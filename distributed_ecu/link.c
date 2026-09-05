/*
 * link.c - UART transport to the Arduino nodes.
 */

#include "link.h"

#include "app.h"
#include "fsl_lpuart.h"
#include "fsl_lpi2c.h"
#include "fsl_common.h"

#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Must be a power of two - the index wrap uses a mask, not a modulo. */
#define LINK_RX_RING_SIZE 256u
#define LINK_RX_RING_MASK (LINK_RX_RING_SIZE - 1u)

/* Compile-time description of a channel. */
typedef struct
{
    LPUART_Type *base;
    uint32_t     instance; /* for CLOCK_GetLpuartClkFreq() */
    IRQn_Type    irqn;
    const char  *name;
} link_config_t;

/* Runtime state of a channel. */
typedef struct
{
    /* Single-producer (ISR) / single-consumer (main) ring. One writer of
     * head and one writer of tail means no critical section is needed. */
    volatile uint8_t  ring[LINK_RX_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t dropped;

    /* Line reassembly, owned by the main context only. */
    char   line[LINK_LINE_MAX];
    size_t lineLen;
} link_state_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

#define LINK_UART_COUNT 2u   /* LINK_ARD1, LINK_ARD2 - LINK_GW is I2C */

/* Consecutive failed gateway transfers before the controller is reset.
 *
 * A FIFO error or a lost arbitration leaves the LPI2C in a state the next
 * transfer inherits, so a single glitch can look permanent. Re-initialising
 * costs microseconds and clears the state machine and both FIFOs. Six, at the
 * 500 ms offline retry rate, is three seconds - long enough that a gateway
 * that is merely unplugged does not churn, short enough to heal itself well
 * inside a demo. */
#define LINK_GW_RECOVER_AFTER 6u

static const link_config_t k_config[LINK_UART_COUNT] = {
    [LINK_ARD1] = {LINK_ARD1_LPUART, LINK_ARD1_INSTANCE, LINK_ARD1_IRQN, "ARD1"},
    [LINK_ARD2] = {LINK_ARD2_LPUART, LINK_ARD2_INSTANCE, LINK_ARD2_IRQN, "ARD2"},
};

static link_state_t s_state[LINK_UART_COUNT];

/* Gateway line reassembly (main context only). */
static char     s_gwLine[LINK_LINE_MAX];
static size_t   s_gwLen;
static uint32_t s_gwDropped;

/* One I2C read can carry several complete lines - at 20 Hz teleop it usually
 * does. The chunk is therefore held between calls and consumed a line at a
 * time; reading a fresh chunk per line would throw away everything after the
 * first newline, which silently drops three key updates out of every four. */
static uint8_t  s_gwChunk[LINK_GW_CHUNK];
static size_t   s_gwChunkLen;
static size_t   s_gwChunkPos;

/* Gateway link health, for the console diagnostics. */
static bool     s_gwOnline;
static uint32_t s_gwPolls;
static uint32_t s_gwFails;
static status_t s_gwLastStatus = kStatus_Success;   /* of the last read */
static uint32_t s_gwConsecFails;
static uint32_t s_gwRecoveries;

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Shared receive handler. Both vectors funnel into this. */
static void link_isr(link_id_t id)
{
    LPUART_Type  *base  = k_config[id].base;
    link_state_t *state = &s_state[id];
    uint32_t      flags = LPUART_GetStatusFlags(base);

    /* An overrun means the hardware already discarded a byte. Clear it so
     * the receiver keeps running, and count it. */
    if ((flags & (uint32_t)kLPUART_RxOverrunFlag) != 0u)
    {
        (void)LPUART_ClearStatusFlags(base, (uint32_t)kLPUART_RxOverrunFlag);
        state->dropped++;
    }

    while ((LPUART_GetStatusFlags(base) & (uint32_t)kLPUART_RxDataRegFullFlag) != 0u)
    {
        uint8_t  byte = LPUART_ReadByte(base);
        uint16_t next = (uint16_t)((state->head + 1u) & LINK_RX_RING_MASK);

        if (next != state->tail)
        {
            state->ring[state->head] = byte;
            state->head              = next;
        }
        else
        {
            state->dropped++; /* main loop is not draining fast enough */
        }
    }

    SDK_ISR_EXIT_BARRIER;
}

void LINK_ARD1_IRQHANDLER(void)
{
    link_isr(LINK_ARD1);
}

void LINK_ARD2_IRQHANDLER(void)
{
    link_isr(LINK_ARD2);
}

/* Defined below, next to the other gateway helpers; LINK_Init() needs it. */
static void link_gw_i2c_init(void);

void LINK_Init(void)
{
    lpuart_config_t config;
    uint32_t        i;

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = LINK_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;

    for (i = 0u; i < LINK_UART_COUNT; i++)
    {
        const link_config_t *cfg   = &k_config[i];
        link_state_t        *state = &s_state[i];

        (void)LPUART_Init(cfg->base, &config, CLOCK_GetLpuartClkFreq(cfg->instance));

        state->head    = 0u;
        state->tail    = 0u;
        state->dropped = 0u;
        state->lineLen = 0u;

        LPUART_EnableInterrupts(cfg->base, (uint32_t)kLPUART_RxDataRegFullInterruptEnable);
        NVIC_SetPriority(cfg->irqn, 3);
        (void)EnableIRQ(cfg->irqn);
    }

    link_gw_i2c_init();

    s_gwLen      = 0u;
    s_gwDropped  = 0u;
    s_gwChunkLen = 0u;
    s_gwChunkPos = 0u;
}

/* Bring up (or reset) the LPI2C master. LPI2C_MasterInit() resets the
 * peripheral, so calling it again is the cheapest full recovery there is. */
static void link_gw_i2c_init(void)
{
    lpi2c_master_config_t i2cCfg;

    LPI2C_MasterGetDefaultConfig(&i2cCfg);
    i2cCfg.baudRate_Hz = LINK_GW_BAUDRATE;
    /* Only one LPI2C instance on this part, so the accessor takes no index. */
    LPI2C_MasterInit(LINK_GW_LPI2C, &i2cCfg, CLOCK_GetLpi2cClkFreq());
}

/* Blocking master write of one line to the gateway. */
static void link_gw_send(const char *line)
{
    lpi2c_master_transfer_t xfer;
    char                    buf[LINK_LINE_MAX + 1u];
    size_t                  n = strlen(line);

    if (n > (LINK_LINE_MAX - 1u))
    {
        n = LINK_LINE_MAX - 1u;
    }
    (void)memcpy(buf, line, n);
    buf[n] = '\n';
    n++;

    (void)memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress   = LINK_GW_ADDR;
    xfer.direction      = kLPI2C_Write;
    xfer.data           = (uint8_t *)buf;
    xfer.dataSize       = n;
    xfer.flags          = (uint32_t)kLPI2C_TransferDefaultFlag;

    if (LPI2C_MasterTransferBlocking(LINK_GW_LPI2C, &xfer) != kStatus_Success)
    {
        s_gwDropped++;
    }
}

/* Read one chunk from the gateway: [len][payload...]. len == 0 means idle.
 * Returns the number of payload bytes placed in @p out. */
static size_t link_gw_read(uint8_t *out, size_t outSize)
{
    lpi2c_master_transfer_t xfer;
    uint8_t                 raw[LINK_GW_CHUNK + 1u];
    size_t                  n;

    (void)memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = LINK_GW_ADDR;
    xfer.direction    = kLPI2C_Read;
    xfer.data         = raw;
    xfer.dataSize     = sizeof(raw);
    xfer.flags        = (uint32_t)kLPI2C_TransferDefaultFlag;

    s_gwLastStatus = LPI2C_MasterTransferBlocking(LINK_GW_LPI2C, &xfer);
    if (s_gwLastStatus != kStatus_Success)
    {
        /* Worth counting after all. A failed transfer and an idle gateway
         * both produce no bytes, and telling them apart is the difference
         * between "nobody is typing" and "the I2C link is dead" - which is
         * the first question you ask when the browser shows nothing. The
         * status is kept too: WHICH way it failed names the wire. */
        s_gwOnline = false;
        s_gwFails++;

        /* A run of failures usually means the controller itself is wedged -
         * a FIFO error or an aborted transfer leaves state the next attempt
         * inherits. Reset it rather than waiting for a power cycle. */
        s_gwConsecFails++;
        if (s_gwConsecFails >= LINK_GW_RECOVER_AFTER)
        {
            s_gwConsecFails = 0u;
            s_gwRecoveries++;
            link_gw_i2c_init();
        }
        return 0u;
    }

    s_gwOnline      = true;
    s_gwConsecFails = 0u;
    s_gwPolls++;

    n = raw[0];
    if ((n == 0u) || (n > LINK_GW_CHUNK))
    {
        return 0u;
    }
    if (n > outSize)
    {
        n = outSize;
    }
    (void)memcpy(out, &raw[1], n);
    return n;
}

void LINK_SendLine(link_id_t id, const char *line)
{
    static const uint8_t newline = (uint8_t)'\n';

    if ((line == NULL) || (id >= LINK_COUNT))
    {
        return;
    }

    if (id == LINK_GW)
    {
        link_gw_send(line);
        return;
    }

    LPUART_WriteBlocking(k_config[id].base, (const uint8_t *)line, strlen(line));
    LPUART_WriteBlocking(k_config[id].base, &newline, 1u);
}

void LINK_Broadcast(const char *line, link_id_t exclude)
{
    uint32_t i;

    for (i = 0u; i < (uint32_t)LINK_COUNT; i++)
    {
        if ((link_id_t)i != exclude)
        {
            LINK_SendLine((link_id_t)i, line);
        }
    }
}

/* Pull one byte out of a channel's ring. Returns false when empty. */
static bool link_ring_get(link_state_t *state, uint8_t *byte)
{
    if (state->tail == state->head)
    {
        return false;
    }

    *byte       = state->ring[state->tail];
    state->tail = (uint16_t)((state->tail + 1u) & LINK_RX_RING_MASK);
    return true;
}

/* Reassemble lines from the gateway, one line per call.
 *
 * Bytes left over from the previous chunk are consumed first, and a new chunk
 * is only fetched once the held one runs out. That is what makes several lines
 * in one I2C read survive: the position is kept, so the next call resumes
 * immediately after the newline instead of discarding the tail. */
static bool link_gw_poll(char *out, size_t outSize)
{
    if (s_gwChunkPos >= s_gwChunkLen)
    {
        s_gwChunkLen = link_gw_read(s_gwChunk, sizeof(s_gwChunk));
        s_gwChunkPos = 0u;
    }

    while (s_gwChunkPos < s_gwChunkLen)
    {
        char ch = (char)s_gwChunk[s_gwChunkPos];
        s_gwChunkPos++;

        if ((ch == '\n') || (ch == '\r'))
        {
            if (s_gwLen == 0u)
            {
                continue;
            }

            size_t n = (s_gwLen < (outSize - 1u)) ? s_gwLen : (outSize - 1u);
            (void)memcpy(out, s_gwLine, n);
            out[n]  = '\0';
            s_gwLen = 0u;
            return true;
        }

        if (s_gwLen < (LINK_LINE_MAX - 1u))
        {
            s_gwLine[s_gwLen] = ch;
            s_gwLen++;
        }
        else
        {
            s_gwLen = 0u;
            s_gwDropped++;
        }
    }

    return false;
}

bool LINK_PollLine(link_id_t id, char *out, size_t outSize)
{
    link_state_t *state;
    uint8_t       byte;

    if ((out == NULL) || (outSize == 0u) || (id >= LINK_COUNT))
    {
        return false;
    }

    if (id == LINK_GW)
    {
        return link_gw_poll(out, outSize);
    }

    state = &s_state[id];

    while (link_ring_get(state, &byte))
    {
        char c = (char)byte;

        if ((c == '\n') || (c == '\r'))
        {
            if (state->lineLen == 0u)
            {
                continue; /* swallow blanks and the second half of CRLF */
            }

            size_t n = (state->lineLen < (outSize - 1u)) ? state->lineLen : (outSize - 1u);
            (void)memcpy(out, state->line, n);
            out[n]         = '\0';
            state->lineLen = 0u;
            return true;
        }

        if (state->lineLen < (LINK_LINE_MAX - 1u))
        {
            state->line[state->lineLen] = c;
            state->lineLen++;
        }
        else
        {
            /* Overlong line: discard and resynchronise on the next newline
             * rather than emitting a truncated message. */
            state->lineLen = 0u;
            state->dropped++;
        }
    }

    return false;
}

uint32_t LINK_GetDroppedCount(link_id_t id)
{
    if (id == LINK_GW)
    {
        return s_gwDropped;
    }
    return (id < LINK_UART_COUNT) ? s_state[id].dropped : 0u;
}

bool LINK_PollByte(link_id_t id, uint8_t *byte)
{
    if ((byte == NULL) || (id >= LINK_UART_COUNT))
    {
        return false;
    }
    return link_ring_get(&s_state[id], byte);
}

bool LINK_GwOnline(void)
{
    return s_gwOnline;
}

void LINK_GwStats(uint32_t *polls, uint32_t *fails)
{
    if (polls != NULL) { *polls = s_gwPolls; }
    if (fails != NULL) { *fails = s_gwFails; }
}

const char *LINK_I2CStatusName(int32_t status)
{
    static char other[24];

    switch (status)
    {
        case kStatus_Success:               return "ok";
        case kStatus_LPI2C_Busy:            return "BUSY";
        case kStatus_LPI2C_Idle:            return "IDLE";
        case kStatus_LPI2C_Nak:             return "NAK";
        case kStatus_LPI2C_FifoError:       return "FIFO ERROR";
        case kStatus_LPI2C_BitError:        return "BIT ERROR";
        case kStatus_LPI2C_ArbitrationLost: return "ARB LOST";
        case kStatus_LPI2C_PinLowTimeout:   return "PIN LOW";
        case kStatus_LPI2C_NoTransferInProgress: return "NO TRANSFER";
        case kStatus_LPI2C_DmaRequestFail:  return "DMA FAIL";
        case kStatus_LPI2C_Timeout:         return "TIMEOUT";
        default:
            (void)snprintf(other, sizeof(other), "status %ld", (long)status);
            return other;
    }
}

const char *LINK_I2CStatusHint(int32_t status)
{
    switch (status)
    {
        case kStatus_Success:
            return "that transfer succeeded";

        case kStatus_LPI2C_Nak:
            return "the bus clocked but NOTHING ACKED that address. For 0x42: the "
                   "ESP32 is unpowered, not running esp32_gateway.ino, or its "
                   "GPIO21/GPIO22 are not on the two pins the MCX is driving";

        case kStatus_LPI2C_FifoError:
            /* This is the one worth spelling out. The SDK ranks NAK above FIFO
             * error, so this status means NDF was NOT set: the slave answered
             * its address and the transfer came apart afterwards. Sending
             * someone to check wiring on this is sending them the wrong way. */
            return "NOT a wiring fault - a NAK would outrank this, so the slave DID "
                   "ACK and the DATA phase then broke. Suspect the far end's timing: "
                   "an ESP32 that stretches SCL while its onRequest runs, or a "
                   "previous odd-sized read leaving it out of frame. Try "
                   "LINK_GW_BAUDRATE at 100000, and check the ESP32's own 'i' "
                   "counters - reads climbing there proves it is being addressed";

        case kStatus_LPI2C_Busy:
            return "SDA or SCL read LOW before the START: a missing pull-up, a wire "
                   "on the wrong header pin, or no common ground with the ESP32";

        case kStatus_LPI2C_Timeout:
        case kStatus_LPI2C_PinLowTimeout:
            return "a line did not come back HIGH in time: the 2k pull-ups are "
                   "missing or not on these pins, or a slave is holding SCL down";

        case kStatus_LPI2C_BitError:
        case kStatus_LPI2C_ArbitrationLost:
            return "the level driven was not seen back on the pin: a short, another "
                   "master, or the wire is not on P3_27/P3_28 at all";

        default:
            return "unexpected LPI2C status - look it up in fsl_lpi2c.h";
    }
}

const char *LINK_GwFailReason(void)
{
    return LINK_I2CStatusName((int32_t)s_gwLastStatus);
}

const char *LINK_GwFailHint(void)
{
    return LINK_I2CStatusHint((int32_t)s_gwLastStatus);
}

uint32_t LINK_GwRecoveries(void)
{
    return s_gwRecoveries;
}

void LINK_GwResync(void)
{
    link_gw_i2c_init();

    s_gwChunkLen    = 0u;
    s_gwChunkPos    = 0u;
    s_gwLen         = 0u;
    s_gwConsecFails = 0u;
}

const char *LINK_GetName(link_id_t id)
{
    if (id == LINK_GW)
    {
        return "GW";
    }
    return (id < LINK_UART_COUNT) ? k_config[id].name : "?";
}
