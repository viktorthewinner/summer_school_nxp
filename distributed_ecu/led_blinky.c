/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * Four-node serial hub: FRDM-MCXA153 + 2 Arduinos + ESP32 gateway.
 *
 * A line typed in any serial monitor, or sent from the laptop over WiFi,
 * reaches every other node tagged with its source.
 *
 *   LPUART0  P0_2  / P0_3    MCU-Link VCOM, 115200   this terminal
 *   LPUART2  P3_14 / P3_15   Arduino 1,      38400   J2 pins 4 / 2
 *   LPUART1  P1_8  / P1_9    Arduino 2,      38400   J2 pins 18 / 20
 *   LPI2C0   P3_27 / P3_28   ESP32 gateway, 400 kHz  mikroBUS J5 pins 5 / 6
 *
 * Wire format: "[SRC] text", SRC being MCX, ARD1, ARD2 or GW.
 *
 * The ESP32 is a gateway, not a controller: remote traffic arrives here as a
 * request and this node decides what to do with it. That distinction is what
 * keeps a dropped WiFi link from being a safety problem.
 *
 * The red LED toggles once per second as a liveness indicator.
 *
 * Drive commands. Typing "d 40 40" here sends D,40,40 to ARD2 and then keeps
 * repeating it at 20 Hz, because ACT stops the motors after 300 ms without a
 * command. That repeat is the point: a command you type once is not a command
 * the actuator can safely act on, and the timeout is what makes pulling this
 * board's cable a safe event rather than a runaway.
 *
 * ---------------------------------------------------------------------------
 * TELEOP - the browser drives, but this board decides
 *
 * The laptop sends key STATE, not motor commands. W A S D and space arrive
 * here as flags and nothing else; the throttle ramp, the steering mix, the
 * duty cap, the obstacle veto and the two watchdogs all live on this node.
 * That is the point of the ESP32 being a gateway: WiFi can drop, stutter or
 * be attacked and the worst it can do is stop sending keys.
 *
 *   from GW   K,<w>,<a>,<s>,<d>,<horn>   key state, 10 Hz plus on change
 *             M,<text>                   message for the ACT display
 *             E                          stop and disarm
 *
 *   to GW     T,<...>                    telemetry, 10 Hz - see telemetry_send
 *
 *   to ARD2   D,<left>,<right>           20 Hz while the browser is alive
 *   to ARD1   H,<0|1>                    horn, repeated so it cannot stick on
 *   from ARD1 P,<front_cm>,<back_cm>,<pir>
 *   from ARD2 S,<left>,<right>,<age_ms>
 *
 * There is no reverse anywhere in this car - a low-side switch conducts one
 * way - so S is a brake, not a gear. A turn on the spot is one side driven
 * and the other held off.
 *
 * Three timeouts in series, each independent of the ones above it:
 *
 *   browser  stops sending keys when the tab loses focus
 *   VCU      TELEOP_TIMEOUT_MS without a K line -> throttle 0, stop sending
 *   ACT      300 ms without a D line            -> motors off in hardware
 *
 * The file name is inherited from the SDK led_blinky example this project was
 * generated from; rename it in CMakeLists.txt if you prefer.
 */

#include "board.h"
#include "app.h"
#include "link.h"

#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#include "fsl_lpi2c.h"
#include "fsl_port.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* The debug console owns transmit on LPUART0. We only read from it, so
 * polling the receiver here does not conflict with PRINTF. */
#define CONSOLE_LPUART ((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR)

#define KEY_BACKSPACE 0x08u
#define KEY_DELETE    0x7Fu

/* How often the gateway is polled. I2C transactions are blocking, so this is
 * rate-limited rather than run every loop: 50 Hz is one 49-byte read every
 * 20 ms, about 6 % of the bus, and it bounds keypress latency at 20 ms. */
#define GW_POLL_MS      20u

/* ...but back right off once the gateway stops answering. Every failed
 * transfer burns the full I2C_RETRY_TIMES wait, and at 12 MHz a dead bus
 * polled at 50 Hz would eat the loop the wheels depend on. Retry twice a
 * second instead: it still recovers on its own within half a second of the
 * wire going back in, and the drive loop never notices. */
#define GW_RETRY_MS    500u

/* Drive command repeat. ACT's timeout is 300 ms; 20 Hz leaves plenty of margin
 * for a dropped line without the motors stuttering. */
#define DRIVE_REPEAT_MS 50u
#define DRIVE_DUTY_MAX  70u
#define LED_BLINK_MS    500u

/* ---------------------------------------------------------------------------
 * Teleop tuning. Everything the car feels like is in these ten numbers.
 * ------------------------------------------------------------------------ */

/* One control update, and one D, line to ACT, every tick. */
#define TELEOP_TICK_MS     50u

/* No key line for this long and the browser is assumed gone. Four times the
 * 10 Hz keepalive, so it survives three lost lines before it fires. */
#define TELEOP_TIMEOUT_MS  400u

#define THROTTLE_MAX       65u   /* under DRIVE_DUTY_MAX, on purpose        */
#define THROTTLE_RISE       3u   /* per tick: standstill to full in ~1.1 s  */
#define THROTTLE_COAST      2u   /* per tick with W released - it rolls off */

/* How much duty comes off the inside pair in a turn. Steering is the
 * difference between the two channels and nothing else. */
#define TURN_BIAS          28u

/* Turning on the spot: one side driven, the other held off. Below about 40 %
 * a stationary car will not break static friction and only buzzes. */
#define PIVOT_DUTY         45u

/* Front sonar veto. Closer than this and forward drive is refused - but a
 * pivot is still allowed, because with no reverse turning away is the only
 * way out. Ignored while PERC is silent; see teleop_guard(). */
#define GUARD_STOP_CM      25
#define SENSOR_STALE_MS   500u

#define HORN_REPEAT_MS    250u   /* PERC drops the horn after 500 ms silent */
#define HEARTBEAT_MS     1000u   /* V, line to BOTH Arduinos, always        */
#define TELEMETRY_MS      100u
#define IMU_SAMPLE_MS      50u

/* MPU6050 - the VCU's only sensor, and the only feedback in the whole car. */
#define IMU_ADDR        0x68u
#define IMU_WHO_AM_I    0x75u
#define IMU_PWR_MGMT_1  0x6Bu
#define IMU_ACCEL_XOUT  0x3Bu   /* 14 bytes: accel, temp, gyro */

/*******************************************************************************
 * Variables
 ******************************************************************************/

static char     s_consoleLine[LINK_LINE_MAX];
static size_t   s_consoleLen;
static uint32_t s_gwPollAt;

static volatile uint32_t s_ms;      /* free-running millisecond counter */

static char     s_actStatus[LINK_LINE_MAX];   /* newest S, line from ACT */
static uint32_t s_actPrintAt;
static uint32_t s_actAt;            /* when ACT last said anything        */

static bool     s_imuAwake;
static bool     s_imuOk;            /* false = absent; telemetry sends 0s */
static uint32_t s_imuAt;
static uint32_t s_imuRetryAt;
static int16_t  s_imuAx, s_imuAy, s_imuAz, s_imuGz;

static uint32_t s_driveSentAt;
static uint8_t  s_driveL;
static uint8_t  s_driveR;
static bool     s_driveArmed;       /* false = send nothing, ACT times out */

/* Perception, as last reported by PERC. -1 cm means no echo, which on an
 * ultrasonic means "nothing within range" and not "sensor broken". */
static int      s_front = -1;
static int      s_back  = -1;
static uint8_t  s_pir;
static uint32_t s_percAt;

/* Teleop input state. Set only from a K, line, cleared by the watchdog. */
static bool     s_keyW, s_keyA, s_keyS, s_keyD, s_keyHorn;
static uint32_t s_keyAt;
static bool     s_teleopLive;       /* true = the browser owns the wheels  */
static uint32_t s_teleopTickAt;
static uint8_t  s_throttle;
static bool     s_guard;            /* front sonar is currently vetoing W  */

static bool     s_horn;
static uint32_t s_hornSentAt;
static uint32_t s_beatAt;

static uint32_t s_telemetryAt;

/* Link diagnostics. Each node's state is remembered so the console can report
 * the CHANGE rather than the state - a line the moment something comes up or
 * goes away, which is what you actually want to see, instead of a status you
 * have to sit and watch. */
#define DIAG_PERIOD_MS   2000u
#define DIAG_SILENT_MS    600u   /* a node that has not spoken for this long */

static bool     s_diagLoud = true;   /* periodic summary on/off - 'diag'   */
static uint32_t s_diagAt;
static int      s_wasPerc = -1;      /* -1 = never seen, 0 = down, 1 = up  */
static int      s_wasAct  = -1;
static int      s_wasGw   = -1;
static int      s_wasKeys = -1;

/* Raw byte view, per channel. While on, that channel's bytes go to the
 * console instead of being assembled into lines - so it stops working as
 * a link for as long as it is on. That is the point. */
static bool     s_raw[2];

/*******************************************************************************
 * Code
 ******************************************************************************/

/* ===========================================================================
 * Hard fault reporter
 *
 * The SDK's HardFault_Handler is `b .` - an infinite loop. Under a debugger
 * that lands you on that instruction in startup_MCXA153.S, which tells you a
 * fault happened and nothing whatsoever about where or why. Without a
 * debugger it looks identical to a board that is simply dead.
 *
 * This replaces it (the SDK's is .weak) and prints the fault status registers
 * and the stacked return address over the VCOM before parking. Feed the PC it
 * prints to:
 *
 *     arm-none-eabi-addr2line -e debug/distributed_ecu.elf <pc>
 *
 * and you have the source line that faulted.
 *
 * The naked wrapper picks the right stack - MSP or PSP, from bit 2 of the
 * EXC_RETURN value in LR - and hands the exception frame to the C function.
 * Getting that wrong is the classic way to print convincing nonsense.
 * ======================================================================== */

extern uint32_t __StackLimit;   /* from the linker script */

void hard_fault_report(uint32_t *frame)
{
    uint32_t cfsr = SCB->CFSR;

    PRINTF("\r\n\r\n*** HARD FAULT ***\r\n");
    PRINTF("  PC   %08X   <- addr2line this one\r\n", (unsigned int)frame[6]);
    PRINTF("  LR   %08X   caller\r\n", (unsigned int)frame[5]);
    PRINTF("  PSR  %08X\r\n", (unsigned int)frame[7]);
    PRINTF("  R0-R3 %08X %08X %08X %08X   R12 %08X\r\n",
           (unsigned int)frame[0], (unsigned int)frame[1],
           (unsigned int)frame[2], (unsigned int)frame[3],
           (unsigned int)frame[4]);
    PRINTF("  HFSR %08X   CFSR %08X\r\n",
           (unsigned int)SCB->HFSR, (unsigned int)cfsr);

    /* The bits that actually name the fault. */
    if ((cfsr & (1u << 0))  != 0u) { PRINTF("  IACCVIOL  - fetched from a bad address\r\n"); }
    if ((cfsr & (1u << 1))  != 0u) { PRINTF("  DACCVIOL  - data access to a bad address\r\n"); }
    if ((cfsr & (1u << 7))  != 0u) { PRINTF("  MMARVALID - at %08X\r\n", (unsigned int)SCB->MMFAR); }
    if ((cfsr & (1u << 8))  != 0u) { PRINTF("  IBUSERR   - instruction bus error\r\n"); }
    if ((cfsr & (1u << 9))  != 0u) { PRINTF("  PRECISERR - bad data address\r\n"); }
    if ((cfsr & (1u << 10)) != 0u) { PRINTF("  IMPRECISERR - late bus error, PC is approximate\r\n"); }
    if ((cfsr & (1u << 15)) != 0u) { PRINTF("  BFARVALID - at %08X\r\n", (unsigned int)SCB->BFAR); }
    if ((cfsr & (1u << 16)) != 0u) { PRINTF("  UNDEFINSTR - undefined instruction\r\n"); }
    if ((cfsr & (1u << 17)) != 0u) { PRINTF("  INVSTATE  - bad execution state, usually a corrupt LR\r\n"); }
    if ((cfsr & (1u << 18)) != 0u) { PRINTF("  INVPC     - bad return, usually a smashed stack\r\n"); }
    if ((cfsr & (1u << 24)) != 0u) { PRINTF("  UNALIGNED - unaligned access\r\n"); }
    if ((cfsr & (1u << 25)) != 0u) { PRINTF("  DIVBYZERO - divide by zero\r\n"); }

    /* A frame pointer near the bottom of the stack means it overflowed, and
     * then everything above is suspect - including the PC just printed. */
    PRINTF("  SP   %08X   limit %08X%s\r\n",
           (unsigned int)(uint32_t)frame, (unsigned int)(uint32_t)&__StackLimit,
           ((uint32_t)frame < ((uint32_t)&__StackLimit + 64u))
               ? "   <- STACK OVERFLOW" : "");

    PRINTF("\r\n  Halted. Reset the board.\r\n");

    for (;;)
    {
    }
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "  tst   lr, #4          \n" /* EXC_RETURN bit 2: which stack? */
        "  ite   eq              \n"
        "  mrseq r0, msp         \n"
        "  mrsne r0, psp         \n"
        "  b     hard_fault_report\n");
}

/* 1 kHz. Everything timed in this file hangs off s_ms.
 *
 * Note this handler existed before but SysTick was never started, so the
 * "heartbeat" LED was not actually blinking. It is now. */
void SysTick_Handler(void)
{
    s_ms++;

    if ((s_ms % LED_BLINK_MS) == 0u)
    {
        GPIO_PortToggle(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
    }
}

static uint32_t now_ms(void)
{
    return s_ms;
}

/* Reprint the prompt plus whatever the user had half-typed, so an incoming
 * message does not appear to swallow their input. */
static void console_redraw_prompt(void)
{
    PRINTF("> ");

    if (s_consoleLen > 0u)
    {
        s_consoleLine[s_consoleLen] = '\0';
        PRINTF("%s", s_consoleLine);
    }
}

/* Non-blocking read of one byte typed in this terminal.
 * GETCHAR() blocks, which would stall the loop and stop every other node
 * being serviced, so read the peripheral directly. */
static bool console_try_read(uint8_t *byte)
{
    uint32_t flags = LPUART_GetStatusFlags(CONSOLE_LPUART);

    if ((flags & (uint32_t)kLPUART_RxOverrunFlag) != 0u)
    {
        (void)LPUART_ClearStatusFlags(CONSOLE_LPUART, (uint32_t)kLPUART_RxOverrunFlag);
    }

    if ((flags & (uint32_t)kLPUART_RxDataRegFullFlag) == 0u)
    {
        return false;
    }

    *byte = LPUART_ReadByte(CONSOLE_LPUART);
    return true;
}

/* =========================================================================
 * Bench self-test. Everything here is driven from the console and touches no
 * other node, so the VCU can be proven on its own before an Arduino is near it.
 * ========================================================================= */

/* LPI2C0 is already a configured master - LINK_Init() did it for the gateway,
 * so these helpers just borrow it. */
/* Status of the last i2c_probe(), so the scan can report ITS OWN result
 * instead of whatever the background gateway poll last saw. */
static status_t s_probeStatus = kStatus_Success;

/* Presence check: START, address, ONE byte, STOP.
 *
 * Two things this must not be, both learned the hard way on this bus.
 *
 * NOT a one-byte READ. The ESP32 answers every request with a fixed
 * LINK_GW_CHUNK + 1 bytes; take one and the other 48 stay in its transmit
 * buffer, so every later poll reads a frame behind until that board is power
 * cycled. The scan you run BECAUSE the link looks broken would break it.
 *
 * NOT a zero-length write either, which is the textbook scan and what this
 * tried next. LPI2C answers an address-only transfer with
 * kStatus_LPI2C_FifoError - measured, against a gateway that was at that
 * moment serving five thousand successful reads - so every address came back
 * a FIFO error and the scan reported an empty bus while the link was provably
 * up. This command FIFO wants a data phase to go with the START.
 *
 * So: write one byte, and make it a newline. On the gateway it lands in
 * onI2CReceive(), where a newline with an empty assembly buffer does nothing
 * at all and with a partial line merely terminates it. On the MPU6050 a lone
 * byte only sets the register pointer; no register is written. Harmless to
 * both, and an absent device still NAKs its address, which is all a scan
 * needs to see. */
static bool i2c_probe(uint8_t addr)
{
    lpi2c_master_transfer_t xfer;
    uint8_t                 quiet = (uint8_t)'\n';

    (void)memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = addr;
    xfer.direction    = kLPI2C_Write;
    xfer.data         = &quiet;
    xfer.dataSize     = 1u;
    xfer.flags        = (uint32_t)kLPI2C_TransferDefaultFlag;

    s_probeStatus = LPI2C_MasterTransferBlocking(LINK_GW_LPI2C, &xfer);

    /* A FIFO error leaves the controller wedged and the next probe inherits
     * it, which is how one bad address turns into a whole empty bus. */
    if (s_probeStatus == kStatus_LPI2C_FifoError)
    {
        LINK_GwResync();
    }

    return (s_probeStatus == kStatus_Success);
}

static bool i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    lpi2c_master_transfer_t xfer;

    (void)memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress   = addr;
    xfer.direction      = kLPI2C_Read;
    xfer.subaddress     = reg;
    xfer.subaddressSize = 1u;
    xfer.data           = buf;
    xfer.dataSize       = len;
    xfer.flags          = (uint32_t)kLPI2C_TransferDefaultFlag;

    return (LPI2C_MasterTransferBlocking(LINK_GW_LPI2C, &xfer) == kStatus_Success);
}

static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    lpi2c_master_transfer_t xfer;

    (void)memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress   = addr;
    xfer.direction      = kLPI2C_Write;
    xfer.subaddress     = reg;
    xfer.subaddressSize = 1u;
    xfer.data           = &val;
    xfer.dataSize       = 1u;
    xfer.flags          = (uint32_t)kLPI2C_TransferDefaultFlag;

    return (LPI2C_MasterTransferBlocking(LINK_GW_LPI2C, &xfer) == kStatus_Success);
}

/* ---------------------------------------------------------------------------
 * The IMU, sampled continuously rather than only on the 'imu' command.
 *
 * It is the only feedback device in the whole car, so the telemetry stream
 * carries it whether or not anyone asked. If the module is absent the reads
 * fail, s_imuOk stays false and the telemetry sends four zeros - a live
 * MPU6050 never reads exactly zero on all four axes, so that doubles as the
 * "no IMU" marker the browser looks for.
 * ------------------------------------------------------------------------ */

static bool imu_wake(void)
{
    uint8_t who = 0u;

    if (!i2c_read_regs(IMU_ADDR, IMU_WHO_AM_I, &who, 1u))
    {
        return false;
    }
    if (!i2c_write_reg(IMU_ADDR, IMU_PWR_MGMT_1, 0x00u)) /* it boots asleep */
    {
        return false;
    }

    s_imuAwake = true;
    return true;
}

static void imu_service(void)
{
    uint8_t raw[14];

    if ((now_ms() - s_imuAt) < IMU_SAMPLE_MS)
    {
        return;
    }
    s_imuAt = now_ms();

    if (!s_imuAwake)
    {
        /* Retry quietly - it may be plugged in later - but once a second, not
         * twenty times. A failing transfer still ties up the bus the gateway
         * poll needs. */
        if ((now_ms() - s_imuRetryAt) < 1000u)
        {
            return;
        }
        s_imuRetryAt = now_ms();
        (void)imu_wake();
        return;
    }

    if (!i2c_read_regs(IMU_ADDR, IMU_ACCEL_XOUT, raw, sizeof(raw)))
    {
        s_imuOk    = false;
        s_imuAwake = false;    /* it may have been unplugged - wake it again */
        return;
    }

    s_imuAx = (int16_t)(((uint16_t)raw[0]  << 8) | raw[1]);
    s_imuAy = (int16_t)(((uint16_t)raw[2]  << 8) | raw[3]);
    s_imuAz = (int16_t)(((uint16_t)raw[4]  << 8) | raw[5]);
    s_imuGz = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);
    s_imuOk = true;
}

static void test_i2c_scan(void)
{
    unsigned found = 0u;

    PRINTF("\r\n-- I2C scan on LPI2C0 (P3_27 SCL / P3_28 SDA)\r\n");

    for (uint8_t a = 0x08u; a < 0x78u; a++)
    {
        if (i2c_probe(a))
        {
            found++;
            PRINTF("   0x%02X", a);
            if (a == LINK_GW_ADDR) { PRINTF("  ESP32 gateway"); }
            if (a == IMU_ADDR)     { PRINTF("  MPU6050"); }
            if (a == 0x69u)        { PRINTF("  MPU6050 with AD0 high - tie AD0 to GND for 0x68"); }
            PRINTF("\r\n");
        }
    }

    /* Probe the gateway once more on its own, so what is reported below is
     * THIS scan's verdict on THAT address - not the background poll's, which
     * is what made a failed scan print "the gateway is answering". */
    (void)i2c_probe(LINK_GW_ADDR);

    if (found == 0u)
    {
        uint32_t polls, fails;

        LINK_GwStats(&polls, &fails);

        PRINTF("   nothing answered.\r\n");
        PRINTF("   Probing 0x%02X on its own gives %s:\r\n",
               (unsigned int)LINK_GW_ADDR, LINK_I2CStatusName((int32_t)s_probeStatus));
        PRINTF("   %s.\r\n", LINK_I2CStatusHint((int32_t)s_probeStatus));

        /* Believe the poll loop over this scan. It runs continuously, against
         * the real gateway, with the real transfer size. If it is succeeding
         * then the bus, the pull-ups, the pins and the address are all proven,
         * and an empty scan is this tool being wrong - not the link. */
        if (LINK_GwOnline() || (polls > 0u))
        {
            PRINTF("   BUT the gateway poll loop has %u successful reads%s.\r\n",
                   (unsigned)polls, LINK_GwOnline() ? " and is up right now" : "");
            PRINTF("   The bus therefore WORKS - trust that over this scan, and\r\n");
            PRINTF("   use 'gw', which reads the gateway the way the poll does.\r\n");
        }

        if (s_probeStatus == kStatus_LPI2C_Nak)
        {
            PRINTF("   A NAK is the electrical case: SDA is mikroBUS J5 pin 6 and SCL\r\n");
            PRINTF("   is J5 pin 5 (silkscreen SDA / SCL). Both must idle at 3.3 V\r\n");
            PRINTF("   against the star point, through the two 2k pull-ups to J3-8.\r\n");
            PRINTF("   About 2.3 V on BOTH is a device on the bus with no VDD - its\r\n");
            PRINTF("   clamp diodes eat the pull-up. Unplug one node at a time.\r\n");
        }
    }
    else
    {
        PRINTF("   %u device(s).\r\n", found);
    }

    /* Scanning walks every address on the bus the gateway lives on. Put the
     * controller and the line assembly back to a known state before the poll
     * loop resumes. */
    LINK_GwResync();
}

/* One raw gateway read, printed. The poll path cannot show this: it yields
 * lines or nothing, so a slave that answers with the WRONG BYTES looks the
 * same as one that answers with none. The first byte is the payload length
 * and must be 0 (idle) or 1..LINK_GW_CHUNK. Anything else - 0xFF, a stuck
 * value, ASCII - means the ESP32 is out of frame or not running the sketch. */
static void test_gw_read(void)
{
    lpi2c_master_transfer_t xfer;
    uint8_t                 raw[LINK_GW_CHUNK + 1u];
    status_t                s;

    PRINTF("\r\n-- gateway raw read, 0x%02X, %u bytes\r\n",
           (unsigned int)LINK_GW_ADDR, (unsigned int)sizeof(raw));

    (void)memset(raw, 0, sizeof(raw));
    (void)memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = LINK_GW_ADDR;
    xfer.direction    = kLPI2C_Read;
    xfer.data         = raw;
    xfer.dataSize     = sizeof(raw);
    xfer.flags        = (uint32_t)kLPI2C_TransferDefaultFlag;

    s = LPI2C_MasterTransferBlocking(LINK_GW_LPI2C, &xfer);

    PRINTF("   status %s - %s.\r\n", LINK_I2CStatusName((int32_t)s),
           LINK_I2CStatusHint((int32_t)s));

    if (s == kStatus_Success)
    {
        PRINTF("   len byte %u %s\r\n", (unsigned)raw[0],
               (raw[0] == 0u)              ? "(idle - nothing pending, this is normal)" :
               (raw[0] <= LINK_GW_CHUNK)   ? "(plausible)" :
                                             "(IMPOSSIBLE - the slave is out of frame)");

        PRINTF("   payload:");
        for (unsigned i = 1u; i < sizeof(raw); i++)
        {
            if (((i - 1u) % 16u) == 0u) { PRINTF("\r\n     "); }
            PRINTF(" %02X", (unsigned)raw[i]);
        }
        PRINTF("\r\n     \"");
        for (unsigned i = 1u; i < sizeof(raw); i++)
        {
            PUTCHAR(((raw[i] >= 0x20u) && (raw[i] < 0x7Fu)) ? (char)raw[i] : '.');
        }
        PRINTF("\"\r\n");
    }

    LINK_GwResync();
}

static void test_imu(void)
{
    uint8_t who = 0u;
    uint8_t raw[14];
    int16_t ax, ay, az, gx, gy, gz;
    int16_t t;

    PRINTF("\r\n-- MPU6050\r\n");

    if (!i2c_read_regs(IMU_ADDR, IMU_WHO_AM_I, &who, 1u))
    {
        PRINTF("   no answer at 0x%02X. Run 'i2c' first.\r\n", IMU_ADDR);
        return;
    }

    PRINTF("   WHO_AM_I = 0x%02X %s\r\n", who,
           (who == 0x68u) ? "(MPU6050)" :
           (who == 0x70u) ? "(MPU6500 - different register map)" : "(unexpected)");

    /* It boots asleep. Nothing reads sensibly until PWR_MGMT_1 is cleared. */
    if (!s_imuAwake)
    {
        if (!i2c_write_reg(IMU_ADDR, IMU_PWR_MGMT_1, 0x00u))
        {
            PRINTF("   could not wake it.\r\n");
            return;
        }
        s_imuAwake = true;
        PRINTF("   woken (PWR_MGMT_1 = 0)\r\n");
    }

    for (unsigned i = 0u; i < 10u; i++)
    {
        uint32_t t0 = now_ms();

        if (!i2c_read_regs(IMU_ADDR, IMU_ACCEL_XOUT, raw, sizeof(raw)))
        {
            PRINTF("   read failed\r\n");
            return;
        }

        ax = (int16_t)(((uint16_t)raw[0]  << 8) | raw[1]);
        ay = (int16_t)(((uint16_t)raw[2]  << 8) | raw[3]);
        az = (int16_t)(((uint16_t)raw[4]  << 8) | raw[5]);
        t  = (int16_t)(((uint16_t)raw[6]  << 8) | raw[7]);
        gx = (int16_t)(((uint16_t)raw[8]  << 8) | raw[9]);
        gy = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
        gz = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);

        /* Defaults: accel +-2 g = 16384 LSB/g, gyro +-250 dps = 131 LSB/dps,
         * temperature = raw/340 + 36.53 degC. */
        PRINTF("   a %6d %6d %6d   g %6d %6d %6d   %d.%01d C\r\n",
               ax, ay, az, gx, gy, gz,
               (int)(t / 340 + 36), (int)((t % 340) * 10 / 340));

        while ((now_ms() - t0) < 200u) { }
    }

    PRINTF("   Flat and still: az near +16384, gyro all near zero.\r\n");
    PRINTF("   Turn the board about its vertical axis and gz should swing.\r\n");
}

/* The red LED is muxed by the generated pin_mux.c. Green, blue and the two
 * buttons are not, so they are set up here instead - in this file, on purpose,
 * because Config Tools would regenerate pin_mux.c and drop them again. */
/* Turn on the MCU's own pull-ups on the I2C bus.
 *
 * The generated pin_mux.c sets these two pins kPORT_PullDisable, which is the
 * correct default for a board that has real pull-up resistors on the bus.
 * This one now does - two 2 kOhm to 3V3, see the schema - so these internal
 * ones are belt and braces, not the mechanism.
 *
 * Internal pull-ups are weak, tens of kOhm against the 2.2 k these want, so
 * this is insurance and not a substitute: with them alone the bus may need
 * LINK_GW_BAUDRATE dropped to 100 kHz, and long wires may not work at all.
 * What it does guarantee is that the bus idles HIGH instead of floating, so
 * a scan result means something and a meter reads 3.3 V rather than noise.
 *
 * In this file rather than pin_mux.c on purpose - Config Tools regenerates
 * that file and would drop this, along with the six UART pins. */
static void i2c_pullups_init(void)
{
    const port_pin_config_t i2c = {
        kPORT_PullUp,                /* the point of the whole function */
        kPORT_LowPullResistor,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable,  /* the filter would blunt 400 kHz edges */
        kPORT_OpenDrainEnable,       /* mandatory for I2C - never push-pull  */
        kPORT_LowDriveStrength,
        kPORT_NormalDriveStrength,
        kPORT_MuxAlt2,               /* keep LPI2C0, do not steal the pin    */
        kPORT_InputBufferEnable,
        kPORT_InputNormal,
        kPORT_UnlockRegister,
    };

    PORT_SetPinConfig(PORT3, 27U, &i2c);   /* SCL, J5 pin 5 */
    PORT_SetPinConfig(PORT3, 28U, &i2c);   /* SDA, J5 pin 6 */
}

static void test_extra_pins_init(void)
{
    /* Eleven fields, in the order the generated pin_mux.c uses. Supplying
     * fewer silently zero-fills the tail, which lands inputBuffer in the wrong
     * slot and leaves the buttons reading nothing. */
    const port_pin_config_t out = {
        kPORT_PullDisable,           /* pull select        */
        kPORT_LowPullResistor,       /* pull value         */
        kPORT_FastSlewRate,          /* slew rate          */
        kPORT_PassiveFilterDisable,  /* passive filter     */
        kPORT_OpenDrainDisable,      /* open drain         */
        kPORT_LowDriveStrength,      /* drive strength     */
        kPORT_NormalDriveStrength,   /* drive strength 1   */
        kPORT_MuxAlt0,               /* mux: plain GPIO    */
        kPORT_InputBufferEnable,     /* input buffer       */
        kPORT_InputNormal,           /* input not inverted */
        kPORT_UnlockRegister,        /* PCR not locked     */
    };
    const port_pin_config_t in = {
        kPORT_PullUp,                /* buttons are active low, so pull up */
        kPORT_LowPullResistor,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterEnable,   /* debounces the worst of the contact  */
        kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength,
        kPORT_NormalDriveStrength,
        kPORT_MuxAlt0,
        kPORT_InputBufferEnable,     /* without this the pin reads nothing  */
        kPORT_InputNormal,
        kPORT_UnlockRegister,
    };
    const gpio_pin_config_t gpioOut = { kGPIO_DigitalOutput, 1 };
    const gpio_pin_config_t gpioIn  = { kGPIO_DigitalInput, 0 };

    /* Ungate and un-reset GPIO1 and GPIO3 BEFORE touching either of them.
     *
     * GPIO_PinInit() looks like it does this for you - it calls
     * GPIO_PortClockEnable() and then releases the reset. The clock part
     * works. The reset part does not, on this part:
     *
     *     fsl_gpio.c:  #if defined(GPIO_RSTS)   -> GPIO_RESETS_ARRAY
     *     MCXA153:     #define GPIO_RSTS_N ...
     *
     * The names do not match, so GPIO_RESETS_ARRAY is never defined and the
     * release is compiled out. GPIO1 therefore stays in reset, and the first
     * write to its PDDR is a bus fault at 0x40103054 - which is exactly what
     * the hard fault handler reported.
     *
     * pin_mux.c releases PORT1 and PORT3, which are different modules from
     * GPIO1 and GPIO3 and do not cover this. GPIO3 happens to be released
     * already by the board's red-LED init, which is why only SW3 on P1_7
     * ever tripped it - and why this looked like a link problem for so long.
     * Both are listed here anyway: relying on someone else's side effect is
     * how this stayed hidden. */
    CLOCK_EnableClock(kCLOCK_GateGPIO1);
    CLOCK_EnableClock(kCLOCK_GateGPIO3);
    RESET_ReleasePeripheralReset(kGPIO1_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kGPIO3_RST_SHIFT_RSTn);

    PORT_SetPinConfig(PORT3, 13U, &out);   /* green LED */
    PORT_SetPinConfig(PORT3, 0U,  &out);   /* blue  LED */
    PORT_SetPinConfig(PORT3, 29U, &in);    /* SW2 */
    PORT_SetPinConfig(PORT1, 7U,  &in);    /* SW3 */

    GPIO_PinInit(GPIO3, 13U, &gpioOut);
    GPIO_PinInit(GPIO3, 0U,  &gpioOut);
    GPIO_PinInit(GPIO3, 29U, &gpioIn);
    GPIO_PinInit(GPIO1, 7U,  &gpioIn);
}

static void test_leds(void)
{
    PRINTF("\r\n-- RGB LED: red, green, blue, then off\r\n");

    struct { GPIO_Type *g; uint32_t p; const char *n; } led[3] = {
        { GPIO3, 12U, "red" }, { GPIO3, 13U, "green" }, { GPIO3, 0U, "blue" },
    };

    for (unsigned i = 0u; i < 3u; i++)
    {
        PRINTF("   %s\r\n", led[i].n);
        GPIO_PinWrite(led[i].g, led[i].p, 0U);      /* active low */
        for (uint32_t t0 = now_ms(); (now_ms() - t0) < 700u; ) { }
        GPIO_PinWrite(led[i].g, led[i].p, 1U);
    }

    PRINTF("   Red is the heartbeat and resumes blinking on its own.\r\n");
}

static void test_buttons(void)
{
    uint32_t end = now_ms() + 8000u;
    uint32_t s2 = 2u, s3 = 2u;

    PRINTF("\r\n-- SW2 / SW3, 8 seconds. Press them.\r\n");

    while ((int32_t)(now_ms() - end) < 0)
    {
        uint32_t a = GPIO_PinRead(GPIO3, 29U);
        uint32_t b = GPIO_PinRead(GPIO1, 7U);

        if (a != s2) { s2 = a; PRINTF("   SW2 %s\r\n", (a == 0u) ? "pressed" : "released"); }
        if (b != s3) { s3 = b; PRINTF("   SW3 %s\r\n", (b == 0u) ? "pressed" : "released"); }
    }

    PRINTF("   done.\r\n");
}

/* =========================================================================
 * Link debugging
 *
 * Three tools, in the order you should reach for them. Between them they cut
 * the link into pieces that can each be blamed or cleared on their own,
 * which is what "the link does not work" never lets you do.
 * ====================================================================== */

/* --- 1. LOOPBACK. The only test that needs no Arduino and no assumptions.
 *
 * One jumper from this board's TX straight to its own RX, and the whole
 * question becomes "can this MCU talk to itself". If that fails, no amount of
 * looking at wires or Arduinos will help: it is the pin mux, the clock or the
 * LPUART setup, all of which are on this board.
 *
 * That matters here more than it usually would. LPUART2's ALT2 comes from
 * NXP's own generated example, but LPUART1's was worked out by elimination
 * and never actually proven - so "LPUART2 loops, LPUART1 does not" is a real
 * possible outcome, and it means the ALT is wrong, not the wiring.
 * ------------------------------------------------------------------------ */
static bool test_loopback(link_id_t id)
{
    static const char probe[] = "LOOPBACK.0123456789.abcdefghij";
    char     line[LINK_LINE_MAX];
    uint32_t t0;
    bool     heard = false;

    PRINTF("\r\n== loopback on %s ==========================================\r\n",
           LINK_GetName(id));
    PRINTF("   Jumper this board's TX straight to its own RX:\r\n");
    if (id == LINK_ARD1)
    {
        PRINTF("      J2-2 (P3_15, marked D8)  ->  J2-4 (P3_14, marked D9)\r\n");
    }
    else
    {
        PRINTF("      J2-20 (P1_9, marked D19) ->  J2-18 (P1_8, marked D18)\r\n");
    }
    PRINTF("   Unplug that Arduino's two wires first - its divider would\r\n");
    PRINTF("   fight the jumper. Both ends are 3.3 V, so no divider here.\r\n\r\n");

    while (LINK_PollLine(id, line, sizeof(line))) { }   /* drain */

    LINK_SendLine(id, probe);

    t0 = now_ms();
    while ((now_ms() - t0) < 300u)
    {
        if (LINK_PollLine(id, line, sizeof(line)))
        {
            if (strcmp(line, probe) == 0)
            {
                PRINTF("   PASS - this MCU transmits and receives on %s.\r\n",
                       LINK_GetName(id));
                PRINTF("   Mux, clock and LPUART are all good. Everything left\r\n");
                PRINTF("   is outside this board: the wire, the divider, the\r\n");
                PRINTF("   ground, or the Arduino.\r\n");
                PRINTF("===========================================================\r\n");
                return true;
            }
            heard = true;
            PRINTF("   got back: \"%s\"\r\n", line);
        }
    }

    if (heard)
    {
        PRINTF("   FAIL - bytes came back, but not the ones sent.\r\n");
        PRINTF("   Both directions work, so this is a BAUD or clock problem,\r\n");
        PRINTF("   not a pin. Check CLOCK_AttachClk for this LPUART.\r\n");
    }
    else
    {
        PRINTF("   FAIL - nothing came back at all.\r\n");
        PRINTF("   With the jumper really in place this is ON THIS BOARD:\r\n");
        PRINTF("     - the ALT setting for these pins in pin_mux.c\r\n");
        PRINTF("     - the LPUART clock attach in hardware_init.c\r\n");
        PRINTF("   Try the other channel: if one loops and the other does not,\r\n");
        PRINTF("   compare their two pin_mux entries - that is your answer.\r\n");
        PRINTF("   Measure the TX pin with a meter too: idle UART sits at\r\n");
        PRINTF("   3.3 V. Reading 0 V means the pin is not muxed to the\r\n");
        PRINTF("   LPUART at all, and 'mark' below makes that easy to see.\r\n");
    }
    PRINTF("===========================================================\r\n");
    return false;
}

/* --- 2. MARK. Transmit continuously so a meter can see it.
 *
 * 0x55 is alternating bits, so the line spends half its time low: a 3.3 V pin
 * reads about 1.8 V average on any cheap multimeter while this runs, against
 * 3.3 V when idle and 0 V when dead. Three clearly different numbers, no
 * oscilloscope needed. ------------------------------------------------- */
static void test_mark(link_id_t id)
{
    uint32_t t0 = now_ms();

    PRINTF("\r\n-- %s: transmitting 0x55 for 5 seconds.\r\n", LINK_GetName(id));
    PRINTF("   Measure %s against GND with a meter now:\r\n",
           (id == LINK_ARD1) ? "J2-2 (marked D8)" : "J2-20 (marked D19)");
    PRINTF("      ~1.8 V  transmitting - the pin and mux are fine\r\n");
    PRINTF("       3.3 V  idle - muxed, but nothing is coming out\r\n");
    PRINTF("       0.0 V  not muxed to the LPUART, or the pin is dead\r\n");

    while ((now_ms() - t0) < 5000u)
    {
        LINK_SendLine(id, "UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU");
    }

    PRINTF("   done.\r\n");
}

/* --- 4. SENSE. Is anything connected to this channel's RX pin at all?
 *
 * Every other test here assumes there is a wire and asks what is on it. This
 * one asks whether the wire exists, and it needs no meter.
 *
 * The pin comes off the LPUART and is read as a GPIO twice - once with the
 * internal pull-DOWN on, then with the pull-UP - and the answer is in whether
 * the two readings agree:
 *
 *   HIGH both times    something outside is driving it high, which is exactly
 *                      what an idle UART line looks like. The wire is real.
 *   LOW both times     something outside is holding it low. An idle UART does
 *                      not do that: suspect the 1k/2k divider assembled the
 *                      wrong way round, a wire sitting in a ground pin, or a
 *                      far end powered off and clamping the line.
 *   follows the pull   NOTHING IS CONNECTED. A floating pin obeys whichever
 *                      resistor is switched on; a driven one cannot. The wire
 *                      is off, broken, or in a different hole than you think.
 *
 * Then one second of watching with the pull-up on, counting transitions,
 * which separates a good idle line from one actually carrying traffic.
 *
 * The pin is handed back to the LPUART before this returns.
 * ------------------------------------------------------------------------ */
static void test_sense(link_id_t id)
{
    /* Eleven fields, in the order the generated pin_mux.c uses - see the note
     * in test_extra_pins_init() about what a short initializer does here. */
    const port_pin_config_t senseDown = {
        kPORT_PullDown, kPORT_LowPullResistor, kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_NormalDriveStrength,
        kPORT_MuxAlt0, kPORT_InputBufferEnable, kPORT_InputNormal,
        kPORT_UnlockRegister,
    };
    const port_pin_config_t senseUp = {
        kPORT_PullUp, kPORT_LowPullResistor, kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_NormalDriveStrength,
        kPORT_MuxAlt0, kPORT_InputBufferEnable, kPORT_InputNormal,
        kPORT_UnlockRegister,
    };
    /* Back onto the LPUART exactly as pin_mux.c left it. */
    const port_pin_config_t restore = {
        kPORT_PullUp, kPORT_LowPullResistor, kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_NormalDriveStrength,
        kPORT_MuxAlt2, kPORT_InputBufferEnable, kPORT_InputNormal,
        kPORT_UnlockRegister,
    };
    const gpio_pin_config_t gpioIn = { kGPIO_DigitalInput, 0 };

    PORT_Type  *port  = (id == LINK_ARD1) ? PORT3 : PORT1;
    GPIO_Type  *gpio  = (id == LINK_ARD1) ? GPIO3 : GPIO1;
    uint32_t    pin   = (id == LINK_ARD1) ? 14u : 8u;
    const char *where = (id == LINK_ARD1) ? "P3_14, J2 pin 4, marked D9"
                                          : "P1_8, J2 pin 18, marked D18";

    uint32_t down, up, prev, t0;
    uint32_t edges = 0u, highs = 0u, samples = 0u;

    PRINTF("\r\n== sense %s RX ==========================================\r\n",
           LINK_GetName(id));
    PRINTF("   %s, off the LPUART for one second.\r\n", where);

    PORT_SetPinConfig(port, pin, &senseDown);
    GPIO_PinInit(gpio, pin, &gpioIn);
    for (t0 = now_ms(); (now_ms() - t0) < 10u; ) { }
    down = GPIO_PinRead(gpio, pin);

    PORT_SetPinConfig(port, pin, &senseUp);
    for (t0 = now_ms(); (now_ms() - t0) < 10u; ) { }
    up = GPIO_PinRead(gpio, pin);

    prev = up;
    for (t0 = now_ms(); (now_ms() - t0) < 1000u; )
    {
        uint32_t level = GPIO_PinRead(gpio, pin);

        if (level != prev) { edges++; prev = level; }
        if (level != 0u)   { highs++; }
        samples++;
    }

    PORT_SetPinConfig(port, pin, &restore);

    PRINTF("   pulled down: %s      pulled up: %s\r\n",
           (down != 0u) ? "HIGH" : "LOW", (up != 0u) ? "HIGH" : "LOW");
    PRINTF("   watched 1 s: %u transitions, high %u%% of the time\r\n",
           (unsigned)edges,
           (unsigned)((highs * 100u) / ((samples != 0u) ? samples : 1u)));

    if (down != up)
    {
        PRINTF("\r\n   FLOATING - nothing is connected to this pin.\r\n");
        PRINTF("   It followed the internal pull in both directions, and a pin\r\n");
        PRINTF("   with a driver on the far end cannot do that. The wire is\r\n");
        PRINTF("   off, broken, or in a different hole than you think it is.\r\n");
        PRINTF("   Count the header again: %s.\r\n", where);
    }
    else if (down == 0u)
    {
        PRINTF("\r\n   HELD LOW by something outside this board.\r\n");
        PRINTF("   An idle UART line sits HIGH, so this is not a quiet link -\r\n");
        PRINTF("   it is a wrong one. Suspect the 1k/2k divider assembled the\r\n");
        PRINTF("   wrong way round, the wire sitting in a ground pin, or the\r\n");
        PRINTF("   far end powered off and clamping the line through its pin.\r\n");
    }
    else if (edges < 4u)
    {
        PRINTF("\r\n   DRIVEN HIGH and idle - THE WIRE IS GOOD.\r\n");
        PRINTF("   Something outside is holding it at the UART idle level, so\r\n");
        PRINTF("   the connection is real and the far end is simply not\r\n");
        PRINTF("   sending. Look at that node, not at this cable.\r\n");
    }
    else
    {
        PRINTF("\r\n   DRIVEN AND ACTIVE - %u transitions in one second.\r\n",
               (unsigned)edges);
        PRINTF("   Traffic is arriving on this pin. If the link still says\r\n");
        PRINTF("   SILENT the bytes are malformed rather than missing: baud,\r\n");
        PRINTF("   or a level that never reaches the threshold. Look at the\r\n");
        PRINTF("   frame error count on the diag line, and at 'raw%c'.\r\n",
               (id == LINK_ARD1) ? '1' : '2');
    }
    PRINTF("=========================================================\r\n");
}

/* --- 3. RAW. Show every byte, not every line.
 *
 * link.c assembles lines and silently discards anything that never ends, so a
 * channel receiving garbage - wrong baud, no common ground, a floating pin -
 * looks identical to one receiving nothing at all. This shows the difference:
 * silence is silence, and garbage is visible as garbage. */
static void raw_service(link_id_t id)
{
    uint8_t  byte;
    unsigned n = 0;

    while (LINK_PollByte(id, &byte))
    {
        if (n == 0u)
        {
            PRINTF("\r\n  [raw %s]", LINK_GetName(id));
        }
        PRINTF(" %02X", (unsigned)byte);
        if ((byte >= 0x20u) && (byte < 0x7Fu)) { PRINTF("'%c'", (char)byte); }
        n++;
        if (n >= 16u) { PRINTF("\r\n"); n = 0u; }
    }

    if (n > 0u)
    {
        PRINTF("\r\n");
        console_redraw_prompt();
    }
}

/* How long to wait for a reply. A 12-byte round trip at 38400 is about 6 ms,
 * so this is roughly eighty times what a healthy link needs. */
#define LINK_TEST_TIMEOUT_MS  500u

/* Round-trip test on one UART.
 *
 * Sends Q,<n> and waits for R,<n>,<node>. The far end answers automatically,
 * but ONLY when it is running test_link_perc.ino or test_link_act.ino - the
 * production sketches ignore Q, lines, so a FAIL here against perc.ino or
 * act.ino means nothing at all.
 *
 * Sending on its own proves nothing, which is what was wrong with the old
 * version of this: a line goes out whether or not anything is listening. */
static bool test_link(link_id_t id)
{
    static uint16_t seq = 0u;

    char     probe[LINK_LINE_MAX];
    char     want[LINK_LINE_MAX];
    char     line[LINK_LINE_MAX];
    uint32_t t0;
    bool     heard = false;   /* the node said SOMETHING, just not the answer */

    seq++;
    (void)snprintf(probe, sizeof(probe), "Q,%u", (unsigned int)seq);
    (void)snprintf(want,  sizeof(want),  "R,%u,", (unsigned int)seq);

    /* Throw away anything already queued - a heartbeat sitting in the buffer
     * must not be mistaken for a reply. The sequence number would catch it
     * anyway, but this keeps the output readable. */
    while (LINK_PollLine(id, line, sizeof(line))) { }

    PRINTF("\r\n-- %s  %s ... ", LINK_GetName(id), probe);
    LINK_SendLine(id, probe);

    t0 = now_ms();
    while ((now_ms() - t0) < LINK_TEST_TIMEOUT_MS)
    {
        if (LINK_PollLine(id, line, sizeof(line)))
        {
            if (strncmp(line, want, strlen(want)) == 0)
            {
                PRINTF("PASS\r\n   reply: %s\r\n", line);
                return true;
            }
            heard = true;
            PRINTF("\r\n   ignoring: %s\r\n", line);
        }
    }

    if (heard)
    {
        PRINTF("   FAIL - that node is talking, but it did not answer the probe.\r\n");
        PRINTF("   Both directions work, so the wiring is fine. It is running the\r\n");
        PRINTF("   wrong sketch: flash test_link_perc.ino / test_link_act.ino.\r\n");
        return false;
    }

    PRINTF("FAIL - silence\r\n");
    PRINTF("   Now look at that Arduino's own USB console:\r\n");
    PRINTF("     it shows [rx] %s   -> this board transmits fine and the fault\r\n", probe);
    PRINTF("                            is on the RETURN path, which is the only\r\n");
    PRINTF("                            one with parts in it: the 1k/2k divider\r\n");
    PRINTF("                            on that Arduino's D5. Swapped, it gives\r\n");
    PRINTF("                            1.7 V and this pin never sees a start bit.\r\n");
    PRINTF("     it shows nothing    -> the forward path is a bare wire, so it is\r\n");
    PRINTF("                            the wire, the pin, or no common ground.\r\n");
    PRINTF("   Garbage instead of silence is a baud or ground problem, not a pin.\r\n");
    return false;
}

/* Both UARTs, one after the other, with a verdict. */
static void test_links_both(void)
{
    bool a1, a2;

    PRINTF("\r\n== link test ==============================================\r\n");
    PRINTF("   Needs test_link_perc.ino on ARD1 and test_link_act.ino on\r\n");
    PRINTF("   ARD2. Against the production sketches this always fails.\r\n");

    a1 = test_link(LINK_ARD1);
    a2 = test_link(LINK_ARD2);

    PRINTF("\r\n   ARD1 (PERC, LPUART2, J2-2 / J2-4)   %s\r\n", a1 ? "PASS" : "FAIL");
    PRINTF("   ARD2 (ACT,  LPUART1, J2-20 / J2-18) %s\r\n", a2 ? "PASS" : "FAIL");

    if (a1 && a2)
    {
        PRINTF("\r\n   Both links are good in both directions, dividers included.\r\n");
    }
    else if (a1 != a2)
    {
        PRINTF("\r\n   One link works and the other does not, so the MCX, the baud\r\n");
        PRINTF("   rate and the ground are all fine - they are shared. Compare the\r\n");
        PRINTF("   two wirings against each other; the good one is the reference.\r\n");
    }
    else
    {
        PRINTF("\r\n   Neither link works. Something shared is wrong: the star\r\n");
        PRINTF("   ground, or both Arduinos are unpowered. Run j on either\r\n");
        PRINTF("   Arduino first - it tests that board alone, with one jumper.\r\n");
    }
    PRINTF("===========================================================\r\n");
}

/* ===========================================================================
 * SW2 and SW3 - the two tests you need most, with no console at all.
 *
 * A console that will not accept input is not only an annoyance: it takes
 * every diagnostic on this board away at exactly the moment they are wanted.
 * Both buttons are already muxed and read (test_extra_pins_init), so binding
 * them costs nothing and removes that dependency.
 *
 *   SW2   sense1  - is anything connected to the PERC receive pin
 *   SW3   busfix  - free an I2C bus a slave is holding low, then scan it
 *
 * A 400 ms lockout instead of a debounce: these launch a test that runs for
 * a second or more, so a bouncing contact cannot start a second one.
 * ======================================================================== */
static void button_service(void)
{
    static uint32_t pressAt;
    static uint32_t sw2Prev = 1u, sw3Prev = 1u;

    uint32_t sw2 = GPIO_PinRead(GPIO3, 29U);
    uint32_t sw3 = GPIO_PinRead(GPIO1, 7U);
    bool     ready = ((now_ms() - pressAt) > 400u);

    if ((sw2 == 0u) && (sw2Prev != 0u) && ready)      /* active low */
    {
        pressAt = now_ms();
        PRINTF("\r\n  [SW2] sense1 - is the PERC receive wire there?\r\n");
        test_sense(LINK_ARD1);
        console_redraw_prompt();
    }
    else if ((sw3 == 0u) && (sw3Prev != 0u) && ready)
    {
        pressAt = now_ms();
        PRINTF("\r\n  [SW3] I2C bus recovery, then a scan\r\n");
        LINK_GwBusRecover();
        test_i2c_scan();
        console_redraw_prompt();
    }

    sw2Prev = sw2;
    sw3Prev = sw3;
}

/* Send the current drive command to ACT. Called on change and then repeatedly,
 * because ACT deliberately forgets a command it has not heard for 300 ms. */
static void drive_send(void)
{
    char cmd[32];

    (void)snprintf(cmd, sizeof(cmd), "D,%u,%u",
                   (unsigned)s_driveL, (unsigned)s_driveR);
    LINK_SendLine(LINK_ARD2, cmd);
    s_driveSentAt = now_ms();
}

/* Clamp, arm and send. Silent, because teleop calls this twenty times a
 * second and printing it would make the console unusable. */
static void drive_apply(uint32_t l, uint32_t r)
{
    s_driveL     = (uint8_t)((l > DRIVE_DUTY_MAX) ? DRIVE_DUTY_MAX : l);
    s_driveR     = (uint8_t)((r > DRIVE_DUTY_MAX) ? DRIVE_DUTY_MAX : r);
    s_driveArmed = true;
    drive_send();
}

static void drive_set(uint32_t l, uint32_t r)
{
    drive_apply(l, r);
    PRINTF("\r\n  drive L=%u%% R=%u%%\r\n", (unsigned)s_driveL, (unsigned)s_driveR);
}

static void drive_stop(void)
{
    s_driveL = 0u;
    s_driveR = 0u;

    /* Send the zero, then stop sending. ACT's timeout then holds it stopped
     * even if this board dies a moment later. */
    drive_send();
    s_driveArmed = false;

    PRINTF("\r\n  drive STOP\r\n");
}

static void drive_service(void)
{
    if (!s_driveArmed)
    {
        return;
    }
    if ((now_ms() - s_driveSentAt) >= DRIVE_REPEAT_MS)
    {
        drive_send();
    }
}

/* ===========================================================================
 * Teleop
 * ======================================================================== */

/* The horn lives on PERC, so it is a line on the ARD1 link. It is repeated
 * while it is on, and PERC drops it after 500 ms of silence - otherwise a
 * single lost "off" would leave the thing sounding until someone pulled a
 * battery, which at a demo is the worst failure in the car. */
static void horn_set(bool on)
{
    if (on == s_horn)
    {
        return;
    }

    s_horn       = on;
    s_hornSentAt = now_ms();
    LINK_SendLine(LINK_ARD1, on ? "H,1" : "H,0");
}

/* Repeat while sounding, so PERC's 500 ms watchdog stays fed. Silence needs
 * no repeating - PERC defaults to off. */
static void horn_service(void)
{
    if (!s_horn)
    {
        return;
    }
    if ((now_ms() - s_hornSentAt) < HORN_REPEAT_MS)
    {
        return;
    }

    s_hornSentAt = now_ms();
    LINK_SendLine(LINK_ARD1, "H,1");
}

/* One line a second to BOTH Arduinos, whatever else is happening.
 *
 * Without this neither node can tell a healthy VCU from a dead one at idle,
 * and their consoles say so wrongly. PERC only ever hears the horn, which is
 * usually silent. ACT only hears D, lines, and those stop the moment nobody
 * is driving - so a perfectly good ACT link reports "VCU NEVER HEARD", which
 * is exactly the false alarm that sends you looking for a broken wire.
 *
 * It carries the VCU's uptime so a board that has silently reset is obvious:
 * the number goes backwards.
 *
 * It is deliberately NOT a drive command. ACT's 300 ms motor timeout must
 * keep counting through this - a heartbeat proves the wire works, not that
 * anyone is asking for motion, and confusing the two would defeat the whole
 * safe state. */
static void heartbeat_service(void)
{
    char line[24];

    if ((now_ms() - s_beatAt) < HEARTBEAT_MS)
    {
        return;
    }
    s_beatAt = now_ms();

    (void)snprintf(line, sizeof(line), "V,%u", (unsigned)now_ms());
    LINK_SendLine(LINK_ARD1, line);
    LINK_SendLine(LINK_ARD2, line);
}

/* True while the front sonar says the way ahead is blocked.
 *
 * Deliberately permissive in two directions. A -1 reading is no echo, which
 * on an ultrasonic means nothing is close enough to return one, so it opens
 * the guard rather than closing it. And a silent PERC does not stop the car
 * either: you need to be able to drive with PERC unflashed or unplugged
 * while bringing the rest up, and a human is watching the wheels. The
 * telemetry reports both facts, so the browser can say so out loud. */
static bool teleop_guard(void)
{
    if ((now_ms() - s_percAt) > SENSOR_STALE_MS)
    {
        return false;                       /* PERC is not talking */
    }
    return (s_front >= 0) && (s_front < GUARD_STOP_CM);
}

/* Keys in, two duties out. This is the whole vehicle dynamics model.
 *
 *   W  ramp the throttle up            S  brake - straight to zero
 *   A  take TURN_BIAS off the left     D  take it off the right
 *
 * With the throttle at zero, A or D alone spins the car on the spot instead,
 * because a stationary car that cannot reverse has no other way to aim. */
static void teleop_mix(uint32_t *outL, uint32_t *outR)
{
    uint32_t l, r;

    s_guard = teleop_guard();

    if (s_keyS || s_guard)
    {
        s_throttle = 0u;                    /* both are stops, not ramps */
    }
    else if (s_keyW)
    {
        s_throttle = (uint8_t)((s_throttle + THROTTLE_RISE > THROTTLE_MAX)
                                   ? THROTTLE_MAX
                                   : (s_throttle + THROTTLE_RISE));
    }
    else
    {
        s_throttle = (uint8_t)((s_throttle > THROTTLE_COAST)
                                   ? (s_throttle - THROTTLE_COAST)
                                   : 0u);
    }

    if (s_throttle > 0u)
    {
        l = s_throttle;
        r = s_throttle;

        if (s_keyA) { l = (s_throttle > TURN_BIAS) ? (s_throttle - TURN_BIAS) : 0u; }
        if (s_keyD) { r = (s_throttle > TURN_BIAS) ? (s_throttle - TURN_BIAS) : 0u; }
    }
    else if (!s_keyS && (s_keyA != s_keyD))
    {
        /* Pivot. Drive the outside pair only - left turn means the right
         * wheels push and the left ones hold. Still allowed under the guard,
         * because turning away is the only escape from a wall. */
        l = s_keyD ? PIVOT_DUTY : 0u;
        r = s_keyA ? PIVOT_DUTY : 0u;
    }
    else
    {
        l = 0u;
        r = 0u;
    }

    *outL = l;
    *outR = r;
}

/* Let go of everything and go quiet, so ACT's own timeout is what actually
 * cuts the motors. Called when the key stream stops, when the browser asks,
 * and when someone types 'x' here - hence the reason being passed in. */
static void teleop_release(const char *why)
{
    s_teleopLive = false;
    s_throttle   = 0u;
    s_keyW = s_keyA = s_keyS = s_keyD = s_keyHorn = false;
    s_guard      = false;

    horn_set(false);

    s_driveL = 0u;
    s_driveR = 0u;
    drive_send();               /* one explicit zero on the way out */
    s_driveArmed = false;

    PRINTF("\r\n  [teleop] released - %s\r\n", why);
    console_redraw_prompt();
}

static void teleop_service(void)
{
    uint32_t l, r;

    if ((now_ms() - s_teleopTickAt) < TELEOP_TICK_MS)
    {
        return;
    }
    s_teleopTickAt = now_ms();

    if ((now_ms() - s_keyAt) > TELEOP_TIMEOUT_MS)
    {
        if (s_teleopLive)
        {
            teleop_release("no keys from the browser");
        }
        return;
    }

    teleop_mix(&l, &r);
    drive_apply(l, r);
    horn_set(s_keyHorn);
}

/* "K,<w>,<a>,<s>,<d>,<horn>". Every field is one character, 0 or 1.
 *
 * A malformed line is dropped rather than half-applied: a corrupted key state
 * that leaves W set is exactly the bug that runs a car into a wall. */
static bool teleop_keys(const char *line)
{
    bool k[5];
    int  i;

    if ((line[0] != 'K') || (line[1] != ','))
    {
        return false;
    }

    for (i = 0; i < 5; i++)
    {
        /* K , w , a , s , d , h  ->  the value of field i is at 2 + 2*i */
        size_t v = (size_t)(2 + (2 * i));
        size_t c = v + 1u;

        if ((line[v] != '0') && (line[v] != '1'))
        {
            return true;                    /* consumed, but not trusted */
        }
        if ((i < 4) && (line[c] != ','))
        {
            return true;
        }
        k[i] = (line[v] == '1');
    }

    s_keyW    = k[0];
    s_keyA    = k[1];
    s_keyS    = k[2];
    s_keyD    = k[3];
    s_keyHorn = k[4];
    s_keyAt   = now_ms();

    if (!s_teleopLive)
    {
        s_teleopLive = true;
        s_throttle   = 0u;                  /* never inherit an old ramp */
        PRINTF("\r\n  [teleop] browser connected - it has the wheels\r\n");
        console_redraw_prompt();
    }
    return true;
}

/* One line, ten times a second, carrying everything the car knows.
 *
 * Field order, and it must match the parser in esp32_gateway.ino:
 *
 *   T,live,L,R,throttle,front,back,pir,horn,guard,ax,ay,az,gz,actAge,percAge
 *
 * front and back are centimetres, -1 for no echo. The two ages are
 * milliseconds since that node last spoke, capped at 9999 - they are the link
 * health readout, and a pegged one is how you see a dead Arduino from the
 * browser. All four IMU fields zero means no MPU6050 on the bus. */
static void telemetry_send(void)
{
    char     line[LINK_LINE_MAX];
    uint32_t actAge, percAge;

    if ((now_ms() - s_telemetryAt) < TELEMETRY_MS)
    {
        return;
    }
    s_telemetryAt = now_ms();

    /* No gateway, no point: the write would only fail slowly, ten times a
     * second, and there is nobody at the far end to read it. The poll above
     * is what notices it coming back. */
    if (!LINK_GwOnline())
    {
        return;
    }

    actAge  = now_ms() - s_actAt;
    percAge = now_ms() - s_percAt;
    if (actAge  > 9999u) { actAge  = 9999u; }
    if (percAge > 9999u) { percAge = 9999u; }

    (void)snprintf(line, sizeof(line),
                   "T,%u,%u,%u,%u,%d,%d,%u,%u,%u,%d,%d,%d,%d,%u,%u",
                   (unsigned)(s_teleopLive ? 1 : 0),
                   (unsigned)s_driveL, (unsigned)s_driveR,
                   (unsigned)s_throttle,
                   s_front, s_back, (unsigned)s_pir,
                   (unsigned)(s_horn ? 1 : 0),
                   (unsigned)(s_guard ? 1 : 0),
                   s_imuOk ? s_imuAx : 0, s_imuOk ? s_imuAy : 0,
                   s_imuOk ? s_imuAz : 0, s_imuOk ? s_imuGz : 0,
                   (unsigned)actAge, (unsigned)percAge);

    LINK_SendLine(LINK_GW, line);
}

/* ===========================================================================
 * Link diagnostics
 *
 * Four questions, answered on this console because the VCU is the only node
 * that can see all four links at once:
 *
 *   is the ESP32 answering I2C?   LINK_GwOnline() - an idle gateway and an
 *                                 absent one both send no bytes, so the
 *                                 transaction status is the only difference
 *   is PERC talking?              a P, line inside DIAG_SILENT_MS
 *   is ACT talking?               an S, line inside DIAG_SILENT_MS
 *   is the browser talking?       a K, line inside TELEOP_TIMEOUT_MS
 * ======================================================================== */

/* Print only when the answer changes. Returns the new state so the caller
 * can store it. */
static int diag_edge(const char *who, int was, bool now, const char *upMsg,
                     const char *downMsg)
{
    int is = now ? 1 : 0;

    if (was == is)
    {
        return is;
    }

    PRINTF("\r\n  [%s] %s\r\n", who, now ? upMsg : downMsg);
    console_redraw_prompt();
    return is;
}

static void diag_service(void)
{
    uint32_t polls, fails;
    uint32_t percAge, actAge, keyAge;
    bool     gwUp;

    percAge = now_ms() - s_percAt;
    actAge  = now_ms() - s_actAt;
    keyAge  = now_ms() - s_keyAt;
    gwUp    = LINK_GwOnline();

    {
        int wasGw = s_wasGw;

        s_wasGw = diag_edge("GW", s_wasGw, gwUp,
                            "ESP32 answering on I2C",
                            "ESP32 NOT answering on I2C");

        /* The one-word reason names the wire; say it once, on the edge. */
        if ((wasGw != s_wasGw) && !gwUp)
        {
            PRINTF("       %s: %s.\r\n", LINK_GwFailReason(), LINK_GwFailHint());
            PRINTF("       SDA = J5 pin 6, SCL = J5 pin 5, 2k to 3V3 on each; "
                   "'i2c' to scan.\r\n");
            console_redraw_prompt();
        }
    }
    s_wasPerc = diag_edge("PERC", s_wasPerc, percAge < DIAG_SILENT_MS,
                          "up - ranges arriving",
                          "SILENT - no P, line. Check J2-2/J2-4 and its divider");
    s_wasAct  = diag_edge("ACT",  s_wasAct,  actAge < DIAG_SILENT_MS,
                          "up - status arriving",
                          "SILENT - no S, line. Check J2-20/J2-18 and its divider");
    s_wasKeys = diag_edge("WEB",  s_wasKeys, keyAge < TELEOP_TIMEOUT_MS,
                          "browser sending keys",
                          "no keys - tab closed, unfocused, or WiFi gone");

    if (!s_diagLoud)
    {
        return;
    }
    if ((now_ms() - s_diagAt) < DIAG_PERIOD_MS)
    {
        return;
    }
    s_diagAt = now_ms();

    LINK_GwStats(&polls, &fails);

    PRINTF("\r\n  [diag] GW %s %u ok / %u fail", gwUp ? "up  " : "DOWN",
           (unsigned)polls, (unsigned)fails);
    if (!gwUp) { PRINTF(" %s", LINK_GwFailReason()); }
    if (LINK_GwRecoveries() != 0u)
    {
        PRINTF(" (%u resets)", (unsigned)LINK_GwRecoveries());
    }
    PRINTF(" | PERC ");
    if (percAge < DIAG_SILENT_MS) { PRINTF("%u ms", (unsigned)percAge); }
    else                          { PRINTF("SILENT"); }
    PRINTF(" | ACT ");
    if (actAge < DIAG_SILENT_MS)  { PRINTF("%u ms", (unsigned)actAge); }
    else                          { PRINTF("SILENT"); }
    PRINTF(" | WEB ");
    if (keyAge < TELEOP_TIMEOUT_MS) { PRINTF("%u ms", (unsigned)keyAge); }
    else                            { PRINTF("silent"); }

    /* Only when there are any. A frame error means bytes ARE arriving on that
     * pin and cannot be read - the opposite fault from a dead wire, and
     * invisible without this. */
    {
        uint32_t fe1 = LINK_GetFrameErrorCount(LINK_ARD1);
        uint32_t fe2 = LINK_GetFrameErrorCount(LINK_ARD2);

        uint32_t to1 = LINK_GetTxTimeoutCount(LINK_ARD1);
        uint32_t to2 = LINK_GetTxTimeoutCount(LINK_ARD2);

        if ((fe1 | fe2) != 0u)
        {
            PRINTF(" | frame err PERC %u ACT %u", (unsigned)fe1, (unsigned)fe2);
        }
        if ((to1 | to2) != 0u)
        {
            PRINTF(" | TX STALLED PERC %u ACT %u", (unsigned)to1, (unsigned)to2);
        }
    }
    PRINTF("\r\n");
    console_redraw_prompt();
}

/* Returns true if the line was a local command and must not be broadcast. */
static bool drive_try_command(const char *line)
{
    unsigned l, r;

    if ((line[0] == 'd') && ((line[1] == ' ') || (line[1] == '\0')))
    {
        /* Two drivers on one set of wheels is how people get hurt. While the
         * browser is sending keys it owns the drive, and this refuses rather
         * than fighting it at 20 Hz. */
        if (s_teleopLive)
        {
            PRINTF("\r\n  the browser is driving. 'x' takes the wheels back.\r\n");
            return true;
        }

        if (sscanf(line + 1, "%u %u", &l, &r) == 2)
        {
            drive_set(l, r);
        }
        else if (sscanf(line + 1, "%u", &l) == 1)
        {
            drive_set(l, l);
        }
        else
        {
            PRINTF("\r\n  usage: d <both> | d <left> <right>   (0-%u)\r\n",
                   (unsigned)DRIVE_DUTY_MAX);
        }
        return true;
    }

    /* A stop is never refused, whoever is driving. Releasing teleop clears
     * the held keys and the throttle ramp with them, so a browser that still
     * has W down restarts from zero rather than snapping back to speed. */
    if ((line[0] == 'x') && (line[1] == '\0'))
    {
        if (s_teleopLive)
        {
            teleop_release("stopped from this console");
        }
        drive_stop();
        return true;
    }

    if (strcmp(line, "loop1") == 0) { (void)test_loopback(LINK_ARD1); return true; }
    if (strcmp(line, "loop2") == 0) { (void)test_loopback(LINK_ARD2); return true; }
    if (strcmp(line, "mark1") == 0) { test_mark(LINK_ARD1); return true; }
    if (strcmp(line, "mark2") == 0) { test_mark(LINK_ARD2); return true; }
    if (strcmp(line, "sense1") == 0) { test_sense(LINK_ARD1); return true; }
    if (strcmp(line, "sense2") == 0) { test_sense(LINK_ARD2); return true; }

    if ((strcmp(line, "raw1") == 0) || (strcmp(line, "raw2") == 0))
    {
        link_id_t id = (line[3] == '1') ? LINK_ARD1 : LINK_ARD2;

        s_raw[id] = !s_raw[id];
        PRINTF("\r\n  raw byte view on %s %s.%s\r\n", LINK_GetName(id),
               s_raw[id] ? "ON" : "off",
               s_raw[id] ? " That channel is not a link while this is on."
                         : "");
        return true;
    }

    if (strcmp(line, "diag") == 0)
    {
        s_diagLoud = !s_diagLoud;
        PRINTF("\r\n  periodic link summary %s. Up/down messages stay on.\r\n",
               s_diagLoud ? "ON, every 2 s" : "off");
        return true;
    }

    if (strcmp(line, "sens") == 0)
    {
        PRINTF("\r\n  front %d cm   back %d cm   PIR %u   (%u ms old)\r\n",
               s_front, s_back, (unsigned)s_pir,
               (unsigned)(now_ms() - s_percAt));
        PRINTF("  ACT %s   (%u ms old)\r\n",
               (s_actStatus[0] != '\0') ? s_actStatus : "- nothing yet -",
               (unsigned)(now_ms() - s_actAt));
        PRINTF("  IMU %s  a %6d %6d %6d   gz %6d\r\n",
               s_imuOk ? "ok " : "absent",
               s_imuAx, s_imuAy, s_imuAz, s_imuGz);
        return true;
    }

    if (strcmp(line, "i2c") == 0)   { test_i2c_scan();        return true; }
    if (strcmp(line, "gw") == 0)    { test_gw_read();         return true; }
    if (strcmp(line, "busfix") == 0)
    {
        PRINTF("\r\n  clocking the bus free, then re-initialising.\r\n");
        LINK_GwBusRecover();
        test_i2c_scan();
        return true;
    }
    if (strcmp(line, "imu") == 0)   { test_imu();             return true; }
    if (strcmp(line, "led") == 0)   { test_leds();            return true; }
    if (strcmp(line, "btn") == 0)   { test_buttons();         return true; }
    if (strcmp(line, "tx1") == 0)   { (void)test_link(LINK_ARD1); return true; }
    if (strcmp(line, "tx2") == 0)   { (void)test_link(LINK_ARD2); return true; }
    if (strcmp(line, "link") == 0)  { test_links_both();       return true; }

    if ((line[0] == '?') && (line[1] == '\0'))
    {
        PRINTF("\r\n  d 40        both pairs to 40 %%"
               "\r\n  d 40 20     left 40 %%, right 20 %% - this is how it steers"
               "\r\n  d 40 0      pivot"
               "\r\n  x           stop, and take the wheels back from the browser"
               "\r\n  sens        ranges, PIR, ACT status and IMU, once"
               "\r\n  diag        toggle the 2 s link summary (up/down msgs stay on)"
               "\r\n"
               "\r\n  i2c         scan the bus - expect 0x42 ESP32, 0x68 MPU6050"
               "\r\n  gw          one raw read of the gateway, bytes and status"
               "\r\n  busfix      clock a stuck I2C bus free - the cure for BUSY"
               "\r\n              SW2 = sense1, SW3 = busfix, for a dead console"
               "\r\n  imu         MPU6050 WHO_AM_I then 10 samples"
               "\r\n  led         cycle the RGB LED       btn   watch SW2 / SW3"
               "\r\n  tx1 / tx2   round-trip test one link (needs test_link_*.ino)"
               "\r\n  link        round-trip test both, with a verdict"
               "\r\n"
               "\r\n  LINK DEBUGGING - use these in this order:"
               "\r\n  loop1/loop2 THIS BOARD ONLY. Jumper its TX to its own RX."
               "\r\n              Fails = mux, clock or LPUART. Nothing external."
               "\r\n  sense1/2    IS THE RX WIRE EVEN THERE? Reads the pin pulled"
               "\r\n              down then up - follows the pull = nothing wired"
               "\r\n  mark1/mark2 transmit 0x55 for 5 s - measure TX with a meter"
               "\r\n              ~1.8 V sending, 3.3 V idle, 0 V not muxed"
               "\r\n  raw1/raw2   show every BYTE received, not every line."
               "\r\n              Silence and garbage look identical otherwise."

               "\r\n  anything else is broadcast to every node as before\r\n");
        return true;
    }

    return false;
}

/* Anything typed here is tagged [MCX] and sent to every other node. */
static void console_service(void)
{
    uint8_t byte;
    char    out[LINK_LINE_MAX];

    while (console_try_read(&byte))
    {
        if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
        {
            if (s_consoleLen == 0u)
            {
                continue;
            }

            s_consoleLine[s_consoleLen] = '\0';

            if (drive_try_command(s_consoleLine))
            {
                /* handled locally - do not flood the other nodes with it */
            }
            else
            {
                (void)snprintf(out, sizeof(out), "[MCX] %s", s_consoleLine);

                LINK_Broadcast(out, LINK_COUNT); /* LINK_COUNT excludes nobody */

                PRINTF("\r\n  sent to ARD1 + ARD2 + GW: %s\r\n", s_consoleLine);
            }

            s_consoleLen = 0u;
            console_redraw_prompt();
        }
        else if ((byte == KEY_BACKSPACE) || (byte == KEY_DELETE))
        {
            if (s_consoleLen > 0u)
            {
                s_consoleLen--;
                PRINTF("\b \b"); /* erase the character on screen */
            }
        }
        else if ((byte >= 0x20u) && (byte < 0x7Fu) && (s_consoleLen < (LINK_LINE_MAX - 1u)))
        {
            s_consoleLine[s_consoleLen] = (char)byte;
            s_consoleLen++;
            PUTCHAR(byte); /* local echo, so you can see what you type */
        }
        else
        {
            /* control character or line full - ignore */
        }
    }
}

/* Protocol traffic, as opposed to chatter.
 *
 * Everything the four nodes say to each other by machine - status, ranges,
 * keys, telemetry requests - is swallowed here and never printed or relayed.
 * That matters more than it sounds: between them ACT and PERC produce fifteen
 * lines a second, and broadcasting those to the other two nodes would fill
 * both links with traffic nobody reads.
 *
 * Returns true when the line was protocol and has been dealt with. */
static bool vcu_consume(link_id_t from, const char *line)
{
    if (from == LINK_ARD2)
    {
        /* ACT status at 5 Hz. Keep the newest, show it once a second - any
         * more and this terminal cannot be typed into. */
        if ((line[0] == 'S') && (line[1] == ','))
        {
            s_actAt = now_ms();
            (void)snprintf(s_actStatus, sizeof(s_actStatus), "%s", line);

            if ((now_ms() - s_actPrintAt) >= 1000u)
            {
                s_actPrintAt = now_ms();
                PRINTF("\r\n  [ACT] %s\r\n", s_actStatus);
                console_redraw_prompt();
            }
            return true;
        }
        return false;
    }

    if (from == LINK_ARD1)
    {
        if ((line[0] == 'P') && (line[1] == ','))
        {
            int      f, b;
            unsigned p;

            if (sscanf(line + 2, "%d,%d,%u", &f, &b, &p) == 3)
            {
                s_front  = f;
                s_back   = b;
                s_pir    = (uint8_t)((p != 0u) ? 1 : 0);
                s_percAt = now_ms();
            }
            return true;   /* even a malformed one - it is not for a human */
        }
        return false;
    }

    if (from == LINK_GW)
    {
        if (teleop_keys(line))
        {
            return true;
        }

        /* A message for the display goes straight through to ACT. The VCU
         * has no opinion about text. */
        if ((line[0] == 'M') && (line[1] == ','))
        {
            LINK_SendLine(LINK_ARD2, line);
            PRINTF("\r\n  [lcd] %s\r\n", line + 2);
            console_redraw_prompt();
            return true;
        }

        if ((line[0] == 'E') && (line[1] == '\0'))
        {
            teleop_release("STOP pressed in the browser");
            return true;
        }
        return false;
    }

    return false;
}

/* A line from one node is printed here and relayed to all the others. */
static void link_service(link_id_t from)
{
    char line[LINK_LINE_MAX];
    char out[LINK_LINE_MAX];

    while (LINK_PollLine(from, line, sizeof(line)))
    {
        if (vcu_consume(from, line))
        {
            continue;
        }

        PRINTF("\r\n  [%s] %s\r\n", LINK_GetName(from), line);

        (void)snprintf(out, sizeof(out), "[%s] %s", LINK_GetName(from), line);
        LINK_Broadcast(out, from); /* everyone except the sender */

        console_redraw_prompt();
    }
}

int main(void)
{
    /* Board pin init, clocks, peripherals, debug console */
    BOARD_InitHardware();

    LINK_Init();

    /* 1 kHz tick. Nothing was starting SysTick before, so both the heartbeat
     * LED and the drive repeat below depend on this line. */
    (void)SysTick_Config(SystemCoreClock / 1000u);

    test_extra_pins_init();
    i2c_pullups_init();   /* after LINK_Init muxed them - see the note there */

    /* Nothing has spoken yet, so make the ages say so. Left at zero they
     * would read "0 ms old" for the first ten seconds and every link would
     * look healthy before a single line had arrived. */
    s_keyAt  = now_ms() - 10000u;
    s_actAt  = s_keyAt;
    s_percAt = s_keyAt;

    /* SAY SOMETHING BEFORE TOUCHING THE I2C BUS.
     *
     * This banner used to come after the IMU wake-up, and that was a real
     * bug: an LPI2C transfer on a stuck bus does not fail, it waits, so a
     * bus with no pull-ups fitted hung the board here with NOTHING printed.
     * Identical, from the outside, to a board never flashed or never powered.
     *
     * The retry limit in CMakeLists.txt means it can no longer hang at all.
     * The ordering stays anyway: the first thing this node does should be to
     * prove it is alive, and no diagnostic should sit behind the thing it is
     * meant to diagnose. */
    PRINTF("\r\n");
    PRINTF("=============================================================\r\n");
    PRINTF(" FRDM-MCXA153 hub   -   MCX + ARD1 + ARD2 + ESP32 gateway\r\n");
    PRINTF("=============================================================\r\n");
    PRINTF(" ARD1 : LPUART2  P3_15 TX (J2-2)   P3_14 RX (J2-4)   %u baud\r\n",
           (unsigned int)LINK_BAUDRATE);
    PRINTF(" ARD2 : LPUART1  P1_9  TX (J2-20)  P1_8  RX (J2-18)  %u baud\r\n",
           (unsigned int)LINK_BAUDRATE);
    PRINTF(" GW   : LPI2C0   P3_27 SCL         P3_28 SDA         addr 0x%02X\r\n",
           (unsigned int)LINK_GW_ADDR);
    PRINTF("\r\n Teleop: join WiFi \"NXP-AV\" and open http://192.168.4.1\r\n");
    PRINTF("         W throttle   A / D steer   S brake   SPACE horn\r\n");
    PRINTF("         The browser sends keys. This board decides everything.\r\n");
    PRINTF("\r\n Type a line and press Enter to send it to every node.\r\n");
    PRINTF(" Drive:  d 40    both     d 40 20   steer     d 40 0   pivot\r\n");
    PRINTF("         x       stop     sens      one sensor dump    ?  help\r\n");
    PRINTF(" Duty is capped at %u %%. WHEELS OFF THE FLOOR until proven.\r\n",
           (unsigned)DRIVE_DUTY_MAX);
    PRINTF(" Self-test: i2c  imu  led  btn  tx1  tx2\r\n");
    PRINTF("-------------------------------------------------------------\r\n");

    /* Now the bus, with the banner already out. A failure here is reported,
     * not fatal - the two UART links and the drive loop do not need I2C. */
    if (imu_wake())
    {
        PRINTF(" IMU  : MPU6050 awake at 0x%02X\r\n", (unsigned int)IMU_ADDR);
    }
    else
    {
        PRINTF(" IMU  : NO ANSWER at 0x%02X.\r\n", (unsigned int)IMU_ADDR);
        PRINTF("        If the ESP32 is silent too, suspect the bus rather than\r\n");
        PRINTF("        either device: SDA and SCL must idle at 3.3 V via the\r\n");
        PRINTF("        two 2k pull-ups to J3-8.\r\n");
        PRINTF("        Run 'i2c' to scan. The Arduino links work regardless.\r\n");
    }
    PRINTF("-------------------------------------------------------------\r\n");
    PRINTF("> ");

    for (;;)
    {
        console_service();       /* this terminal -> everyone      */
        button_service();        /* SW2 / SW3 when it will not type */
        teleop_service();        /* keys -> duties, 20 Hz          */
        drive_service();         /* repeat the drive command       */
        imu_service();           /* the only feedback in the car   */
        horn_service();          /* horn repeat while sounding     */
        heartbeat_service();     /* V, to both Arduinos, 1 Hz      */
        if (s_raw[LINK_ARD1]) { raw_service(LINK_ARD1); }
        else                  { link_service(LINK_ARD1); }
        if (s_raw[LINK_ARD2]) { raw_service(LINK_ARD2); }
        else                  { link_service(LINK_ARD2); }

        /* The gateway is polled, not interrupt-driven, and each poll is a
         * blocking I2C transaction. Rate-limit it so the UART channels keep
         * their share of the loop - and so keypress latency stays bounded
         * by a number rather than by however fast this loop happens to run. */
        if ((now_ms() - s_gwPollAt) >=
            (LINK_GwOnline() ? GW_POLL_MS : GW_RETRY_MS))
        {
            s_gwPollAt = now_ms();
            link_service(LINK_GW); /* laptop -> here + both Arduinos */
        }

        telemetry_send();        /* here -> laptop, 10 Hz          */
        diag_service();          /* who is talking, who stopped    */
    }
}
