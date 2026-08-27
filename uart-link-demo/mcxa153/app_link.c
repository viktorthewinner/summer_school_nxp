/*
 * app_link.c - LPUART2 link driver for the FRDM-MCXA153 <-> Arduino demo.
 */
#include "app_link.h"

#include "fsl_lpuart.h"
#include "fsl_clock.h"

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------------
 * Board configuration
 *
 * LPUART2 is the link to the Arduino:
 *      P1_5 / LPUART2_TXD -> J1 pin 4  (ARD_D1)
 *      P1_4 / LPUART2_RXD -> J1 pin 2  (ARD_D0)
 *
 * LPUART0 stays on the MCU-Link VCOM and remains your debug console, so the
 * two never share wires.
 *
 * LINK_LPUART_CLK_FREQ mirrors how the SDK examples define their own clock
 * macro in app.h - open
 *   mcuxsdk-examples/_boards/frdmmcxa153/driver_examples/lpuart/polling/app.h
 * and copy the pattern, changing the instance index from 0 to 2.
 * ---------------------------------------------------------------------- */
#define LINK_LPUART           LPUART2
#define LINK_LPUART_CLK_FREQ  CLOCK_GetLpuartClkFreq(2u)

#define LINK_RX_BUF_SIZE      192u

static char   s_rx[LINK_RX_BUF_SIZE];
static size_t s_rxLen;

void LINK_Init(uint32_t baudRate)
{
    lpuart_config_t cfg;

    LPUART_GetDefaultConfig(&cfg);
    cfg.baudRate_Bps = baudRate;
    cfg.enableTx     = true;
    cfg.enableRx     = true;

    (void)LPUART_Init(LINK_LPUART, &cfg, LINK_LPUART_CLK_FREQ);

    s_rxLen = 0u;
}

void LINK_SendLine(const char *line)
{
    static const uint8_t nl = (uint8_t)'\n';

    if (line == NULL)
    {
        return;
    }

    LPUART_WriteBlocking(LINK_LPUART, (const uint8_t *)line, strlen(line));
    LPUART_WriteBlocking(LINK_LPUART, &nl, 1u);
}

bool LINK_Poll(char *out, size_t outSize)
{
    uint32_t flags;

    if ((out == NULL) || (outSize == 0u))
    {
        return false;
    }

    for (;;)
    {
        flags = LPUART_GetStatusFlags(LINK_LPUART);

        /* An overrun means we lost bytes; drop the partial line rather than
         * hand a truncated one to the parser. */
        if ((flags & (uint32_t)kLPUART_RxOverrunFlag) != 0u)
        {
            LPUART_ClearStatusFlags(LINK_LPUART, (uint32_t)kLPUART_RxOverrunFlag);
            s_rxLen = 0u;
        }

        if ((flags & (uint32_t)kLPUART_RxDataRegFullFlag) == 0u)
        {
            return false; /* nothing more waiting */
        }

        char c = (char)LPUART_ReadByte(LINK_LPUART);

        if ((c == '\n') || (c == '\r'))
        {
            if (s_rxLen == 0u)
            {
                continue; /* ignore empty lines and CRLF pairs */
            }

            size_t n = (s_rxLen < (outSize - 1u)) ? s_rxLen : (outSize - 1u);
            (void)memcpy(out, s_rx, n);
            out[n]  = '\0';
            s_rxLen = 0u;
            return true;
        }

        if (s_rxLen < (LINK_RX_BUF_SIZE - 1u))
        {
            s_rx[s_rxLen] = c;
            s_rxLen++;
        }
        else
        {
            s_rxLen = 0u; /* line too long - resynchronise on the next '\n' */
        }
    }
}

bool LINK_GetField(const char *line, const char *key, char *out, size_t outSize)
{
    char   pattern[40];
    const char *p;
    size_t n = 0u;

    if ((line == NULL) || (key == NULL) || (out == NULL) || (outSize == 0u))
    {
        return false;
    }

    (void)snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    p = strstr(line, pattern);
    if (p == NULL)
    {
        return false;
    }

    p = strchr(p, ':');
    if (p == NULL)
    {
        return false;
    }
    p++;

    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }

    if (*p == '"')
    {
        p++;
        while ((*p != '"') && (*p != '\0') && (n < (outSize - 1u)))
        {
            out[n] = *p;
            n++;
            p++;
        }
    }
    else
    {
        while ((*p != ',') && (*p != '}') && (*p != '\0') && (n < (outSize - 1u)))
        {
            out[n] = *p;
            n++;
            p++;
        }
    }

    out[n] = '\0';
    return true;
}

void LINK_Fmt2(char *out, size_t outSize, float value)
{
    bool neg = (value < 0.0f);
    int  whole;
    int  frac;

    if (neg)
    {
        value = -value;
    }

    whole = (int)value;
    frac  = (int)(((value - (float)whole) * 100.0f) + 0.5f);

    if (frac >= 100)
    {
        whole++;
        frac -= 100;
    }

    (void)snprintf(out, outSize, "%s%d.%02d", neg ? "-" : "", whole, frac);
}
