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
 *   LPI2C0   P3_27 / P3_28   ESP32 gateway, 400 kHz  mikroBUS J6
 *
 * Wire format: "[SRC] text", SRC being MCX, ARD1, ARD2 or GW.
 *
 * The ESP32 is a gateway, not a controller: remote traffic arrives here as a
 * request and this node decides what to do with it. That distinction is what
 * keeps a dropped WiFi link from being a safety problem.
 *
 * The red LED toggles once per second as a liveness indicator.
 *
 * The file name is inherited from the SDK led_blinky example this project was
 * generated from; rename it in CMakeLists.txt if you prefer.
 */

#include "board.h"
#include "app.h"
#include "link.h"

#include "fsl_debug_console.h"
#include "fsl_lpuart.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* The debug console owns transmit on LPUART0. We only read from it, so
 * polling the receiver here does not conflict with PRINTF. */
#define CONSOLE_LPUART ((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR)

#define KEY_BACKSPACE 0x08u
#define KEY_DELETE    0x7Fu

/* How often the gateway is polled, in main-loop iterations. I2C transactions
 * are blocking, so hammering the bus would starve the UART channels. */
#define GW_POLL_DIVIDER 200u

/*******************************************************************************
 * Variables
 ******************************************************************************/

static char     s_consoleLine[LINK_LINE_MAX];
static size_t   s_consoleLen;
static uint32_t s_gwTick;

/*******************************************************************************
 * Code
 ******************************************************************************/

void SysTick_Handler(void)
{
    /* Heartbeat: if this stops blinking, the firmware is stuck. */
    GPIO_PortToggle(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
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
            (void)snprintf(out, sizeof(out), "[MCX] %s", s_consoleLine);

            LINK_Broadcast(out, LINK_COUNT); /* LINK_COUNT excludes nobody */

            PRINTF("\r\n  sent to ARD1 + ARD2 + GW: %s\r\n", s_consoleLine);
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

/* A line from one node is printed here and relayed to all the others. */
static void link_service(link_id_t from)
{
    char line[LINK_LINE_MAX];
    char out[LINK_LINE_MAX];

    while (LINK_PollLine(from, line, sizeof(line)))
    {
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
    PRINTF("\r\n Type a line and press Enter to send it to every node.\r\n");
    PRINTF("-------------------------------------------------------------\r\n");
    PRINTF("> ");

    for (;;)
    {
        console_service();       /* this terminal -> everyone      */
        link_service(LINK_ARD1); /* ARD1 -> here + ARD2 + GW       */
        link_service(LINK_ARD2); /* ARD2 -> here + ARD1 + GW       */

        /* The gateway is polled, not interrupt-driven, and each poll is a
         * blocking I2C transaction. Rate-limit it so the UART channels keep
         * their share of the loop. */
        s_gwTick++;
        if (s_gwTick >= GW_POLL_DIVIDER)
        {
            s_gwTick = 0u;
            link_service(LINK_GW); /* laptop -> here + both Arduinos */
        }
    }
}
