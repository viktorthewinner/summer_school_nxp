/*
 * main.c - FRDM-MCXA153 <-> Arduino UART link demo.
 *
 * The MCX sends a JSON line once per second and prints whatever the Arduino
 * sends back on the MCU-Link VCOM debug console.
 *
 *   MCX  -> Arduino : {"code":"data","value":21.50}
 *   MCX  -> Arduino : {"code":"chat","value":"hello from MCXA153"}   (on SW3)
 *   Arduino -> MCX  : {"code":"ack","of":"data","value":"21.50"}
 *   Arduino -> MCX  : {"code":"chat","value":"typed in serial monitor"}
 *
 * Drop this into an SDK lpuart example project. See ../README.md.
 */
#include "board.h"
#include "app.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"

#include "app_link.h"

#include <stdio.h>
#include <string.h>

/* Must match LINK_BAUD in the Arduino sketch.
 * 38400 because the Arduino uses SoftwareSerial - see README section 5. */
#define LINK_BAUD          38400u

#define TX_PERIOD_MS       1000u

/* SW3 "Wake-up" button is on P1_7 (UM12012 Table 5). Optional - set
 * USE_BUTTON to 0 if you have not configured the pin in Config Tools. */
#define USE_BUTTON         1
#define BUTTON_GPIO        GPIO1
#define BUTTON_PIN         7u

static volatile uint32_t s_ms;

void SysTick_Handler(void)
{
    s_ms++;
}

int main(void)
{
    char     line[192];
    char     num[16];
    char     code[24];
    char     value[128];
    uint32_t nextTx  = 0u;
    float    sample  = 20.0f;
#if USE_BUTTON
    bool     prevBtn = true;
#endif

    BOARD_InitHardware();
    SysTick_Config(SystemCoreClock / 1000u);

    LINK_Init(LINK_BAUD);

    PRINTF("\r\n");
    PRINTF("FRDM-MCXA153 <-> Arduino UART link demo\r\n");
    PRINTF("LPUART2 @ %u baud on P1_5 (TX, J1-4) / P1_4 (RX, J1-2)\r\n", LINK_BAUD);
    PRINTF("----------------------------------------------------\r\n");

    for (;;)
    {
        /* ---- periodic data frame ------------------------------------- */
        if ((int32_t)(s_ms - nextTx) >= 0)
        {
            nextTx = s_ms + TX_PERIOD_MS;

            LINK_Fmt2(num, sizeof(num), sample);
            (void)snprintf(line, sizeof(line),
                           "{\"code\":\"data\",\"value\":%s}", num);
            LINK_SendLine(line);
            PRINTF("TX  %s\r\n", line);

            sample += 0.25f;
            if (sample > 30.0f)
            {
                sample = 20.0f;
            }
        }

#if USE_BUTTON
        /* ---- SW3 press sends a chat frame ---------------------------- */
        {
            bool btn = (GPIO_PinRead(BUTTON_GPIO, BUTTON_PIN) != 0u);

            if (prevBtn && !btn) /* active low - falling edge */
            {
                (void)snprintf(line, sizeof(line),
                               "{\"code\":\"chat\",\"value\":\"hello from MCXA153\"}");
                LINK_SendLine(line);
                PRINTF("TX  %s\r\n", line);
            }
            prevBtn = btn;
        }
#endif

        /* ---- anything coming back from the Arduino ------------------- */
        if (LINK_Poll(line, sizeof(line)))
        {
            PRINTF("RX  %s\r\n", line);

            if (LINK_GetField(line, "code", code, sizeof(code)))
            {
                if ((strcmp(code, "chat") == 0) &&
                    LINK_GetField(line, "value", value, sizeof(value)))
                {
                    PRINTF("    >> chat from Arduino: %s\r\n", value);
                }
                else if (strcmp(code, "ack") == 0)
                {
                    if (LINK_GetField(line, "value", value, sizeof(value)))
                    {
                        PRINTF("    >> Arduino acked value %s\r\n", value);
                    }
                }
                else
                {
                    /* unknown code - ignore */
                }
            }
        }
    }
}
