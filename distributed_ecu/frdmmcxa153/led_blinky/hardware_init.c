/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*${header:start}*/
#include "pin_mux.h"
#include "peripherals.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "board.h"
#include "app.h"
#include <stdbool.h>
/*${header:end}*/

/*${function:start}*/
void BOARD_InitHardware(void)
{
    /* Attach 12 MHz FRO to LPUART2 (link to Arduino 1).
     * LPUART0, the VCOM debug console, is attached inside
     * BOARD_InitDebugConsole() below. */
    CLOCK_SetClockDiv(kCLOCK_DivLPUART2, 1u);
    CLOCK_AttachClk(kFRO12M_to_LPUART2);

    /* Attach 12 MHz FRO to LPUART1 (link to Arduino 2). */
    CLOCK_SetClockDiv(kCLOCK_DivLPUART1, 1u);
    CLOCK_AttachClk(kFRO12M_to_LPUART1);

    /* Attach 12 MHz FRO to LPI2C0 (link to the ESP32 gateway). */
    CLOCK_SetClockDiv(kCLOCK_DivLPI2C0, 1u);
    CLOCK_AttachClk(kFRO12M_to_LPI2C0);

    BOARD_InitPins();
    BOARD_BootClockFRO12M();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    LED_RED_INIT(LOGIC_LED_OFF);
}
/*${function:end}*/
