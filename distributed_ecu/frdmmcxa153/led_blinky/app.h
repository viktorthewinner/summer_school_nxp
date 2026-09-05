/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define BOARD_LED_GPIO     BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

/* ---------------------------------------------------------------------------
 * Links to the two Arduino nodes.
 *
 * UART is point-to-point, so three boards form a star with the MCX at the
 * centre - two separate LPUARTs, not one shared pair.
 *
 *   ARD1  LPUART2  P3_15 TX (J2 pin 2,  ARD_D8)   P3_14 RX (J2 pin 4,  ARD_D9)
 *   ARD2  LPUART1  P1_9  TX (J2 pin 20, ARD_D19)  P1_8  RX (J2 pin 18, ARD_D18)
 *
 * LPUART0 stays on P0_2 / P0_3 for the MCU-Link VCOM debug console.
 *
 * All four link pins mux at ALT2. For LPUART2 that is copied from NXP's
 * generated lpuart example. For LPUART1 it was established by triangulation:
 * on P1_8 and P1_9, LPI2C0 sits at ALT3 and CT_INP8 at ALT4 in NXP's own
 * generated files, which places LPUART1 at ALT2. Note P1_9 has an empty ALT1
 * slot, so reading its position in the pin_signal string would wrongly
 * suggest ALT1.
 *
 * Using P1_8 / P1_9 for LPUART1 gives up LPI2C0 on the Arduino header;
 * I2C is still reachable on P3_27 / P3_28 (mikroBUS and Pmod).
 *
 * 38400 baud, not 115200: the Arduino side uses SoftwareSerial, which is
 * bit-banged and unreliable above roughly 38400 on a 16 MHz AVR. Both ends
 * must agree - see LINK_BAUD in the sketch.
 * ------------------------------------------------------------------------ */
#define LINK_BAUDRATE 38400U

#define LINK_ARD1_LPUART     LPUART2
#define LINK_ARD1_INSTANCE   2u
#define LINK_ARD1_IRQN       LPUART2_IRQn
#define LINK_ARD1_IRQHANDLER LPUART2_IRQHandler

#define LINK_ARD2_LPUART     LPUART1
#define LINK_ARD2_INSTANCE   1u
#define LINK_ARD2_IRQN       LPUART1_IRQn
#define LINK_ARD2_IRQHANDLER LPUART1_IRQHandler

/* ---------------------------------------------------------------------------
 * Link to the ESP32 gateway - LPI2C0 on P3_27 (SCL) / P3_28 (SDA), ALT2.
 *
 * These are the mikroBUS I2C pins: J5 pin 5 (SCL) and J5 pin 6 (SDA), with
 * GND on J5 pin 8 - per NXP's own lpi2c example readme for this board. NOT
 * J6: that is the SPI/analog half of the socket, and its pins 5/6 are
 * P1_2/P1_0. Pull-ups go to 3.3 V: the ESP32 is not 5 V tolerant.
 *
 * I2C is master-polled, so the gateway cannot initiate. The VCU reads it
 * every cycle; a remote command waits at most one poll period.
 * ------------------------------------------------------------------------ */
#define LINK_GW_LPI2C        LPI2C0
#define LINK_GW_INSTANCE     0u
#define LINK_GW_ADDR         0x42u   /* must match GW_I2C_ADDR in the sketch */
#define LINK_GW_BAUDRATE     400000u
/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
/*${prototype:end}*/

#endif /* _APP_H_ */
