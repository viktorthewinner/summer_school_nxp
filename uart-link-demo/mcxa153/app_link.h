/*
 * app_link.h - UART link between FRDM-MCXA153 (LPUART2) and an Arduino.
 *
 * Transport only: newline-terminated ASCII lines in both directions.
 * The JSON payload is composed and parsed by the caller.
 *
 * Adapted from the ESP8266 lesson at
 * https://alexp25.github.io/ipcei-lab/lessons/lp4-iot-app/
 * The pins are identical; the baud rate is lower because the Arduino side
 * uses SoftwareSerial. See README.md.
 */
#ifndef APP_LINK_H
#define APP_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Bring up LPUART2 at the given baud rate, 8N1.
 * Pin mux and clock attach are done by Config Tools (BOARD_InitHardware). */
void LINK_Init(uint32_t baudRate);

/* Send one line. A '\n' terminator is appended automatically. Blocking. */
void LINK_SendLine(const char *line);

/* Non-blocking receive. Returns true once per complete line received.
 * The line is copied into out[] with the terminator stripped. */
bool LINK_Poll(char *out, size_t outSize);

/* Extract a JSON field value by key from a flat one-level object.
 * Handles both "key":"string" and "key":number.
 * Returns false if the key is absent. */
bool LINK_GetField(const char *line, const char *key, char *out, size_t outSize);

/* Format a float with 2 decimals without pulling float support into printf. */
void LINK_Fmt2(char *out, size_t outSize, float value);

#endif /* APP_LINK_H */
