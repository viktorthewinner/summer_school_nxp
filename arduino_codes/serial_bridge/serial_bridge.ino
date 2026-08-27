/*
 * serial_bridge.ino - Arduino node for the three-board serial hub.
 *
 * UPLOAD THIS SAME SKETCH TO BOTH ARDUINOS - it is identical on each.
 * A node's identity comes from which MCX port it is wired to, not from its
 * firmware, so there is no ID to set and no way to get two boards wrong.
 * The MCX tags every relayed message with its source.
 *
 * Type a line here and it reaches the MCX terminal and the other Arduino.
 *
 *   USB Serial (D0/D1)   115200   this Serial Monitor
 *   SoftwareSerial D4/D5  38400   link to the FRDM-MCXA153
 *
 * WIRING - see WIRING_3_BOARDS.svg
 *
 *   Arduino 1                                   Arduino 2
 *   MCX J2 pin 2  (P3_15) -> D4   direct        MCX J2 pin 20 (P1_9) -> D4
 *   MCX J2 pin 4  (P3_14) <- D5   VIA DIVIDER   MCX J2 pin 18 (P1_8) <- D5
 *   MCX J3 pin 12 or 14   <-> GND               same
 *
 * The divider (1 kOhm series + 2 kOhm to GND) on D5 is mandatory on BOTH
 * boards: the Arduino drives 5 V and the MCX pins are not 5 V tolerant.
 *
 * D0/D1 are deliberately left free - they are shared with the USB bootloader,
 * and anything driving them during an upload breaks avrdude.
 */

#include <SoftwareSerial.h>

/* ---- configuration ------------------------------------------------------ */

static const uint8_t  LINK_RX_PIN = 4;      /* <- MCX TX, direct wire       */
static const uint8_t  LINK_TX_PIN = 5;      /* -> MCX RX, THROUGH DIVIDER   */
static const uint32_t LINK_BAUD   = 38400;  /* must match LINK_BAUDRATE     */
static const uint32_t USB_BAUD    = 115200;

static const uint8_t  LINE_MAX    = 96;

/* ---- state -------------------------------------------------------------- */

SoftwareSerial link(LINK_RX_PIN, LINK_TX_PIN);

static char    linkLine[LINE_MAX];
static uint8_t linkLen = 0;

static char    usbLine[LINE_MAX];
static uint8_t usbLen = 0;

/* ---- helpers ------------------------------------------------------------ */

/* Collect bytes into buf until a newline arrives.
 * Returns true once per complete line; the terminator is stripped. */
static bool collectLine(Stream &src, char *buf, uint8_t &len, uint8_t cap)
{
  while (src.available()) {
    char c = (char)src.read();

    if (c == '\n' || c == '\r') {
      if (len == 0) continue;          /* skip blanks and the CRLF tail */
      buf[len] = '\0';
      len = 0;
      return true;
    }

    if (len < cap - 1) {
      buf[len++] = c;
    } else {
      len = 0;                          /* overlong: drop and resync */
    }
  }
  return false;
}

/* ---- sketch ------------------------------------------------------------- */

void setup()
{
  Serial.begin(USB_BAUD);
  link.begin(LINK_BAUD);

  Serial.println();
  Serial.println(F("====================================================="));
  Serial.println(F(" Arduino node  ->  FRDM-MCXA153 hub"));
  Serial.println(F("====================================================="));
  Serial.print(F(" link : SoftwareSerial  RX=D"));
  Serial.print(LINK_RX_PIN);
  Serial.print(F("  TX=D"));
  Serial.print(LINK_TX_PIN);
  Serial.print(F("  @ "));
  Serial.println(LINK_BAUD);
  Serial.println();
  Serial.println(F(" Type a line + Enter -> goes to the MCX and the other Arduino."));
  Serial.println(F(" Set the Serial Monitor line ending to Newline."));
  Serial.println(F("-----------------------------------------------------"));
}

void loop()
{
  /* this Serial Monitor -> MCX hub -> other Arduino */
  if (collectLine(Serial, usbLine, usbLen, LINE_MAX)) {
    link.print(usbLine);
    link.print('\n');
    Serial.print(F("  [you] "));
    Serial.println(usbLine);
  }

  /* hub -> this Serial Monitor. Already tagged [MCX], [ARD1] or [ARD2]. */
  if (collectLine(link, linkLine, linkLen, LINE_MAX)) {
    Serial.print(F("  "));
    Serial.println(linkLine);
  }
}
