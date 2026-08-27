/*
 * link_node.ino - Arduino side of the FRDM-MCXA153 <-> Arduino UART link.
 *
 * Adapted from the ESP8266 lesson at
 * https://alexp25.github.io/ipcei-lab/lessons/lp4-iot-app/
 *
 * Differences from the ESP8266 version, and why:
 *
 *  1. SoftwareSerial on D4/D5 instead of the hardware UART, so D0/D1 stay
 *     free for the USB serial monitor. The ESP8266 has a spare UART; an
 *     UNO/Nano does not.
 *  2. 38400 baud instead of 115200. SoftwareSerial is bit-banged and is not
 *     reliable at 115200 on a 16 MHz AVR.
 *  3. A resistor divider is REQUIRED on D5 -> MCX P1_4. The ESP8266 is a
 *     3.3 V part so the lesson wires it straight through; the Arduino is
 *     5 V and P1_4 is not 5 V tolerant. See README section 3.
 *
 * Protocol - newline-terminated JSON lines:
 *   in   {"code":"data","value":21.50}
 *   in   {"code":"chat","value":"hello from MCXA153"}
 *   out  {"code":"ack","of":"data","value":"21.50"}
 *   out  {"code":"chat","value":"whatever you type in the serial monitor"}
 */

#include <SoftwareSerial.h>

#define LINK_RX_PIN 4      /* <- MCX P1_5 / TX  (J1 pin 4), direct wire     */
#define LINK_TX_PIN 5      /* -> MCX P1_4 / RX  (J1 pin 2), VIA DIVIDER     */
#define LINK_BAUD   38400  /* must match LINK_BAUD in the MCX main.c        */

#define USB_BAUD    115200

SoftwareSerial link(LINK_RX_PIN, LINK_TX_PIN);

static char linkBuf[128];
static uint8_t linkLen = 0;

static char usbBuf[96];
static uint8_t usbLen = 0;

/* ---------------------------------------------------------------- helpers */

/* Extract a JSON field by key from a flat one-level object.
 * Handles both "key":"string" and "key":number. */
static bool getField(const char *src, const char *key,
                     char *out, uint8_t outSize)
{
  char pattern[24];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);

  const char *p = strstr(src, pattern);
  if (!p) return false;

  p = strchr(p, ':');
  if (!p) return false;
  p++;

  while (*p == ' ' || *p == '\t') p++;

  uint8_t n = 0;
  if (*p == '"') {
    p++;
    while (*p && *p != '"' && n < outSize - 1) out[n++] = *p++;
  } else {
    while (*p && *p != ',' && *p != '}' && n < outSize - 1) out[n++] = *p++;
  }
  out[n] = '\0';
  return true;
}

static void sendLine(const char *s)
{
  link.print(s);
  link.print('\n');
  Serial.print(F("TX  "));
  Serial.println(s);
}

/* ------------------------------------------------------- inbound handling */

static void handleFromMcx(const char *line)
{
  Serial.print(F("RX  "));
  Serial.println(line);

  char code[24];
  char value[80];

  if (!getField(line, "code", code, sizeof(code))) {
    Serial.println(F("    !! no \"code\" field"));
    return;
  }

  if (!strcmp(code, "data")) {
    if (getField(line, "value", value, sizeof(value))) {
      Serial.print(F("    >> data value = "));
      Serial.println(value);

      char out[96];
      snprintf(out, sizeof(out),
               "{\"code\":\"ack\",\"of\":\"data\",\"value\":\"%s\"}", value);
      sendLine(out);
    }
  }
  else if (!strcmp(code, "chat")) {
    if (getField(line, "value", value, sizeof(value))) {
      Serial.print(F("    >> chat from MCX: "));
      Serial.println(value);
    }
  }
  else {
    Serial.print(F("    >> unhandled code: "));
    Serial.println(code);
  }
}

static void sendChat(const char *text)
{
  char out[128];
  snprintf(out, sizeof(out), "{\"code\":\"chat\",\"value\":\"%s\"}", text);
  sendLine(out);
}

/* ------------------------------------------------------------------ setup */

void setup()
{
  Serial.begin(USB_BAUD);
  link.begin(LINK_BAUD);

  Serial.println();
  Serial.println(F("Arduino link node ready"));
  Serial.print(F("SoftwareSerial RX=D"));
  Serial.print(LINK_RX_PIN);
  Serial.print(F(" TX=D"));
  Serial.print(LINK_TX_PIN);
  Serial.print(F(" @ "));
  Serial.println(LINK_BAUD);
  Serial.println(F("Type a message + Enter to send it to the MCX."));
  Serial.println(F("----------------------------------------------"));
}

void loop()
{
  /* ---- bytes from the MCX ---- */
  while (link.available()) {
    char c = link.read();
    if (c == '\n' || c == '\r') {
      if (linkLen) {
        linkBuf[linkLen] = '\0';
        handleFromMcx(linkBuf);
        linkLen = 0;
      }
    } else if (linkLen < sizeof(linkBuf) - 1) {
      linkBuf[linkLen++] = c;
    } else {
      linkLen = 0;   /* overlong line - resync on the next newline */
    }
  }

  /* ---- bytes typed into the serial monitor ---- */
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbLen) {
        usbBuf[usbLen] = '\0';
        sendChat(usbBuf);
        usbLen = 0;
      }
    } else if (usbLen < sizeof(usbBuf) - 1) {
      usbBuf[usbLen++] = c;
    }
  }
}
