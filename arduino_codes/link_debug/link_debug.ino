/*
 * link_debug.ino  --  raw link debugger. Flash to EITHER Arduino.
 *
 * The production sketches parse. This one does not: it shows every byte that
 * arrives on the link, as hex, whatever it is. That difference is the whole
 * point. A parser silently discards anything malformed, so a link receiving
 * garbage - wrong baud, no common ground, a floating pin - looks exactly like
 * a link receiving nothing, and those two faults have nothing in common.
 *
 * It is also symmetric with the MCX console: anything you type here goes out
 * on the link, so you can drive the link from either end and watch the other.
 *
 * ---------------------------------------------------------------------------
 * WIRING - identical on both boards, only the MCX pins differ
 *
 *   PERC (Arduino 1)                    ACT (Arduino 2)
 *   D4  <- MCX J2-2  (P3_15, "D8")      D4  <- MCX J2-20 (P1_9, "D19")
 *   D5  -> MCX J2-4  (P3_14, "D9")      D5  -> MCX J2-18 (P1_8, "D18")
 *          THROUGH THE 1k/2k DIVIDER           THROUGH THE 1k/2k DIVIDER
 *   GND <-> star point                  GND <-> star point
 *
 * D4 is a bare wire: the MCX drives 3.3 V into a 5 V input, which clears the
 * ATmega's 3.0 V threshold with 0.3 V to spare. D5 must go through the
 * divider or the Arduino puts 5 V on a pin that is not 5 V tolerant.
 *
 * ---------------------------------------------------------------------------
 * COMMANDS   (USB Serial Monitor, 115200, line ending Newline)
 *
 *   t          send one probe line to the MCX
 *   a          probe once a second, until x
 *   e          ECHO MODE: send every received byte straight back
 *   j          JUMPER TEST: prove THIS board's SoftwareSerial, no MCX needed
 *   h          toggle hex / text view of received bytes
 *   x          stop whatever is running
 *   s <text>   send any line you like
 *   ?          help
 *
 * ---------------------------------------------------------------------------
 * HOW TO USE IT - cut the link in half, then in half again
 *
 *   1. j here.               Fails -> this board. Nothing else matters yet.
 *   2. loop1 / loop2 on the  Fails -> the MCX: pin mux, clock, LPUART.
 *      MCX console.                  Neither wire nor Arduino is involved.
 *   3. e here, then tx1 on   MCX sees its own line come back -> BOTH wires
 *      the MCX.                      and both directions are good.
 *   4. t here.               MCX prints it -> the return path (with the
 *                                     divider in it) works on its own.
 *
 * Steps 1 and 2 each test one board alone. Only after both pass is it worth
 * looking at a wire.
 */

#include <SoftwareSerial.h>

/* ---- configuration ------------------------------------------------------ */

static const uint8_t  PIN_LINK_RX = 4;
static const uint8_t  PIN_LINK_TX = 5;

static const uint32_t LINK_BAUD = 38400;    /* must match LINK_BAUDRATE */
static const uint32_t USB_BAUD  = 115200;
static const uint8_t  LINE_MAX  = 64;

/* ---- state -------------------------------------------------------------- */

SoftwareSerial link(PIN_LINK_RX, PIN_LINK_TX);

static char     g_line[LINE_MAX];
static uint8_t  g_len;

static bool     g_hex   = true;     /* show bytes as hex, not as text */
static bool     g_echo  = false;    /* mirror everything back to the MCX */
static bool     g_auto  = false;    /* probe once a second */
static uint32_t g_autoAt;
static uint16_t g_seq;

static uint32_t g_rxBytes, g_rxLines;
static uint32_t g_lastRxAt;
static uint8_t  g_col;

/* ---- receive ------------------------------------------------------------
 * No parsing, no line assembly, no filtering. Every byte is printed as it
 * arrives, and only newline is given any meaning at all - and then only to
 * break the display into readable rows.
 * ------------------------------------------------------------------------ */

static void showByte(uint8_t b)
{
  if (g_col == 0) {
    Serial.print(F("[rx] "));
  }

  if (g_hex) {
    if (b < 0x10) Serial.print('0');
    Serial.print(b, HEX);
    Serial.print((b >= 0x20 && b < 0x7F) ? (char)b : '.');
    Serial.print(' ');
    if (++g_col >= 16) { Serial.println(); g_col = 0; }
  } else {
    if (b == '\n' || b == '\r') {
      Serial.println();
      g_col = 0;
    } else {
      Serial.print((b >= 0x20 && b < 0x7F) ? (char)b : '.');
      g_col = 1;
    }
  }
}

static void linkService(void)
{
  while (link.available()) {
    uint8_t b = (uint8_t)link.read();

    g_rxBytes++;
    g_lastRxAt = millis();
    if (b == '\n') g_rxLines++;

    showByte(b);

    if (g_echo) link.write(b);   /* straight back, byte for byte */
  }
}

/* ---- send --------------------------------------------------------------- */

static void probe(void)
{
  char out[LINE_MAX];
  snprintf(out, sizeof(out), "R,%u,DEBUG", (unsigned)(++g_seq));
  link.println(out);
  Serial.print(F("[tx] "));
  Serial.println(out);
}

/* ---- jumper test --------------------------------------------------------
 * Short D4 to D5 with one wire, with NOTHING else on either pin, and this
 * board talks to itself. It proves SoftwareSerial, both pins and the baud
 * rate without the MCX, the loom or the divider being involved at all.
 *
 * Take the two link wires off first. The divider on D5 would load the pin,
 * and the MCX driving D4 would fight the jumper.
 * ------------------------------------------------------------------------ */
static void jumperTest(void)
{
  const char *probeText = "JUMPER.0123456789";
  char        got[LINE_MAX];
  uint8_t     n = 0;
  uint32_t    t0;

  Serial.println(F("\n  JUMPER TEST"));
  Serial.println(F("  Take BOTH link wires off, then short D4 to D5."));
  Serial.println(F("  Sending in 3 s..."));
  delay(3000);

  while (link.available()) link.read();     /* drain */

  link.println(probeText);

  t0 = millis();
  while ((millis() - t0) < 500 && n < (LINE_MAX - 1)) {
    if (link.available()) {
      char c = (char)link.read();
      if (c == '\n' || c == '\r') break;
      got[n++] = c;
    }
  }
  got[n] = '\0';

  if (strcmp(got, probeText) == 0) {
    Serial.println(F("  PASS - this board's link pins and SoftwareSerial work."));
    Serial.println(F("  Any remaining fault is the MCX, the wires or the divider."));
  } else if (n > 0) {
    Serial.print(F("  FAIL - got back garbage: \""));
    Serial.print(got);
    Serial.println(F("\"\n  That is a baud problem, not a wiring one."));
  } else {
    Serial.println(F("  FAIL - nothing came back."));
    Serial.println(F("  With the jumper really on D4-D5 this board is the fault:"));
    Serial.println(F("  wrong pins, a dead pin, or SoftwareSerial not started."));
  }
  Serial.println(F("  Put the link wires back when you are done.\n"));
}

/* ---- console ------------------------------------------------------------ */

static void help(void)
{
  Serial.println(F(
    "\n  t   one probe to the MCX        a   probe every second"
    "\n  e   echo mode - mirror bytes    j   jumper test (D4 to D5)"
    "\n  h   toggle hex / text view      x   stop"
    "\n  s <text>  send a line           ?   this help"
    "\n"
    "\n  Order to debug in: j here, then loop1/loop2 on the MCX, then"
    "\n  e here with tx1 there. Each step blames one board only.\n"));
}

static void status(void)
{
  Serial.print(F("  rx "));
  Serial.print(g_rxBytes);
  Serial.print(F(" bytes, "));
  Serial.print(g_rxLines);
  Serial.print(F(" lines"));

  if (g_rxBytes > 0) {
    Serial.print(F(", last "));
    Serial.print(millis() - g_lastRxAt);
    Serial.print(F(" ms ago"));
  } else {
    Serial.print(F(" - NOTHING has ever arrived on D4"));
  }

  Serial.print(F("   [hex "));
  Serial.print(g_hex ? F("on") : F("off"));
  Serial.print(F(", echo "));
  Serial.print(g_echo ? F("ON") : F("off"));
  Serial.println(F("]"));
}

static void handleLine(char *s)
{
  while (*s == ' ') s++;

  switch (*s) {
    case 't': probe(); break;
    case 'a': g_auto = true;  g_autoAt = 0; Serial.println(F("  probing")); break;
    case 'x': g_auto = false; g_echo = false; Serial.println(F("  stopped")); break;
    case 'e': g_echo = !g_echo;
              Serial.print(F("  echo mode "));
              Serial.println(g_echo ? F("ON - mirroring every byte back")
                                    : F("off"));
              break;
    case 'h': g_hex = !g_hex; g_col = 0;
              Serial.println(g_hex ? F("  hex view") : F("  text view"));
              break;
    case 'j': jumperTest(); break;
    case 'i': status(); break;
    case 's': if (s[1] == ' ') {
                link.println(s + 2);
                Serial.print(F("[tx] ")); Serial.println(s + 2);
              }
              break;
    case '?': help(); break;
    case '\0': status(); break;
    default:  Serial.println(F("  ? unknown - type ? for help")); break;
  }
}

static void consoleService(void)
{
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_len == 0) continue;
      g_line[g_len] = '\0';
      g_len = 0;
      if (g_col != 0) { Serial.println(); g_col = 0; }
      handleLine(g_line);
    } else if (g_len < (LINE_MAX - 1)) {
      g_line[g_len++] = c;
    }
  }
}

static void autoService(void)
{
  if (!g_auto) return;
  if ((millis() - g_autoAt) < 1000UL) return;
  g_autoAt = millis();
  probe();
}

/* ---- entry -------------------------------------------------------------- */

void setup(void)
{
  Serial.begin(USB_BAUD);
  link.begin(LINK_BAUD);

  Serial.println(F("\n============================================"));
  Serial.println(F(" link_debug - raw view of the MCX link"));
  Serial.println(F("============================================"));
  Serial.println(F("  D4 <- MCX TX (direct)   D5 -> MCX RX (divider)"));
  Serial.print  (F("  link "));
  Serial.print  (LINK_BAUD);
  Serial.println(F(" baud, 8N1"));
  Serial.println(F("  Every byte arriving on D4 is printed. Nothing is parsed."));
  help();
}

void loop(void)
{
  consoleService();
  linkService();
  autoService();
}
