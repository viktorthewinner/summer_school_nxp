/*
 * test_link_perc.ino  --  PERC (Arduino 1) link test.
 *
 * The other self-tests are standalone and prove the parts ON this board. This
 * one proves the wiring BETWEEN this board and the MCX - the three wires that
 * no standalone test can reach.
 *
 * Run it with the MCX flashed and its VCOM open too. Two terminals, one per
 * board, is the whole point: each shows one direction, so a failure tells you
 * which half is broken instead of "the link does not work".
 *
 * ---------------------------------------------------------------------------
 * THE LINK  (SoftwareSerial, 38400)
 *
 *   D4  <-  MCX J2-2   (P3_15, marked D8)   DIRECT WIRE
 *   D5  ->  MCX J2-4   (P3_14, marked D9)   THROUGH THE 1k / 2k DIVIDER
 *   GND <-> star point                      not optional, see below
 *
 * The two directions are not built the same, and that is what makes a one-way
 * failure worth something:
 *
 *   MCX -> here works, here -> MCX does not
 *        The return path is the only one with parts in it. Suspect the
 *        divider: 1k from D5 to the junction, 2k from the junction to ground,
 *        MCX on the junction. Swapped, that gives 1.7 V and the MCX sees
 *        nothing. Missing altogether, D5 puts 5 V on a 3.3 V pin.
 *
 *   here -> MCX works, MCX -> here does not
 *        The forward path is a bare wire, so it is the wire, the pin, or the
 *        far end. J2-2 is the pin marked D8 - count the two-row header twice.
 *
 *   Both directions garbled rather than silent
 *        Almost always no common ground, or a baud mismatch. Both ends must be
 *        at 38400 and all three supply negatives must meet at the star point.
 *
 *   Neither direction, and j below also fails
 *        Not the cabling at all - stop looking at it.
 *
 * ---------------------------------------------------------------------------
 * COMMANDS   (USB Serial Monitor, 115200, line ending Newline)
 *
 *   t          send one probe up to the MCX
 *   a          probe once a second until you stop it
 *   x          stop probing
 *   s <text>   send any line you like up the link
 *   j          jumper test - see below
 *   ?          help
 *
 * Everything arriving on the link is printed as [rx], always, whatever mode
 * you are in. A Q,<n> from the MCX is answered automatically with R,<n>,PERC.
 *
 *   j  is the bisection. Put ONE jumper between D4 and D5 - nothing else, the
 *      MCX can stay unplugged - and this sends a line out of D5 and expects it
 *      back on D4. That exercises the sketch and SoftwareSerial with no
 *      cabling involved. Passing j and failing everything else puts the fault
 *      firmly between the boards. REMOVE THE JUMPER AFTERWARDS.
 */

#include <SoftwareSerial.h>

/* ---- configuration ------------------------------------------------------ */

static const char     NODE_NAME[]  = "PERC";

static const uint8_t  PIN_LINK_RX  = 4;   /* <- MCX P3_15, direct wire         */
static const uint8_t  PIN_LINK_TX  = 5;   /* -> MCX P3_14, THROUGH THE DIVIDER */

static const uint32_t LINK_BAUD    = 38400;
static const uint32_t USB_BAUD     = 115200;
static const uint16_t AUTO_MS      = 1000;
static const uint8_t  LINE_MAX     = 48;

/* ---- state -------------------------------------------------------------- */

SoftwareSerial link(PIN_LINK_RX, PIN_LINK_TX);

static bool     g_auto;
static uint32_t g_autoAt;
static uint16_t g_seq;
static uint16_t g_rxCount;

static char     g_linkLine[LINE_MAX];
static uint8_t  g_linkLen;
static char     g_usbLine[LINE_MAX];
static uint8_t  g_usbLen;

/* ---- link --------------------------------------------------------------- */

/* One complete line arrived on D4. Show it, and answer it if it was a probe. */
static void linkLine(char *s)
{
  g_rxCount++;
  Serial.print(F("[rx] "));
  Serial.println(s);

  if (s[0] == 'Q' && s[1] == ',') {
    char out[LINE_MAX];
    snprintf(out, sizeof(out), "R,%s,%s", s + 2, NODE_NAME);
    link.println(out);
    Serial.print(F("     answered: "));
    Serial.println(out);
  }
}

static void linkService(void)
{
  while (link.available()) {
    char c = (char)link.read();
    if (c == '\n' || c == '\r') {
      if (g_linkLen == 0) continue;
      g_linkLine[g_linkLen] = '\0';
      g_linkLen = 0;
      linkLine(g_linkLine);
    } else if (g_linkLen < (LINE_MAX - 1)) {
      g_linkLine[g_linkLen++] = c;
    }
  }
}

static void probe(void)
{
  char out[LINE_MAX];
  snprintf(out, sizeof(out), "T,%u,%s", (unsigned)(++g_seq), NODE_NAME);
  link.println(out);
  Serial.print(F("[tx] "));
  Serial.print(out);
  Serial.println(F("   -> expect [PERC] ... on the MCX console"));
}

/* ---- jumper test -------------------------------------------------------- */

static void jumperTest(void)
{
  Serial.println(F("\n  Jumper D4 to D5, then this should come straight back."));
  Serial.println(F("  Nothing else needs to be connected."));

  while (link.available()) (void)link.read();   /* clear anything stale */
  g_linkLen = 0;

  link.println(F("J,loopback"));

  char     got[LINE_MAX];
  uint8_t  n    = 0;
  uint32_t t0   = millis();
  bool     done = false;

  while (!done && (millis() - t0) < 500) {
    if (link.available()) {
      char c = (char)link.read();
      if (c == '\n' || c == '\r') { if (n) done = true; }
      else if (n < (LINE_MAX - 1)) got[n++] = c;
    }
  }
  got[n] = '\0';

  if (!done || n == 0) {
    Serial.println(F("  FAIL - nothing came back."));
    Serial.println(F("  If the jumper really is on D4-D5, the fault is on this"));
    Serial.println(F("  board or in the sketch, not in the cabling to the MCX."));
  } else if (strcmp(got, "J,loopback") == 0) {
    Serial.println(F("  PASS - SoftwareSerial and both pins are good."));
    Serial.println(F("  REMOVE THE JUMPER before wiring the MCX back on."));
  } else {
    Serial.print(F("  GARBLED - sent J,loopback and got: "));
    Serial.println(got);
    Serial.println(F("  Corrupt rather than absent is usually a poor jumper"));
    Serial.println(F("  contact. Reseat it and run j again."));
  }
}

/* ---- console ------------------------------------------------------------ */

static void help(void)
{
  Serial.println(F(
    "\n  t          one probe to the MCX      a   probe once a second"
    "\n  x          stop probing              s <text>  send any line"
    "\n  j          D4-D5 jumper test         ?   help"
    "\n  Received lines always print as [rx].\n"));
}

static void handleLine(char *s)
{
  while (*s == ' ') s++;
  char c = *s;
  char *arg = s + 1;
  while (*arg == ' ') arg++;

  switch (c) {
    case 't': probe(); break;
    case 'a': g_auto = true;  g_autoAt = millis() - AUTO_MS;
              Serial.println(F("  probing once a second - x to stop")); break;
    case 'x': g_auto = false;
              Serial.print(F("  stopped. lines received so far: "));
              Serial.println(g_rxCount); break;
    case 's': if (*arg) { link.println(arg);
                          Serial.print(F("[tx] ")); Serial.println(arg); }
              else Serial.println(F("  s needs something to send"));
              break;
    case 'j': jumperTest(); break;
    case '?': help(); break;
    case '\0': break;
    default:  Serial.println(F("  ? unknown - type ? for help")); break;
  }
}

static void consoleService(void)
{
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_usbLen == 0) continue;
      g_usbLine[g_usbLen] = '\0';
      g_usbLen = 0;
      handleLine(g_usbLine);
    } else if (g_usbLen < (LINE_MAX - 1)) {
      g_usbLine[g_usbLen++] = c;
    }
  }
}

/* ---- entry -------------------------------------------------------------- */

void setup(void)
{
  /* Everything else on this board stays inert - the buzzer especially, so it
   * does not shriek through the whole test. */
  pinMode(6, OUTPUT); digitalWrite(6, LOW);     /* TRIG left  */
  pinMode(7, OUTPUT); digitalWrite(7, LOW);     /* TRIG right */
  pinMode(9, OUTPUT); digitalWrite(9, LOW);     /* SFM-27     */

  Serial.begin(USB_BAUD);
  link.begin(LINK_BAUD);

  Serial.println(F("\n====================================="));
  Serial.println(F(" PERC link test - Arduino 1"));
  Serial.println(F("====================================="));
  Serial.println(F(" D4 <- MCX J2-2 (P3_15, marked D8)  direct wire"));
  Serial.println(F(" D5 -> MCX J2-4 (P3_14, marked D9)  through the 1k/2k divider"));
  Serial.println(F(" GND to the star point. 38400 at both ends."));
  Serial.println(F(" Open the MCX VCOM as well and type link there."));
  help();
}

void loop(void)
{
  consoleService();
  linkService();

  if (g_auto && (millis() - g_autoAt) >= AUTO_MS) {
    g_autoAt = millis();
    probe();
  }
}
