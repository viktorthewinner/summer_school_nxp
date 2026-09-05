/*
 * perc.ino  --  PERC node (Arduino 1): ranging, PIR, horn.
 *
 * This is the teleop build. PERC measures and reports; it decides nothing.
 * The VCU owns every decision in this car, including whether an obstacle
 * matters and whether the horn sounds. PERC's whole contract is two lines:
 *
 *   out   P,<front_cm>,<back_cm>,<pir>     10 Hz     -1 cm = no echo
 *   in    H,<0|1>                          horn off / on, from the VCU
 *
 * Anything else arriving on the link is printed to the USB monitor and
 * ignored, which is how the hub's "[SRC] text" chatter stays harmless.
 *
 * ---------------------------------------------------------------------------
 * LINK  (SoftwareSerial D4/D5, 38400, to the MCX on LPUART2 - J2 pins 2/4)
 *
 * ---------------------------------------------------------------------------
 * PINS   (SCHEMA_WIRING.svg block C)
 *
 *   D3  ECHO front  (INT1)      D7  TRIG front
 *   D2  ECHO back   (INT0)      D6  TRIG back
 *   D8  HC-SR501 PIR            D9  SFM-27 buzzer
 *
 *   Free: D10-D13, A0-A5.  D0/D1 stay free - CH340 bootloader.
 *
 * FRONT IS D3/D7 AND BACK IS D2/D6. That is the only place the pair-to-pin
 * mapping exists; if you move the sensors on the chassis, swap them here and
 * nowhere else - the VCU only ever sees "front" and "back".
 *
 * ---------------------------------------------------------------------------
 * WHY THE RANGING IS INTERRUPT-DRIVEN AND NOT pulseIn()
 *
 * pulseIn() sits and waits: up to 10 ms per ping with the timeout below, and
 * during those 10 ms this node services nothing. Not blocking the loop is the
 * entire reason PERC is its own node, so the echo is timed by INT0/INT1
 * instead - which is what those two pins were held open for.
 *
 * The one wart: SoftwareSerial disables interrupts for a whole character
 * (~260 us at 38400) while it receives, so an echo edge landing in that
 * window is timestamped late. Worst case that is about 4 cm of error on one
 * reading, and it cannot accumulate. Live with it; the alternative is a
 * hardware UART this board does not have spare.
 *
 * THEY STILL FIRE ALTERNATELY. Two HC-SR04 pinging together hear each other's
 * burst and both report nonsense - front, wait 60 ms, back, wait 60 ms.
 */

#include <SoftwareSerial.h>

/* ---- configuration ------------------------------------------------------ */

static const uint8_t  PIN_LINK_RX  = 4;   /* <- MCX P3_15, direct wire         */
static const uint8_t  PIN_LINK_TX  = 5;   /* -> MCX P3_14, THROUGH THE DIVIDER */

static const uint8_t  PIN_ECHO_F   = 3;   /* INT1 - front */
static const uint8_t  PIN_TRIG_F   = 7;
static const uint8_t  PIN_ECHO_B   = 2;   /* INT0 - back  */
static const uint8_t  PIN_TRIG_B   = 6;
static const uint8_t  PIN_PIR      = 8;
static const uint8_t  PIN_BUZZ     = 9;

static const uint32_t LINK_BAUD    = 38400;
static const uint32_t USB_BAUD     = 115200;
static const uint8_t  LINE_MAX     = 48;

/* Cap the echo wait at the range that matters. Full scale is 38 ms and would
 * halve the update rate for nothing; 10 ms is about 1.7 m. Past that we
 * report -1, which is honest. */
static const uint32_t ECHO_TIMEOUT_US = 10000UL;

/* 60 ms between pings, alternating, so each sensor gets 8.3 Hz. */
static const uint16_t PING_GAP_MS  = 60;

static const uint16_t REPORT_MS    = 100;   /* P, line rate - 10 Hz */

/* Console diagnostics. The VCU sends H, at least once a second even with
 * the horn off, precisely so this node can tell a live VCU from a dead
 * one - nothing else ever comes down this link. */
static const uint16_t VCU_SILENT_MS = 3000;
static const uint16_t STATUS_MS     = 2000;

/* An echo of a centimetre or two is the sensor hearing its own ring-down,
 * not an object. Treat anything under this as no reading. */
static const int      RANGE_MIN_CM = 2;

/* ---- horn ---------------------------------------------------------------
 * The SFM-27 comes in two kinds and they need opposite drive - FULL_SCHEMA.md
 * section 5, and test_perc.ino's b1 / b2 tell you which one you have. tone()
 * is the default here because it is audible on both: a passive buzzer gives a
 * clean 2 kHz, an active one rattles at its own pitch. If yours turned out to
 * be ACTIVE and you want it clean, set this to 1.
 * ------------------------------------------------------------------------ */
#define BUZZER_IS_ACTIVE 0
static const uint16_t HORN_HZ = 2000;

/* The VCU repeats H,1 every 250 ms while the key is held, so silence for
 * twice that means the horn is no longer wanted - or nobody is left to want
 * anything. A single lost "off" leaving the buzzer sounding until someone
 * pulls a battery is the most embarrassing failure available to this car,
 * and it costs four lines to make impossible. */
static const uint16_t HORN_TIMEOUT_MS = 500;

/* ---- state -------------------------------------------------------------- */

SoftwareSerial link(PIN_LINK_RX, PIN_LINK_TX);

/* Echo timing. Only one sensor is ever in flight, so both ISRs share these
 * and the s_echoArmed gate keeps the idle sensor's pin out of it. */
static volatile uint32_t s_echoRiseUs;
static volatile uint32_t s_echoWidthUs;
static volatile bool     s_echoDone;
static volatile uint8_t  s_echoArmed;      /* 0 none, 1 front, 2 back */
static volatile uint32_t s_echoInts;       /* every edge, armed or not      */

static int      g_frontCm = -1;
static int      g_backCm  = -1;
static uint8_t  g_pir;

static uint8_t  g_pingWho = 1;             /* whose turn: 1 front, 2 back */
static uint32_t g_pingAtMs;
static uint32_t g_pingAtUs;
static bool     g_pingInFlight;

static uint32_t g_reportAt;
static bool     g_horn;
static uint32_t g_hornAt;

static char     g_linkLine[LINE_MAX];
static uint8_t  g_linkLen;

static uint32_t g_vcuAt;
static int8_t   g_vcuUp = -1;      /* -1 never heard, 0 down, 1 up */
static uint32_t g_statusAt;
static uint32_t g_rxLines;
static uint32_t g_rxBytes;         /* raw bytes, whatever they turned out to be */
static uint32_t g_txLines;         /* P, lines handed to SoftwareSerial         */
static uint32_t g_echoPrev;        /* s_echoInts at the last status print       */

/* ---- ranging ------------------------------------------------------------ */

static inline void echoEdge(uint8_t who, uint8_t pin)
{
  /* Counted BEFORE the gate below, deliberately. An echo pin with nothing
   * driving it chatters, and every one of those edges costs an interrupt
   * whether or not it was this sensor's turn. SoftwareSerial times its bits
   * by counting CPU cycles, so a storm here is one of the very few things
   * that can kill the link without anyone touching a wire. diagService()
   * turns this counter into an answer. */
  s_echoInts++;

  if (s_echoArmed != who) return;          /* not this sensor's turn */

  if (digitalRead(pin)) {
    s_echoRiseUs = micros();
  } else if (s_echoRiseUs != 0UL) {
    s_echoWidthUs = micros() - s_echoRiseUs;
    s_echoDone    = true;
  }
}

static void echoFrontISR(void) { echoEdge(1, PIN_ECHO_F); }
static void echoBackISR(void)  { echoEdge(2, PIN_ECHO_B); }

static void pingFire(uint8_t who)
{
  uint8_t trig = (who == 1) ? PIN_TRIG_F : PIN_TRIG_B;

  noInterrupts();
  s_echoRiseUs  = 0UL;
  s_echoWidthUs = 0UL;
  s_echoDone    = false;
  s_echoArmed   = who;
  interrupts();

  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);                   /* the datasheet's 10 us trigger */
  digitalWrite(trig, LOW);

  g_pingAtUs     = micros();
  g_pingAtMs     = millis();
  g_pingInFlight = true;
}

static void pingStore(uint8_t who, int cm)
{
  if (who == 1) g_frontCm = cm;
  else          g_backCm  = cm;
}

static void pingNext(void)
{
  s_echoArmed    = 0;
  g_pingInFlight = false;
  g_pingWho      = (g_pingWho == 1) ? 2 : 1;
}

/* Fire, collect, swap. Never blocks: the echo arrives in an ISR and this only
 * ever looks at a flag. */
static void rangingService(void)
{
  if (!g_pingInFlight) {
    if ((millis() - g_pingAtMs) < PING_GAP_MS) return;
    pingFire(g_pingWho);
    return;
  }

  if (s_echoDone) {
    uint32_t us = s_echoWidthUs;           /* the ISR is finished with it */
    int      cm = (int)(us / 58UL);        /* 343 m/s, out and back      */

    pingStore(g_pingWho, (cm < RANGE_MIN_CM) ? -1 : cm);
    pingNext();
    return;
  }

  if ((micros() - g_pingAtUs) > ECHO_TIMEOUT_US) {
    pingStore(g_pingWho, -1);              /* nothing out there, or too far */
    pingNext();
  }
}

/* ---- PIR ----------------------------------------------------------------
 * Slow by nature, and slower still for the first minute after power-up while
 * the HC-SR501 settles. Poll it; there is nothing here worth an interrupt.
 * ------------------------------------------------------------------------ */
static void pirService(void)
{
  g_pir = (uint8_t)(digitalRead(PIN_PIR) ? 1 : 0);
}

/* ---- horn --------------------------------------------------------------- */

static void hornSet(bool on)
{
  if (on) g_hornAt = millis();             /* refresh the watchdog either way */
  if (on == g_horn) return;
  g_horn = on;

#if BUZZER_IS_ACTIVE
  digitalWrite(PIN_BUZZ, on ? HIGH : LOW);
#else
  if (on) {
    tone(PIN_BUZZ, HORN_HZ);
  } else {
    noTone(PIN_BUZZ);
    digitalWrite(PIN_BUZZ, LOW);           /* noTone() can leave it high */
  }
#endif
}

static void hornService(void)
{
  if (!g_horn) return;
  if ((millis() - g_hornAt) > HORN_TIMEOUT_MS) {
    hornSet(false);
    Serial.println(F("[safe] horn timed out - the VCU stopped asking"));
  }
}

/* ---- link --------------------------------------------------------------- */

static void handleLinkLine(const char *s)
{
  /* Anything at all arriving means the VCU is alive and the wiring works in
   * this direction. That is worth recording before we look at what it says. */
  g_vcuAt = millis();
  g_rxLines++;

  /* The hub tags relayed traffic "[SRC] text"; step over that if present. */
  if (*s == '[') {
    const char *close = strchr(s, ']');
    if (close != NULL) { s = close + 1; while (*s == ' ') s++; }
  }

  if ((s[0] == 'H' || s[0] == 'h') && s[1] == ',') {
    hornSet(s[2] != '0');
    return;
  }

  /* V,<vcu_uptime_ms> - the 1 Hz heartbeat. It exists so this node can tell
   * a live VCU from a dead one at idle; consumed silently, or it would fill
   * the console with one line a second. */
  if ((s[0] == 'V' || s[0] == 'v') && s[1] == ',') {
    return;
  }

  Serial.print(F("[link] "));
  Serial.println(s);
}

/* ---- diagnostics --------------------------------------------------------
 * Two questions this console can answer that nothing else can:
 *
 *   are the sensors reading?   the status line below
 *   is the VCU reaching me?    g_vcuUp, from the H, keep-alive
 *
 * The second one used to be unanswerable here. Ranges only travel outward, so
 * with the horn quiet this node heard nothing at all from a perfectly healthy
 * VCU - and a silent link looked identical to a working one.
 * ------------------------------------------------------------------------ */

static void diagService(void)
{
  bool up = (g_rxLines > 0) && ((millis() - g_vcuAt) < VCU_SILENT_MS);

  if (g_vcuUp != (int8_t)(up ? 1 : 0)) {
    g_vcuUp = up ? 1 : 0;
    if (up) {
      Serial.println(F("[link] VCU UP - lines arriving on D4"));
    } else if (g_rxLines > 0) {
      Serial.println(F("[link] VCU SILENT - nothing on D4 for 3 s"));
    } else {
      Serial.println(F("[link] no VCU yet. Check D4 <- MCX J2-2, and ground."));
    }
  }

  if ((millis() - g_statusAt) < STATUS_MS) return;
  g_statusAt = millis();

  Serial.print(F("[perc] front "));
  if (g_frontCm < 0) Serial.print(F("--")); else Serial.print(g_frontCm);
  Serial.print(F(" cm   back "));
  if (g_backCm < 0)  Serial.print(F("--")); else Serial.print(g_backCm);
  Serial.print(F(" cm   PIR "));
  Serial.print(g_pir ? F("MOTION") : F("still"));
  Serial.print(F("   horn "));
  Serial.println(g_horn ? F("ON") : F("off"));

  /* The link in numbers, because "SILENT" has three causes and they need
   * three different things done about them.
   *
   * BYTES are counted apart from LINES on purpose. A line only exists if the
   * byte stream was clean enough to contain a newline, so "no bytes at all"
   * is a dead wire and "bytes but no lines" is corruption - two faults with
   * nothing in common, and indistinguishable if you only ever count lines.
   * That is what this console could not tell you before. */
  uint32_t ints;
  noInterrupts();
  ints = s_echoInts;             /* 32-bit read the ISR could tear */
  interrupts();

  Serial.print(F("[link] rx "));
  Serial.print(g_rxBytes);
  Serial.print(F(" bytes / "));
  Serial.print(g_rxLines);
  Serial.print(F(" lines    tx "));
  Serial.print(g_txLines);
  Serial.print(F(" lines    echo int "));
  Serial.println(ints);

  if (g_rxBytes == 0UL) {
    Serial.println(F("       NOT ONE BYTE has reached D4 since power-up, so"));
    Serial.println(F("       nothing is arriving to be misread. In this order:"));
    Serial.println(F("         1. at the MCX console type 'mark1', then meter"));
    Serial.println(F("            MCX J2-2: ~1.8 V sending, 3.3 V idle, 0 V dead."));
    Serial.println(F("         2. continuity from MCX J2-2 to this board's D4."));
    Serial.println(F("         3. this GND to the MCX GND - the star point."));
    Serial.println(F("       'j' in link_debug.ino clears this board first."));
  } else if (g_rxLines == 0UL) {
    Serial.println(F("       Bytes ARE arriving and none has formed a line, so"));
    Serial.println(F("       this is corruption, not a broken wire: a baud that"));
    Serial.println(F("       is not 38400 at both ends, no common ground, or the"));
    Serial.println(F("       echo interrupts above."));
  } else if (g_vcuUp != 1) {
    Serial.print(F("       Heard "));
    Serial.print(g_rxLines);
    Serial.print(F(" lines and then nothing for "));
    Serial.print((millis() - g_vcuAt) / 1000UL);
    Serial.println(F(" s."));
    Serial.println(F("       It worked once, so suspect the far end or a joint"));
    Serial.println(F("       that moves - not the design of the link."));
  }

  /* One ping in flight makes two edges, so about 33 a second is normal and
   * ten thousand is a pin that nothing is driving. */
  if ((ints - g_echoPrev) > 20000UL) {
    Serial.println(F("       ECHO PINS ARE CHATTERING. Thousands of edges a"));
    Serial.println(F("       second with one ping in flight means D2/D3 are not"));
    Serial.println(F("       being driven: check both HC-SR04P have 5 V and a"));
    Serial.println(F("       ground at the star point."));
  }
  g_echoPrev = ints;
}

static void linkService(void)
{
  while (link.available()) {
    char c = (char)link.read();
    g_rxBytes++;               /* before any judgement about what it is */
    if (c == '\n' || c == '\r') {
      if (g_linkLen == 0) continue;
      g_linkLine[g_linkLen] = '\0';
      g_linkLen = 0;
      handleLinkLine(g_linkLine);
    } else if (g_linkLen < (LINE_MAX - 1)) {
      g_linkLine[g_linkLen++] = c;
    }
  }
}

static void reportService(void)
{
  if ((millis() - g_reportAt) < REPORT_MS) return;
  g_reportAt = millis();

  char out[LINE_MAX];
  snprintf(out, sizeof(out), "P,%d,%d,%u",
           g_frontCm, g_backCm, (unsigned)g_pir);
  link.println(out);
  g_txLines++;
}

/* ---- entry -------------------------------------------------------------- */

void setup(void)
{
  pinMode(PIN_TRIG_F, OUTPUT); digitalWrite(PIN_TRIG_F, LOW);
  pinMode(PIN_TRIG_B, OUTPUT); digitalWrite(PIN_TRIG_B, LOW);
  /* INPUT_PULLUP, not INPUT. An HC-SR04P drives ECHO push-pull, so when the
   * sensor is present and powered the pull-up changes nothing - it loses to
   * the driven level and costs 0.16 mA. When the sensor is absent, unpowered
   * or its ground is off, INPUT leaves the pin floating around the threshold
   * and it oscillates, firing echoEdge() thousands of times a second and
   * wrecking SoftwareSerial's bit timing on the link. Pulled up, that same
   * fault is silent and harmless: the pin sits high, no edge ever arrives,
   * the ping times out and the range reports -1, which is the truth. A dead
   * sensor should cost you that sensor, not the whole node. */
  pinMode(PIN_ECHO_F, INPUT_PULLUP);
  pinMode(PIN_ECHO_B, INPUT_PULLUP);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_BUZZ, OUTPUT);   digitalWrite(PIN_BUZZ, LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_ECHO_F), echoFrontISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO_B), echoBackISR,  CHANGE);

  Serial.begin(USB_BAUD);
  link.begin(LINK_BAUD);

  Serial.println(F("\nPERC node ready."));
  Serial.println(F("  link  D4/D5 @ 38400 -> MCX LPUART2"));
  Serial.println(F("  out   P,<front_cm>,<back_cm>,<pir>   10 Hz, -1 = no echo"));
  Serial.println(F("  in    H,<0|1>                        horn"));
  Serial.println(F("  front = D3/D7,   back = D2/D6"));
  Serial.println(F("  Give the PIR a full minute to settle before judging it."));
}

void loop(void)
{
  linkService();
  rangingService();
  pirService();
  hornService();
  reportService();
  diagService();
}
