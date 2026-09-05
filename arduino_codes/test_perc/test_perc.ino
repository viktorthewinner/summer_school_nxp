/*
 * test_perc.ino  --  PERC (Arduino 1) bench self-test.
 *
 * Standalone. USB cable only, no MCX, no link. Tests every component on this
 * board, one command at a time, so a failure tells you which part is wrong.
 *
 * Serial Monitor at 115200, line ending Newline.
 *
 * ---------------------------------------------------------------------------
 * WHAT IS ON THIS BOARD   (SCHEMA_WIRING.svg block C)
 *
 *   D3  ECHO front  (INT1)      D7  TRIG front
 *   D2  ECHO back   (INT0)      D6  TRIG back
 *   D8  HC-SR501 PIR            D9  SFM-27 buzzer
 *
 *   Free: D10, D11, D12, D13, A0-A5.  Ten pins, including both I2C.
 *
 * ---------------------------------------------------------------------------
 * ONE FORWARD, ONE REARWARD
 *
 * D3/D7 looks where the car is going and is the only sensor allowed to override
 * the driver. D2/D6 looks behind and is a readout only - there is no reverse for
 * it to protect.
 *
 * They must fire ALTERNATELY. Two HC-SR04 pinging at the same moment hear each
 * other's burst and both report nonsense, and facing them apart does not fix it:
 * the burst travels through the chassis as well as the air. 60 ms apart, front
 * then back, gives each sensor 8.3 Hz.
 *
 * ---------------------------------------------------------------------------
 * COMMANDS
 *   u        stream both ranges, alternating
 *   p        PIR - report every change for 60 s
 *   b1       buzzer: steady DC          b2   buzzer: 2 kHz square wave
 *   x        stop whatever is streaming
 *   ?        help
 */

/* ---- pins --------------------------------------------------------------- */

static const uint8_t PIN_ECHO_F = 3;   /* INT1 - front */
static const uint8_t PIN_ECHO_B = 2;   /* INT0 - back  */
static const uint8_t PIN_TRIG_F = 7;
static const uint8_t PIN_TRIG_B = 6;
static const uint8_t PIN_PIR    = 8;
static const uint8_t PIN_BUZZ   = 9;

static const uint32_t USB_BAUD  = 115200;
static const uint8_t  LINE_MAX  = 32;

/* Cap the echo wait to the range we care about: 1.5 m is about 9 ms, against a
 * 38 ms full-scale default that would halve the update rate for nothing. */
static const unsigned long ECHO_TIMEOUT_US = 9000UL;

/* The HC-SR04 wants ~60 ms between pings so the previous burst has died away.
 * Alternating two sensors on that cadence is what stops them hearing each
 * other. */
static const uint16_t PING_GAP_MS = 60;

static char     g_line[LINE_MAX];
static uint8_t  g_len;
static char     g_stream;
static uint32_t g_streamAt, g_streamUntil;
static bool     g_pingFront;
static int      g_pirLast = -1;

/* ---- ultrasonic --------------------------------------------------------- */

/* Returns cm, or -1 if nothing answered inside the timeout. */
static int rangeCm(uint8_t trig, uint8_t echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long us = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (us == 0UL) return -1;
  return (int)(us / 58UL);            /* 343 m/s, out and back */
}

/* ---- buzzer -------------------------------------------------------------
 * SFM-27 is almost certainly a magnetic buzzer, but there are two kinds and
 * they need opposite drive. These two commands tell you which you have:
 *
 *   b1  steady DC     ACTIVE  buzzes at its own pitch     PASSIVE  one click
 *   b2  2 kHz square  ACTIVE  warbles or rattles          PASSIVE  clean tone
 *
 * If it draws more than about 20 mA it needs a transistor rather than a pin.
 * A 220 ohm in series costs volume but protects the pin either way, and is the
 * safe thing to fit until you have measured it.
 * ------------------------------------------------------------------------ */

static void buzzDC(uint16_t ms)
{
  noTone(PIN_BUZZ);
  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZ, LOW);
}

static void buzzTone(uint16_t hz, uint16_t ms)
{
  tone(PIN_BUZZ, hz, ms);
  delay(ms + 20);
  noTone(PIN_BUZZ);
  digitalWrite(PIN_BUZZ, LOW);
}

/* ---- console ------------------------------------------------------------ */

static void help(void)
{
  Serial.println(F(
    "\n  u    stream both ranges        p    PIR for 60 s"
    "\n  b1   buzzer, steady DC         b2   buzzer, 2 kHz square"
    "\n  x    stop streaming            ?    this help\n"));
}

static void handleLine(char *s)
{
  while (*s == ' ') s++;

  switch (*s) {
    case 'u':
      Serial.println(F("  ranges, alternating front / back - x to stop"));
      g_stream = 'u'; g_streamAt = 0; g_streamUntil = millis() + 60000UL;
      break;
    case 'p':
      Serial.println(F("  PIR - needs ~1 min to settle after power-up"));
      g_stream = 'p'; g_pirLast = -1; g_streamUntil = millis() + 60000UL;
      break;
    case 'b':
      if (s[1] == '2') {
        Serial.println(F("  2 kHz square. Clean tone = PASSIVE buzzer."));
        buzzTone(2000, 400);
      } else {
        Serial.println(F("  steady DC. Continuous buzz = ACTIVE buzzer, one click = PASSIVE."));
        buzzDC(400);
      }
      break;
    case 'x': g_stream = 0; Serial.println(F("  stopped")); break;
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
      if (g_len == 0) continue;
      g_line[g_len] = '\0'; g_len = 0;
      handleLine(g_line);
    } else if (g_len < (LINE_MAX - 1)) {
      g_line[g_len++] = c;
    }
  }
}

static void streamService(void)
{
  if (g_stream == 0) return;
  if ((long)(millis() - g_streamUntil) > 0) {
    g_stream = 0; Serial.println(F("  (stream ended)")); return;
  }

  if (g_stream == 'p') {                       /* PIR: only on change */
    int v = digitalRead(PIN_PIR);
    if (v != g_pirLast) {
      g_pirLast = v;
      Serial.print(F("  PIR ")); Serial.println(v ? F("MOTION") : F("clear"));
    }
    return;
  }

  if ((millis() - g_streamAt) < PING_GAP_MS) return;
  g_streamAt = millis();

  /* One sensor per slot, never both. */
  int cm = g_pingFront ? rangeCm(PIN_TRIG_F, PIN_ECHO_F)
                       : rangeCm(PIN_TRIG_B, PIN_ECHO_B);

  Serial.print(g_pingFront ? F("  front ") : F("  back  "));
  if (cm < 0) Serial.println(F("--   (nothing inside 1.5 m, or no echo at all)"));
  else       { Serial.print(cm); Serial.println(F(" cm")); }

  g_pingFront = !g_pingFront;
}

/* ---- entry -------------------------------------------------------------- */

void setup(void)
{
  pinMode(PIN_TRIG_F, OUTPUT); digitalWrite(PIN_TRIG_F, LOW);
  pinMode(PIN_TRIG_B, OUTPUT); digitalWrite(PIN_TRIG_B, LOW);
  pinMode(PIN_ECHO_F, INPUT);
  pinMode(PIN_ECHO_B, INPUT);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_BUZZ, OUTPUT);   digitalWrite(PIN_BUZZ, LOW);

  Serial.begin(USB_BAUD);
  Serial.println(F("\n====================================="));
  Serial.println(F(" PERC self-test - Arduino 1"));
  Serial.println(F("====================================="));
  Serial.println(F(" Two fixed HC-SR04P, splayed ~25 deg left and right."));
  Serial.println(F(" They fire alternately - together they hear each other."));
  Serial.println(F(" The PIR needs about a minute to settle after power-up."));
  help();
}

void loop(void)
{
  consoleService();
  streamService();
}
