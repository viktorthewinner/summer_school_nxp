/*
 * act_motor_test.ino  --  MILESTONE 1: prove the car moves.
 *
 * Standalone. No VCU, no link, no sensors. Upload to ACT (Arduino 2), open the
 * Serial Monitor at 115200, and drive the wheels by hand.
 *
 *   >>> WHEELS OFF THE FLOOR until every command has done what it says. <<<
 *
 * ---------------------------------------------------------------------------
 * HARDWARE - see SCHEMA_WIRING.svg, blocks D and E
 *
 *   D9  --[470R]--+-- base of Q1        D10 --[470R]--+-- base of Q3
 *                 +--[10k]-- emitter                  +--[10k]-- emitter
 *       --[470R]--+-- base of Q2            --[470R]--+-- base of Q4
 *                 +--[10k]-- emitter                  +--[10k]-- emitter
 *
 *   Each collector -> motor -.  Motor + -> PACK +.  1N4001 across each motor,
 *   band (cathode) to PACK +.  All four emitters -> star ground.
 *
 * ---------------------------------------------------------------------------
 * WHY 2 kHz AND NOT 20 kHz
 *
 * The flyback diodes are 1N4001, a standard-recovery part: it keeps conducting
 * backwards for ~2 us after the transistor turns on. At 20 kHz that costs about
 * 96 mW in every transistor. At 2 kHz it is ~10 mW, and better still the motor
 * current has fully decayed during the off time (L/R ~ 330 us), so the diode is
 * already blocking by the time the transistor turns back on.
 *
 * You will hear a whine from the motors. That is the 2 kHz. It is not a fault.
 *
 * ---------------------------------------------------------------------------
 * WHAT PROTECTS THE TRANSISTORS
 *
 * Base current is fixed at (5 - 0.9) / 470 = 8.7 mA, so a PN2222A can pass at
 * most h_FE x 8.7 mA - roughly 435 mA with a typical part. It therefore
 * CURRENT-LIMITS on stall instead of passing the 800 mA the motor would take.
 * That is helpful, but it is not protection: limiting means it comes out of
 * saturation, and ~0.4 to 1.0 W in a TO-92 rated 0.625 W will cook it if you
 * hold it there. Never leave the car pushing against something.
 *
 * DUTY_MAX starts at 70 %. Leave it there until the wiring is proven.
 *
 * ---------------------------------------------------------------------------
 * COMMANDS  (type, press Enter)
 *
 *   b 40    both pairs to 40 %          x       stop
 *   l 40    left pair only              t       run the self test
 *   r 40    right pair only             ?       help
 */

/* ---- configuration ------------------------------------------------------ */

static const uint8_t  PIN_PWM_L = 9;      /* OC1A -> 470R -> Q1, Q2 bases    */
static const uint8_t  PIN_PWM_R = 10;     /* OC1B -> 470R -> Q3, Q4 bases    */

static const uint16_t PWM_TOP   = 4000;   /* ICR1: 16e6/(2*1*4000) = 2 kHz   */
static const uint8_t  DUTY_MAX  = 70;     /* %, cap until the wiring is proven */
static const uint8_t  SLEW_STEP = 4;      /* % per tick, softens the inrush  */
static const uint16_t SLEW_MS   = 20;

static const uint32_t USB_BAUD  = 115200;
static const uint8_t  LINE_MAX  = 32;

/* ---- state -------------------------------------------------------------- */

static uint8_t  g_targetL, g_targetR;     /* what you asked for  */
static uint8_t  g_dutyL,   g_dutyR;       /* what is applied now */
static uint32_t g_slewAt;

static char     g_line[LINE_MAX];
static uint8_t  g_len;

/* ---- drive -------------------------------------------------------------- */

static void pwmApply(void)
{
  OCR1A = (uint16_t)(((uint32_t)PWM_TOP * g_dutyL) / 100UL);
  OCR1B = (uint16_t)(((uint32_t)PWM_TOP * g_dutyR) / 100UL);
}

static void driveInit(void)
{
  /* Drive the pins low BEFORE making them outputs. The 10k base pull-downs
   * hold the transistors off until now; nothing should twitch on the way. */
  digitalWrite(PIN_PWM_L, LOW);
  digitalWrite(PIN_PWM_R, LOW);
  pinMode(PIN_PWM_L, OUTPUT);
  pinMode(PIN_PWM_R, OUTPUT);

  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A  = 0;
  OCR1B  = 0;
  ICR1   = PWM_TOP;

  /* Mode 10: phase-correct PWM, TOP = ICR1. Non-inverting on OC1A and OC1B,
   * prescaler 1. Phase correct rather than fast PWM because it keeps the
   * pulse centred, which matters once two channels switch together. */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13)  | _BV(CS10);
}

static uint8_t clampDuty(long v)
{
  if (v < 0)              return 0;
  if (v > (long)DUTY_MAX) return DUTY_MAX;
  return (uint8_t)v;
}

static void setTarget(long l, long r)
{
  g_targetL = clampDuty(l);
  g_targetR = clampDuty(r);
}

/* Move the applied duty toward the target a step at a time. A step straight to
 * 70 % is survivable but pointless; ramping keeps the inrush down. */
static void slewService(void)
{
  if ((millis() - g_slewAt) < SLEW_MS) return;
  g_slewAt = millis();

  bool changed = false;

  if (g_dutyL != g_targetL) {
    if (g_dutyL < g_targetL) g_dutyL = (g_targetL - g_dutyL > SLEW_STEP) ? g_dutyL + SLEW_STEP : g_targetL;
    else                     g_dutyL = (g_dutyL - g_targetL > SLEW_STEP) ? g_dutyL - SLEW_STEP : g_targetL;
    changed = true;
  }
  if (g_dutyR != g_targetR) {
    if (g_dutyR < g_targetR) g_dutyR = (g_targetR - g_dutyR > SLEW_STEP) ? g_dutyR + SLEW_STEP : g_targetR;
    else                     g_dutyR = (g_dutyR - g_targetR > SLEW_STEP) ? g_dutyR - SLEW_STEP : g_targetR;
    changed = true;
  }
  if (changed) pwmApply();
}

/* Immediate, no ramp. Used by the self test and by every stop. */
static void stopNow(void)
{
  g_targetL = g_targetR = 0;
  g_dutyL   = g_dutyR   = 0;
  pwmApply();
}

/* ---- self test ---------------------------------------------------------- */

/* Hold for ms, but bail out the moment anything arrives on USB. That keystroke
 * is your abort - it does not have to be a particular key. */
static bool holdFor(uint16_t ms)
{
  uint32_t t0 = millis();
  while ((millis() - t0) < ms) {
    slewService();
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println(F("  ABORTED"));
      return false;
    }
  }
  return true;
}

static bool step(const __FlashStringHelper *what, uint8_t l, uint8_t r, uint16_t ms)
{
  Serial.print(F("  ")); Serial.println(what);
  setTarget(l, r);
  return holdFor(ms);
}

static void selfTest(void)
{
  Serial.println(F("\nSELF TEST - press any key to abort"));

  bool ok =
    step(F("left pair  40 %  - only the LEFT wheels should turn"),  40,  0, 2500) &&
    step(F("stop"),                                                  0,  0, 1000) &&
    step(F("right pair 40 %  - only the RIGHT wheels should turn"),   0, 40, 2500) &&
    step(F("stop"),                                                  0,  0, 1000) &&
    step(F("both       40 %  - all four, same direction"),           40, 40, 2500) &&
    step(F("stop"),                                                  0,  0, 1000) &&
    step(F("both       65 %  - listen for the pitch rising"),        65, 65, 2500);

  stopNow();
  Serial.println(ok ? F("SELF TEST DONE\n") : F("SELF TEST STOPPED\n"));
}

/* ---- console ------------------------------------------------------------ */

static void help(void)
{
  Serial.println(F(
    "\n  b <0-70>   both pairs      l <0-70>   left pair"
    "\n  r <0-70>   right pair      x          stop"
    "\n  t          self test       ?          this help"
    "\n  duty is a percentage; below about 25 % the motors may only buzz.\n"));
}

static void report(void)
{
  Serial.print(F("  L=")); Serial.print(g_targetL);
  Serial.print(F("%  R=")); Serial.print(g_targetR);
  Serial.println(F("%"));
}

static void handleLine(char *s)
{
  while (*s == ' ') s++;
  char c = *s;
  long v = atol(s + 1);

  switch (c) {
    case 'b': setTarget(v, v); report(); break;
    case 'l': setTarget(v, g_targetR); report(); break;
    case 'r': setTarget(g_targetL, v); report(); break;
    case 'x':
    case 's': stopNow(); Serial.println(F("  STOP")); break;
    case 't': selfTest(); break;
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
      g_line[g_len] = '\0';
      g_len = 0;
      handleLine(g_line);
    } else if (g_len < (LINE_MAX - 1)) {
      g_line[g_len++] = c;
    }
  }
}

/* ---- entry -------------------------------------------------------------- */

void setup(void)
{
  driveInit();
  stopNow();

  Serial.begin(USB_BAUD);
  Serial.println(F("\n==========================================="));
  Serial.println(F(" ACT motor test - 4 x PN2222A, 2 kHz PWM"));
  Serial.println(F("==========================================="));
  Serial.println(F(" WHEELS OFF THE FLOOR."));
  Serial.println(F(" D9 = left pair (Q1,Q2)   D10 = right pair (Q3,Q4)"));
  help();
}

void loop(void)
{
  consoleService();
  slewService();
}
