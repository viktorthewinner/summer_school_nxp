/*
 * test_act.ino  --  ACT (Arduino 2) bench self-test.
 *
 * Standalone. USB cable only, no MCX, no link. Everything on this board, one
 * command at a time.
 *
 * Serial Monitor at 115200, line ending Newline.
 *
 *   >>> WHEELS OFF THE FLOOR. <<<
 *
 * If this is the very first power-up of the drive stage, run
 * act_motor_test.ino instead - it is the same motor code with nothing else in
 * the sketch, so a fault can only be the motors.
 *
 * ---------------------------------------------------------------------------
 * WHAT IS ON THIS BOARD   (SCHEMA_WIRING.svg blocks D and E)
 *
 *   D9  PWM left pair  (Q1, Q2 via 470R)   D10 PWM right pair (Q3, Q4)
 *   D6  LCD RS   D7 LCD E   D8/D11/D12/A2  LCD DB4..DB7
 *
 *   Free: D13, A0, A1, A3, A4, A5.
 *
 *   LCD RW to ground, V0 to the 10k trimpot wiper, backlight through 220R.
 *
 * ---------------------------------------------------------------------------
 * COMMANDS
 *   b 40 / l 40 / r 40   motors: both, left pair, right pair (0-70 %)
 *   x                    stop the motors
 *   t                    motor self test
 *   d <text>             write text to the LCD
 *   d                    LCD test pattern
 *   ?                    help
 */

#include <LiquidCrystal.h>

/* ---- pins --------------------------------------------------------------- */

static const uint8_t PIN_PWM_L  = 9;
static const uint8_t PIN_PWM_R  = 10;
/* LiquidCrystal(RS, E, DB4, DB5, DB6, DB7) */
LiquidCrystal lcd(6, 7, 8, 11, 12, A2);

static const uint16_t PWM_TOP   = 4000;   /* 2 kHz - the 1N4001 sets this */
static const uint8_t  DUTY_MAX  = 70;
static const uint8_t  SLEW_STEP = 4;
static const uint16_t SLEW_MS   = 20;
static const uint32_t USB_BAUD  = 115200;
static const uint8_t  LINE_MAX  = 40;

static uint8_t  g_targetL, g_targetR, g_dutyL, g_dutyR;
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
  digitalWrite(PIN_PWM_L, LOW);
  digitalWrite(PIN_PWM_R, LOW);
  pinMode(PIN_PWM_L, OUTPUT);
  pinMode(PIN_PWM_R, OUTPUT);

  TCCR1A = 0; TCCR1B = 0; OCR1A = 0; OCR1B = 0;
  ICR1   = PWM_TOP;
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);  /* phase correct, TOP=ICR1 */
  TCCR1B = _BV(WGM13)  | _BV(CS10);                 /* prescaler 1             */
}

static uint8_t clampDuty(long v)
{
  if (v < 0) return 0;
  if (v > (long)DUTY_MAX) return DUTY_MAX;
  return (uint8_t)v;
}

static void slewService(void)
{
  if ((millis() - g_slewAt) < SLEW_MS) return;
  g_slewAt = millis();
  bool ch = false;
  if (g_dutyL != g_targetL) {
    if (g_dutyL < g_targetL) g_dutyL = (g_targetL - g_dutyL > SLEW_STEP) ? g_dutyL + SLEW_STEP : g_targetL;
    else                     g_dutyL = (g_dutyL - g_targetL > SLEW_STEP) ? g_dutyL - SLEW_STEP : g_targetL;
    ch = true;
  }
  if (g_dutyR != g_targetR) {
    if (g_dutyR < g_targetR) g_dutyR = (g_targetR - g_dutyR > SLEW_STEP) ? g_dutyR + SLEW_STEP : g_targetR;
    else                     g_dutyR = (g_dutyR - g_targetR > SLEW_STEP) ? g_dutyR - SLEW_STEP : g_targetR;
    ch = true;
  }
  if (ch) pwmApply();
}

static void stopNow(void)
{
  g_targetL = g_targetR = g_dutyL = g_dutyR = 0;
  pwmApply();
}

static bool holdFor(uint16_t ms)
{
  uint32_t t0 = millis();
  while ((millis() - t0) < ms) {
    slewService();
    if (Serial.available()) { while (Serial.available()) Serial.read();
                              Serial.println(F("  ABORTED")); return false; }
  }
  return true;
}

static void motorTest(void)
{
  Serial.println(F("\n  motor test - any key aborts"));
  bool ok = true;
  struct { const __FlashStringHelper *t; uint8_t l, r; uint16_t ms; } seq[] = {
    { F("left pair 40 %"),  40,  0, 2500 },
    { F("stop"),             0,  0, 1000 },
    { F("right pair 40 %"),  0, 40, 2500 },
    { F("stop"),             0,  0, 1000 },
    { F("both 40 %"),       40, 40, 2500 },
  };
  for (uint8_t i = 0; i < 5 && ok; i++) {
    Serial.print(F("  ")); Serial.println(seq[i].t);
    g_targetL = seq[i].l; g_targetR = seq[i].r;
    ok = holdFor(seq[i].ms);
  }
  stopNow();
  Serial.println(ok ? F("  done") : F("  stopped"));
}

/* ---- LCD ---------------------------------------------------------------- */

static void lcdTest(void)
{
  Serial.println(F("  LCD: banner, then a counter for 3 s"));
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("ACT self-test"));
  lcd.setCursor(0, 1); lcd.print(F("1602A 4-bit OK"));
  delay(1500);

  for (int i = 0; i < 30; i++) {
    lcd.setCursor(0, 1);
    lcd.print(F("count "));
    lcd.print(i);
    lcd.print(F("   "));
    delay(100);
  }
  lcd.clear();
  lcd.print(F("ACT ready"));

  Serial.println(F("  Blank screen or a row of solid blocks means the contrast pot:"));
  Serial.println(F("  turn the 10k trimpot slowly through its whole range."));
}

/* ---- console ------------------------------------------------------------ */

static void help(void)
{
  Serial.println(F(
    "\n  b <0-70>  both      l <0-70>  left      r <0-70>  right"
    "\n  x         stop      t         motor self test"
    "\n  d <text>  to LCD    d         LCD test pattern"
    "\n  ?         help\n"));
}

static void handleLine(char *s)
{
  while (*s == ' ') s++;
  char c = *s;
  char *arg = s + 1;
  while (*arg == ' ') arg++;

  switch (c) {
    case 'b': g_targetL = g_targetR = clampDuty(atol(arg)); break;
    case 'l': g_targetL = clampDuty(atol(arg)); break;
    case 'r': g_targetR = clampDuty(atol(arg)); break;
    case 'x': stopNow(); Serial.println(F("  STOP")); break;
    case 't': motorTest(); break;
    case 'd': if (*arg) { lcd.clear(); lcd.print(arg); Serial.print(F("  LCD ")); Serial.println(arg); }
              else lcdTest();
              break;
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

/* ---- entry -------------------------------------------------------------- */

void setup(void)
{
  driveInit();
  stopNow();

  lcd.begin(16, 2);
  lcd.print(F("ACT self-test"));

  Serial.begin(USB_BAUD);
  Serial.println(F("\n====================================="));
  Serial.println(F(" ACT self-test - Arduino 2"));
  Serial.println(F("====================================="));
  Serial.println(F(" WHEELS OFF THE FLOOR."));
  Serial.println(F(" Motors whine at 2 kHz - that is the 1N4001 diodes, not a fault."));
  help();
}

void loop(void)
{
  consoleService();
  slewService();
}
