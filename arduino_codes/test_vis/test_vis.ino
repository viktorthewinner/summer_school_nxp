/*
 * test_vis.ino  --  GW node (ESP32) bench self-test.
 *
 * Standalone. USB cable only, no MCX. The ESP32 has two jobs and this checks
 * both of them.
 *
 *   1  WiFi access point, so the laptop can reach the car
 *   2  I2C slave at 0x42, so the VCU can reach the laptop
 *
 * Serial Monitor at 115200.
 *
 * ---------------------------------------------------------------------------
 * PINS   (SCHEMA_WIRING.svg block B)
 *
 *   GPIO21 SDA   GPIO22 SCL   -> MCX P3_28 / P3_27, 3.3 V, 2k pull-ups at MCX
 *   VIN          9 V logic pack direct, 470 uF close by. Needs an AMS1117
 *                (SOT-223) on the board, and the 5 V fan pointed at it.
 *   GND          star point
 *
 * Four wires. This node has no sensors and no display: it is the route between
 * the laptop and the VCU, and nothing else. Roughly 20 GPIO are unused, so if
 * something later needs pins, this is the board with room.
 *
 * ---------------------------------------------------------------------------
 * IT IS A GATEWAY, NOT A CONTROLLER
 *
 * Everything arriving from the laptop is handed to the VCU as a request; the
 * VCU decides what to act on. That is what stops a dropped WiFi link being a
 * safety problem, and it is what makes the fault-injection demo worth showing.
 *
 * ---------------------------------------------------------------------------
 * COMMANDS
 *   w   WiFi status and client count
 *   i   I2C slave counters
 *   e   echo test: queue a line for the VCU's next read
 *   ?   help
 */

#include <WiFi.h>
#include <Wire.h>

/* ---- configuration ------------------------------------------------------ */

static const char *AP_SSID = "NXP-AV";
static const char *AP_PASS = "distributed";     /* >= 8 characters */

static const uint8_t PIN_SDA = 21, PIN_SCL = 22;
static const uint8_t GW_ADDR = 0x42;            /* must match LINK_GW_ADDR */
static const uint8_t GW_CHUNK = 48;             /* must match LINK_GW_CHUNK */

/* ---- state -------------------------------------------------------------- */

static volatile uint32_t g_i2cReads, g_i2cWrites;
static volatile char     g_pending[64];
static volatile uint8_t  g_pendingLen;
static char    g_line[32];
static uint8_t g_len;

/* ---- I2C slave, seen by the MCX -----------------------------------------
 * Both callbacks run in ISR context: buffer moves only, no Serial, no
 * allocation, nothing that can block.
 * ------------------------------------------------------------------------ */

static void onI2CReceive(int count)
{
  (void)count;
  g_i2cWrites++;
  while (Wire.available()) (void)Wire.read();
}

static void onI2CRequest(void)
{
  uint8_t buf[GW_CHUNK + 1];
  uint8_t n = 0;

  g_i2cReads++;

  while (n < GW_CHUNK && n < g_pendingLen) { buf[n + 1] = (uint8_t)g_pending[n]; n++; }
  buf[0] = n;
  while (n < GW_CHUNK) { buf[n + 1] = 0; n++; }

  g_pendingLen = 0;
  Wire.write(buf, GW_CHUNK + 1);   /* fixed length keeps the master simple */
}

/* ---- reports ------------------------------------------------------------ */

static void wifiReport(void)
{
  Serial.println(F("\n-- WiFi"));
  Serial.print(F("  SSID    ")); Serial.println(AP_SSID);
  Serial.print(F("  IP      ")); Serial.println(WiFi.softAPIP());
  Serial.print(F("  clients ")); Serial.println(WiFi.softAPgetStationNum());
  Serial.println(F("  Join the network, then browse to the IP above."));
}

static void i2cReport(void)
{
  Serial.println(F("\n-- I2C slave (the MCX is the master)"));
  Serial.print(F("  address 0x")); Serial.println(GW_ADDR, HEX);
  Serial.print(F("  reads   ")); Serial.println(g_i2cReads);
  Serial.print(F("  writes  ")); Serial.println(g_i2cWrites);

  if (g_i2cReads == 0 && g_i2cWrites == 0) {
    Serial.println(F("  Nothing yet. Normal until the MCX is powered and running;"));
    Serial.println(F("  it polls this node rather than the other way round."));
    Serial.println(F("  If it stays at zero with the MCX up, check the two I2C wires"));
    Serial.println(F("  and check the two 2k pull-ups to 3V3 are fitted - a floating"));
    Serial.println(F("  bus cannot ACK, which looks exactly like a dead slave."));
  } else if (g_i2cReads > 0) {
    Serial.println(F("  Reads climbing = the VCU is polling. The link works."));
  }
}

static void echoTest(void)
{
  const char *msg = "GW,hello from the ESP32";
  uint8_t n = 0;
  while (msg[n] != '\0' && n < sizeof(g_pending) - 1) { g_pending[n] = msg[n]; n++; }
  g_pendingLen = n;
  Serial.println(F("\n  queued. It leaves on the VCU's next poll and should appear"));
  Serial.println(F("  on the MCX console as [GW] GW,hello from the ESP32"));
}

static void help(void)
{
  Serial.println(F("\n  w  WiFi    i  I2C counters    e  queue a line for the VCU    ?  help\n"));
}

static void consoleService(void)
{
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_len == 0) continue;
      g_line[g_len] = '\0'; g_len = 0;
      switch (g_line[0]) {
        case 'w': wifiReport(); break;
        case 'i': i2cReport();  break;
        case 'e': echoTest();   break;
        default:  help();       break;
      }
    } else if (g_len < sizeof(g_line) - 1) {
      g_line[g_len++] = c;
    }
  }
}

/* ---- entry -------------------------------------------------------------- */

void setup(void)
{
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n====================================="));
  Serial.println(F(" GW self-test - ESP32 gateway"));
  Serial.println(F("====================================="));

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  wifiReport();

  Wire.begin((int)GW_ADDR, (int)PIN_SDA, (int)PIN_SCL, 400000);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  i2cReport();

  help();
}

void loop(void)
{
  consoleService();
}
