/*
 * esp32_gateway.ino - WiFi gateway for the distributed ECU vehicle.
 *
 * Bridges a laptop browser to the FRDM-MCXA153 hub:
 *
 *   laptop  <--WiFi/WebSocket-->  ESP32  <--I2C slave-->  FRDM-MCXA153
 *
 * The ESP32 is a GATEWAY, not a controller. Everything it receives from the
 * laptop is forwarded to the VCU as a request; the VCU decides what to act on.
 * That is what keeps a dropped WiFi link from being a safety problem.
 *
 * ---------------------------------------------------------------------------
 * WIRING - see WIRING_4_NODES.svg
 *
 *   ESP32 GPIO22 (SCL)  <->  MCX P3_27  (mikroBUS J6)
 *   ESP32 GPIO21 (SDA)  <->  MCX P3_28  (mikroBUS J6)
 *   ESP32 GND           <->  MCX GND    (J3 pin 12 or 14)
 *
 *   4.7 kOhm pull-ups on SDA and SCL to 3.3 V. NOT to 5 V - no ESP32 pin is
 *   5 V tolerant.
 *
 * ---------------------------------------------------------------------------
 * BOARD SETUP
 *
 *   Tools > Board            : ESP32 Dev Module
 *   Tools > Partition Scheme : default
 *   Library needed           : "WebSockets" by Markus Sattler (Library Manager)
 *
 * ---------------------------------------------------------------------------
 * USE
 *
 *   1. Upload, then open the Serial Monitor at 115200.
 *   2. On the laptop, join the WiFi network below.
 *   3. Browse to http://192.168.4.1
 *
 * The access point is deliberate: campus and corporate WiFi usually block
 * device-to-device traffic, which would kill this on demo day.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>

/* ---- configuration ------------------------------------------------------ */

static const char *AP_SSID = "NXP-AV";
static const char *AP_PASS = "distributed";   /* >= 8 characters */

static const uint8_t GW_I2C_ADDR = 0x42;      /* must match LINK_GW_ADDR      */
static const uint8_t PIN_SDA     = 21;
static const uint8_t PIN_SCL     = 22;

static const uint8_t  LINE_MAX  = 96;
static const uint8_t  GW_CHUNK  = 48;         /* must match LINK_GW_CHUNK     */
static const uint16_t QUEUE_MAX = 512;

/* ---- state -------------------------------------------------------------- */

WebServer       http(80);
WebSocketsServer ws(81);

/* Bytes waiting to be collected by the MCX on its next I2C read. */
static volatile char     toMcx[QUEUE_MAX];
static volatile uint16_t toMcxHead = 0;   /* written by loop()  */
static volatile uint16_t toMcxTail = 0;   /* written by ISR     */

/* Line being assembled from I2C writes by the MCX. */
static volatile char    fromMcx[LINE_MAX];
static volatile uint8_t fromMcxLen = 0;
static volatile bool    fromMcxReady = false;
static char             fromMcxLine[LINE_MAX];

/* ---- I2C slave callbacks ------------------------------------------------
 * These run in ISR context. Keep them to buffer moves only - no Serial, no
 * WebSocket, no allocation.
 * ------------------------------------------------------------------------ */

void onI2CReceive(int count)
{
  while (count-- > 0) {
    char c = (char)Wire.read();

    if (c == '\n' || c == '\r') {
      if (fromMcxLen > 0 && !fromMcxReady) {
        fromMcx[fromMcxLen] = '\0';
        fromMcxReady = true;
        fromMcxLen = 0;
      }
    } else if (fromMcxLen < LINE_MAX - 1) {
      fromMcx[fromMcxLen++] = c;
    }
  }
}

void onI2CRequest()
{
  uint8_t buf[GW_CHUNK + 1];
  uint8_t n = 0;

  while (n < GW_CHUNK && toMcxTail != toMcxHead) {
    buf[1 + n] = (uint8_t)toMcx[toMcxTail];
    toMcxTail = (uint16_t)((toMcxTail + 1) % QUEUE_MAX);
    n++;
  }

  buf[0] = n;                       /* 0 means "nothing pending" */
  Wire.write(buf, GW_CHUNK + 1);    /* fixed length keeps the master simple */
}

/* Queue one line for the MCX. Called from loop() context only. */
static void queueToMcx(const char *s)
{
  while (*s) {
    uint16_t next = (uint16_t)((toMcxHead + 1) % QUEUE_MAX);
    if (next == toMcxTail) return;           /* full - drop the rest */
    toMcx[toMcxHead] = *s++;
    toMcxHead = next;
  }
  uint16_t next = (uint16_t)((toMcxHead + 1) % QUEUE_MAX);
  if (next != toMcxTail) {
    toMcx[toMcxHead] = '\n';
    toMcxHead = next;
  }
}

/* ---- web page ----------------------------------------------------------- */

static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta charset="utf-8"><meta name=viewport content="width=device-width,initial-scale=1">
<title>NXP AV Gateway</title><style>
:root{--bg:#12161c;--fg:#e8edf4;--mut:#8b98a9;--acc:#4da3ff;--ok:#39c07a}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);
font:15px/1.5 system-ui,sans-serif;padding:16px;max-width:820px;margin:auto}
h1{font-size:19px;margin:8px 0 2px}p.s{color:var(--mut);margin:0 0 16px;font-size:13px}
#log{background:#0b0e13;border:1px solid #232b36;border-radius:8px;padding:10px;
height:300px;overflow:auto;font:13px/1.45 ui-monospace,monospace;white-space:pre-wrap}
form{display:flex;gap:8px;margin-top:12px}
input{flex:1;padding:10px;border-radius:8px;border:1px solid #2a3442;background:#0b0e13;color:var(--fg)}
button{padding:10px 16px;border:0;border-radius:8px;background:var(--acc);color:#04121f;font-weight:700}
#st{font-size:13px;color:var(--mut);margin-top:10px}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px;background:#c0392b}
.dot.on{background:var(--ok)}
</style></head><body>
<h1>Distributed ECU &mdash; gateway</h1>
<p class=s>ESP32 &rarr; I&sup2;C &rarr; FRDM-MCXA153 &rarr; Arduino 1 + Arduino 2</p>
<div id=log></div>
<form onsubmit="send();return false">
<input id=msg autocomplete=off placeholder="type a message and press Enter">
<button>Send</button></form>
<div id=st><span class=dot id=d></span><span id=stx>connecting...</span></div>
<script>
let ws,log=document.getElementById('log'),d=document.getElementById('d'),stx=document.getElementById('stx');
function add(t,c){let e=document.createElement('div');if(c)e.style.color=c;e.textContent=t;
log.appendChild(e);log.scrollTop=log.scrollHeight;}
function conn(){ws=new WebSocket('ws://'+location.hostname+':81/');
ws.onopen=()=>{d.className='dot on';stx.textContent='connected';add('-- connected --','#8b98a9');};
ws.onclose=()=>{d.className='dot';stx.textContent='disconnected - retrying';setTimeout(conn,1200);};
ws.onmessage=e=>add(e.data);}
function send(){let m=document.getElementById('msg');
if(ws&&ws.readyState==1&&m.value){ws.send(m.value);add('[you] '+m.value,'#4da3ff');m.value='';}}
conn();
</script></body></html>)HTML";

/* ---- WebSocket ---------------------------------------------------------- */

void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len)
{
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[ws] client %u connected\n", num);
      ws.sendTXT(num, "[GW] gateway ready");
      break;

    case WStype_DISCONNECTED:
      Serial.printf("[ws] client %u gone\n", num);
      break;

    case WStype_TEXT: {
      char line[LINE_MAX];
      size_t n = (len < LINE_MAX - 1) ? len : LINE_MAX - 1;
      memcpy(line, payload, n);
      line[n] = '\0';

      Serial.printf("[laptop -> MCX] %s\n", line);
      queueToMcx(line);            /* forwarded as a REQUEST, not a command */
      break;
    }

    default:
      break;
  }
}

/* ---- setup / loop ------------------------------------------------------- */

void setup()
{
  Serial.begin(115200);
  delay(200);

  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  Wire.begin(GW_I2C_ADDR, PIN_SDA, PIN_SCL, 400000);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  http.on("/", []() { http.send_P(200, "text/html", PAGE); });
  http.onNotFound([]() { http.send(404, "text/plain", "not found"); });
  http.begin();

  ws.begin();
  ws.onEvent(onWsEvent);

  Serial.println();
  Serial.println(F("============================================="));
  Serial.println(F(" ESP32 gateway  -  WiFi to FRDM-MCXA153"));
  Serial.println(F("============================================="));
  Serial.printf(" SSID    : %s\n", AP_SSID);
  Serial.printf(" pass    : %s\n", AP_PASS);
  Serial.print(  " open    : http://");
  Serial.println(WiFi.softAPIP());
  Serial.printf(" I2C     : slave 0x%02X on SDA=%u SCL=%u\n",
                GW_I2C_ADDR, PIN_SDA, PIN_SCL);
  Serial.println(F("---------------------------------------------"));
}

void loop()
{
  http.handleClient();
  ws.loop();

  /* A complete line arrived from the MCX - push it to every browser. */
  if (fromMcxReady) {
    noInterrupts();
    strncpy(fromMcxLine, (const char *)fromMcx, LINE_MAX - 1);
    fromMcxLine[LINE_MAX - 1] = '\0';
    fromMcxReady = false;
    interrupts();

    Serial.printf("[MCX -> laptop] %s\n", fromMcxLine);
    ws.broadcastTXT(fromMcxLine);
  }
}
