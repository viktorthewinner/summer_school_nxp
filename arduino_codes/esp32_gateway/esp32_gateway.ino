/*
 * esp32_gateway.ino - WiFi gateway for the distributed ECU vehicle.
 *
 * Bridges a laptop browser to the FRDM-MCXA153 hub:
 *
 *   laptop  <--WiFi/WebSocket-->  ESP32  <--I2C slave-->  FRDM-MCXA153
 *
 * The ESP32 is a GATEWAY, not a controller. It moves bytes and nothing else.
 * The page below sends KEY STATE - which of W A S D and space are down right
 * now - and the VCU turns that into motor duty, applies the throttle ramp,
 * the steering mix, the duty cap and the obstacle veto, and runs the
 * watchdogs. Nothing on this node decides anything, which is exactly what
 * keeps a dropped WiFi link from being a safety problem: the worst it can do
 * is stop sending keys, and both nodes downstream time out on their own.
 *
 * ---------------------------------------------------------------------------
 * PROTOCOL  (must match distributed_ecu/led_blinky.c)
 *
 *   browser -> VCU    K,<w>,<a>,<s>,<d>,<horn>   10 Hz plus on every change
 *                     M,<text>                   message for the ACT display
 *                     E                          stop and release
 *
 *   VCU -> browser    T,<16 fields>              telemetry at 10 Hz, parsed
 *                                                by tel() below
 *                     anything else              log line, shown as text
 *
 * ---------------------------------------------------------------------------
 * WIRING - see SCHEMA_WIRING.svg, blocks B and F
 *
 *   ESP32 GPIO22 (SCL)  <->  MCX J5 pin 5, SCL (P3_27, mikroBUS)
 *   ESP32 GPIO21 (SDA)  <->  MCX J5 pin 6, SDA (P3_28, mikroBUS)
 *                              J5, NOT J6 - J6 is the SPI half of the socket
 *                              and its pins 5/6 are P1_2/P1_0. Go by the
 *                              silkscreen, which prints SCL and SDA.
 *                              On a 30-pin DevKit D21 and D22 are NOT next
 *                              to each other - TX0 and RX0 sit between them,
 *                              which is how a board swap ends up with the two
 *                              wires reversed. The boot banner's "verdict"
 *                              line catches that; 'p' on the console proves it.
 *   470 Ohm IN SERIES with each of those two wires, at the ESP32 end. They
 *                              limit the current the MCX's 2k pull-ups push
 *                              into this chip's pins whenever it is unpowered
 *                              and the MCX is not - which is what latches up
 *                              an ESP32 and killed two of them here. 400 kHz
 *                              passes through 470 Ohm without trouble.
 *   PLUG ORDER: I2C wires off BEFORE either board's USB comes out, USB back in
 *                              BEFORE the wires go back on. On the car both
 *                              boards share one pack and come up together.
 *   ESP32 GND           <->  MCX GND    (J3 pin 12 or 14), star point
 *   ESP32 VIN           <-   9 V logic pack DIRECT, 470 uF close by.
 *                              Board must have an AMS1117 (SOT-223), not a
 *                              SOT-23-5 LDO - those stop at ~6 V. Put the
 *                              5 V fan on the regulator: 0.68 W at 9 V in.
 *
 *   FIT THE I2C PULL-UPS: 2 kOhm from SDA to 3V3 and 2 kOhm from SCL to 3V3,
 *   at the MCX end (J3-8). They are not optional and they are not on any
 *   board here.
 *
 *   This schema used to borrow the 2.2 kOhm on the MPU6050 module instead,
 *   which made a sensor breakout a hard dependency for this WiFi link: with
 *   the IMU unplugged, or one of its wires off, SDA and SCL float, nothing
 *   can ACK, and this node goes silent for a reason it cannot report.
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
 *   3. Browse to http://192.168.4.1 and click the page once so it has focus.
 *
 * The access point is deliberate: campus and corporate WiFi usually block
 * device-to-device traffic, which would kill this on demo day. It also means
 * the address is fixed, and that this node depends on nothing but itself.
 *
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

static const uint8_t  GW_LINE_MAX  = 128;

/* GW_CHUNK must match LINK_GW_CHUNK in distributed_ecu/link.c. */
static const uint8_t  GW_CHUNK  = 48;
static const uint16_t QUEUE_MAX = 512;

/* Lines held between the I2C callback and loop(). Telemetry arrives at 10 Hz
 * and a log line can land in the same millisecond, so a single "one line
 * waiting" slot drops traffic - this is a ring instead. */
static const uint8_t  RX_LINES  = 8;

/* ---- state -------------------------------------------------------------- */

WebServer        http(80);
WebSocketsServer ws(81);

/* Bytes waiting to be collected by the MCX on its next I2C read. */
static volatile char     toMcx[QUEUE_MAX];
static volatile uint16_t toMcxHead = 0;   /* written by loop()  */
static volatile uint16_t toMcxTail = 0;   /* written by ISR     */

/* Lines arriving from the MCX. rxHead is written by the I2C callback only,
 * rxTail by loop() only, so no lock is needed. */
static volatile char     rxRing[RX_LINES][GW_LINE_MAX];
static volatile uint8_t  rxHead = 0;
static volatile uint8_t  rxTail = 0;
static volatile char     rxAsm[GW_LINE_MAX];
static volatile uint8_t  rxAsmLen = 0;
static volatile uint32_t rxDropped = 0;

static uint32_t i2cReads = 0, i2cWrites = 0;
static char     line[GW_LINE_MAX];

/* ---- I2C slave callbacks ------------------------------------------------
 * These run in callback context. Keep them to buffer moves only - no Serial,
 * no WebSocket, no allocation.
 * ------------------------------------------------------------------------ */

static void rxPush(void)
{
  uint8_t next = (uint8_t)((rxHead + 1u) % RX_LINES);

  if (next == rxTail) {          /* loop() is behind - drop the oldest line */
    rxTail = (uint8_t)((rxTail + 1u) % RX_LINES);
    rxDropped++;
  }

  for (uint8_t i = 0; i < rxAsmLen; i++) rxRing[rxHead][i] = rxAsm[i];
  rxRing[rxHead][rxAsmLen] = '\0';
  rxHead = next;
}

void onI2CReceive(int count)
{
  i2cWrites++;

  while (count-- > 0) {
    char c = (char)Wire.read();

    if (c == '\n' || c == '\r') {
      if (rxAsmLen > 0) {
        rxPush();
        rxAsmLen = 0;
      }
    } else if (rxAsmLen < GW_LINE_MAX - 1) {
      rxAsm[rxAsmLen++] = c;
    }
  }
}

void onI2CRequest()
{
  uint8_t buf[GW_CHUNK + 1];
  uint8_t n = 0;

  i2cReads++;

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
<title>NXP AV &mdash; teleop</title><style>
:root{--bg:#0e1218;--pan:#151b24;--line:#232c39;--fg:#e8edf4;--mut:#8b98a9;
--acc:#4da3ff;--ok:#39c07a;--warn:#e8a33d;--bad:#e0524a}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
font:14px/1.5 system-ui,-apple-system,Segoe UI,sans-serif;padding:14px;
max-width:1080px;margin:auto;-webkit-user-select:none;user-select:none}
h1{font-size:18px;margin:4px 0 2px;letter-spacing:.2px}
.sub{color:var(--mut);font-size:12px;margin:0 0 14px}
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.card{background:var(--pan);border:1px solid var(--line);border-radius:10px;padding:12px 14px}
.card h2{font-size:11px;letter-spacing:.10em;text-transform:uppercase;
color:var(--mut);margin:0 0 10px;font-weight:600}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;
margin-right:7px;background:var(--bad);vertical-align:1px}
.dot.on{background:var(--ok)}
.dot.warn{background:var(--warn)}

/* state banner */
#state{display:flex;align-items:center;gap:12px;margin-bottom:12px;
padding:10px 14px;border-radius:10px;background:var(--pan);
border:1px solid var(--line);font-weight:600}
#state.live{border-color:#1d6b46;background:#10231a}
#state.stop{border-color:#6b2420;background:#241211}
#state .why{font-weight:400;color:var(--mut);font-size:12px;margin-left:auto;text-align:right}

/* keypad */
.pad{display:grid;grid-template-columns:repeat(3,52px);gap:6px;justify-content:center}
.key{height:46px;display:flex;align-items:center;justify-content:center;
border:1px solid var(--line);border-radius:8px;background:#0d1219;
font-weight:700;font-size:15px;color:var(--mut);transition:.06s}
.key.wide{grid-column:1/4;height:34px;font-size:12px;letter-spacing:.12em}
.key.hit{background:var(--acc);border-color:var(--acc);color:#04121f}
.key.hit.brake{background:var(--bad);border-color:var(--bad);color:#fff}
.key.hit.horn{background:var(--warn);border-color:var(--warn);color:#1a1204}
.pad .sp{visibility:hidden}
.hint{color:var(--mut);font-size:11px;text-align:center;margin-top:9px}

/* rows and bars */
.row{display:flex;align-items:center;gap:10px;margin:7px 0;font-size:13px}
.row .lb{width:74px;color:var(--mut);flex:none}
.row .vl{width:88px;text-align:right;font-variant-numeric:tabular-nums;flex:none}
.bar{flex:1;height:9px;background:#0a0e14;border-radius:5px;overflow:hidden}
.bar i{display:block;height:100%;width:0;background:var(--acc);
border-radius:5px;transition:width .1s linear}
.bar i.ok{background:var(--ok)}.bar i.warn{background:var(--warn)}
.bar i.bad{background:var(--bad)}

table.kv{width:100%;border-collapse:collapse;font-size:13px}
table.kv td{padding:3px 0}
table.kv td:first-child{color:var(--mut)}
table.kv td:last-child{text-align:right;font-variant-numeric:tabular-nums}

form{display:flex;gap:8px}
input{flex:1;min-width:0;padding:9px 10px;border-radius:8px;
border:1px solid var(--line);background:#0a0e14;color:var(--fg);font:inherit}
button{padding:9px 15px;border:0;border-radius:8px;background:var(--acc);
color:#04121f;font-weight:700;font:inherit;cursor:pointer}
button.gh{background:#222c3a;color:var(--fg)}
#log{background:#0a0e14;border:1px solid var(--line);border-radius:8px;
padding:9px;height:150px;overflow:auto;
font:12px/1.45 ui-monospace,SFMono-Regular,Consolas,monospace;white-space:pre-wrap}
.foot{color:var(--mut);font-size:11px;margin-top:12px}
</style></head><body>

<h1>Distributed ECU &mdash; teleop</h1>
<p class=sub>browser &rarr; ESP32 &rarr; I&sup2;C &rarr; <b>FRDM-MCXA153</b> &rarr; PERC + ACT.
The keys go to the MCX; the MCX decides what the wheels do.</p>

<div id=state><span class=dot id=dot></span><span id=stx>connecting&hellip;</span>
<span class=why id=why></span></div>

<div class=grid>

  <div class=card>
    <h2>Control</h2>
    <div class=pad>
      <div class="key sp"></div><div class=key id=kW>W</div><div class="key sp"></div>
      <div class=key id=kA>A</div><div class=key id=kS>S</div><div class=key id=kD>D</div>
      <div class="key wide" id=kH>SPACE &mdash; HORN</div>
    </div>
    <div class=hint>W throttle &middot; A/D steer &middot; S brake (no reverse) &middot; SPACE horn</div>
    <div class=hint>Click the page once so it has keyboard focus.</div>
  </div>

  <div class=card>
    <h2>Drive</h2>
    <div class=row><span class=lb>throttle</span><span class=bar><i id=bT></i></span><span class=vl id=vT>0 %</span></div>
    <div class=row><span class=lb>left</span><span class=bar><i id=bL class=ok></i></span><span class=vl id=vL>0 %</span></div>
    <div class=row><span class=lb>right</span><span class=bar><i id=bR class=ok></i></span><span class=vl id=vR>0 %</span></div>
    <div class=row><span class=lb>horn</span><span class=vl id=vH style="width:auto">off</span></div>
  </div>

  <div class=card>
    <h2>Range &mdash; HC-SR04P</h2>
    <div class=row><span class=lb>front</span><span class=bar><i id=bF></i></span><span class=vl id=vF>&ndash;</span></div>
    <div class=row><span class=lb>back</span><span class=bar><i id=bB></i></span><span class=vl id=vB>&ndash;</span></div>
    <div class=hint id=gtx>front stops forward drive under 25 cm</div>
  </div>

  <div class=card>
    <h2>Motion &amp; presence</h2>
    <table class=kv>
      <tr><td>PIR (HC-SR501)</td><td id=vP>&ndash;</td></tr>
      <tr><td>accel X / Y / Z</td><td id=vA>&ndash;</td></tr>
      <tr><td>tilt Z</td><td id=vG>&ndash;</td></tr>
      <tr><td>yaw rate</td><td id=vY>&ndash;</td></tr>
    </table>
  </div>

  <div class=card>
    <h2>Links</h2>
    <table class=kv>
      <tr><td>browser &rarr; ESP32</td><td id=lW>&ndash;</td></tr>
      <tr><td>VCU &rarr; browser</td><td id=lT>&ndash;</td></tr>
      <tr><td>ACT (Arduino 2)</td><td id=lA>&ndash;</td></tr>
      <tr><td>PERC (Arduino 1)</td><td id=lP>&ndash;</td></tr>
    </table>
  </div>

  <div class=card>
    <h2>Display &mdash; 1602A on ACT</h2>
    <form onsubmit="sendMsg();return false">
      <input id=msg maxlength=40 autocomplete=off placeholder="text for the LCD">
      <button>Send</button>
    </form>
    <div class=hint>Over 16 characters and it scrolls. Row 2 stays live drive state.</div>
    <div style="margin-top:9px"><button class=gh type=button onclick="estop()">STOP &mdash; release the wheels</button></div>
  </div>

</div>

<div class=card style="margin-top:12px"><h2>Log</h2><div id=log></div></div>
<p class=foot>Key state leaves this page at 10 Hz. Lose focus, close the tab or drop
WiFi and the VCU releases after 400 ms, then ACT cuts the motors after another 300 ms.</p>

<script>
var $=function(i){return document.getElementById(i)};
var ws,keys={w:0,a:0,s:0,d:0,h:0},lastT=0,sentAt=0;

function log(t,c){var e=document.createElement('div');if(c)e.style.color=c;
e.textContent=t;var l=$('log');l.appendChild(e);
while(l.childNodes.length>200)l.removeChild(l.firstChild);l.scrollTop=l.scrollHeight}

/* ---- keys -------------------------------------------------------------- */
var MAP={KeyW:'w',KeyA:'a',KeyS:'s',KeyD:'d',Space:'h',
         ArrowUp:'w',ArrowLeft:'a',ArrowDown:'s',ArrowRight:'d'};

function paint(){
  $('kW').className='key'+(keys.w?' hit':'');
  $('kA').className='key'+(keys.a?' hit':'');
  $('kD').className='key'+(keys.d?' hit':'');
  $('kS').className='key'+(keys.s?' hit brake':'');
  $('kH').className='key wide'+(keys.h?' hit horn':'');
}
function sendKeys(){
  if(!ws||ws.readyState!=1)return;
  ws.send('K,'+keys.w+','+keys.a+','+keys.s+','+keys.d+','+keys.h);
  sentAt=Date.now();
}
function setKey(k,v){
  if(keys[k]===v)return;
  keys[k]=v;paint();sendKeys();          /* on change, not only on the tick */
}
function clearKeys(){
  keys={w:0,a:0,s:0,d:0,h:0};paint();sendKeys();
}
addEventListener('keydown',function(e){
  /* An input box has focus: let the user type, and do not drive. */
  if(e.target&&e.target.tagName=='INPUT')return;
  var k=MAP[e.code];if(!k)return;
  e.preventDefault();                    /* space scrolls the page otherwise */
  if(!e.repeat)setKey(k,1);
});
addEventListener('keyup',function(e){
  var k=MAP[e.code];if(!k)return;e.preventDefault();setKey(k,0);
});
/* Every way of leaving the page clears the keys. A tab switched away with W
   still down is the one failure mode this page can actually cause. */
addEventListener('blur',clearKeys);
document.addEventListener('visibilitychange',function(){
  if(document.hidden)clearKeys();
});
function estop(){keys={w:0,a:0,s:0,d:0,h:0};paint();
  if(ws&&ws.readyState==1){ws.send('E');sendKeys()}log('-- STOP sent --','#e0524a')}

function sendMsg(){
  var m=$('msg');
  if(ws&&ws.readyState==1&&m.value){ws.send('M,'+m.value);
    log('[lcd] '+m.value,'#4da3ff');m.value='';m.blur()}
}

/* ---- telemetry --------------------------------------------------------- */
function bar(el,pct,cls){el.style.width=Math.max(0,Math.min(100,pct))+'%';
  el.className=cls||''}

function range(bEl,vEl,cm){
  if(cm<0){bar(bEl,100,'ok');vEl.textContent='clear';return}
  var pct=Math.min(100,cm/1.5);                 /* 150 cm is a full bar */
  bar(bEl,pct,cm<25?'bad':cm<50?'warn':'ok');
  vEl.textContent=cm+' cm';
}
function age(el,ms){
  if(ms>=9999){el.textContent='silent';el.style.color='#e0524a';return}
  el.textContent=ms+' ms';
  el.style.color=ms<500?'#39c07a':'#e8a33d';
}

function tel(f){
  lastT=Date.now();
  var live=f[1]=='1',L=+f[2],R=+f[3],thr=+f[4];
  var fr=+f[5],bk=+f[6],pir=f[7]=='1',horn=f[8]=='1',guard=f[9]=='1';
  var ax=+f[10],ay=+f[11],az=+f[12],gz=+f[13];

  var st=$('state');
  st.className=guard?'stop':live?'live':'';
  $('stx').textContent=guard?'BLOCKED - obstacle ahead'
                      :live?'LIVE - the VCU has your keys':'connected, released';
  $('why').textContent=guard?'pivot with A or D to turn away'
                      :live?'':'press a key to take the wheels';

  bar($('bT'),thr,'');bar($('bL'),L,'ok');bar($('bR'),R,'ok');
  $('vT').textContent=thr+' %';$('vL').textContent=L+' %';$('vR').textContent=R+' %';
  $('vH').textContent=horn?'SOUNDING':'off';
  $('vH').style.color=horn?'#e8a33d':'';

  range($('bF'),$('vF'),fr);range($('bB'),$('vB'),bk);
  $('gtx').textContent=guard?'guard ACTIVE - forward drive refused'
                            :'front stops forward drive under 25 cm';

  $('vP').textContent=pir?'MOTION':'still';
  $('vP').style.color=pir?'#e8a33d':'';

  if(ax==0&&ay==0&&az==0&&gz==0){
    $('vA').textContent='no MPU6050';$('vG').textContent='-';$('vY').textContent='-';
  }else{
    /* MPU6050 defaults: accel +-2 g = 16384/g, gyro +-250 dps = 131/dps */
    $('vA').textContent=ax+' / '+ay+' / '+az;
    $('vG').textContent=(az/16384).toFixed(2)+' g';
    $('vY').textContent=(gz/131).toFixed(1)+' °/s';
  }
  age($('lA'),+f[14]);age($('lP'),+f[15]);
}

/* ---- socket ------------------------------------------------------------ */
function conn(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=function(){$('dot').className='dot on';$('stx').textContent='connected';
    log('-- connected --','#8b98a9');clearKeys()};
  ws.onclose=function(){$('dot').className='dot';
    $('state').className='';$('stx').textContent='disconnected - retrying';
    $('why').textContent='';keys={w:0,a:0,s:0,d:0,h:0};paint();
    setTimeout(conn,1200)};
  ws.onmessage=function(e){
    var d=e.data;
    if(d.charCodeAt(0)==84&&d.charCodeAt(1)==44){tel(d.split(','));return}
    log(d);
  };
}

setInterval(function(){
  sendKeys();                                   /* 10 Hz keepalive */
  var t=Date.now();
  $('lW').textContent=(ws&&ws.readyState==1)?'up':'down';
  $('lW').style.color=(ws&&ws.readyState==1)?'#39c07a':'#e0524a';
  var d=lastT?(t-lastT):9999;
  $('lT').textContent=lastT?(d+' ms'):'never';
  $('lT').style.color=d<600?'#39c07a':'#e0524a';

  /* Everything below the VCU row is only ever written by tel(). Until the
     VCU has spoken once, those rows are not "unknown" - they are unasked,
     and a bare dash reads as if the Arduinos were the thing at fault. Say
     what is actually true instead. */
  if(!lastT){
    ['lA','lP'].forEach(function(i){
      $(i).textContent='no telemetry';$(i).style.color='#8b98a9'});
    $('stx').textContent='no telemetry from the VCU';
    $('why').textContent='check the MCX console and the ESP32 i2c counters';
  }
},100);

paint();conn();
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
      char buf[GW_LINE_MAX];
      size_t n = (len < GW_LINE_MAX - 1) ? len : GW_LINE_MAX - 1;
      memcpy(buf, payload, n);
      buf[n] = '\0';

      /* Key lines arrive ten times a second. Printing them would make this
       * monitor useless and slow the loop down for no reason. */
      if (buf[0] != 'K') Serial.printf("[laptop -> MCX] %s\n", buf);

      queueToMcx(buf);             /* forwarded as a REQUEST, not a command */
      break;
    }

    default:
      break;
  }
}

/* ---- serial console -----------------------------------------------------
 * Two commands, and 'i' is the one that matters. The MCX is the I2C master
 * here, so this node never speaks first: READS CLIMBING MEANS THE VCU IS
 * POLLING AND THE LINK WORKS. Reads stuck at zero with the MCX running is a
 * wiring fault. Check the two 2 kOhm pull-ups to 3V3 are fitted and that the
 * bus idles at 3.3 V: a floating bus cannot ACK, and that is indistinguishable
 * from this node being dead.
 * ------------------------------------------------------------------------ */

static void consoleService()
{
  if (!Serial.available()) return;
  char c = (char)Serial.read();

  if (c == 'i') {
    Serial.printf("[i2c] reads %lu  writes %lu  queued %u  lines dropped %lu\n",
                  (unsigned long)i2cReads, (unsigned long)i2cWrites,
                  (unsigned)((toMcxHead - toMcxTail + QUEUE_MAX) % QUEUE_MAX),
                  (unsigned long)rxDropped);
  } else if (c == 'w') {
    Serial.printf("[wifi] SSID %s  clients %d  http://%s\n",
                  AP_SSID, WiFi.softAPgetStationNum(),
                  WiFi.softAPIP().toString().c_str());
  } else if (c == 'p') {
    /* Tug test. Hold SDA low for three seconds. If this wire is really on
     * the MCX's bus, the MCX console shows every I2C transfer failing with
     * BUSY or PIN LOW for those three seconds and the IMU scan finds nothing.
     * If the MCX does not notice, this pin is not on that bus, whatever the
     * pull-up says. */
    Serial.println(F("[bus] tug test - watch the MCX [diag] line while this runs"));
    Wire.end();
    Serial.println(F("[bus] 1/2  SDA held LOW for 3 s  -> MCX should say BUSY"));
    pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_SDA, LOW);
    delay(3000);
    digitalWrite(PIN_SDA, HIGH);
    pinMode(PIN_SDA, INPUT);
    Serial.println(F("[bus]      SDA released, 3 s pause"));
    delay(3000);
    Serial.println(F("[bus] 2/2  SCL held LOW for 3 s  -> MCX should say BUSY too"));
    Serial.println(F("           (the LPI2C calls either line low a busy bus)"));
    pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_SCL, LOW);
    delay(3000);
    digitalWrite(PIN_SCL, HIGH);
    pinMode(PIN_SCL, INPUT);
    Serial.println(F("[bus]      SCL released. A wire the MCX never noticed is"));
    Serial.println(F("           not on its bus. Restarting so the slave comes"));
    Serial.println(F("           back clean."));
    Serial.flush();
    delay(100);
    ESP.restart();
  } else if (c == '?') {
    Serial.println(F("  i  I2C counters - reads climbing = the VCU is polling"));
    Serial.println(F("  w  WiFi status and connected clients"));
    Serial.println(F("  p  tug test - hold SDA low 3 s, the MCX must complain"));
  }
}

/* ---- link diagnostics ---------------------------------------------------
 * The single most useful fact this node holds is whether the read counter is
 * moving. The MCX is the I2C master, so this node never speaks first: reads
 * climbing means the VCU is alive, running the right firmware, and the two
 * I2C wires are good. Reads frozen means it is not, and no amount of staring
 * at the browser will tell you that.
 *
 * Printed on a change, and summarised every few seconds, so you do not have
 * to know to type 'i'.
 * ------------------------------------------------------------------------ */

static const uint32_t DIAG_MS = 3000;

static uint32_t diagAt, lastReads;
static int      mcxUp = -1;

static void diagService()
{
  if (millis() - diagAt < DIAG_MS) return;
  diagAt = millis();

  bool up = (i2cReads != lastReads);

  if (mcxUp != (int)up) {
    mcxUp = up;
    if (up) {
      Serial.println(F("[i2c] VCU IS POLLING - link up"));
    } else {
      Serial.println(F("[i2c] VCU NOT POLLING. Either it is not running this"));
      Serial.println(F("      firmware, or SDA/SCL are wrong - and remember the"));
      Serial.println(F("      bus needs its 2k pull-ups to 3V3 fitted - a"));
      Serial.println(F("      floating bus cannot ACK and looks just like this."));
    }
  }

  Serial.printf("[gw] reads %lu (+%lu)  writes %lu  queued %u  dropped %lu  ws %u\n",
                (unsigned long)i2cReads,
                (unsigned long)(i2cReads - lastReads),
                (unsigned long)i2cWrites,
                (unsigned)((toMcxHead - toMcxTail + QUEUE_MAX) % QUEUE_MAX),
                (unsigned long)rxDropped,
                ws.connectedClients());

  lastReads = i2cReads;
}

/* ---- boot-time wiring test ----------------------------------------------
 * Runs once, before the I2C peripheral owns the pins. Counts falling edges
 * on each pin for WIRE_TEST_MS while the MCX is (presumably) already
 * polling the bus - it reads the IMU at 10 Hz and this node several times a
 * second, so a correctly wired SCL sees well over a thousand edges a second
 * and SDA a few hundred. The four possible answers each name one fault.
 * ------------------------------------------------------------------------ */

static const uint32_t WIRE_TEST_MS = 1000;   /* count window, after the first edge */
static const uint32_t WIRE_WAIT_MS = 5000;   /* how long to wait for that edge      */

static volatile uint32_t edgeSda = 0, edgeScl = 0;
static void IRAM_ATTR onSdaEdge() { edgeSda++; }
static void IRAM_ATTR onSclEdge() { edgeScl++; }

static void printWireVerdict(bool sdaUp, bool sclUp, uint32_t sda, uint32_t scl)
{
  Serial.printf(" bus test: %lu falling edges on SCL, %lu on SDA (%lu ms after the first)\n",
                (unsigned long)scl, (unsigned long)sda, (unsigned long)WIRE_TEST_MS);

  if (!sdaUp || !sclUp) {
    Serial.println(F(" verdict : a line idles LOW - that wire is off, or its"));
    Serial.println(F("           2k pull-up to 3V3 is missing at the MCX end"));
  } else if (scl == 0 && sda == 0) {
    Serial.println(F(" verdict : pulled up but SILENT. Either the MCX is not"));
    Serial.println(F("           running, or these wires do not reach J5 pins"));
    Serial.println(F("           5/6 - check they are not on J6, and not on"));
    Serial.println(F("           the IMU's VCC/AD0/INT pins next to its SDA/SCL."));
    Serial.println(F("           (With the gateway down the MCX retries only"));
    Serial.println(F("           twice a second, so a quiet second is not proof."));
    Serial.println(F("           Type p for the tug test - that one is.)"));
  } else if (scl == 0 || sda == 0) {
    Serial.printf(" verdict : only %s carries traffic - the %s wire is off the\n",
                  scl ? "SCL" : "SDA", scl ? "SDA" : "SCL");
    Serial.println(F("           bus (a bad jumper, or the wrong header pin)"));
  } else if (sda > scl) {
    Serial.println(F(" verdict : *** SDA AND SCL ARE SWAPPED *** - the clock is"));
    Serial.printf("           arriving on GPIO%u. Move SCL to GPIO%u (D%u) and\n",
                  PIN_SDA, PIN_SCL, PIN_SCL);
    Serial.printf("           SDA to GPIO%u (D%u). On the 30-pin board they are\n",
                  PIN_SDA, PIN_SDA);
    Serial.println(F("           NOT adjacent: TX0 and RX0 sit between them"));
  } else {
    Serial.println(F(" verdict : wiring looks right - clock on SCL, data on SDA"));
  }
}

/* ---- setup / loop ------------------------------------------------------- */

void setup()
{
  Serial.begin(115200);
  delay(200);

  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  /* Before touching the bus: do the MCX's 2k pull-ups actually reach these
   * pins? An external 2k to 3V3 beats the ~45k internal pulldown, and a wire
   * that is not connected cannot. Sampled repeatedly so a poll in progress
   * cannot read as a dead line. */
  pinMode(PIN_SDA, INPUT_PULLDOWN);
  pinMode(PIN_SCL, INPUT_PULLDOWN);
  bool sdaUp = false, sclUp = false;
  for (uint8_t n = 0; n < 20u; n++) {
    if (digitalRead(PIN_SDA)) { sdaUp = true; }
    if (digitalRead(PIN_SCL)) { sclUp = true; }
    delay(2);
  }

  /* A HIGH line only proves a pull-up. It does not prove the wire lands on
   * the bus the MCX is actually clocking, nor that SDA and SCL are on the
   * pins this sketch thinks they are - and on the 30-pin DevKit D21 and D22
   * are not neighbours, TX0/RX0 sit between them. So listen for one second:
   * a live I2C bus puts nine falling edges per byte on SCL and far fewer on
   * SDA. Which pin sees the clock names the wiring. */
  attachInterrupt(digitalPinToInterrupt(PIN_SDA), onSdaEdge, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_SCL), onSclEdge, FALLING);
  /* With this node down the MCX backs off to a poll every 500 ms and, after
   * a run of failures, resets its controller - so it may touch the bus only
   * once every few seconds. Wait for the first edge, up to WIRE_WAIT_MS,
   * and only then count for WIRE_TEST_MS: one START plus one address byte
   * is enough to tell which pin has the clock. */
  uint32_t t0 = millis();
  while (edgeSda == 0 && edgeScl == 0 && millis() - t0 < WIRE_WAIT_MS) delay(10);
  delay(WIRE_TEST_MS);
  detachInterrupt(digitalPinToInterrupt(PIN_SDA));
  detachInterrupt(digitalPinToInterrupt(PIN_SCL));
  uint32_t sdaEdges = edgeSda, sclEdges = edgeScl;

  /* Wire.begin() returns false on a failed slave init and reports why only
   * through log_e, which prints nothing at Core Debug Level "None". Discarding
   * it is how a dead I2C slave still produces a clean-looking boot. */
  bool i2cOk = Wire.begin(GW_I2C_ADDR, PIN_SDA, PIN_SCL, 400000);

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
  Serial.printf(" I2C init: %s\n", i2cOk ? "ok" : "*** Wire.begin FAILED ***");
  Serial.printf(" SDA line: %s\n",
                sdaUp ? "HIGH - wire and pull-up present"
                      : "*** LOW - nothing external on this pin ***");
  Serial.printf(" SCL line: %s\n",
                sclUp ? "HIGH - wire and pull-up present"
                      : "*** LOW - nothing external on this pin ***");
  printWireVerdict(sdaUp, sclUp, sdaEdges, sclEdges);
  Serial.println(F(" Type i for the I2C counters, w for WiFi, ? for help."));
  Serial.println(F(" Reads climbing means the VCU is polling. It is the"));
  Serial.println(F(" master here, so this node never speaks first."));
  Serial.println(F("---------------------------------------------"));
}

void loop()
{
  http.handleClient();
  ws.loop();
  consoleService();
  diagService();

  /* Drain everything the MCX has said since the last pass. */
  while (rxTail != rxHead) {
    uint8_t i = 0;
    for (; i < GW_LINE_MAX - 1 && rxRing[rxTail][i] != '\0'; i++) line[i] = rxRing[rxTail][i];
    line[i] = '\0';
    rxTail = (uint8_t)((rxTail + 1u) % RX_LINES);

    /* Telemetry is for the page, not for a human reading the monitor. */
    if (line[0] != 'T' || line[1] != ',') Serial.printf("[MCX -> laptop] %s\n", line);

    ws.broadcastTXT(line);
  }
}
