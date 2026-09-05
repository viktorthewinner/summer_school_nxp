# Firmware — getting the car moving

Five programs. **Run them in this order**; each one only adds a layer to the one before, so
when something breaks you already know which layer is new.

| # | Node | Sketch / file | What it proves |
|---|---|---|---|
| **1** | ACT | [`arduino_codes/act_motor_test/`](arduino_codes/act_motor_test/act_motor_test.ino) | **The car moves.** Transistors, diodes, PWM, wiring |
| 2 | ACT | [`arduino_codes/act/`](arduino_codes/act/act.ino) | Same drive stage, now taking orders and stopping safely |
| 3 | VCU | `distributed_ecu/led_blinky.c` | Drive commands from the MCX console |
| 4 | PERC | [`arduino_codes/perc/`](arduino_codes/perc/perc.ino) | Ranging, PIR and horn on the link |
| **5** | GW | [`arduino_codes/esp32_gateway/`](arduino_codes/esp32_gateway/esp32_gateway.ino) | **Driving it from a browser with WASD** |

Steps 1 to 3 need no ESP32. Step 5 is the only one that does.

## Testing each board on its own

Before any of that, there is one self-test per board. Each is standalone — USB cable only, no
other board attached — and each exercises everything on that board from a menu, so a failure
tells you which part is wrong rather than that "it does not work".

| Board | Program | Tests |
|---|---|---|
| VCU | the normal `distributed_ecu` build — console commands | I²C bus scan, MPU6050, RGB LED, SW2/SW3, both UARTs, `sens` |
| PERC | [`test_perc/`](arduino_codes/test_perc/test_perc.ino) | Both ultrasonics, PIR, buzzer |
| ACT | [`test_act/`](arduino_codes/test_act/test_act.ino) | Motors, 1602A |
| GW | [`test_vis/`](arduino_codes/test_vis/test_vis.ino) | WiFi AP, I²C slave |

Those four are standalone, and that is their limit: **none of them can reach the wiring
*between* boards.** Two more cover that, and they are the ones to reach for once the loom
is in:

| Link | Program | Tests |
|---|---|---|
| MCX ↔ PERC | [`test_link_perc/`](arduino_codes/test_link_perc/test_link_perc.ino) | J2-2 / J2-4, the PERC divider, common ground |
| MCX ↔ ACT | [`test_link_act/`](arduino_codes/test_link_act/test_link_act.ino) | J2-20 / J2-18, the ACT divider, common ground |

### VCU — FRDM-MCXA153

Flash the normal build, open the VCOM at 115200:

```
i2c    scan the bus - expect 0x42 (ESP32) and 0x68 (MPU6050)
imu    WHO_AM_I, wake the sensor, then stream ten samples
led    cycle red, green, blue
btn    watch SW2 and SW3 for eight seconds
sens   one dump of ranges, PIR, ACT status and the IMU, with ages
tx1    round-trip test the PERC link
tx2    round-trip test the ACT link
link   round-trip test both, with a verdict
```

Flat and still, `imu` should show `az` near **+16384** and all three gyro axes near zero. Turn
the board about its vertical axis and `gz` should swing.

> **The I²C bus has no pull-ups of its own** — it borrows the 2.2 kΩ on the MPU6050 board.
> Unplug that module and the whole bus goes dead, the ESP32 included. If `i2c` finds nothing at
> all, check the IMU is connected before suspecting anything else.

`led` and `btn` need `P3_13`, `P3_0`, `P3_29` and `P1_7` muxed, which the generated `pin_mux.c`
does not do. That setup lives in `led_blinky.c` on purpose: Config Tools would regenerate
`pin_mux.c` and drop it again, along with your six UART pins.

### PERC — Arduino 1

```
u     stream both ranges, alternating front / back   p   watch the PIR
b1    buzzer, steady DC                          b2   buzzer, 2 kHz square
x     stop streaming
```

Give the PIR a **full minute** after power-up before you judge it.

`b1` and `b2` identify the SFM-27: a steady DC that buzzes means **active**, a clean 2 kHz tone
from `b2` means **passive**. They need opposite drive, so find out before you write anything
that depends on it.

Both sonars reading the same value regardless of what you put in front of them usually means
they are firing together and hearing each other — the alternation is not optional, and pointing
them in opposite directions does not fix it. The burst reaches the other sensor through the
chassis as readily as through the air.

**`D3`/`D7` is the front sensor, `D2`/`D6` is the back one.** That mapping lives only in the pin
constants at the top of the sketch; nothing above PERC knows a pin number.

### ACT — Arduino 2

```
b 40 / l 40 / r 40    motors: both, left pair, right pair
t                     motor self test          x   stop
d <text> / d          LCD
```

Blank LCD or a row of solid blocks is the **contrast trimpot** — turn it slowly through its
whole range before assuming anything is broken.

### GW — ESP32

```
w   WiFi status and clients      i   I2C slave counters
e   queue a line for the VCU     ?   help
```

`i` is the useful one. **Reads climbing means the VCU is polling and the link works** — the MCX
is the master here, so this node never speaks first. If reads stay at zero with the MCX
running, check the two I²C wires, and remember the bus has no pull-ups of its own: it borrows
the 2.2 kΩ on the MPU6050 board, so that module has to be present for anything on the bus to
work at all.

`e` queues a line that leaves on the VCU's next poll and should appear on the MCX console as
`[GW] GW,hello from the ESP32`. That proves the return path end to end.

---

## Step 1 — prove the car moves

**Nothing but ACT and its USB cable. Wheels off the floor.**

Upload `act_motor_test.ino`, open the Serial Monitor at **115200**, line ending **Newline**.

```
t          run the self test: left pair, right pair, both, then faster
b 40       both pairs to 40 %
l 40       left pair only
r 40       right pair only
x          stop
```

What you are checking, in order:

1. **`l 40` turns only the left two wheels.** If both sides move, the two channels are
   shorted somewhere. If the wrong side moves, D9 and D10 are swapped.
2. **`r 40` turns only the right two.**
3. **`b 40` turns all four the same way.** A wheel spinning backwards means that motor's two
   leads are swapped — fix it at the motor, not in software.
4. **`x` stops everything**, and so does pressing reset mid-run.

Below about 25 % the motors may only buzz — that is friction, not a fault. Duty is capped at
70 % (`DUTY_MAX`) until you trust the wiring.

You will hear a **2 kHz whine**. That is the PWM frequency, forced by the 1N4001 diodes
(see [`FULL_SCHEMA.md`](FULL_SCHEMA.md) §1). It is not a fault.

> **If a transistor gets hot,** stop and check the base resistor on that channel is really
> 470 Ω (or 480). A too-large base resistor takes the transistor out of saturation and turns
> it into a heater — §7 of the schema doc has the numbers.

---

## Step 2 — ACT takes orders

Upload `act.ino`. Same drive stage plus the link on D4/D5.

```
in    D,<left>,<right>          duty 0..100 each
out   S,<left>,<right>,<age>    5 Hz status
```

**No command for 300 ms and the motors stop.** That is the entire safety story on this node —
it never needs to know *why* the VCU went quiet. On its own with nothing connected to D4, ACT
therefore sits stopped and prints `[safe] no command from the VCU`. That is correct behaviour,
not a fault.

You can drive it without the MCX: short D4 to a USB-serial adapter and send `D,40,40` twenty
times a second. Easier to just do step 3.

---

## Step 3 — drive from the MCX

```bash
cmake --build --preset debug
```

Then flash:

```bash
"C:/nxp/LinkServer_26.6.137/LinkServer.exe" flash MCXA153:FRDM-MCXA153 load distributed_ecu/debug/distributed_ecu.elf
```

Open the MCU-Link VCOM at 115200. New commands:

```
d 40        both pairs to 40 %
d 40 20     left 40 %, right 20 %  — this is how it steers
d 40 0      pivot
x           stop, and take the wheels back from the browser
sens        one sensor dump
?           command help
```

Anything else you type is broadcast to all nodes exactly as before.

**Two things this build changes in `led_blinky.c`:**

- **SysTick is now started.** It never was — `SysTick_Handler` was dead code, so the "heartbeat"
  red LED was not actually blinking. It blinks at 1 Hz now, and the drive repeat below depends
  on the same tick.
- **The drive command repeats at 20 Hz.** A command you type once is not a command an actuator
  can safely act on. `x` sends one zero and then stops sending, so ACT's own timeout holds the
  motors stopped even if this board dies immediately afterwards.

ACT's 5 Hz status is captured but only printed once a second — otherwise it makes the terminal
impossible to type in.

---

## Step 4 — PERC reports what it can see

Upload `perc.ino`. It now measures rather than just staying alive:

```
out   P,<front_cm>,<back_cm>,<pir>    10 Hz.  -1 cm means no echo
in    H,<0|1>                         horn
```

Check it on the USB monitor first, with `sens` typed at the MCX console as the cross-check.
A hand 30 cm in front should move the first number and leave the second alone.

**The ranging does not block.** The echo is timed on INT0/INT1 rather than in `pulseIn()`,
which would stop this node for up to 10 ms per ping. Not blocking is why PERC is its own node,
so if you rewrite this, keep that property.

**The horn has its own watchdog.** The VCU repeats `H,1` every 250 ms while you hold space;
PERC silences the buzzer after 500 ms without one. A single lost "off" leaving it sounding
until someone pulls a battery is the most embarrassing failure available to this car, and it
costs four lines to make impossible.

---

## Step 5 — drive it from a browser

Upload `esp32_gateway.ino` (Board: **ESP32 Dev Module**, library: **WebSockets** by Markus
Sattler). Then:

1. Join the WiFi network **`NXP-AV`**, password **`distributed`**.
2. Open **http://192.168.4.1**
3. **Click the page once** so it has keyboard focus.

**The ESP32 is the access point, and that is deliberate.** A network you do not own can block
device-to-device traffic, which would end the demo; this node depends on nothing but itself,
and the address is fixed. It also avoids station mode's modem sleep, which wakes on the beacon
roughly every 100 ms and would add that to every key press on a 20 ms control path.

```
W        throttle - ramps up, it is not a switch
A / D    steer: takes duty off that side. At a standstill they pivot instead
S        brake. NOT reverse - there is none
SPACE    horn
```

The message box writes to the 1602A on ACT. Over 16 characters it marquees; row 2 stays the
live drive state, so the car keeps reporting itself with the laptop shut.

### What the browser is actually allowed to do

**It sends key state and nothing else.** `K,<w>,<a>,<s>,<d>,<horn>`, ten times a second plus
one on every change. The throttle ramp, the steering mix, the duty cap, the obstacle veto and
the watchdogs are all on the MCX. That is not decoration — it is the reason a WiFi dropout is
not a safety event. The worst a broken, lagging or hostile browser can do is stop sending keys.

**Three timeouts in series**, each independent of the one above it:

| Layer | Trigger | Effect |
|---|---|---|
| browser | tab loses focus, closes, or WiFi drops | stops sending keys |
| **VCU** | 400 ms with no `K` line | throttle to zero, stops sending `D` |
| **ACT** | 300 ms with no `D` line | motors off, held off in hardware |

Test it. Hold W with the wheels off the floor and alt-tab away — the car should stop, and the
console should print `[teleop] browser silent - released`.

### The front sensor can overrule you

Under **25 cm** the VCU refuses forward throttle and the page goes red. A pivot still works,
because with no reverse turning away is the only way out. Nothing acts on the back sensor —
it is a readout, for the same reason.

If PERC is silent the guard is off rather than on: you need to be able to drive with PERC
unflashed while bringing the rest up, and a human is watching the wheels. The page shows PERC's
age in milliseconds so you can see which of the two it is.

### Debugging a link that does not work

Do not start at the wire. Start by proving each board alone, because a test that
involves two boards and a cable can only tell you that something is wrong.

| # | Do this | Where | A failure means |
|---|---|---|---|
| 1 | `j` — jumper D4 to D5, both link wires off | [`link_debug/`](arduino_codes/link_debug/link_debug.ino) on that Arduino | **That Arduino.** Pins, SoftwareSerial, baud |
| 2 | `loop1` / `loop2` — jumper the MCX's own TX to its own RX | MCX console | **The MCX.** Pin mux, clock, LPUART setup |
| 3 | `e` on the Arduino, then `tx1` / `tx2` on the MCX | both | The **wires** — and it tests both at once |
| 4 | `t` on the Arduino | both | The **return path**: the 1 k/2 k divider |

Steps 1 and 2 involve one board each and no cable at all. Only when both pass is a
wire worth suspecting — and step 3 then tests both wires in a single round trip,
because echo mode sends every byte straight back the way it came.

**Flash [`link_debug.ino`](arduino_codes/link_debug/link_debug.ino) to whichever Arduino you
are chasing.** It is the same sketch for both, and unlike the production sketches it parses
nothing: every byte arriving on D4 is printed as hex. That matters more than it sounds, because
a parser throws away anything malformed — so a link carrying garbage looks identical to a
dead one, and those two faults share no causes.

> **`loop1` and `loop2` are the ones to reach for first if both links are dead.** LPUART2's
> ALT2 mux value came from NXP's own generated example, but **LPUART1's was worked out by
> elimination and has never been proven**. So "`loop1` passes, `loop2` fails" is a real
> possible outcome, and it would mean the ALT value in `pin_mux.c` is wrong — not the loom.

`mark1` / `mark2` transmit `0x55` for five seconds so a multimeter can settle on the TX pin:
**~1.8 V** while sending, **3.3 V** idle, **0 V** if the pin is not muxed to the LPUART at all.
Three clearly different readings, no oscilloscope.

`raw1` / `raw2` put a channel into raw byte view on the MCX side, the same way round.

### Reading the three consoles

Every board now says what it can see, so you can find a dead link without guessing.

**MCX, 115200** — the only node that sees all four links. It prints the moment one changes:

```
  [GW] ESP32 answering on I2C
  [PERC] up - ranges arriving
  [ACT] SILENT - no S, line. Check J2-20/J2-18 and its divider
  [diag] GW up  4821 ok / 0 fail | PERC 92 ms | ACT SILENT | WEB 104 ms
```

`diag` toggles the 2 s summary; the up/down messages stay on either way. `sens` dumps the
sensors once. The `ok / fail` pair is I²C transactions: an idle gateway and an absent one both
return no bytes, so the failure count is the only thing that tells them apart.

**ESP32, 115200** — answers one question, and it is the right one:

```
  [i2c] VCU IS POLLING - link up
  [gw] reads 1482 (+150)  writes 30  queued 0  dropped 0  ws 1
```

**Reads climbing means the VCU is alive and the I²C wiring is good.** The MCX is the master, so
this node never speaks first. Frozen reads mean the MCX is not running this firmware, or SDA/SCL
are wrong, or — the one people miss — the MPU6050 is unplugged, since its 2.2 kΩ are the bus's
only pull-ups.

**PERC and ACT, 115200** — each says whether the VCU is reaching it, and what its own hardware
is doing:

```
  [link] VCU UP - lines arriving on D4
  [perc] front 41 cm   back -- cm   PIR still   horn off   VCU 240 ms ago, 96 lines
  [act] L0 R0  SAFE  VCU 12 ms ago, 1904 lines (1902 drive)
```

> PERC could not answer that question before. Ranges only travel outward, so with the horn
> quiet it heard nothing at all from a perfectly healthy VCU — a working link and a dead one
> looked identical. The VCU now sends `H,0` once a second purely as a keep-alive.

### When it does not work

| Symptom | Look at |
|---|---|
| page will not load at all | you are on the wrong WiFi. `w` on the ESP32 console lists connected clients |
| page loads, everything reads `-` | the VCU is not polling. `i` on the ESP32 console: reads must climb |
| `PERC silent`, `ACT silent` | that Arduino's own link. `tx1` / `tx2` at the MCX console |
| wheels do nothing, page looks live | `sens` at the MCX console, then ACT's USB monitor |
| horn will not stop | it already has: PERC drops it 500 ms after the last `H,1` |
| keys do nothing | the page does not have focus, or the cursor is in the message box |

Remember the I²C bus borrows the 2.2 kΩ pull-ups on the MPU6050 module. Unplug the IMU and
the browser link dies with it.

---

## Protocol summary

| Direction | Line | Rate |
|---|---|---|
| browser → VCU | `K,<w>,<a>,<s>,<d>,<horn>` | 10 Hz + on change |
| browser → VCU | `M,<text>` | as typed |
| browser → VCU | `E` | stop and release |
| VCU → browser | `T,<16 fields>` | 10 Hz |
| VCU → ACT | `D,<left>,<right>` | 20 Hz while armed |
| VCU → ACT | `M,<text>` | on change |
| VCU → PERC | `H,<0\|1>` | 4 Hz while held |
| ACT → VCU | `S,<left>,<right>,<age_ms>` | 5 Hz |
| PERC → VCU | `P,<front_cm>,<back_cm>,<pir>` | 10 Hz |
| any → any | `[SRC] text` | as typed |

Plain ASCII lines at 38400, same framing the serial hub already used. There is no sign
anywhere in the drive command: a low-side switch conducts one way, so duty is 0..100 and the
car steers on the difference between the two channels.

The telemetry line is the one that needs its field order written down, because two files parse
it — `telemetry_send()` in `led_blinky.c` and `tel()` in the gateway's page:

```
T, live, L, R, throttle, front, back, pir, horn, guard, ax, ay, az, gz, actAge, percAge
```

Ranges are centimetres with `-1` for no echo. The two ages are milliseconds since that node
last spoke, capped at 9999 — they are the link health readout. All four IMU fields reading
exactly zero means no MPU6050 answered; a live one always has noise on it.

---

## What is deliberately not here yet

The heading loop, the state machine, autonomous behaviour, fault injection. Teleop is a driver
with a radio, not a self-driving car — the only thing on board deciding anything for itself is
the 25 cm front guard.

The one piece of firmware design already fixed by the hardware: **there is no closed loop on
speed.** With no encoders, duty is a request, not a measurement. Heading will close on the
MPU6050 at the VCU when you get there; distance stays open loop.

And one thing the front/back split cost: **there is no left/right range comparison any more.**
The splayed pair existed to tell an autonomous car which way round an obstacle to go. A driven
car does not need that and does need to see behind itself. If autonomy comes back, so does the
splayed pair — or a third sensor.
