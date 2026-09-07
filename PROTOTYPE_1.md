# Distributed-ECU Car — Prototype 1

**NXP Summer School project · FRDM-MCXA153 + 2 × Arduino + ESP32 · September 2026**

This is the story of the first working prototype: what it is, what is in it, how the pieces
talk to each other, and what we learned getting it to move. It is written to be read from the
top, by someone who has never seen the car. If you want to *build* one, the pin-level truth is
in [`FULL_SCHEMA.md`](FULL_SCHEMA.md) and the wiring sheet; if you want to *flash* one, go to
[`FIRMWARE.md`](FIRMWARE.md). This document is the map that tells you why those two exist and
how they fit together.

---

## 1. What it is, in one paragraph

A small four-wheel car that drives itself the way a real vehicle does: not with one big
microcontroller doing everything, but with **four separate boards, each with one job**, wired
together into a network. One board is the brain, one board only measures, one board only turns
the wheels, and one board only talks to the laptop. If any one of them fails, the others notice
and the car stops safely. That last sentence is the whole point of the project. The car is the
excuse; the distributed safety architecture is the lesson.

You join its WiFi network from a laptop, open a web page, and steer it with the W, A, S, D
keys. It stops itself if you drive it at a wall,
stops itself if the laptop goes quiet, stops itself if the brain board goes quiet, and it
reports what it is doing on a little LCD on its back.

---

## 2. The idea: four boards, four jobs

Real cars do not run on one computer. They run on dozens of small ones (ECUs, electronic
control units) that each own one part of the car and talk over a network. We copied that shape
at desk scale.

```
                      laptop (browser, WiFi)
                              │
                              ▼
                     ┌──────────────┐
                     │  GW  ESP32   │   the gateway: WiFi in, bytes out. Decides nothing.
                     └──────┬───────┘
                            │ I²C
                            ▼
 ┌────────────────┐  UART  ┌──────────────────┐  UART  ┌────────────────┐
 │ PERC  Arduino 1│ ─────► │ VCU  FRDM-MCXA153│ ─────► │ ACT  Arduino 2 │
 │ two sonars     │        │ the brain, the   │        │ four motors    │
 │ PIR, buzzer    │ ◄───── │ hub, the referee │ ◄───── │ LCD            │
 └────────────────┘        └────────┬─────────┘        └────────────────┘
                                    │ I²C
                              ┌─────┴─────┐
                              │  MPU6050  │  gyro + accelerometer
                              └───────────┘
```

| Node | Board | What it owns | What it must never do |
|---|---|---|---|
| **VCU** (vehicle control unit) | FRDM-MCXA153 | Every decision. Reads the sensors, applies the rules, sends the motor commands, watches the other three | Nothing is off limits — it is the only board allowed to decide |
| **PERC** (perception) | Arduino UNO/Nano | Measures: front range, back range, motion (PIR). Sounds the horn when told | Decide anything. It reports, it never acts on its own readings |
| **ACT** (actuation) | Arduino UNO/Nano | Turns the four motors, runs the LCD | Move without a fresh command. 300 ms of silence and it stops |
| **GW** (gateway) | ESP32 DevKit | Is the WiFi access point and the web page. Moves bytes between browser and VCU | Touch the wheels. It only forwards *which keys are pressed* |

Why not one board? The honest answer is fault containment. WiFi is bursty and retries; it must
never be able to reach the motors. Ultrasonic ranging needs microsecond timing and would fight a
control loop on the same chip. Motor PWM needs the only 16-bit timer an Arduino has. Splitting
them is not about CPU load, it is about making sure one thing going wrong cannot become three.

**The MCX is the hub, and there is no bus.** The MCXA153 has no CAN controller (we checked
three ways, see [`CONNECTING_THE_BOARDS.md`](CONNECTING_THE_BOARDS.md)), so the links are
point-to-point: two UARTs to the two Arduinos, one I²C bus to the ESP32 and the IMU. Every
message from an Arduino goes to the MCX and the MCX decides whether to relay it. That is
actually a good shape for a safety system — the referee sits in the middle of every
conversation.

---

## 3. What is in the car

### The boards

| Qty | Part | Role |
|---|---|---|
| 1 | **FRDM-MCXA153** (NXP MCX A-series, Cortex-M33) | VCU |
| 2 | **Arduino UNO / Nano** (ATmega328P, CH340 USB) | PERC and ACT |
| 1 | **ESP32 DevKit V1** (30-pin) | GW |

### Sensors

| Qty | Part | Where | What it gives us |
|---|---|---|---|
| 2 | **HC-SR04P** ultrasonic | PERC, one facing forward, one facing back | Distance to the nearest thing, about 8 times a second each, up to about 1.7 m |
| 1 | **HC-SR501** PIR | PERC | "Something warm moved" — a person-detector |
| 1 | **MPU6050** IMU (GY-521 board, silkscreened HW-123) | VCU, on the I²C bus | Gyro and accelerometer. The only motion feedback in the whole car |

### Actuators and displays

| Qty | Part | Where | Notes |
|---|---|---|---|
| 4 | **TT gear motor** with wheel | ACT via the drive stage | The chassis kit's motors. 3–6 V, about 250 mA running |
| 4 | **PN2222A** NPN transistor | ACT | One per motor, low-side switch. This *is* the motor driver |
| 4 | **1N4001** diode | ACT | Flyback protection across each motor |
| 1 | **1602A** LCD (bare HD44780, no I²C backpack) | ACT | Two rows of 16 characters. Row 1 is your message or the rear warning, row 2 is the live drive state |
| 1 | **SFM-27** buzzer | PERC | The horn, and the safe-state alarm |
| 1 | **12 V fan** | On the 9 V pack terminals | Cools the ESP32's regulator. Permanently on |

### Chassis and power

| Qty | Part | Notes |
|---|---|---|
| 1 | **ZX-4WD** style kit chassis | Two plates, four motors, four wheels. It came with encoder disks that nothing reads yet |
| 1 | **6 V** pack | The four motors and the ESP32 |
| 1 | **9 V** pack | The MCX, PERC and ACT |

---

## 4. How the car moves

The car moves like this:

**Each motor has its own switch.** One PN2222A transistor sits on the ground side of each
motor. Turn it on and that wheel pulls; the four together are the whole drive stage, built from
four transistors, four diodes and eight resistors. There is no driver module anywhere in the
car, which means every part of it can be measured with a meter and understood on paper.

**Speed comes from pulsing that switch.** The transistors are switched on and off two thousand
times a second, and the fraction of time they spend on is the speed. Fully on, fully off,
nothing in between, which is why they stay cool.

**Steering comes from the difference between the two sides.** The left pair runs from one pin
and the right pair from another. Left at 60 % and right at 30 % curves the car right; left at
60 % and right at zero pivots it on the spot. Wherever it needs to go, it turns to face that
way and drives, and a pivot takes it out of a corner without needing to back up.

**The gyro is what closes the loop.** The MPU6050 measures how fast the car is really turning,
which makes heading a measurement rather than a hope. Wheel speed stays open loop: throttle is
a request to the motors, so the car runs slower on carpet and slower as the pack drains. The
two interrupt pins held free on ACT (D2, D3) are the upgrade path, and the encoder disks are
already on the wheels.

**The whine you hear is the drive stage working.** 2 kHz is deliberate. The 1N4001 diodes are
slow to stop conducting, and at this frequency the motor current has fully decayed before the
transistor switches back on, so that slowness costs nothing. If a transistor ever runs warm the
fix is a *lower* frequency, never a higher one.

**The transistors run cool by design, and the front guard keeps them there.** A PN2222A driving
a motor burns about 100 mW in a package rated for 625 mW. The one case that would change that
is holding a *stalled* motor against a wall, which is exactly what the 25 cm front guard exists
to prevent. That guard, and the habit of never leaving the car pushing against something, are
what keep the drive stage comfortable.

### The safe state is a resistor

Every one of the four transistors has a 10 kΩ resistor from its base to ground. When the
Arduino pin driving that base goes quiet — reset button, upload in progress, a crashed sketch,
a pulled cable — the resistor pulls the base down and the transistor turns *off*. The motors
stop because of a passive component, not because software told them to. Everything else in the
safety story sits on top of that.

---

## 5. How the car sees

**Two ultrasonic sensors, fixed, one forward and one back.** Earlier designs swept a single
sensor on a servo, then had two sensors splayed left and right for obstacle avoidance. For a
car driven by a person, neither is right: the driver already knows which way to turn, and what
they cannot see is what is directly ahead and what is behind them. So one sensor covers the
direction of travel and one covers the end you cannot watch. They fire alternately, 60 ms
apart, because two ultrasonic sensors pinging at once hear each other's echo through the
chassis and both report nonsense.

**The front sensor can overrule the driver.** Under **25 cm** the VCU refuses forward throttle
and the web page turns red. A pivot is still allowed, so turning away is always available.

**The back sensor warns instead of acting.** There is no reverse gear for it to veto, so what
it looks after is the person standing behind the car rather than the car itself. Inside
**25 cm** the VCU writes `ATTENTION` across the top row of the LCD, and puts the previous
message back once the rear is clear. Three details make that trustworthy:

- It **clears at 32 cm, not 25**, so a wall sitting right on the threshold cannot flicker the
  screen several times a second.
- The warning is **repeated once a second** while it stands. The message line is
  fire-and-forget over a link with no acknowledgement, and one dropped line must not be the
  difference between a warned bystander and an unwarned one.
- A **silent PERC or a no-echo reading counts as clear**, the same rule the front guard uses,
  so a missing sensor can never invent an alert.

The MCX console logs each transition as `[rear] ATTENTION (n cm)` and `[rear] clear`, which is
the quickest way to check the feature without a tape measure.

**The PIR sees people.** It is slow (it needs a full minute after power-up to settle) and it
only says "yes, something warm moved". In this prototype it is reported to the browser and
nothing acts on it; in the next one it is the "stop for a pedestrian" input.

**The IMU is the only motion sensor.** The MPU6050 sits on the MCX's I²C bus, powered from the
board's own 3.3 V. Its gyro gives heading rate, its accelerometer gives vibration (a moving car
shakes; a stalled one does not). The prototype reads it and streams it; the heading hold that
uses it is next.

---

## 6. How the boards talk

Everything is plain ASCII text, one message per line, at 38400 baud on the UARTs and 400 kHz on
I²C. You can read every message on a serial monitor with your eyes, which is worth more during
bring-up than any amount of bandwidth.

| From → to | Line | Rate | Meaning |
|---|---|---|---|
| browser → VCU | `K,<w>,<a>,<s>,<d>,<horn>` | 10 Hz + on change | Which keys are held right now |
| browser → VCU | `M,<text>` | when typed | Text for the LCD |
| browser → VCU | `E` | on release | Stop and let go of the wheels |
| VCU → browser | `T,<16 fields>` | 10 Hz | Telemetry for the page |
| VCU → ACT | `D,<left>,<right>` | 20 Hz while driving | Motor duty, 0–100 each |
| VCU → ACT | `M,<text>` | on change, then 1 Hz while a rear warning stands | Text for the LCD, including `ATTENTION` |
| VCU → PERC | `H,<0 or 1>` | 4 Hz while the horn is held | Horn on and off |
| VCU → both Arduinos | `V,<ms>` | 1 Hz | Heartbeat, so each node can tell a live VCU from a dead one |
| ACT → VCU | `S,<left>,<right>,<age_ms>` | 5 Hz | What ACT is actually doing, and how stale its last command is |
| PERC → VCU | `P,<front_cm>,<back_cm>,<pir>` | 10 Hz | Ranges (-1 means no echo) and PIR |

Two things about this protocol are deliberate and worth keeping when it changes:

- **Commands are repeated, not sent once.** The VCU resends the drive command 20 times a
  second even if nothing changed. A command you send once is not a command an actuator can
  safely act on, because it cannot tell "still valid" from "the sender died".
- **The browser sends key *state*, not commands.** The throttle ramp, the steering mix, the
  duty cap, the obstacle veto and every watchdog live on the MCX. The worst a laggy, broken
  or hostile browser can do is stop sending keys, and then the car stops.

The telemetry line is the one whose field order matters, because two programs parse it:

```
T, live, L, R, throttle, front, back, pir, horn, guard, ax, ay, az, gz, actAge, percAge
```

The two ages at the end are milliseconds since that node last spoke. They are the link health
readout, and the web page shows them so you can see which board has gone quiet.

---

## 7. Power: two packs, one ground

```
6 V PACK ──► the four motors, through their transistors
         ──► ESP32 VIN, with 470 uF right beside the board

9 V PACK ──► FRDM-MCXA153, PERC Arduino, ACT Arduino
         ──► the 12 V fan, aimed at the ESP32's regulator

         both negatives meet at ONE star point
```

The reasons behind the shape:

- **Six volts suits the motors and the ESP32 equally well.** TT motors take 3-6 V, so this is
  the top of their window and they run properly quick. The ESP32's on-board regulator drops
  6 V to 3.3 V burning about 0.3 W, which is comfortable — at 9 V it would be twice that and
  genuinely hot.
- **Watch the sag, because they share.** The ESP32 needs at least about 4.4 V at VIN, and four
  motors starting or stalling pull the pack down hard. The 470 uF beside the board is what
  rides out its own WiFi bursts; if the ESP32 ever resets when the car lurches, that pack is
  sagging and the two loads want separating again.
- **Nine volts suits the two Arduinos.** Their regulators need 7 V or more to hold 5 V, so 9 V
  is right and 6 V would not be.
- **Check which regulator the ESP32 DevKit has** before connecting anything. The fat 4-pin
  SOT-223 (AMS1117) is happy to 15 V. The tiny 5-pin types stop at about 6 V, which puts them
  right at the edge of this pack.
- **The fan is wired permanently on**, taken from the 9 V pack terminals rather than through
  a board's supply pin — brushless fans put commutation spikes on their supply wires and those
  must not arrive at the MCX. Switching it would need a flyback diode there is none spare for.
  It runs at about 70 % speed on 9 V, which is quieter and still plenty of air. Take it from
  the 9 V pack even though it cools a board on the 6 V one: a 12 V fan is marginal to start at
  6 V.
- **One ground.** A UART line is a voltage relative to ground. If two boards disagree about
  where zero is, the link is garbage and it looks exactly like a firmware bug. Both pack
  negatives, all four board grounds and all four transistor emitters meet at one point.

### One open item: how the 9 V reaches the MCX

The FRDM board's VIN pin is a dead end. It is silkscreened VIN and the manual rates it 5-9 V,
but on a stock board it feeds an unpopulated regulator footprint, and the path onward is
disabled by default. Nine volts there powers nothing at all.

The board's only working input for a pack is **5 V into J3-10**, so something has to bring the
9 V down before it gets there. Until that part is picked, the MCX runs from its MCU-Link USB
cable — which is what the prototype has done all along, and it is also what keeps the console
and the debugger alive. Note that J3-10 is the same rail the USB connectors feed, so it is one
or the other, never both.

Full detail, including the regulator heat maths and why a resistor divider cannot feed an ESP32,
is in `FULL_SCHEMA.md` §11.

---

## 8. The safety net

There are four independent layers, and each one works without the others.

| Layer | Where | Trigger | What happens |
|---|---|---|---|
| **Hardware off-state** | ACT, the 10 kΩ pull-downs | Arduino pin goes quiet for any reason | All four transistors off. No software involved |
| **ACT watchdog** | ACT firmware | 300 ms without a `D,` line | Motors to zero. ACT never needs to know *why* the VCU stopped talking |
| **VCU watchdog** | MCX firmware | 400 ms without a `K,` line from the browser | Throttle to zero, stop sending drive commands |
| **Browser** | the web page | tab loses focus, closes, WiFi drops | Stops sending keys |

Plus two rules the VCU applies on top:

- **The front guard.** Under 25 cm, no forward throttle. Pivoting is still allowed. If PERC
  itself is silent the guard is *off*, not on, so the car can be driven while PERC is being
  brought up — a human is watching the wheels.
- **The horn has its own watchdog.** PERC silences it 500 ms after the last "on" message, so
  one lost "off" cannot leave it sounding until someone pulls a battery.

You can test the whole chain in ten seconds: hold W with the wheels off the floor, alt-tab
away from the browser, and watch the car stop. The MCX console prints
`[teleop] browser silent - released`.

---

## 9. The firmware

Five programs, one per node plus a standalone motor test, and they build on each other in
order. When something breaks, the order tells you which layer is new.

| Order | Node | File | What it proves |
|---|---|---|---|
| 1 | ACT | [`arduino_codes/act_motor_test/`](arduino_codes/act_motor_test/act_motor_test.ino) | The car moves at all. Transistors, diodes, PWM, wiring |
| 2 | ACT | [`arduino_codes/act/`](arduino_codes/act/act.ino) | Same drive stage, now taking orders and stopping safely |
| 3 | VCU | [`distributed_ecu/led_blinky.c`](distributed_ecu/led_blinky.c) + [`link.c`](distributed_ecu/link.c) | Drive commands from the MCX console, the hub, all three links |
| 4 | PERC | [`arduino_codes/perc/`](arduino_codes/perc/perc.ino) | Ranging, PIR and horn on the link |
| 5 | GW | [`arduino_codes/esp32_gateway/`](arduino_codes/esp32_gateway/esp32_gateway.ino) | Driving it from a browser |

Alongside those, one self-test per board and one link test per UART, so a fault can be pinned
to a board or a wire before anyone starts guessing:

| Test | File | Covers |
|---|---|---|
| PERC alone | [`test_perc/`](arduino_codes/test_perc/test_perc.ino) | Both sonars, PIR, buzzer |
| ACT alone | [`test_act/`](arduino_codes/test_act/test_act.ino) | Motors, LCD |
| GW alone | [`test_vis/`](arduino_codes/test_vis/test_vis.ino) | WiFi AP, I²C slave |
| VCU alone | the normal build, console commands `i2c`, `imu`, `led`, `btn` | I²C bus, IMU, LED, buttons |
| MCX ↔ PERC wire | [`test_link_perc/`](arduino_codes/test_link_perc/test_link_perc.ino) | Both directions and the divider |
| MCX ↔ ACT wire | [`test_link_act/`](arduino_codes/test_link_act/test_link_act.ino) | Both directions and the divider, wheels never move |
| Any Arduino link, raw | [`link_debug/`](arduino_codes/link_debug/link_debug.ino) | Prints every byte in hex, parses nothing |

### What the MCX firmware does, in words

The MCX runs a single main loop with no operating system. A 1 ms tick drives everything: the
20 Hz drive repeat, the 50 Hz gateway poll, the 10 Hz telemetry, the 1 Hz heartbeat LED. Each
UART has an interrupt-driven receive ring so both Arduinos can talk at once. The I²C bus is
polled from the main loop, and it has learned to heal itself: after a run of failures it
re-initialises the peripheral, and if a slave is holding a line low it clocks the bus free by
hand. Two buttons on the board run the two most useful diagnostics without the console, so a
dead terminal cannot take every tool away with it.

The console over the MCU-Link USB port (115200 baud) is the main window into the car during
development. `?` lists the commands; `sens` dumps every sensor with its age; `diag` toggles a
two-second summary of all four links.

---

## 10. Driving it

1. Power up. Both packs together, or the 6 V one first so the ESP32 is never sitting
   unpowered on a live I²C bus.
2. On the laptop, join the WiFi network **NXP-AV**, password **distributed**. The ESP32 is the
   access point, so this works anywhere, with no router.
3. Open **http://192.168.4.1** and click the page once so it has keyboard focus.

| Key | Does |
|---|---|
| **W** | Throttle. It ramps up over about a second — it is not a switch |
| **A / D** | Steer, by taking throttle off that side. At a standstill they pivot |
| **S** | Brake. Not reverse; there is none |
| **Space** | Horn |

The text box on the page writes to the LCD. Row 2 of the LCD always shows the live drive
state, so the car keeps reporting itself with the laptop shut. The page also shows the two
ranges, the PIR, the IMU, and the age of the last message from each board; a red page means the
front guard has tripped.

---

## 11. What we learned building it

The prototype took longer to wire than to write, and nearly every delay was one of these. They
are recorded here because each one looked like something else at the time.

**A latched ESP32 looks like a dead ESP32, and it is.** The I²C bus idles at 3.3 V through its
pull-ups. Unplug the ESP32's USB while the MCX is still powered and that current flows into the
ESP32's I/O pins and back-feeds the whole chip. Power it up in that state and it latches up:
3.3 V and EN read fine, no access point ever appears, the ROM never answers. Two boards died
that way. The fix is two 470 Ω resistors in series with the I²C wires at the ESP32 end, and a
plug order: wires off before either USB comes out, USB back in before the wires go back on.
It matters more now that the ESP32 sits on its own pack — the two boards no longer come up
together by default, so switch both packs at once, or the ESP32's first and off last.

**The I²C bus needs its own pull-ups.** For a while the design borrowed the 2.2 kΩ resistors on
the MPU6050 board. Electrically that worked; as a design it made a sensor breakout a hard
dependency of the WiFi gateway. Unplug the IMU and the whole bus floats, nothing can acknowledge
an address, and the gateway goes silent for a reason nothing can report. Two 2 kΩ resistors to
3.3 V and the bus stands on its own.

**Wires in the wrong holes look exactly like firmware.** The PERC link was dead in both
directions for most of an evening while both boards were provably healthy. The MCX side was
checked three ways; the Arduino side was transmitting fine. A GPIO test on the MCX finally showed
the receive pin was floating — nothing was connected to it — and the two wires were simply on
the wrong holes of a two-row header. The lesson became a console command (`sense1`), which
answers "is anything connected to this pin at all" in one second with no meter.

**The VIN pin on the FRDM board does nothing.** It is silkscreened VIN, it is rated 5–9 V in the
manual, and on a stock board it feeds an unpopulated footprint. Nine volts on it does nothing at
all. That is still the open question in the power design, and it is why the MCX is on USB.

**A one-byte I²C probe broke the link it was meant to check.** The ESP32 answers every read with
a fixed 49-byte frame. A bus scan that reads one byte from each address left 48 bytes stranded
in the ESP32's transmit buffer, and every later poll was a frame behind until the board was
power-cycled. The scan you run *because* the link is broken was breaking it. The probe is now
a one-byte write, which is a no-op on both slaves.

**A floating input can kill a serial link on the same chip.** PERC's echo pins are wired to
interrupts. With a sonar unplugged, the pin floats, the interrupt fires thousands of times a
second, and the bit-banged serial port loses its timing. Pulling the pins up made that fault
report "-1 cm" and stay local, instead of taking the node off the network.

**An LCD that shows nothing, or shows solid blocks, is almost never broken.** It is the contrast
trimpot. Turn it slowly through its whole range before suspecting anything else.

**The blocking write nobody noticed.** The MCX's UART transmit had no timeout. A stalled
transmitter would have frozen the whole main loop, console included. A car must not be one loose
wire away from a hang, so every UART and I²C wait is now bounded, and the diagnostics say
`TX STALLED` if one trips.

---

## 12. Document map

| You want to… | Read |
|---|---|
| Understand the car | this file |
| Wire the car, every terminal and every wire | [`FULL_SCHEMA.md`](FULL_SCHEMA.md) Appendix A, and [`SCHEMA_WIRING.svg`](SCHEMA_WIRING.svg) |
| Understand the electrical design and why each part is there | [`FULL_SCHEMA.md`](FULL_SCHEMA.md) §1–§11 |
| See the architecture on one sheet | [`SCHEMA_SIGNALS.svg`](SCHEMA_SIGNALS.svg), [`WIRING_4_NODES.svg`](WIRING_4_NODES.svg) |
| See the power rails on one sheet | [`SCHEMA_POWER.svg`](SCHEMA_POWER.svg) |
| Flash it, test it, debug a link | [`FIRMWARE.md`](FIRMWARE.md) |
| Know the FRDM-MCXA153 pinout and why there is no CAN | [`CONNECTING_THE_BOARDS.md`](CONNECTING_THE_BOARDS.md) §5 |
| See where the serial hub came from | [`SERIAL_BRIDGE.md`](SERIAL_BRIDGE.md), [`uart-link-demo/`](uart-link-demo/README.md) |
