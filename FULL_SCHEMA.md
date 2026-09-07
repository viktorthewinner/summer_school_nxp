# Full System Schema — Boards and Components

**Distributed-ECU car · NXP Summer School · PN2222A transistor drive, forward only**

| Sheet | File | Covers |
|---|---|---|
| 1 — Signals | [`SCHEMA_SIGNALS.svg`](SCHEMA_SIGNALS.svg) | Every board, every component, every pin |
| 2 — Power | [`SCHEMA_POWER.svg`](SCHEMA_POWER.svg) | Traction pack, USB logic, star ground |
| 3 — **Wiring** | [`SCHEMA_WIRING.svg`](SCHEMA_WIRING.svg) | **Every component, every terminal, every wire** |

Wiring the car? Work from **sheet 3** and the netlist in Appendix A. Sheets 1 and 2 are the
architecture and the rails — useful for understanding it, not for holding a wire.

**Shopping list: fourteen resistors, one trimpot, one buck module.** The four flyback diodes and
the four PN2222A you already have.

| Qty | Part | For |
|---|---|---|
| 4 | 470 Ω resistor | Transistor base drive |
| 4 | 10 kΩ resistor | Base pull-down — this is what makes the safe state real |
| 2 + 2 | 1 kΩ and 2 kΩ | The two level dividers on the Arduino TX lines |
| 1 | **10 kΩ trimpot** | 1602A contrast. Without it the screen is blank or solid blocks |
| 1 | 220 Ω resistor | 1602A backlight — only if the module has no on-board resistor |
| **2** | **2 kΩ resistor** | **I²C pull-ups**, SDA and SCL up to 3V3. Without these nothing on the bus can answer — §3 |
| 1 | **9 V → 5 V buck module** (MP1584 / LM2596 mini) | The only way to run the MCX off the pack — its `VIN` pin is a dead end. §4 and §11 |

> **Your diodes are 1N4001, so the motor PWM runs at 2 kHz, not 20 kHz.** The ratings are
> fine (1 A, 50 V against 250 mA running and a 4.8 V pack) — it is the switching speed that
> sets the frequency. See §1.

Two spare PN2222A are still worth having — you have exactly four and no margin.

---

## 1. The drive stage

Four PN2222A as low-side switches, one per motor, driven in two groups of two. All four wheels
turn one direction only; **you steer on the duty difference between the sides**, and a side at
zero pivots the car.

That is the whole drive stage: four transistors, four diodes, eight resistors, no bridge and no
module.

### Sizing, because the PN2222A is a tight part

| | |
|---|---|
| PN2222A rating | **600 mA** continuous, 625 mW in TO-92 free air |
| One motor, running | ~250 mA — **2× margin** |
| One *pair* on one device | ~500 mA — no margin, do not do this |
| Base resistor | **470 Ω** → I<sub>B</sub> 8.7 mA, forced β = 29, safely saturated |
| Arduino pin load | 2 bases × 8.7 mA = **17 mA** — inside the comfortable limit |
| Dissipation, running | V<sub>CE(sat)</sub> ~0.4 V × 0.25 A = **100 mW**, no heatsink |

**One transistor per motor, not per pair.** A pair on a single 2222 sits at 500 mA against a
600 mA rating, and TT motors pull more than their free-running current once they are carrying a
car. Per-motor gives you the margin, and since each device has its own motor there is no
current-sharing problem between them.

**Two PWM pins, not four.** D9 drives the two left transistors through their own base
resistors, D10 the two right. Pin load is 2 × 8.7 mA = 17 mA, well inside what an Arduino pin
will give you.

> ### A stalled motor destroys the transistor
>
> Base current is fixed at 8.7 mA, so the transistor can pass at most
> h<sub>FE</sub> × 8.7 mA — roughly 435 mA with a typical part. It therefore **current-limits**
> on stall rather than passing the 800 mA the motor would take. That helps, but it is not
> protection: limiting means it leaves saturation, and it then burns **0.5 to 1.0 W in a package
> rated 0.625 W** depending on the individual device gain. Marginal to about 1.5× over — it will
> not die instantly, but it will not survive being held there. With no encoders you cannot
> detect a stall directly, so use what you have:
>
> - **Cap the duty cycle.** You do not need full speed for this demo.
> - **Watch the ultrasonic.** If range stops changing while you are commanding forward, you are
>   blocked — cut the motors.
> - **Watch the accelerometer.** A car that is really moving vibrates; a stalled one is still.
>   The MPU6050 gives you this for free.
>
> None of these is a real current limit. The rule that actually protects the parts is: never
> let the car sit pushing against something.

**No reverse.** A single transistor conducts one way, so the car drives forward and turns, but
it cannot back out of a dead end. Plan the course around that — and note that with two
channels you can always pivot away from an obstacle rather than reversing off it.

### Why the PWM is 2 kHz and not 20 kHz

The 1N4001 is a standard-recovery rectifier: after the transistor turns on, the diode keeps
conducting **backwards** for a couple of microseconds before it blocks. That is a current spike
straight through the PN2222A, once per PWM cycle.

| PWM | `ICR1` | Recovery loss in each transistor |
|---|---|---|
| 20 kHz | 400 | ~96 mW — **doubles its dissipation** |
| 8 kHz | 1000 | ~38 mW |
| **2 kHz** | **4000** | **~10 mW** |
| 490 Hz | 16327 | ~2 mW |

There is a second, better reason to go low. The motor's L/R time constant is roughly 330 µs, so
**below about 4 kHz the winding current decays to zero during the OFF time** — the diode is
already off when the transistor turns back on, and reverse recovery stops happening at all.

**2 kHz, `ICR1 = 4000`.** You will hear a whine from the motors; that is expected, not a fault.
At 2 kHz the torque ripple is far above the gearbox's mechanical bandwidth, so it does not
affect how the car drives. If a transistor ever runs warm, drop to 1 kHz (`ICR1 = 8000`) rather
than going up.

**The 0.4 V drop is real.** Out of a 4.8 V pack that is about 8 % of your voltage, so the
motors run slightly slower than the same pack would give a MOSFET. Not a problem, just
something to know when you calibrate speed.

## 2. How the car navigates without encoders

With no optocouplers there is no wheel feedback anywhere in the vehicle. That is a real
constraint, and it moves the control loop:

```
MPU6050 gyro ─> VCU: heading PID ──> (speed, yaw_rate) ──> ACT: two PWM duties
                                                                 (open loop, no feedback)
```

- **Heading is closed loop**, on the gyro, at the VCU. This is now the only feedback loop in
  the car — which makes the MPU6050 the single most important sensor you own.
- **Distance is open loop**: commanded speed × time, calibrated once by driving a measured
  two metres and adjusting the constant.
- **Wheel speed is open loop.** A motor at 60 % duty turns at whatever the load and the pack
  voltage allow. Expect the car to slow on carpet and to slow further as the cells sag.

This is honest dead reckoning and it is fine for the demo, because the ultrasonic and the gyro
are what actually drive the behaviour. It would not be fine for anything needing repeatable
position.

> **The upgrade path is already wired.** ACT's D2 and D3 (INT0/INT1) are deliberately left
> empty. If you ever buy two LM393 slot sensors, they drop straight in with no rewiring and no
> schema change — the encoder disks are already on the wheels.

---

## 3. Node allocation

| Node | Board | Job | Attached |
|---|---|---|---|
| **VCU** | FRDM-MCXA153 | Fusion, state machine, **heading PID**, safety supervisor, hub | MPU6050 IMU on its I²C bus, on-board RGB LED + SW2/SW3 |
| **PERC** | Arduino 1 | Perception | 2 × HC-SR04P (fixed, one front one back), HC-SR501 PIR, SFM-27 buzzer |
| **ACT** | Arduino 2 | Motor PWM, safe state, status readout | 4 × PN2222A → 4 DC motors, 1602A LCD |
| **GW** | ESP32 | WiFi gateway — the laptop's only route in | — |

Still genuinely four nodes, and not because of CPU load:

- **PERC is timing-sensitive.** An ultrasonic echo has to be measured to the microsecond, and
  two sensors alternating means the node is mid-measurement most of the time. The ranging is
  interrupt-driven so it no longer *blocks* — but it still owns INT0 and INT1, and it would be
  the first thing a control loop on the same chip interfered with.
- **ACT owns Timer1.** The 2 kHz motor PWM needs the only 16-bit timer on an ATmega328P, and
  nothing else on that board may touch it.
- **GW is bursty and unreliable.** WiFi retries must never be able to reach the wheels. This is
  the fault-containment argument, and it is the honest one.
- **VCU decides**, holds the only control loop, and supervises the other three.

**Two range sensors, both fixed — one forward, one rearward.** No servo, no sweep, and nine
times the update rate the swept single sensor managed. The front one is the only sensor in the
car that can override the driver; the back one is a readout, because there is no reverse for it
to protect. See §5.

**The IMU hangs off the MCX's I²C bus, not an Arduino.** `LPI2C0` already runs to the ESP32;
I²C is multi-drop, so the MPU6050 joins as a second slave for free. Two reasons: the heading
loop gets its sensor with no link in the path, and the module is 3.3 V native so it matches
the MCX exactly.

> ### The bus has its own pull-ups. Fit them.
>
> | | |
> |---|---|
> | **J5-5 SCL** → `[ 2 kΩ ]` → **J3-8** | 3.3 V, never 5 V |
> | **J5-6 SDA** → `[ 2 kΩ ]` → **J3-8** | 3.3 V, never 5 V |
>
> This schema used to say the opposite — that the MPU6050's on-board 2.2 kΩ were the bus
> pull-ups and no resistors were needed. That was true electrically and wrong as a design.
> It made a **sensor breakout a hard dependency for the WiFi gateway**: unplug the IMU, or
> have one of its four wires off, and SDA and SCL float, nothing can ACK, and the browser
> link dies for a reason nothing in the gateway can report. That is what a scan returning
> `nothing answered` means, and it cost a long evening to find.
>
> Two resistors make the bus stand on its own. The IMU's 2.2 kΩ then sit in parallel, giving
> about 1.05 kΩ and a 2.8 mA sink at V<sub>OL</sub> — inside the 3 mA the I²C spec allows, so
> leaving the module fitted is fine.
>
> **The MCU's internal pull-ups are not an option here.** `P3_27`/`P3_28` are the only 5 V
> tolerant pins on the chip and their weak internal pull-up will not hold a bus at rail —
> measured, it reached 2.3 V with nothing else attached. Enabling them is worth doing anyway
> (`i2c_pullups_init()` in `led_blinky.c`) so the lines idle high rather than float, which
> makes a meter reading and a bus scan mean something. It is not a substitute for the 2 kΩ.

---

---

## 4. VCU — FRDM-MCXA153

Every pin on this board has **three** names and they are all different. The connector pin is
what you count to, the `Pn_m` is what the datasheet calls it, and the silkscreen is what is
actually printed beside the header — that last one is the quickest way to find it with a wire
in your hand.

| Connector pin | MCU pin | Printed on the board | Function | Goes to |
|---|---|---|---|---|
| **J2-2** | `P3_15` | **D8** | LPUART2_TXD | PERC **D4** — direct |
| **J2-4** | `P3_14` | **D9** | LPUART2_RXD | PERC **D5** — **through the divider** |
| **J2-20** | `P1_9` | **D19** | LPUART1_TXD | ACT **D4** — direct |
| **J2-18** | `P1_8` | **D18** | LPUART1_RXD | ACT **D5** — **through the divider** |
| **J5-5** | `P3_27` | **SCL** on the mikroBUS socket | LPI2C0_SCL | ESP32 GPIO22 **+** MPU6050 SCL |
| **J5-6** | `P3_28` | **SDA** on the mikroBUS socket | LPI2C0_SDA | ESP32 GPIO21 **+** MPU6050 SDA |
| **J3-8** | — | **3V3** | `LDO_3V3` out | MPU6050 **VCC** — 3.3 V, not 5 V |
| **J3-12** *(or J3-14)* | — | **GND** | ground | ★ star point |
| **J3-10** | — | **5V** | `SYS_5V0` | **Buck module out, 5 V** — see §11 |

> ### Do not power this board from J3-16, whatever the silkscreen says
>
> `J3-16` is marked **VIN** and the net is `P5-9V_VIN`, so it looks like the obvious place for
> the 9 V pack. **It does nothing on a stock board.** UM12012 Rev 2.0 Table 7 traces it:
>
> ```
> J3-16  P5-9V_VIN (5–9 V)
>    └─> J22   ✗ 1×3-pin footprint for a 5 V regulator module — NOT POPULATED
>        └─> P5V_HDR_IN (5 V)
>            ✗ "third power source option (disabled by default)" for SYS_5V0
>            └─> SYS_5V0 → U2 → LDO_3V3 → MCU
> ```
>
> Two independent breaks, and the manual says so outright: *"By default, the option to produce
> the SYS_5V0 supply from the P5V_HDR_IN supply is disabled."* Nine volts on J3-16 feeds an
> empty footprint. The board will not come up and nothing is wrong with it.
>
> **So the 9 V goes through a buck module to 5 V and into `J3-10` (`SYS_5V0`) instead** — the
> rail both USB inputs already land on. §11 has the wiring.

> **`J2-2` means connector J2, pin 2.** J1–J4 are the Arduino-form headers and are laid out as
> two-row footprints, so count carefully — UM12012 Figures 11–12 show the numbering. If in
> doubt, find the silkscreen name instead: J2-2 is the pin marked **D8**.

> **A trap worth knowing.** `P1_8` / `P1_9` are the Arduino header's *I²C* pins — that is why
> they are silkscreened D18 and D19 next to SDA/SCL markings on many boards. **Here they are
> LPUART1**, not I²C. The real I²C is `P3_27`/`P3_28` on the mikroBUS socket.

On-board, nothing to wire (verified in `distributed_ecu/frdmmcxa153/frdmmcxa153/board.h`):
`P3_12` red LED, `P3_13` green, `P3_0` blue, `P3_29` SW2, `P1_7` SW3, `P0_2`/`P0_3` → VCOM.
This schema uses no J1 pins, so all of it is free for state indication.

---

## 5. PERC — Arduino 1

| Pin | Net | Component | Notes |
|---|---|---|---|
| D0 / D1 | — | — | **Leave free** — CH340 bootloader |
| D2 | ECHO_B | HC-SR04P **back** | **INT0** |
| D3 | ECHO_F | HC-SR04P **front** | **INT1** |
| **D4** | LINK_RX | ← MCX `P3_15` | Direct |
| **D5** | LINK_TX | → MCX `P3_14` | **Through the divider** |
| D6 | TRIG_B | HC-SR04P **back** | |
| D7 | TRIG_F | HC-SR04P **front** | |
| D8 | PIR OUT | HC-SR501 | Slow signal — poll it, no interrupt needed |
| D9 | BUZZER | SFM-27 | 220 Ω in series until you have measured it |
| D10–D13, A0–A5 | — | — | **Spare — ten pins**, both I²C among them |

### One forward, one rearward

**`D3`/`D7` is the front sensor and `D2`/`D6` is the back one.** That mapping exists in exactly
one place — the pin constants at the top of `perc.ino` — and nothing above PERC knows the pin
numbers. The VCU asks for "front" and "back"; move the sensors on the chassis and you edit two
lines in one file.

This replaced an earlier pair splayed 25° left and right. That arrangement gave a left/right
comparison for autonomous obstacle avoidance, which is the right answer for a car that steers
itself. It is the wrong answer for one a human drives: the driver already knows which way to
go, and what they cannot see is what is directly ahead and what is behind them. So one sensor
covers the direction of travel and the other covers the end you cannot watch.

Two consequences worth being straight about:

- **There is no left/right range comparison any more.** The autonomous avoidance logic that
  argument was built for is not in this build, and if it comes back it needs a third sensor or
  the splayed pair returned.
- **The back sensor cannot stop anything**, because the car has no reverse. It is a readout —
  how close you are to the wall behind you when you pivot — and nothing else acts on it.

The front sensor does act. The VCU refuses forward throttle under `GUARD_STOP_CM`, 25 cm by
default, and keeps allowing a pivot, because with no reverse turning away is the only escape.
See §5 of [`FIRMWARE.md`](FIRMWARE.md).

**They must fire alternately.** Two HC-SR04 pinging at the same moment hear each other's burst
and both report nonsense — and pointing them in opposite directions does not fix that, because
the burst reaches the other sensor through the chassis as much as through the air. Front, wait
60 ms, back, wait 60 ms.

| | Update rate |
|---|---|
| One sensor swept 120° on a servo | 0.9 Hz |
| **Two fixed, fired alternately** | **8.3 Hz each** |

**Nine times faster, with nothing moving.** That is why the servos came out: the sweep existed
only to compensate for having one sensor, and a second sensor does the job better. It also frees
`D6`/`D7`, releases Timer1 on this node, and takes the last load off the traction pack.

**Neither ping blocks.** `pulseIn()` would tie this node up for up to 10 ms per reading; the
echo is timed on INT0/INT1 instead, which is what those two pins were held open for. Not
blocking is the whole reason PERC is a separate node, so do not undo it.

Power the HC-SR04P from the Arduino's **5 V**. The "P" variant's 3.3–5 V range earns you nothing
on a 5 V node — it would only matter if you moved one onto the MCX.

### The SFM-27 buzzer

Identify it before you rely on it. There are two kinds and they need opposite drive:

| Drive | Active buzzer | Passive buzzer |
|---|---|---|
| Steady DC (`digitalWrite HIGH`) | Buzzes at its own fixed pitch | One click, then silence |
| 2 kHz square (`tone()`) | Warbles or rattles | Clean 2 kHz tone |

`test_perc.ino` has both as `b1` and `b2` — whichever sounds right tells you which you have.

**Fit a 220 Ω in series until you have measured the current.** A magnetic buzzer can pull
30–40 mA, which is over what an AVR pin should source; the resistor costs volume and protects
the pin. If it turns out to draw under 20 mA, take it out.

Use it for the **safe state**, not for every obstacle. An alarm that is always sounding is an
alarm nobody listens to.

---

## 6. ACT — Arduino 2

| Pin | Net | Component | Notes |
|---|---|---|---|
| D0 / D1 | — | — | **Leave free** — bootloader |
| D2 / D3 | — | — | **Held open for encoders** (INT0/INT1) |
| **D4** | LINK_RX | ← MCX `P1_9` | Direct |
| **D5** | LINK_TX | → MCX `P1_8` | **Through the divider** |
| **D9** | PWM_L | Bases of the two left transistors | **Timer1 OC1A — 2 kHz** |
| **D10** | PWM_R | Bases of the two right transistors | **Timer1 OC1B — 2 kHz** |
| D6 / D7 | RS / E | **1602A** | 4-bit mode |
| D8 / D11 / D12 / A2 | DB4 / DB5 / DB6 / DB7 | **1602A** | DB0–DB3 stay unconnected |
| D13, A0, A1, A3, A4, A5 | — | — | Spare — A4/A5 stay free, so I²C is still available later |

For 2 kHz on D9/D10: phase-correct PWM, prescaler 1, `ICR1 = 16e6 / (2 × 2000) = 4000`.
Not `analogWrite()` — its 490 Hz is fixed and you want the 16-bit resolution around low duty.

The `(speed, yaw_rate)` command from the VCU maps to two duties:

```
duty_L = k * (speed - yaw_rate * track/2)
duty_R = k * (speed + yaw_rate * track/2)
```

Both clamped to `[0, duty_max]`. There is no negative side — that is the forward-only
constraint showing up in the maths. A hard turn is one side at `duty_max` and the other near
zero; that is your pivot.

**The safe state is in hardware.** Duty 0 stops the motors, and the 10 kΩ base pull-downs hold
all four transistors off while the Arduino is in reset or has crashed. That is a genuine
hardware off-state, and it is the main thing the 10 kΩ resistors buy you.

---

## 7. Wiring one drive channel

Build this four times. Two share D9, two share D10.

```
                    PACK +
                      |
          +-----------+-----------+
          |                       |
      [ motor ]              [ 1N4001 ]      band (cathode) to PACK +
          |                       |          anode to the collector
          +-----------+-----------+
                      |
                      C
   D9 --[470R]--+--- B  PN2222A
                |     E
             [10k]    |
                |     |
               GND   GND  -> star point, thick wire
```

### How the switch actually works

The transistor is a **low-side (common-emitter) switch**: emitter to ground, collector to the
motor's negative terminal, motor's positive terminal to PACK +. Current flows
`PACK + → motor → collector → emitter → star ground`, so the transistor sits *in the return
path*. That is what "low side" means, and it is why all four emitters want thick wire.

**Why the emitter has to be the grounded end.** An NPN conducts when its base is ~0.7 V above
its *emitter*. With the emitter at 0 V the base needs 0.7 V, and an Arduino pin gives 5 V —
easy. Put the transistor on the high side instead and its emitter would sit near 4.4 V, so the
base would need ~5.1 V, which is more than the pack itself. That is why a high-side NPN cannot
work without a charge pump, and why every simple transistor motor switch is low-side.

**It is a switch, not an amplifier — and that is deliberate.** A BJT in its active region gives
`I_C = β × I_B`. Here `I_B = (5 − 0.9) / 470 = 8.7 mA`, so with a real h<sub>FE</sub> of ~50 the
transistor *could* pass 436 mA. The motor only draws 250 mA, so it cannot use all that gain —
it runs out of collector current and collapses into **saturation**, where V<sub>CE</sub> falls to
~0.35 V and it behaves like a closed switch dissipating about 90 mW.

The ratio that matters is the **forced β**: `250 mA / 8.7 mA = 29`. As long as that is
comfortably below the real h<sub>FE</sub>, the transistor is hard on. Under-drive the base and it
stops being a switch:

| Base resistor | I<sub>B</sub> | Can pass | Result |
|---|---|---|---|
| **470 Ω** | 8.7 mA | 436 mA | Saturated. V<sub>CE</sub> 0.35 V, **90 mW** |
| 2.2 kΩ | 1.9 mA | 93 mA | Not saturated — acts as a resistor. V<sub>CE</sub> ~3 V, **750 mW**, and the motor only sees 1.8 V |

That second row is the failure mode to understand: a too-large base resistor does not make the
motor "a bit slower", it makes the transistor a heater. Over-driving the base is free; under-
driving it is what cooks TO-92 parts.

**The base resistor is not a precision part.** Anything from about **410 Ω to 570 Ω** works —
below that an Arduino pin driving two bases exceeds 20 mA, above it the forced β climbs out of
saturation. **480 Ω is fine**: I<sub>B</sub> 8.5 mA instead of 8.7 mA, forced β 29.3 instead of
28.7. Nothing in this circuit can tell them apart.

PWM does not change any of this — the transistor is only ever fully on or fully off, 2000 times
a second, and duty cycle sets speed.

> **Check the pinout before you solder.** The usual PN2222A TO-92 arrangement is **E–B–C** left
> to right with the flat face toward you and the leads down — but parts sold as "2N2222" in
> TO-92 are sometimes C–B–E. Confirm with a multimeter's diode test: from the **base**, both
> other pins read like a forward diode (~0.7 V); the pin with the *slightly higher* reading is
> the emitter.

Four things that matter, in order of how badly they bite:

1. **The diode's band goes to PACK +.** Backwards, it is a dead short across the pack the
   moment you power up.
2. **Motor − goes to the collector, never straight to ground.** The transistor is *in* the
   return path; that is what "low side" means.
3. **The 10 kΩ goes base-to-emitter**, not base-to-supply. It holds the transistor off when the
   Arduino pin is floating — during reset, during upload, and if the sketch crashes.
4. **All four emitters go to the star point in thick wire.** On a low-side switch the full
   motor current returns through the ground net, so give it its own thick run rather than
   sharing a thin wire with the signal grounds.

Test one channel on the bench before you build the other three. A single miswired diode is
cheap to find at that point and expensive to find with four of them installed.

---

## 7b. The 1602A display

It is a **bare HD44780**, not an I²C backpack — so it costs six Arduino pins instead of two,
and it needs three things that a backpack would have handled for you.

| Pin | Goes to | Note |
|---|---|---|
| 1 **VSS** | ★ | |
| 2 **VDD** | ACT 5 V | |
| 3 **V0** | **10 kΩ trimpot wiper** | Contrast. Pot ends to +5 V and ★ |
| 4 **RS** | ACT **D6** | |
| 5 **RW** | **★** | Write-only. Leave it floating and the display drives the data bus back at you |
| 6 **E** | ACT **D7** | |
| 7–10 **DB0–DB3** | *unconnected* | 4-bit mode |
| 11–14 **DB4–DB7** | ACT **D8, D11, D12, A2** | |
| 15 **A** | +5 V **through 220 Ω** | Backlight anode |
| 16 **K** | ★ | Backlight cathode |

**The contrast pot is not optional.** With V0 floating or tied to a rail the display is either
blank or a row of solid blocks, and it looks exactly like a dead display or a wiring fault.
This is the single most common way a first 1602A build "fails".

**Check the backlight resistor before you connect A.** Many modules carry one on board (often
marked R8); many do not, and connecting the LED straight to 5 V kills it. Measure between pin
15 and the 5 V pin with the module unpowered — a few tens of ohms means it is fitted, open
circuit means fit your own 220 Ω.

Use `LiquidCrystal lcd(6, 7, 8, 11, 12, A2);` — that is `(RS, E, DB4, DB5, DB6, DB7)`, and it
is in the core, no library to install.

---

## 8. GW — the ESP32 gateway

Four wires. I²C to the VCU on **GPIO21 (SDA) / GPIO22 (SCL)**, slave `0x42`, 400 kHz, **each
through a 470 Ω series resistor at the ESP32 end**; **VIN**
from the 9 V logic pack directly (see §11 — check the regulator, and point the 5 V fan at it)
with a 470 µF beside it;
**GND** to the star point. No sensors, no
display — this node is the route between the laptop and the VCU and nothing else.

**The two 470 Ω are latch-up protection, and they are not optional either.** The bus idles at
3.3 V through the 2 kΩ pull-ups. Whenever the ESP32 is unpowered while the MCX is not — one USB
cable out before the other, on the bench — that current flows into GPIO21/22's clamp diodes and
back-feeds the whole chip through two I/O pins. Re-powering it in that state latches it up, and a
latched ESP32 is dead: 3V3 and EN read fine, GPIO0 sits near 0.5 V, the ROM never answers, no
access point appears. Two boards died that way on this project. With 470 Ω in the line the
injected current stays under 2 mA, below the latch-up threshold, and 400 kHz still passes
cleanly (the RC with the 2 kΩ pull-up and ~50 pF of bus is well under the rise-time budget).

**Plug order, every time:**
1. I²C wires **off** the ESP32 before either board's USB comes out.
2. USB **back in** on both boards before the wires go back on.
3. On the car, both boards on the **same 9 V pack** so they come up together.

**It is a gateway, not a controller.** Everything the laptop sends arrives at the VCU as a
*request*, and the VCU decides what to act on. That is what stops a dropped WiFi link being a
safety problem, and it is what makes the fault-injection demo worth watching: you can pull this
node out mid-run and the car keeps driving, blind to the laptop but not to the world.

Nothing else hangs off it. **Roughly 30 GPIO are unused** — by a wide margin the emptiest
board in the project, so if anything later needs pins, this is where they are.

Pins to avoid on any ESP32 DevKit: **GPIO6–11** (wired to the SPI flash — using one stops the
board booting), **GPIO34/35/36/39** (input only, they cannot drive anything), and
**GPIO0/2/5/12/15** (strapping pins that decide how the chip boots; GPIO12 pulled high at
reset sets the flash to 1.8 V and the board will not start).

---

## 9. The parts you still have to buy

**The 1 kΩ / 2 kΩ dividers.** The MCX is 3.3 V and **only `P3_27`/`P3_28` tolerate 5 V**. Both
Arduino TX lines run into `P3_14` and `P1_8`, which do not. Without dividing them you destroy
those pins, and the hub cannot exist without those two lines.

```
Arduino D5 ----[ 1 kOhm ]----+---- MCX RX
                             |
                        [ 2 kOhm ]      5 V x 2/(1+2) = 3.33 V
                             |
                            GND
```

The MCX→Arduino direction is plain wire: 3.3 V clears the AVR's 3.0 V V<sub>IH</sub>.

**The base resistors and pull-downs.** 4 × 470 Ω and 4 × 10 kΩ. See §7.

**The ESP32 latch-up resistors.** 2 × 470 Ω more, one in each I²C wire at the ESP32 end. See §8.

**The LCD contrast trimpot**, 10 kΩ, and a 220 Ω for its backlight if the module has no
on-board resistor. See §7b.

**The flyback diodes are done — 1N4001, four of them, already in hand.** Ratings are
comfortable: 1 A against 250 mA running and 800 mA at stall, 50 V against a 4.8 V pack. The
only consequence is the PWM frequency, covered in §1.

---

## 10. Assumptions to check before wiring

| Item | Assumed | How to check | If wrong |
|---|---|---|---|
| ~~SFM-27~~ | **Resolved: a buzzer on D9** | `b1` / `b2` in test_perc.ino | Active vs passive needs opposite drive — see §5 |
| ~~16×2 LCD~~ | **Resolved: it is a 1602A**, bare HD44780 | — | Six pins on ACT, plus a contrast pot — see §7b |
| ~~HW-123~~ | **Resolved: it is an MPU6050** | — | HW-123 is a common silkscreen on GY-521 MPU6050 boards. `WHO_AM_I` (reg `0x75`) reads `0x68` |
| **Battery holders** | 4 × AA in series | Count the cells | Two 4-cell packs in series = 12 V, which cooks the TT motors and cooks the buck module's output setting along with them |

---

## 11. Power — three supplies, one star point

Detail on [`SCHEMA_POWER.svg`](SCHEMA_POWER.svg).

```
TRACTION PACK  4 × AA NiMH 4.8 V ────> 4 × DC motor, each via its PN2222A

LOGIC PACK     9 V ................ ┬──> BUCK 9->5 V ──> MCX J3-10 (SYS_5V0)
                                    │                   NOT J3-16 - see below
                                    ├──> ESP32 VIN              direct, fan on the regulator
                                    └──> 12 V fan               runs at ~70 % on 9 V

TRACTION PACK ...................... └──> 5 V fan                runs at ~96 % on 4.8 V

USB 5 V        laptop / power bank ─┬──> PERC Arduino
                                    └──> ACT Arduino

     ALL THREE NEGATIVES ══════ STAR POINT      <-- the only thing that must be shared
```

### Linking boards that run on different supplies

**Independent supplies are fine. A shared ground is not optional.** A UART line is a voltage
*relative to ground*; if two boards do not agree on where zero is, the receiver sees noise, or
current finds a path back through the signal wires. So: traction pack −, logic pack −, USB
ground, all four board grounds, all four transistor emitters and every component GND meet at
one point.

**Nothing else changes.** Logic levels do not depend on which supply feeds a board — an
Arduino still drives 5 V, the MCX still needs 3.3 V — so **both 1 kΩ / 2 kΩ dividers stay
exactly as they are.**

### Do not run the ESP32 from an Arduino 3.3 V pin

| An Arduino 3.3 V pin gives | | An ESP32 needs | |
|---|---|---|---|
| Uno R3 (LP2985) | 150 mA | idle | 40 mA |
| Nano (FT232RL) | 50 mA | WiFi average | **100 mA** |
| CH340 clone | 30 mA | WiFi TX burst | **500 mA** |

Even the *average* exceeds what a CH340 board's 3.3 V pin can give. This is not marginal — the
ESP32 would brown out and reset in a loop, and you would be cooking the Arduino's regulator
while it happened.

**Feed it from the logic pack at VIN instead**, with a 470 µF close to the board to absorb the
transmit bursts. Its on-board AMS1117 makes the 3.3 V. That also puts the ESP32 on the same
supply as the MCX — which is tidy, because they are the two ends of the same I²C link.

*(If you would rather not add the pack yet: an Arduino's **5 V** pin into ESP32 **VIN** does
work, with the same 470 µF. You are then limited by that Arduino's USB port.)*

### Feeding the ESP32 from 9 V

**Two things to settle before the pack goes anywhere near the board.**

#### 1. Check which regulator your DevKit has

`VIN` goes to an on-board linear regulator, and not all DevKits fit the same one:

| What you see next to the USB socket | Part | Max input | 9 V? |
|---|---|---|---|
| Fat 4-pin tab package (SOT-223) | **AMS1117-3.3** | **15 V** | Fine |
| Tiny 5-pin (SOT-23-5) | ME6211, RT9013 and friends | **~6 V** | **Destroys it** |

Most ESP32 DevKit V1 boards use the AMS1117. **Look before you connect the pack** — this is not
a recoverable mistake.

#### 2. A divider is not the answer, and it is worth knowing why

It is the obvious-looking fix and it does not work. A divider sets its output by *ratio*, and
the ratio only holds while nothing draws from it. Put a load on the tap and it sits in parallel
with the lower resistor and the voltage collapses. Sized for 6 V (`R1 = R2/2`), the tap behaves
like a 6 V source behind `R2/3`:

| R2 | R1 | Idle current | Burnt idle | V at 100 mA | V at 500 mA |
|---|---|---|---|---|---|
| 3 kΩ | 1.5 kΩ | 2 mA | 0.02 W | **0 V** | **0 V** |
| 300 Ω | 150 Ω | 20 mA | 0.18 W | **0 V** | **0 V** |
| 30 Ω | 15 Ω | 200 mA | 1.8 W | 5.0 V | **1.0 V** |
| 15 Ω | 7.5 Ω | 400 mA | **3.6 W** | 5.5 V | **3.5 V** |

The ESP32 pulls ~100 mA average and ~500 mA in a WiFi burst, and needs about **4.4 V** at VIN.
So the only divider that even boots it wastes **400 mA continuously into 3.6 W of resistors**
and *still* resets the board on every transmit.

**Four 1N4001 in series would work** — a diode's drop moves only logarithmically with current,
0.70 V at 100 mA and 0.85 V at 500 mA, giving 6.2 V idle and 5.6 V under burst. Worth doing if
you ever buy four spare. The four you have are committed to motor flyback.

#### What we do instead: 9 V direct, and cool it

9 V on VIN is inside the AMS1117's rating. The only real objection is heat — and you have fans.

| VIN | Regulator dissipation | Still air | With the 5 V fan on it |
|---|---|---|---|
| 6.2 V (via diodes) | 0.32 W | +32 °C | +14 °C |
| **9 V direct** | **0.68 W** | +68 °C — hot | **+31 °C — fine** |

Moving air roughly halves the rise on a SOT-223, which takes 9 V direct from *marginal* to
*comfortable* and costs nothing you do not already own.

### The two fans

They are not decoration. **They fix the one genuinely marginal thermal spot in the build.**

| Fan | Supply | Aim it at | Why |
|---|---|---|---|
| **12 V type** | **Logic pack +, 9 V** | The four PN2222A | A stalled transistor puts 0.5–1.0 W into a TO-92 rated 0.625 W |
| **5 V type** | **Traction pack +, 4.8 V** | The ESP32's regulator | Lets VIN take 9 V without cooking, as above |

```
PN2222A, stalled   0.75 W in TO-92    still air  +150 °C      forced air  +68 °C
AMS1117 at 9 V     0.68 W in SOT-223  still air   +68 °C      forced air  +31 °C
```

That +150 °C is the number that matters. The drive stage has always been the weakest thermal
point here — §7 — and the 12 V fan is what turns it from *do not hold it stalled* into
*comfortable*.

**Neither fan runs at its rated voltage, and neither cares.** A 12 V brushless fan on 9 V turns
at roughly 70 % — quieter, still plenty of air. A 5 V fan on 4.8 V is barely slower.

> **Wire both permanently on.** Switching an inductive load needs a flyback diode across it, and
> there are none spare. Permanently-on needs nothing.

> **Take the fan supplies from the pack terminals**, not by daisy-chaining through a board's
> supply pin. Brushless fans put commutation spikes on their supply wires, and you do not want
> those arriving at the MCX's VIN. Keep the fan wiring away from the UART and I²C runs too.

### Supply windows — what the input can be, per board

| Board | Input | Absolute | Use | Why that ceiling |
|---|---|---|---|---|
| **FRDM-MCXA153** | `J3-10` SYS_5V0 | 5 V ±5 % | **5.0 V** | **This is the one.** Regulated 5 V only, no headroom — that is what the buck module is for |
| | `J3-16` P5-9V_VIN | — | **never** | Dead end: J22 not populated, path to SYS_5V0 disabled by default. See §4 |
| | MCU-Link USB | 5 V | 5 V | The only input that also gives you VCOM |
| **Arduino Uno / Nano** | VIN pin or barrel | 6 – 20 V | **7 – 9 V** | Below 7 V the 5 V rail sags on regulator dropout |
| | 5V pin | 4.8 – 5.2 V | 5.0 V | Bypasses the regulator — never with USB as well |
| | USB | 5 V | 5 V | What this design uses |
| **ESP32 DevKit** | VIN / 5V pin | 4.75 – 12 V | **5 – 9 V** | AMS1117 heat, not the datasheet, sets this. 9 V is fine **with the fan on it** |
| | 3V3 pin | 3.0 – 3.6 V | 3.3 V | Bypasses AMS1117 — needs a solid 500 mA |
| | micro-USB | 5 V | 5 V | Fine, but then it is a fourth cable |

**The upper limit is thermal, not electrical.** A linear regulator burns
`(Vin − Vout) × I` as heat, so what the datasheet allows and what the board survives are
different numbers. For the ESP32's AMS1117-3.3 at ~120 mA WiFi average:

| VIN | Dissipation | Junction rise (SOT-223, ~100 °C/W) |
|---|---|---|
| 5 V | 0.20 W | +20 °C |
| **6 V** | **0.32 W** | **+32 °C** — fine |
| 9 V | 0.68 W | +68 °C — hot |
| 12 V | 1.04 W | +104 °C — thermal shutdown |

Transmit bursts are far worse instantaneously (1.35 W at 6 V) but last milliseconds, so they
do not set the junction temperature; the average does. That is also what the 470 µF is for.

> **The 9 V logic pack feeds the MCX and the ESP32 directly.** The Arduinos stay on USB; if you
> ever want them off a pack too, take them from the same 9 V — their regulators want 7 V or
> more, so 9 V suits them and 6 V would not.

### Two packs, two chemistries

This is the non-obvious part — the packs want **opposite** cells:

| Pack | Cells | Why |
|---|---|---|
| **Traction** | 4 × AA **NiMH** (4.8 V) | Either chemistry works now the servos are gone — the TT motors take 3–6 V. NiMH still holds its voltage under a stall where alkalines sag |
| **Logic** | **9 V** (6 × AA alkaline preferred) | Top of the MCX 5–9 V window, taken directly; the ESP32 takes the same 9 V with a fan on its regulator. Also runs the 12 V fan. **Not a PP3 block** — too little capacity and too much internal resistance |

Same holder, different cells. Label them.

### The MCX needs a buck module, and 9 V direct will not do

The FRDM's own `VIN` pin is not wired to anything that works (§4). So the 9 V pack reaches the
MCX through a **9 V → 5 V step-down module** — an MP1584 or LM2596 mini board, whatever the kit
has — and its output goes to `J3-10`:

| Buck module | To |
|---|---|
| IN + | **Logic pack +**, 9 V |
| IN − | ★ star point |
| **OUT +** | MCX **J3-10** (`SYS_5V0`) |
| OUT − | ★ star point |

**Set the output to 5.0 V with the board disconnected, and measure it.** Most of these modules
ship with the trimpot wherever it landed, and `SYS_5V0` is a 5 V ±5 % rail with no headroom —
it goes straight into the LDO that feeds a 3.3 V MCU. Anything much over 5.25 V is out of
spec, and a module wound up to 12 V from a previous project will destroy the board on contact.
A linear 7805 works too: the board draws well under 200 mA, so about 0.8 W in a TO-220, warm
but survivable and one less thing to set wrong.

> ### Never both at once
>
> `SYS_5V0` is the same rail the two USB connectors feed. Putting the buck module on it while
> a USB cable is plugged in back-feeds one source into the other, and that is how boards and
> laptop USB ports die. **USB in, pack off. Pack on, USB out.** No exceptions, and a switch on
> the pack is worth fitting for exactly this reason.

### What the MCX pack costs you

Running the MCX from the pack instead of MCU-Link USB means **no VCOM**: no console, no
FreeMASTER, no debugger. With no cable in J15 the LPC55S69 that provides them is unpowered.
During development keep the USB cable in and the pack switched off; the pack is for the
untethered demo run, when the browser is your only interface anyway.

Wire the pack now anyway — the ESP32 needs it for that run.

### Power-up order

Logic pack first, or everything together. A powered Arduino driving an unpowered MCX pushes
current through the MCX's ESD diodes; the 1 kΩ series resistor in each divider limits it to a
harmless few mA, which is a second reason those dividers are not optional.

---

## 12. Bring-up order

```
[ ] Tie the pack negative to board ground. Verify continuity across all four boards
[ ] UART links + both dividers, both directions            <- you already have this working
[ ] I2C scan from the MCX: expect 0x42 (ESP32) and 0x68 (MPU6050)
[ ] MPU6050: confirm WHO_AM_I (reg 0x75) reads 0x68 before writing driver code
[ ] Build ONE drive channel on a breadboard. Check the diode band before power
[ ] That channel: duty 0, 30 %, 100 %, then pull the Arduino reset with the motor
    spinning and confirm it stops and stays stopped. Only then build the other three
[ ] All four channels, WHEELS OFF THE FLOOR. Watch for a warm transistor
[ ] Link timeout: pull the ACT cable, confirm all four motors stop
[ ] Both sonars: one at a time, then alternating. Watch for cross-talk
[ ] PIR — give it a full minute to settle before you debug it
[ ] LCD alone: banner, then the contrast pot until it is readable
[ ] SFM-27: identify it. Only then wire it
[ ] Gyro heading hold on blocks, then on the floor
[ ] Gateway: I2C reads climbing on the ESP32, browser reaches 192.168.4.1
```

---

## Appendix A — Complete netlist

Every wire in the vehicle. Tick them off as you go. `★` = star ground (§11).

### A.1 Inter-board links

| From | | To | Wire |
|---|---|---|---|
| MCX **J2-2** | `P3_15`, marked **D8** | PERC **D4** | Direct |
| PERC **D5** | | 1 kΩ → MCX **J2-4** (`P3_14`, marked **D9**); 2 kΩ from the junction to ★ | **Divider** |
| MCX **J2-20** | `P1_9`, marked **D19** | ACT **D4** | Direct |
| ACT **D5** | | 1 kΩ → MCX **J2-18** (`P1_8`, marked **D18**); 2 kΩ from the junction to ★ | **Divider** |
| MCX **J5-5 SCL** | `P3_27`, mikroBUS | **470 Ω** → ESP32 **GPIO22**; MPU6050 **SCL** direct | Shared, 3.3 V. **Series R at the ESP32 end** — see §8 |
| MCX **J5-6 SDA** | `P3_28`, mikroBUS | **470 Ω** → ESP32 **GPIO21**; MPU6050 **SDA** direct | Shared, 3.3 V. **Series R at the ESP32 end** — see §8 |
| **J5-5 SCL** | | **2 kΩ** → MCX **J3-8** (3V3) | **Pull-up — not optional** |
| **J5-6 SDA** | | **2 kΩ** → MCX **J3-8** (3V3) | **Pull-up — not optional** |
| MCX **J3-12** or **J3-14** | marked **GND** | ★ | |

> **The I²C pins are on J5, not J6.** An earlier revision of this appendix said the
> opposite, and warned you off J5 — that was wrong. NXP's own LPI2C example readme for this
> board puts **SDA on J5 pin 6, SCL on J5 pin 5, GND on J5 pin 8**. J5 is the mikroBUS half
> that carries PWM · INT · RX · TX · SCL · SDA · 5V · GND, in that order from pin 1. J6 is the
> other half (AN · RST · CS · SCK · MISO · MOSI · 3V3 · GND): its pins 5 and 6 are `P1_2`/`P1_0`,
> unconfigured SPI pins, and a meter on them reads a floating couple of volts that looks
> exactly like a half-pulled-up bus. Go by the **silkscreen**, which prints `SCL` and `SDA`
> beside the socket, and confirm with continuity to the ESP32 end.
>
> One more trap on the same header: **J5 pins 3 and 4 (RX / TX) are `P3_14` / `P3_15`** — the
> same nets as J2-4 / J2-2, i.e. the PERC link. Count J5 from the wrong end and "pins 5 and 6"
> become pins 4 and 3: the ESP32 lands on PERC's UART and takes both links down at once.
>
> **A bus that idles at about 2.3 V on both lines is not a missing pull-up.** A missing
> pull-up floats; 2.3 V on both lines at once, steady, is a 3.3 V pull-up minus a diode drop —
> the signature of a device on the bus whose VDD is off or whose ground is not the star
> point. Its input clamp diodes conduct the pull-up current into its dead rail. Measure the
> ESP32's 3V3 pin and the MPU6050's VCC against the **MCX** ground: 3.3 V, or that is the
> fault. A dev board fed 9 V into a SOT-23-5 regulator (§11) fails exactly this way.

### A.2 VCU — FRDM-MCXA153 and the MPU6050

| From | | To |
|---|---|---|
| MCX **J3-8** | marked **3V3**, `LDO_3V3` | MPU6050 **VCC** — 3.3 V, **not** 5 V |
| MPU6050 **GND** | | ★ |
| MPU6050 **AD0** | | ★ — this is what selects address 0x68 |
| MPU6050 **XDA**, **XCL**, **INT** | | *leave unconnected* |
| MPU6050 **SDA** / **SCL** | | the shared bus — see A.1. Its own 2.2 kΩ now parallel the fitted 2 kΩ, which is fine |
| MCX **MCU-Link USB** | the micro-B socket | Laptop — VCOM, debug, and the board's 5 V during development |
| MCX **J3-10** | marked **5V**, `SYS_5V0` | **BUCK MODULE OUT +**, set to 5.0 V and measured — for the untethered run. **Never with a USB cable also in the board** |
| MCX **J3-16** | marked **VIN**, `P5-9V_VIN` | **nothing.** Dead end on a stock board — §4 |

The MPU6050's pin names are printed on the module itself, so that side needs no decoding —
it is only the MCX end that has three names for everything.

### A.3 PERC — Arduino 1

| From | To |
|---|---|
| PERC **D3** | HC-SR04P **front ECHO** |
| PERC **D7** | HC-SR04P **front TRIG** |
| PERC **D2** | HC-SR04P **back ECHO** |
| PERC **D6** | HC-SR04P **back TRIG** |
| PERC **D8** | HC-SR501 **OUT** |
| PERC **D9** | 220 Ω → SFM-27 **IN** |
| PERC **5V** | both HC-SR04P **VCC**, HC-SR501 **VCC** |
| PERC **GND**, both HC-SR04P **GND**, HC-SR501 **GND**, SFM-27 **GND** | ★ |

Nothing on this node touches the traction pack. With the servos gone it is entirely USB-powered.

### A.4 ACT — Arduino 2

| From | To |
|---|---|
| ACT **D9** | 470 Ω → **Q1 base**, and a second 470 Ω → **Q2 base** (left pair) |
| ACT **D10** | 470 Ω → **Q3 base**, and a second 470 Ω → **Q4 base** (right pair) |
| ACT **D6** | 1602A **RS** (pin 4) |
| ACT **D7** | 1602A **E** (pin 6) |
| ACT **D8** | 1602A **DB4** (pin 11) |
| ACT **D11** | 1602A **DB5** (pin 12) |
| ACT **D12** | 1602A **DB6** (pin 13) |
| ACT **A2** | 1602A **DB7** (pin 14) |
| ACT **5V** | 1602A **VDD** (pin 2), trimpot end 1 |
| ACT **5V** | 220 Ω → 1602A **A** (pin 15) *— skip if the module has its own* |
| 1602A **V0** (pin 3) | **Trimpot wiper** |
| ACT **GND** | ★ |
| 1602A **VSS** (1), **RW** (5), **K** (16), trimpot end 3 | ★ |
| 1602A **DB0–DB3** (7–10) | *unconnected* |

### A.5 Drive — build four times, Q1…Q4

| From | To |
|---|---|
| **PACK +** | Motor *n* terminal 1 |
| Motor *n* terminal 2 | **Q*n* collector** |
| **Q*n* collector** | 1N4001 **anode** |
| 1N4001 **cathode** (banded end) | **PACK +** |
| **Q*n* base** | 470 Ω from ACT D9 or D10 (see A.4) |
| **Q*n* base** | 10 kΩ to **Q*n* emitter** |
| **Q*n* emitter** | ★ — thick wire |

Q1, Q2 = left pair (from D9). Q3, Q4 = right pair (from D10).

### A.6 GW — ESP32

| From | To |
|---|---|
| ESP32 **GPIO22** | **470 Ω** → MCX **J5-5 SCL** — resistor right at the ESP32 pin, see A.1 and §8 |
| ESP32 **GPIO21** | **470 Ω** → MCX **J5-6 SDA** — resistor right at the ESP32 pin, see A.1 and §8 |
| ESP32 **VIN** | **LOGIC PACK +** (9 V) directly. **Not** the 3V3 pin, not an Arduino, and **not through a divider** — see §11. Confirm the board has an AMS1117 first, and put the 5 V fan on it |
| ESP32 **VIN** ↔ **GND** | 470 µF, as close to the board as you can get it |
| ESP32 **GND** | ★ |

### A.7 Power

| From | To |
|---|---|
| **Traction pack +** (NiMH 4.8 V) | 4 × motor terminal 1, 4 × diode cathode, **5 V fan +** |
| **Logic pack +** (9 V) | **Buck module IN +**, ESP32 **VIN**, **12 V fan +** |
| **Buck module OUT +** (5.0 V) | MCX **J3-10** — never while a USB cable is in the MCX |
| **USB 5 V** | PERC and ACT, one cable each |
| **Traction pack −** | ★ |
| **Logic pack −** | ★ |
| **USB ground** (via the boards) | ★ |
| ★ | MCX GND, PERC GND, ACT GND, ESP32 GND, 4 × transistor emitter, **both fan −**, every component GND |

Three supplies, one reference. Miss the star point and the UART links and I²C have no common
zero — which looks exactly like a firmware bug.

---

## 13. Sources

- [UM12012 FRDM-MCXA153 Board User Manual Rev. 2.0](https://akizukidenshi.com/goodsaffix/FRDM-MCXA153_User%20Manual.pdf) — pinouts, Figures 11–12 and Table 14
- `distributed_ecu/frdmmcxa153/frdmmcxa153/board.h` — LED and switch pins, verified in-tree
- [`arduino_codes/esp32_gateway/esp32_gateway.ino`](arduino_codes/esp32_gateway/esp32_gateway.ino) — I²C address, chunk size, AP config
- [`arduino_codes/serial_bridge/serial_bridge.ino`](arduino_codes/serial_bridge/serial_bridge.ino) — D4/D5 link pins, 38400
- [`SERIAL_BRIDGE.md`](SERIAL_BRIDGE.md) — the working 3-board hub this builds on
