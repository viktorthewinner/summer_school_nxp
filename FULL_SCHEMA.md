# Full System Schema — Boards and Components

**Distributed-ECU car · NXP Summer School · built only from parts in the box**

| Sheet | File | Covers |
|---|---|---|
| 1 — Signals | [`SCHEMA_SIGNALS.svg`](SCHEMA_SIGNALS.svg) | Every board, every component, every pin |
| 2 — Power | [`SCHEMA_POWER.svg`](SCHEMA_POWER.svg) | Traction pack, USB logic, star ground |

Anything drawn with a **dashed red border** is not in your box. There are only three such
items and they are listed in §7.

---

## 1. Two things that change the project

**You have no line sensor.** Not a QTR array, not a TCRT5000, nothing. Line following is
therefore only possible through the OV7670 — which is the single highest-risk part you own.
So the demo has to change. What this parts list *is* good at:

> **An obstacle-avoiding rover with heading hold.** Drive straight on IMU yaw + encoder
> odometry, sweep the ultrasonic across the path, steer or stop for what it finds, and halt
> when the PIR sees a person. Then inject the fault: unplug PERC mid-run, watch the VCU lose
> ranging and degrade to a controlled stop instead of driving into a wall.

That keeps the thing that actually makes this an interesting project — distributed nodes,
fault containment, safe degradation — and drops the one capability the hardware cannot do.
If the camera works, line following comes back as a bonus, not as the foundation.

**CAN is off the table, permanently.** Yes, the ESP32 has a native CAN controller (TWAI).
It still needs an external transceiver, you have none, and the MCXA153 has no CAN peripheral
at all. Two of your four nodes could never join the bus. The transport stays UART + I²C —
which is what your firmware already implements.

---

## 2. Node allocation, and why

| Node | Board | Job | Attached |
|---|---|---|---|
| **VCU** | FRDM-MCXA153 | Fusion, state machine, arbitration, safety supervisor, hub | HW-123 IMU (on its I²C bus), on-board RGB LED + SW2/SW3 |
| **PERC** | Arduino 1 | Perception + driver display | Scanning head (HC-SR04P on 2 × MG90S), HC-SR501 PIR, SFM-27, 8×8 matrix |
| **ACT** | Arduino 2 | Motor PWM, encoders, speed PID, local safe state | Motor driver → 4 motors, 2 encoders, 7-segment, 16×2 LCD |
| **VIS** | ESP32 | Camera + WiFi gateway | OV7670 |

Four nodes is genuinely justified here, and not because of CPU load:

- **PERC blocks.** A servo sweep takes hundreds of milliseconds and an ultrasonic ping takes
  up to 25 ms. Neither belongs anywhere near a control loop.
- **ACT is timing-critical.** 20 kHz PWM plus encoder interrupts plus a 100 Hz speed PID.
- **VIS is bursty and unreliable.** Camera DMA and WiFi retries must not be able to touch
  the wheels. This is the fault-containment argument, and it is the honest one.
- **VCU decides.** It holds the state machine and supervises the other three.

### The two allocation decisions worth defending

**Servos on PERC, motors on ACT — this is forced.** The Arduino `Servo` library takes over
Timer1, and Timer1 is the only 16-bit timer on an ATmega328P, so it is also the only way to
get 20 kHz motor PWM. One Uno cannot do both. Splitting them across the two Arduinos costs
nothing and makes the conflict disappear.

**The IMU hangs off the MCX's I²C bus, not an Arduino.** `LPI2C0` already runs to the ESP32
gateway; I²C is multi-drop, so the HW-123 joins it as a second slave for free. Three
benefits: the VCU gets yaw rate with no link in the path (it is a safety input), the module
is 3.3 V native so it matches the MCX exactly, and **its on-board 2.2 kΩ pull-ups to 3.3 V
are the bus pull-ups** — so you need no resistors, which matters because you have none.

---

## 3. VCU — FRDM-MCXA153

| Function | MCU pin | Header | Dir | To | Wiring |
|---|---|---|---|---|---|
| LPUART2_TXD | `P3_15` | **J2 pin 2** | → | PERC **D4** | Direct wire |
| LPUART2_RXD | `P3_14` | **J2 pin 4** | ← | PERC **D5** | **1 kΩ + 2 kΩ divider** |
| LPUART1_TXD | `P1_9` | **J2 pin 20** | → | ACT **D4** | Direct wire |
| LPUART1_RXD | `P1_8` | **J2 pin 18** | ← | ACT **D5** | **1 kΩ + 2 kΩ divider** |
| LPI2C0_SCL | `P3_27` | **mikroBUS J6** | ↔ | ESP32 GPIO22 **+ HW-123 SCL** | No pull-ups to add |
| LPI2C0_SDA | `P3_28` | **mikroBUS J6** | ↔ | ESP32 GPIO21 **+ HW-123 SDA** | |
| 3.3 V | — | **J3 pin 8** (`LDO_3V3`) | → | HW-123 VCC | **Not 5 V** |
| GND | — | **J3 pin 12 / 14** | — | star ground | |

On-board, nothing to wire (verified in `distributed_ecu/frdmmcxa153/frdmmcxa153/board.h`):
`P3_12` red LED, `P3_13` green, `P3_0` blue, `P3_29` SW2, `P1_7` SW3, `P0_2`/`P0_3` →
MCU-Link VCOM. This schema uses no J1 pins, so all of it is free for state indication.

---

## 4. PERC — Arduino 1

| Pin | Net | Component | Notes |
|---|---|---|---|
| D0 / D1 | — | — | **Leave free** — CH340 bootloader |
| D2 | ECHO | HC-SR04P | **INT0** |
| D3 | PIR OUT | HC-SR501 | **INT1** |
| **D4** | LINK_RX | ← MCX `P3_15` | Direct |
| **D5** | LINK_TX | → MCX `P3_14` | **Through the divider** |
| D6 | SERVO_PAN | MG90S #1 | Signal only — V+ from the pack |
| D7 | SERVO_TILT | MG90S #2 | Signal only — V+ from the pack |
| D8 | TRIG | HC-SR04P | |
| D10 | MATRIX_CS | 8×8 matrix | |
| D11 | MATRIX_DIN | 8×8 matrix | Hardware SPI MOSI |
| D13 | MATRIX_CLK | 8×8 matrix | Hardware SPI SCK |
| A0 / A1 | SFM-27 | reserved | Analog + digital, until you identify it |
| D9, D12, A2–A5 | — | — | Spare |

**The scanning head is the interesting part.** You have exactly one range sensor and two
servos. Mount the HC-SR04P on the pan/tilt and sweep it: one sensor becomes a left / centre
/ right picture, and you get a coarse obstacle map that a fixed sensor could never give you.
The blocking sweep is precisely why PERC is its own node.

Power the HC-SR04P from the Arduino's **5 V**. The "P" variant's 3.3–5 V range earns you
nothing on a 5 V node — it would only matter if you ever moved it onto the MCX.

---

## 5. ACT — Arduino 2

| Pin | Net | Component | Notes |
|---|---|---|---|
| D0 / D1 | — | — | **Leave free** — bootloader |
| D2 | ENC_L | Left optocoupler | **INT0** |
| D3 | ENC_R | Right optocoupler | **INT1** |
| **D4** | LINK_RX | ← MCX `P1_9` | Direct |
| **D5** | LINK_TX | → MCX `P1_8` | **Through the divider** |
| D6 / D7 | IN1 / IN2 | Motor driver | Left pair direction |
| D8 / D12 | IN3 / IN4 | Motor driver | Right pair direction |
| **D9** | ENA | Motor driver | **Timer1 OC1A — 20 kHz** |
| **D10** | ENB | Motor driver | **Timer1 OC1B — 20 kHz** |
| A0 / A1 | CLK / DIO | 7-segment | Bit-banged; kept off D13 because the on-board LED loads it |
| A4 / A5 | SDA / SCL | 16×2 LCD | **5 V bus, ACT-local** |
| D11, D13, A2, A3 | — | — | Spare |

**Four motors, two channels, two encoders.** Wire the left pair in parallel to one output
and the right pair to the other. The wheels on each side are mechanically locked together,
so a second encoder per side would measure the same thing — you need 2 of your 4 disks, and
that is why two external interrupts are enough.

For 20 kHz on D9/D10: phase-correct PWM, prescaler 1, `ICR1 = 16e6 / (2 × 20000) = 400`.
`analogWrite()`'s default 490 Hz is audible and puts torque ripple through the gearbox.

> **Known limitation.** `SoftwareSerial` disables interrupts while transmitting, so a link
> message can swallow an encoder edge. At ~200 RPM with a 20-slot disk that is roughly 67
> edges/s per wheel against ~4 ms blocked per 50 ms window — under half a percent. Harmless
> for the speed PID, which measures rate; it accumulates in absolute odometry. If odometry
> drift bothers you, move ACT's link to the hardware UART on D0/D1 and give up the USB
> serial monitor on that board.

---

## 6. VIS — ESP32 and the camera

I²C to the VCU on **GPIO21 (SDA) / GPIO22 (SCL)**, slave `0x42`, 400 kHz — unchanged from
[`esp32_gateway.ino`](arduino_codes/esp32_gateway/esp32_gateway.ino).

| OV7670 | ESP32 | | OV7670 | ESP32 |
|---|---|---|---|---|
| D0 | GPIO5 | | XCLK | **GPIO0** |
| D1 | GPIO18 | | PCLK | GPIO25 |
| D2 | GPIO19 | | HREF | GPIO26 |
| D3 | GPIO23 | | VSYNC | GPIO27 |
| D4 | GPIO36 *(in only)* | | SIOC | GPIO32 |
| D5 | GPIO39 *(in only)* | | SIOD | GPIO33 |
| D6 | GPIO34 *(in only)* | | RESET | tie to 3.3 V |
| D7 | GPIO35 *(in only)* | | PWDN | tie to GND |

Three ESP32 facts this map is built around: **GPIO6–11 are wired to the flash** and must
stay unused; **GPIO34–39 are input-only**, which is exactly what eight camera data lines
want; and **GPIO0 is a strapping pin** — if the camera holds it low at boot the ESP32 will
not start, so unplug the camera before flashing.

`esp32-camera` lists OV7670 support. At **QQVGA grayscale** a frame is 160 × 120 = 19 KB,
which fits in internal RAM with no PSRAM. Expect around 10 fps.

> **Treat this as the part that might not work.** No frame buffer, ~14 wires, and the ESP32
> is also running WiFi. Build the car so that everything works with the camera unplugged,
> and add it last. If it defeats you, the ESP32 goes back to being a pure gateway — which is
> what your current firmware already does.

---

## 7. What is not in your box

Three items. The first is a hard blocker.

| # | Part | Why | Consequence if missing |
|---|---|---|---|
| 1 | **Dual H-bridge — L298N or TB6612FNG** | You have four motors and nothing to drive them | **The car cannot move.** No substitute exists in the box |
| 2 | **2 × slotted optocoupler (LM393 IR speed sensor)** | Your four "speed encoder disks" are chopper wheels — passive plastic | No encoder signal at all: no odometry, no speed PID |
| 3 | **Resistors: 2 × 1 kΩ, 2 × 2 kΩ, 2 × 10 kΩ** | The two level dividers, plus pull-downs on ENA/ENB | The 5 V Arduino TX lines **destroy `P3_14` and `P1_8`** |

Item 3 is unavoidable. The MCX is 3.3 V and only `P3_27`/`P3_28` tolerate 5 V, so every
Arduino→MCX line needs dividing, and there is no way to build the hub without those two
lines. A €2 resistor assortment covers all six.

The 10 kΩ pair is not strictly required but is strongly recommended: the driver's enable
inputs float while an Arduino is in reset, and floating CMOS inputs mean the wheels can
twitch. Until you fit them, **keep the wheels off the floor during every reset.**

Optional, later: a **5 V USB power bank** so the car can run untethered. Not needed for
bench work — see §9.

---

## 8. Assumptions you should check before wiring

Five unknowns. Four are cheap to resolve and one may change the design.

| Item | Assumed | How to check | If wrong |
|---|---|---|---|
| **SFM-27** | Unknown — A0 + A1 held for it | Read the silkscreen and count the pins | May need I²C (A4/A5 are free on PERC) or more pins |
| **8×8 matrix** | MAX7219 module, 3 pins | Is there a 24-pin DIP behind the matrix? | A bare 1088AS needs 16 pins and will not fit — it would have to move to the ESP32 or be dropped |
| **7-segment** | TM1637 module, 2 pins | Count the header pins: 4 = TM1637 | A bare display needs 8–12 pins; ACT has 4 spare, so it would have to be a single digit |
| **16×2 LCD** | PCF8574 I²C backpack, 2 pins | Is there a small board soldered to the back? | Bare HD44780 needs 6 pins — ACT has 4 spare, so something else moves to PERC |
| **HW-123** | MPU6050 at 0x68 | Read `WHO_AM_I` (reg `0x75`) | `0x70` = MPU6500 — different register map, different driver |
| **Battery holders** | 4 × AA in series = 6 V | Count the cells | Two 4-cell packs in series = 12 V, too much for MG90S and for the TT motors |

---

## 9. Power

Detail on [`SCHEMA_POWER.svg`](SCHEMA_POWER.svg). Two supplies:

```
4 × AA pack (6 V alkaline / 4.8 V NiMH) ──┬──> motor driver VM ──> 4 × DC gear motor
                                          └──> MG90S V+ (both servos)

USB 5 V (laptop, or one power bank later) ──> VCU, PERC, ACT, ESP32 — each on its own port

                    PACK NEGATIVE  ══════  BOARD GROUND      <-- the one wire everyone forgets
```

**You have no regulator, so logic runs from USB.** That is not a compromise — it is the
correct bring-up configuration. Every board has a USB port and its own regulator, and you
need all four serial monitors open anyway. The pack powers motion only.

Two things follow from that:

1. **The two grounds must be tied.** Battery negative to board ground. Without it the motor
   driver has no reference for its logic inputs and nothing works — and it looks exactly
   like a firmware bug.
2. **Never feed 6 V into an Arduino 5 V pin or into the MCX.** The pack goes to the driver's
   VM and the servos' V+, and nowhere else.

**Servos on the traction pack, not the Arduino.** An MG90S stalls at ~700 mA; two of them
will brown out a board fed from a USB port. They also share the pack with the motors, so
expect the scanning head to twitch when the motors start — if that matters, a second pack
fixes it.

**Prefer NiMH.** The MG90S window is 4.8–6.0 V. Four NiMH cells sit at 4.8 V, right in it.
Four fresh alkalines sit at ~6.4 V, just over.

---

## 10. Bring-up order

```
[ ] Tie the pack negative to board ground. Verify continuity across all four boards
[ ] UART links + both dividers, both directions            <- you already have this working
[ ] I2C scan from the MCX: expect 0x42 (ESP32) and 0x68 or 0x69 (HW-123)
[ ] HW-123: read WHO_AM_I before writing a single driver line
[ ] Scanning head: servos first, sweep with no ultrasonic mounted. Then add ranging
[ ] PIR: it needs ~1 minute to settle. Do not debug it in the first minute
[ ] Displays: matrix, 7-seg, LCD — each alone, each confirmed
[ ] SFM-27: identify it. Only then wire it
[ ] Motors ON BLOCKS. Prove STBY / ENA-low stops them BEFORE anything else
[ ] Encoders: turn each wheel by hand, check direction and count
[ ] Only then put it on the floor
```

Note on odometry: this is a 4WD skid-steer chassis, so every turn scrubs the tyres and your
encoder odometry degrades during rotation. Trust it going straight, trust the IMU for
heading, and do not expect dead reckoning through a turn to be accurate.

---

## 11. Sources

- [UM12012 FRDM-MCXA153 Board User Manual Rev. 2.0](https://akizukidenshi.com/goodsaffix/FRDM-MCXA153_User%20Manual.pdf) — pinouts, Figures 11–12 and Table 14
- `distributed_ecu/frdmmcxa153/frdmmcxa153/board.h` — LED and switch pins, verified in-tree
- [`arduino_codes/esp32_gateway/esp32_gateway.ino`](arduino_codes/esp32_gateway/esp32_gateway.ino) — I²C address, chunk size, AP config
- [`arduino_codes/serial_bridge/serial_bridge.ino`](arduino_codes/serial_bridge/serial_bridge.ino) — D4/D5 link pins, 38400
- [`SERIAL_BRIDGE.md`](SERIAL_BRIDGE.md) — the working 3-board hub this builds on
