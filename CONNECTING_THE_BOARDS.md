# Connecting the Boards — Step by Step

**FRDM-MCXA153 (VCU) + 2× Arduino (PERC, ACT)**
Distributed-ECU autonomous vehicle — NXP Summer School project

---

## Read this first

**The MCXA153 has no CAN peripheral.** Verified three ways:

| Source | Finding |
|---|---|
| UM12012 Rev 2.0 (board manual) | No CAN reference anywhere in 31 pages; no CAN pin routed to J1–J7 |
| Zephyr `frdm_mcxa153` support matrix | LPUART ×3, LPSPI ×2, LPI2C, I3C, LPADC, PWM, EQDC — no CAN |
| MCUXpresso SDK `_boards/frdmmcxa153/driver_examples/` | 24 peripheral folders, no `flexcan` |

CAN-FD in this family is in the **MCXA154/155/156** (FRDM-MCXA156 board), not the A153.

**Consequence:** all three nodes use an SPI CAN controller (MCP2515). Nothing else in the
architecture changes — the message set, E2E protection, bus-load budget and safety concept
are unaffected.

**Second critical fact:** MCX I/O is 3.3 V and **only `P3_27` and `P3_28` are 5 V tolerant.**
Every 5 V Arduino output that reaches the MCX needs a divider or level shifter. There are no
exceptions to this rule and ignoring it damages the MCU.

---

## What you need

| Item | Qty | Note |
|---|---|---|
| FRDM-MCXA153 | 1 | VCU node |
| Arduino UNO/Nano | 2 | PERC + ACT nodes |
| MCP2515 + TJA1050 module (5 V) | 2 | For the Arduinos |
| MCP2515 breakout + SN65HVD230 module (3.3 V) | 1 | For the MCX — see Stage 3 for why |
| 120 Ω resistors | 2 | Bus termination, **two only** |
| 1 kΩ and 2 kΩ resistors | 1 each | UART level divider |
| USB-CAN analyzer (CANable/USBtin) + SavvyCAN | 1 | Not optional. This is your debugger |
| Twisted pair wire | ~1 m | CAN_H / CAN_L |
| Breadboard + jumpers | — | Stage 0–2 only; crimp a real harness before it goes on the vehicle |

---

## Rule 0 — Grounds and power

Do this before connecting any signal wire. Getting it wrong produces faults that look
like software bugs and will cost you a day.

1. **All three boards share one ground.** Star topology — every ground returns to a single
   point, not daisy-chained through the motor return.
2. **Never power a board from two sources at once.** During bench bring-up each board is
   USB-powered and you connect *signal + ground only* between them. Do not run 5 V or 3.3 V
   between boards while both have USB attached.
3. **Motors stay disconnected** until Stage 4. Every wiring problem is easier to find with a
   silent bench.
4. Before the first power-up, check continuity between the three board grounds with a
   multimeter. It should read near 0 Ω.

---

## Stage 0 — UART link (30 minutes)

**Goal:** prove the toolchains, your byte framing, and your ground wiring before adding CAN
silicon. If you skip this and go straight to CAN, a failure could be SPI, bit timing,
termination, framing or grounding, and you won't know which.

### 0.1 Wiring

`LPUART2` is on the Arduino header J1.

| FRDM-MCXA153 | Direction | Arduino | Wiring |
|---|---|---|---|
| `P1_5` / LPUART2_TXD / D1 — **J1 pin 4** | → | Arduino RX | **Direct wire.** 3.3 V clears the AVR's 3.0 V V<sub>IH</sub> threshold at 5 V VCC |
| `P1_4` / LPUART2_RXD / D0 — **J1 pin 2** | ← | Arduino TX | **Divider required** — see 0.2 |
| GND — **J3 pin 12 or 14** | — | Arduino GND | Direct wire |

### 0.2 The level divider (Arduino TX → MCX RX)

```
Arduino TX ----[ 1 kOhm ]----+---- MCX P1_4 (J1 pin 2)
                             |
                         [ 2 kOhm ]
                             |
                            GND
```

Output = 5 V × 2 / (1 + 2) = **3.33 V**. Safe for the MCX.

> Do not skip this because "it worked for someone on YouTube". `P1_4` is not 5 V tolerant.

### 0.3 Firmware

**MCX side** — start from the SDK example:

```
mcuxsdk-examples/_boards/frdmmcxa153/driver_examples/lpuart/interrupt/
```

Configure LPUART2 at 115200 8N1 and route `P1_4`/`P1_5` using **MCUXpresso Config Tools**.
Do not hand-edit the pin mux registers.

**Arduino side** — use `SoftwareSerial` on spare pins so USB serial stays free for debugging:

```cpp
#include <SoftwareSerial.h>
SoftwareSerial link(4, 5);   // RX = D4, TX = D5

void setup() {
  Serial.begin(115200);      // USB, for your eyes
  link.begin(38400);         // to the MCX — 38400 is a safe SoftwareSerial ceiling
}

void loop() {
  if (link.available()) Serial.write(link.read());
  static uint32_t t = 0;
  if (millis() - t >= 500) { t = millis(); link.print("PING\n"); }
}
```

Set the MCX to 38400 to match. If you use a Nano Every, Leonardo or Mega, use a real
hardware UART instead and run 115200.

> **CHECKPOINT 0** — "PING" arrives at the MCX every 500 ms, and MCX output appears in the
> Arduino's USB serial monitor. Both directions working. Do not proceed until this is true.

---

## Stage 1 — First MCP2515 on an Arduino

**Goal:** get one node talking CAN, verified against the analyzer.

### 1.1 Inspect the module before wiring it

Two things kill more MCP2515 projects than everything else combined:

1. **Read the crystal marking with your eyes.** Modules ship with **8 MHz** or **16 MHz**
   crystals. The wrong setting in software gives you zero frames and *no error message*.
   Write down which one you have.
2. **Find the onboard 120 Ω termination.** Most modules have one, sometimes on a jumper
   (often labelled J1). You will need to remove it on the middle node in Stage 4 — check
   now whether yours is a jumper or a soldered resistor.

### 1.2 Wiring

| MCP2515 module | Arduino UNO |
|---|---|
| VCC | 5 V |
| GND | GND |
| SCK | D13 |
| SI (MOSI) | D11 |
| SO (MISO) | D12 |
| CS | D10 |
| INT | D2 |

### 1.3 Library

Use **`coryjfowler/MCP_CAN_lib`** — its bit-timing tables are better maintained than the
Seeed fork's.

```cpp
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN(10);            // CS on D10

void setup() {
  Serial.begin(115200);
  // MCP_8MHZ or MCP_16MHZ — must match your crystal
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 init failed");
    delay(500);
  }
  CAN.setMode(MCP_LOOPBACK);   // internal loopback — no bus needed yet
  Serial.println("MCP2515 OK");
}

void loop() {
  byte data[8] = {1,2,3,4,5,6,7,8};
  CAN.sendMsgBuf(0x201, 0, 8, data);

  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    long unsigned id; byte len; byte buf[8];
    CAN.readMsgBuf(&id, &len, buf);
    Serial.print("RX id=0x"); Serial.println(id, HEX);
  }
  delay(100);
}
```

> **CHECKPOINT 1a** — In `MCP_LOOPBACK` the node receives its own frames. This proves SPI
> wiring, the crystal setting and bit timing **without any bus wiring at all**. If loopback
> fails, the problem is SPI or the crystal — not the bus.

### 1.4 Against the analyzer

Change `MCP_LOOPBACK` to `MCP_NORMAL`, then wire:

```
Arduino MCP2515  CAN_H ---- twisted pair ---- CAN_H  USB-CAN analyzer
                 CAN_L ----     pair     ---- CAN_L
                 GND   ------------------------ GND
```

Both ends terminated: the module's onboard 120 Ω plus the analyzer's (most have one built
in — check its documentation). Two terminations total, nothing more.

> **CHECKPOINT 1b** — SavvyCAN shows ID `0x201` arriving at 10 Hz. Load your `.dbc` and the
> signals decode by name. A node that transmits with no receiver present will report
> transmit errors — that is normal and is why you connect the analyzer before trusting it.

---

## Stage 2 — Second Arduino, two-node bus

Repeat 1.2 and 1.3 on the second Arduino with a different message ID (`0x300`).

Wire the bus as a **single linear backbone** — no star, no long stubs:

```
[Arduino PERC]---[Arduino ACT]---[analyzer]
     120R                            120R
   (one end)                      (other end)
```

The middle node's termination **must be removed**. Three 120 Ω resistors in parallel = 40 Ω,
which is a dead bus.

> **CHECKPOINT 2** — Measure resistance across CAN_H–CAN_L with everything powered down:
> **60 Ω**. Not 40, not 120. If it isn't 60, fix it before going further.
>
> Both IDs visible in SavvyCAN simultaneously, no error frames.

---

## Stage 3 — The MCX node

### 3.1 Why a different module

The MCP2515 chip runs from 2.7–5.5 V, so it works fine at 3.3 V. But the **TJA1050
transceiver on the cheap blue modules needs 4.75 V minimum** and will not work at 3.3 V.
And you cannot run the module at 5 V, because its MISO and INT outputs would drive 5 V into
non-tolerant MCX pins.

| Approach | ~€ | Verdict |
|---|---|---|
| **MCP2515 breakout + SN65HVD230 module, both at 3.3 V** | 7 | **Recommended** — fully native 3.3 V, no shifters |
| MIKROE CAN click on mikroBUS (uses LPSPI0) | 28 | Zero wiring, but costs you six Arduino header pins (see §5) |
| 5 V module + TXS0108E level shifter | 6 | Cheapest, but auto-direction shifters get unreliable on SPI — keep SCK ≤ 1 MHz |

Mixing a 3.3 V SN65HVD230 with the Arduinos' 5 V TJA1050s **on the same bus is fine**.
CAN_H/CAN_L is differential and the two are interoperable by design.

### 3.2 Wiring — LPSPI1 on the Arduino header

| CAN module | MCX signal | Header pin |
|---|---|---|
| VCC | `LDO_3V3` | **J3 pin 8** |
| GND | GND | **J3 pin 12** or **14** |
| SCK | `P2_12` / LPSPI1_SCK / D13 | **J2 pin 12** |
| SI (MOSI) | `P2_13` / LPSPI1_SDO / D11 | **J2 pin 8** |
| SO (MISO) | `P2_16` / LPSPI1_SDI / D12 | **J2 pin 10** |
| CS | `P2_6` / LPSPI1_PCS1 / D10 | **J2 pin 6** |
| INT | `P2_4` / D2 | **J1 pin 6** |

Then wire MCP2515 `TXCAN`/`RXCAN` to the SN65HVD230's `D`(TXD) and `R`(RXD) pins, and the
transceiver's CAN_H/CAN_L to the bus.

### 3.3 Firmware — you must write this driver

**There is no NXP MCP2515 driver.** You will port one. Budget half a day. The scope is
smaller than it sounds — roughly 300 lines, almost all register writes.

Start from the SDK LPSPI example:

```
mcuxsdk-examples/_boards/frdmmcxa153/driver_examples/lpspi/
```

SPI configuration: **mode 0 (CPOL=0, CPHA=0)**, MSB first, 1–4 MHz (the chip tolerates 10 MHz;
start slow). Route the LPSPI1 pins with MCUXpresso Config Tools.

**SPI commands you need:**

| Command | Byte |
|---|---|
| RESET | `0xC0` |
| READ | `0x03` |
| WRITE | `0x02` |
| READ STATUS | `0xA0` |
| BIT MODIFY | `0x05` |
| RTS (request to send, TXB0) | `0x81` |
| READ RX BUFFER n | `0x90 \| (n << 2)` |

**Registers you need:**

| Register | Addr | Purpose |
|---|---|---|
| `CANCTRL` | `0x0F` | Mode: `0x80` config, `0x00` normal, `0x40` loopback, `0x60` listen-only |
| `CANSTAT` | `0x0E` | Read the mode back to confirm |
| `CNF1/2/3` | `0x2A/0x29/0x28` | Bit timing — **config mode only** |
| `CANINTE` | `0x2B` | Interrupt enables |
| `CANINTF` | `0x2C` | Interrupt flags |
| `TXB0CTRL` | `0x30` | TX buffer 0 control |
| `TXB0SIDH/SIDL` | `0x31/0x32` | Standard ID high/low |
| `TXB0DLC` | `0x35` | Data length |
| `TXB0D0` | `0x36` | First data byte |
| `RXB0CTRL` | `0x60` | RX buffer 0 control |
| `RXB0SIDH` | `0x61` | Received ID |
| `RXB0DLC` | `0x65` | Received length |
| `RXB0D0` | `0x66` | First received byte |
| `RXM0SIDH` | `0x20` | Acceptance mask |
| `RXF0SIDH` | `0x00` | Acceptance filter |

**Bring-up sequence:**

1. Assert RESET (`0xC0`), wait 10 ms
2. Confirm you are in config mode — read `CANSTAT`, top 3 bits should be `100`
3. Write `CNF1`, `CNF2`, `CNF3` for 500 kbit/s
4. Set acceptance masks/filters (or set `RXB0CTRL` to receive-all while debugging)
5. Set `CANCTRL` to **loopback** (`0x40`) and test — same trick as Checkpoint 1a
6. Only then set normal mode (`0x00`)

**Bit timing values:** take them from `mcp_can_dfs.h` in the same coryjfowler library your
Arduinos are using, for your crystal frequency at 500 kbit/s. Copying them from the library
rather than a datasheet calculation guarantees all three nodes are bit-identical, which is
what actually matters. Verify the crystal on the MCX's module too — it may differ from the
Arduinos'.

> **CHECKPOINT 3a** — MCX MCP2515 passes internal loopback. SPI and timing are correct.
>
> **CHECKPOINT 3b** — MCX transmits `0x100` and the analyzer sees it.

---

## Stage 4 — All three nodes

```
      [MCX VCU]        [Arduino PERC]      [Arduino ACT]       [analyzer]
          |                   |                  |                  |
   =======+===================+==================+==================+=======
        120R                                                      120R
     (one end)                                                 (other end)
```

1. Connect all four devices to the single linear backbone.
2. **Exactly two 120 Ω terminations**, at the two physical ends. Remove every other one.
3. Power down and measure CAN_H–CAN_L: **60 Ω**.
4. Power up one node at a time, watching the analyzer.

> **CHECKPOINT 4** — All periodic messages (`0x100`, `0x110`, `0x200`, `0x201`, `0x202`,
> `0x300`, `0x301`) visible at their designed rates, decoded by name from your `.dbc`, with
> no error frames. Measured bus load should be around 5 %.

Only now connect the motors.

---

## 5. Full pinout reference (FRDM-MCXA153)

From **UM12012 Rev. 2.0**, Figures 11–12 and Table 14.

### Arduino header J1 — digital D0–D7

| Pin | MCU | Function |
|---|---|---|
| 2 | `P1_4` | LPUART2_RXD / ARD_D0 |
| 4 | `P1_5` | LPUART2_TXD / ARD_D1 |
| 6 | `P2_4` | GPIO / ARD_D2 |
| 8 | `P3_0` | PWM0_A0 / ARD_D3 |
| 10 | `P2_5` | GPIO / ARD_D4 |
| 12 | `P3_12` | PWM0_X0 / ARD_D5 |
| 14 | `P3_13` | PWM0_X1 / ARD_D6 |
| 16 | `P3_1` | GPIO / ARD_D7 |

### Arduino header J2 — digital D8–D19 (even pins)

| Pin | MCU | Function |
|---|---|---|
| 2 | `P3_15` | GPIO / ARD_D8 |
| 4 | `P3_14` | PWM0_X2 / ARD_D9 |
| 6 | `P2_6` | **LPSPI1_PCS1** / ARD_D10 |
| 8 | `P2_13` | **LPSPI1_SDO (MOSI)** / ARD_D11 |
| 10 | `P2_16` | **LPSPI1_SDI (MISO)** / ARD_D12 |
| 12 | `P2_12` | **LPSPI1_SCK** / ARD_D13 |
| 14 | GND | |
| 16 | VDDA_MCU | |
| 18 | `P1_8` | LPI2C0_SDA / ARD_D18 |
| 20 | `P1_9` | LPI2C0_SCL / ARD_D19 |

J3's **odd** pins carry motor-control analog signals (BEMF, DC-bus current and voltage).

### Header J3 — power and motor drive

| Pin | Signal |
|---|---|
| 4 | VDD_BOARD |
| 6 | `P1_29` / MCU_RESET_B |
| 8 | **LDO_3V3** |
| 10 | **SYS_5V0** |
| 12, 14 | **GND** |
| 16 | P5-9V_VIN |

Odd pins 5–15: `P3_6`–`P3_11` = PWM0_A0/B0/A1/B1/A2/B2 (three half-bridges).
Odd pins 1, 3: `P2_7`, `P3_31` = MC_ENC_B, MC_ENC_A.

### Header J4 — analog A0–A5

`P1_10`(A0), `P1_12`(A1), `P1_13`(A2), `P2_0`(A3), `P3_31`(A4), `P3_30`(A5) on even pins 2–12.

### mikroBUS socket J5 / J6

| Signal | MCU pin | | Signal | MCU pin |
|---|---|---|---|---|
| AN | `P3_30` | | PWM | `P3_12` |
| RST | `P3_1` | | INT | `P2_5` |
| CS | `P1_3` (LPSPI0_PCS) | | RX | `P3_14` (LPUART2_RXD) |
| SCK | `P1_1` (LPSPI0_SCK) | | TX | `P3_15` (LPUART2_TXD) |
| MISO | `P1_2` (LPSPI0_SDI) | | SCL | `P3_27` (LPI2C0) |
| MOSI | `P1_0` (LPSPI0_SDO) | | SDA | `P3_28` (LPI2C0) |
| +3V3 | | | +5V | |
| GND | | | GND | |

### Pin conflicts — the board multiplexes these

| MCU pin | Appears as | …and as |
|---|---|---|
| `P3_14` / `P3_15` | mikroBUS RX / TX | Arduino D9 / D8 |
| `P2_5` | mikroBUS INT | Arduino D4 |
| `P3_1` | mikroBUS RST | Arduino D7, Pmod pin 4 |
| `P3_12` | mikroBUS PWM | Arduino D5 |
| `P3_30` | mikroBUS AN | Arduino A5 |
| `P1_4` / `P1_5` | LPUART2 on D0 / D1 | J3 odd pins as ADC0_A20 / A21 |
| `P1_0`–`P1_3` | mikroBUS SPI | Pmod J7 SPI |

**Using a mikroBUS click board costs you Arduino D4, D5, D7, D8, D9 and A5.** That is the
practical argument for the €7 wired option in Stage 3 over the €28 click.

### Other notes

- `LPUART0` on `P0_2`/`P0_3` is wired to the **MCU-Link VCOM** (disable with JP19). Leave it
  alone — that is your FreeMASTER telemetry channel to the PC.
- **Only `P3_27` and `P3_28` are 5 V tolerant.** They are the mikroBUS/Pmod I²C pins.
- The MCX has a hardware quadrature decoder (`eqdc` in the SDK) on `P3_31`/`P2_7`/`P1_6`.
  Not needed now, but useful if the ACT Arduino's encoder ISR load ever becomes a problem.

---

## 6. Troubleshooting

| Symptom | Most likely cause | Fix |
|---|---|---|
| MCP2515 init fails | SPI wiring, or CS on the wrong pin | Check MOSI/MISO not swapped; verify CS pin matches the constructor |
| Init OK, loopback fails | Wrong crystal constant | Read the crystal marking; switch `MCP_8MHZ` ↔ `MCP_16MHZ` |
| Loopback OK, no frames on bus | Termination, or CAN_H/CAN_L swapped | Measure 60 Ω across the pair with power off |
| Frames appear then stop | Bus-off from error accumulation | Usually one node has wrong bit timing. Put the analyzer in listen-only and check |
| Everything works, then random resets | Motor brownout | Separate rails, star ground, bulk capacitance |
| Intermittent faults on the vehicle | Loose Dupont wires | Crimp a real harness. This looks exactly like a CAN bug and isn't |
| Resistance reads 40 Ω | Three terminations | Remove the middle node's 120 Ω |
| Resistance reads 120 Ω | One termination | Add the second at the other physical end |
| MCX pin dead after wiring an Arduino | 5 V into a 3.3 V pin | The divider was missing. Check the pin against §5 |

---

## 7. Checklist

```
[ ] All three grounds connected, star topology, continuity verified
[ ] Motors disconnected until Stage 4
[ ] Crystal frequency confirmed by eye on every MCP2515 module
[ ] Level divider fitted on Arduino TX -> MCX RX
[ ] CHECKPOINT 0  UART link working both directions
[ ] CHECKPOINT 1a MCP2515 internal loopback passes on Arduino #1
[ ] CHECKPOINT 1b Frames visible in SavvyCAN
[ ] CHECKPOINT 2  Two-node bus, 60 ohm measured, both IDs visible
[ ] CHECKPOINT 3a MCX MCP2515 loopback passes
[ ] CHECKPOINT 3b MCX transmits, analyzer receives
[ ] CHECKPOINT 4  All three nodes, all periodic IDs, no error frames, ~5% load
[ ] Only then: connect motors
```

---

## Sources

- [UM12012 FRDM-MCXA153 Board User Manual Rev. 2.0](https://akizukidenshi.com/goodsaffix/FRDM-MCXA153_User%20Manual.pdf) — pinouts in Figures 11–12, Table 14
- [Zephyr — FRDM-MCXA153 board support](https://docs.zephyrproject.org/latest/boards/nxp/frdm_mcxa153/doc/index.html)
- [MCUXpresso SDK examples — frdmmcxa153](https://github.com/nxp-mcuxpresso/mcuxsdk-examples/tree/release/24.12.00/_boards/frdmmcxa153/driver_examples)
- [NXP — FRDM-MCXA153 product page](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-MCXA153)
- [NXP — FRDM-MCXA156, the CAN-FD sibling](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-MCXA156)
- [MCXA153/A152/A143/A142 datasheet (MCXAP64M96FS3)](https://www.nxp.com/docs/en/data-sheet/MCXAP64M96FS3.pdf)
- [coryjfowler/MCP_CAN_lib](https://github.com/coryjfowler/MCP_CAN_lib)
