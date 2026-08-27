# Serial Hub — FRDM-MCXA153 + 2 × Arduino

Type a line in any of the three serial monitors and it appears in the other two, tagged with
its source.

**Status: builds clean.** Verified with your toolchain — zero warnings, 17 868 B flash
(13.7 %), 4 496 B RAM (18.3 %). Both ISRs confirmed linked as strong symbols.

> The board was unplugged when I finished, so I could not flash it. Reconnect and run the
> flash step in §5.

---

## 1. Topology — a star, not a bus

UART is point-to-point. Three boards therefore need **two separate LPUARTs** on the MCX,
with the MCX acting as hub. The two Arduinos never talk directly; every message is relayed.

```
                     ┌──────────────────┐
   Arduino 1  ◄─────►│  FRDM-MCXA153    │◄─────►  Arduino 2
   LPUART2           │      (hub)       │           LPUART1
   J2 pins 2/4       └────────┬─────────┘         J2 pins 20/18
                              │ LPUART0
                         MCU-Link VCOM
                            (COM3)
```

Each channel has its own interrupt-driven receive ring, so both Arduinos can transmit
simultaneously without colliding.

**Wire format:** `[SRC] text`, where SRC is `MCX`, `ARD1` or `ARD2`.

---

## 2. Wiring

**Schematic: [`WIRING_3_BOARDS.svg`](WIRING_3_BOARDS.svg)**

| Link | MCX pin | Header | Dir | Arduino | Wire |
|---|---|---|---|---|---|
| **ARD1** | `P3_15` / LPUART2_TXD | **J2 pin 2** (ARD_D8) | → | **D4** | Direct |
| | `P3_14` / LPUART2_RXD | **J2 pin 4** (ARD_D9) | ← | **D5** | **1 kΩ + 2 kΩ divider** |
| **ARD2** | `P1_9` / LPUART1_TXD | **J2 pin 20** (ARD_D19) | → | **D4** | Direct |
| | `P1_8` / LPUART1_RXD | **J2 pin 18** (ARD_D18) | ← | **D5** | **1 kΩ + 2 kΩ divider** |
| **GND** | GND | **J3 pin 12** or **14** | — | GND on both | Direct |

Six signal wires plus a common ground. **Both** Arduinos need their own divider — `P3_14`
and `P1_8` are equally not 5 V tolerant.

### Why these pins

`P1_8`/`P1_9` were chosen for LPUART1 because they land on the Arduino header next to the
existing link, and don't collide with the LED (`P3_12`), the VCOM (`P0_2`/`P0_3`) or
LPUART2 (`P3_14`/`P3_15`).

All four link pins mux at **ALT2**. LPUART2's value is copied from NXP's generated example.
LPUART1's had no generated reference for this board, so it was established by
triangulation: NXP's own files put `LPI2C0` at **ALT3** and `CT_INP8` at **ALT4** on these
exact pins, which places LPUART1 at ALT2.

That step mattered. `P1_9`'s signal string is `P1_9/LPUART1_TXD/LPI2C0_SCL/...`, which reads
as ALT1 positionally — but `LPI2C0_SCL` is verifiably ALT3, so `P1_9` has an **empty ALT1
slot** and LPUART1_TXD is ALT2. A positional guess would have produced a dead TX line with
no error message.

**Cost:** using `P1_8`/`P1_9` gives up LPI2C0 on the Arduino header. I²C is still reachable
on `P3_27`/`P3_28` (mikroBUS and Pmod).

---

## 3. Files

**`distributed_ecu/`**

| File | Change |
|---|---|
| `link.c` / `link.h` | Rewritten for **two channels** — config table + per-channel state |
| `led_blinky.c` | Three-way hub with source tagging |
| `frdmmcxa153/led_blinky/app.h` | `LINK_ARD1_*` and `LINK_ARD2_*` macros |
| `frdmmcxa153/led_blinky/pin_mux.c` | Added `P1_8`/`P1_9`, PORT1 gate + LPUART1 reset |
| `frdmmcxa153/led_blinky/hardware_init.c` | LPUART1 clock attach |

**`arduino_codes/serial_bridge/serial_bridge.ino`** — upload the **same sketch to both**
Arduinos. There is no node ID to set: a node's identity comes from which MCX port it is
wired to, and the hub applies the tag. That removes the failure mode where both boards
claim to be ARD1.

> The `.mex` was not modified. If you open Config Tools and hit "Update Code" it will
> regenerate `pin_mux.c` and drop all six UART pins.

---

## 4. Design notes

**Channel abstraction.** `link.c` holds a `const` config table (base, instance, IRQ, name)
plus a parallel runtime state array. Both `LPUART1_IRQHandler` and `LPUART2_IRQHandler`
funnel into one `link_isr(id)`. Adding a third node is a table row.

**Single-producer / single-consumer rings**, one per channel. One writer of `head` (ISR),
one of `tail` (main). No critical sections, no disabled interrupts.

**Nothing blocks.** `GETCHAR()` would stall the loop and starve both Arduinos, so the
console receiver is polled directly while `PRINTF` keeps transmit.

**Prompt redraw.** When a relayed message arrives mid-typing, the hub reprints the prompt
and your partial input, so incoming traffic doesn't appear to eat what you were typing.

**Dropped bytes are counted per channel** — `LINK_GetDroppedCount(id)` covers hardware
overrun, ring overflow and overlong lines.

---

## 5. Build and flash

```bash
cmake --build --preset debug
```

Then, with the board connected:

```bash
"C:/nxp/LinkServer_26.6.137/LinkServer.exe" flash MCXA153:FRDM-MCXA153 load distributed_ecu/debug/distributed_ecu.elf
```

Or use the VS Code MCUXpresso flash task.

**Arduino:** upload `serial_bridge.ino` to each board in turn. Remember to pick the right
COM port — the FRDM's MCU-Link VCOM also enumerates as one, and uploading to it produces
`stk500_getsync() not in sync`.

---

## 6. Running it

| Monitor | Port | Baud |
|---|---|---|
| MCX hub | MCU-Link VCOM (COM3) | 115200 |
| Arduino 1 | its CH340 port | 115200 |
| Arduino 2 | its CH340 port | 115200 |

Links run at **38400** — `LINK_BAUDRATE` in `app.h` and `LINK_BAUD` in the sketch. Set the
Arduino Serial Monitor line ending to **Newline**.

Type `hello` on Arduino 1:

```
Arduino 1     [you] hello
MCX hub       [ARD1] hello
Arduino 2     [ARD1] hello
```

Type `ping` in the MCX terminal:

```
MCX hub       sent to ARD1 + ARD2: ping
Arduino 1     [MCX] ping
Arduino 2     [MCX] ping
```

The red LED blinks once per second. If it stops, the firmware is stuck.

---

## 7. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| One Arduino works, the other doesn't | That link's wires or divider | Compare against the working one — the two links are identical in structure |
| Neither works | No common ground | All three grounds must be tied |
| MCX sends, Arduino silent | TX/RX not crossed | An MCX TX pin always goes to an Arduino **D4** |
| Arduino sends, MCX silent | Divider miswired | Junction must read 3.2–3.4 V with D5 high |
| Garbage characters | Baud mismatch | 38400 on both ends of every link |
| `stk500_getsync() not in sync` | Uploading to the wrong COM port | Pick the CH340 port, not MCU-Link VCOM |
| Nothing on the MCX terminal | VCOM disabled | Open jumper **JP19** |
| Messages echo but never arrive | Wrong MCX port for that Arduino | ARD1 is J2 pins 2/4; ARD2 is J2 pins 20/18 |

`SoftwareSerial` disables interrupts while transmitting, so an Arduino can miss incoming
bytes if it talks at the same moment the hub relays to it. Harmless at typing speed; if it
ever matters, move to a board with a spare hardware UART.

---

## 8. Next step

This is Stage 0 of [`CONNECTING_THE_BOARDS.md`](CONNECTING_THE_BOARDS.md), now with all
three nodes proven. The star is deliberately the same shape as the eventual CAN bus: keep
`LINK_SendLine` / `LINK_PollLine` / `LINK_Broadcast`, swap `link.c` for a CAN
implementation, and `led_blinky.c` doesn't change.
