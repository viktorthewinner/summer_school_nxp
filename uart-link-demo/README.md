# FRDM-MCXA153 ↔ Arduino UART Link

A message-passing demo between the two boards, adapted from the IPCEI lab lesson
[LP4 — IoT App](https://alexp25.github.io/ipcei-lab/lessons/lp4-iot-app/), which does the
same thing with an ESP8266.

The MCX sends a JSON line once per second; the Arduino parses it, prints it, and sends an
acknowledgement back. Both directions are demonstrated.

```
{"code":"data","value":21.50}                       MCX     -> Arduino
{"code":"ack","of":"data","value":"21.50"}          Arduino -> MCX
{"code":"chat","value":"hello from MCXA153"}        MCX     -> Arduino   (button SW3)
{"code":"chat","value":"anything you type"}         Arduino -> MCX       (serial monitor)
```

---

## 1. What changed from the ESP8266 lesson, and why

The pins are **identical** — the lesson uses `LPUART2` on `P1_5`/`P1_4`, which is what the
board routes to the Arduino header. Three things had to change:

| | ESP8266 lesson | This Arduino version | Reason |
|---|---|---|---|
| **Level shifting** | None — direct wires | **Divider on Arduino TX → MCX RX** | ESP8266 is 3.3 V. Arduino is 5 V and `P1_4` is **not** 5 V tolerant. This is the one change you must not skip. |
| **Arduino-side port** | Hardware UART | `SoftwareSerial` on D4/D5 | The ESP8266 has a spare UART. An UNO/Nano does not — D0/D1 are shared with USB. |
| **Baud rate** | 115200 | **38400** | `SoftwareSerial` is bit-banged and unreliable at 115200 on a 16 MHz AVR. |

Everything else — the JSON line protocol, the `LPUART0` debug console, the relay pattern —
carries over unchanged.

> If you use a **Nano Every, Leonardo or Mega**, use its second *hardware* UART instead of
> `SoftwareSerial` and run 115200, exactly like the lesson. Change `LINK_BAUD` in both
> sketches to match.

---

## 2. Files

```
uart-link-demo/
├── README.md                        this file
├── mcxa153/
│   ├── app_link.h                   LPUART2 transport API
│   ├── app_link.c                   driver: send/poll lines, JSON field extraction
│   └── main.c                       the demo application
└── arduino/
    └── link_node/
        └── link_node.ino            Arduino sketch
```

---

## 3. Wiring

**Full schematic: [`WIRING.svg`](WIRING.svg)** — open it in any browser, or print it and keep
it on the bench.

**Do this with both boards powered off.**

| FRDM-MCXA153 | Header pin | Direction | Arduino | Wiring |
|---|---|---|---|---|
| `P1_5` / LPUART2_TXD | **J1 pin 4** (ARD_D1) | → | **D4** (SoftwareSerial RX) | Direct wire |
| `P1_4` / LPUART2_RXD | **J1 pin 2** (ARD_D0) | ← | **D5** (SoftwareSerial TX) | **Through the divider below** |
| GND | **J3 pin 12** or **14** | — | GND | Direct wire |

TX and RX are **crossed** — each board's transmit goes to the other's receive.

### The divider — Arduino D5 → MCX P1_4

```
Arduino D5 ----[ 1 kΩ ]----+---- MCX P1_4  (J1 pin 2)
                           |
                       [ 2 kΩ ]
                           |
                          GND
```

Output = 5 V × 2 / (1 + 2) = **3.33 V**. Safe.

The other direction needs nothing: the MCX drives 3.3 V, and the AVR's input threshold at
5 V VCC is 3.0 V, so 3.3 V registers as a valid high.

### Ground

**Tying GND together is required** — without a shared return the UART will read garbage or
nothing at all. This is the same warning the original lesson gives.

Do **not** connect 3.3 V or 5 V between the boards while both have USB plugged in. Each
board is powered by its own USB during this demo; only signals and ground cross over.

> **Check before power-up:** with a multimeter, confirm continuity between the two board
> grounds (near 0 Ω), and confirm there is no continuity between Arduino 5 V and any MCX pin.

---

## 4. FRDM-MCXA153 project setup

### 4.1 Create the project

1. In **MCUXpresso IDE** (or the VS Code extension), import an SDK example for
   `frdmmcxa153`: **`driver_examples/lpuart/polling`**.
   That example already gives you a working `LPUART0` debug console over the MCU-Link VCOM.
2. Build and flash it once, unmodified. Confirm you see its output in a terminal at
   115200 8N1 on the MCU-Link VCOM port.

   > If nothing appears, check that **JP19 is open** — shorting it disables the VCOM port.

### 4.2 Add LPUART2 in Config Tools

Open **MCUXpresso Config Tools** on the project.

**Pins tool:**
- Route `LPUART2_TXD` to **`P1_5`**
- Route `LPUART2_RXD` to **`P1_4`**
- *(optional, for the button)* set **`P1_7`** as **GPIO input, pull-up enabled** — this is
  the SW3 "Wake-up" button

**Clocks / Peripherals tool:**
- Enable the **LPUART2** clock and attach a source (FRO 12 MHz is fine at 38400)

Click **Update Code**. Config Tools regenerates `pin_mux.c` and `clock_config.c`, which
`BOARD_InitHardware()` already calls. **Do not hand-edit those files.**

### 4.3 Add the application files

1. Copy `mcxa153/app_link.c` and `mcxa153/app_link.h` into the project's `source/` folder.
2. Replace the example's `main.c` with `mcxa153/main.c`.
3. Open `app_link.c` and check this line near the top:

   ```c
   #define LINK_LPUART_CLK_FREQ  CLOCK_GetLpuartClkFreq(2u)
   ```

   Compare it against how the SDK example defines its own clock macro in `app.h` — it will
   be the same call with instance `0`. If your SDK version spells it differently, copy that
   spelling and change the index to `2`.

4. Build and flash.

> If you did not configure `P1_7`, set `#define USE_BUTTON 0` in `main.c`. The demo works
> fine without the button.

---

## 5. Arduino setup

1. Open `arduino/link_node/link_node.ino` in the Arduino IDE.
2. **Tools → Board** → Arduino Uno (or Nano).
3. Upload.
4. **Tools → Serial Monitor**, set to **115200 baud**, and set the line ending to
   **Newline** — the sketch sends on Enter.

No libraries to install. `SoftwareSerial` ships with the IDE.

---

## 6. Running it

Open two terminals:

- **MCX debug console** — MCU-Link VCOM, 115200 8N1
- **Arduino serial monitor** — 115200

### Expected output

**MCX console:**
```
FRDM-MCXA153 <-> Arduino UART link demo
LPUART2 @ 38400 baud on P1_5 (TX, J1-4) / P1_4 (RX, J1-2)
----------------------------------------------------
TX  {"code":"data","value":20.00}
RX  {"code":"ack","of":"data","value":"20.00"}
    >> Arduino acked value 20.00
TX  {"code":"data","value":20.25}
RX  {"code":"ack","of":"data","value":"20.25"}
    >> Arduino acked value 20.25
```

**Arduino serial monitor:**
```
Arduino link node ready
SoftwareSerial RX=D4 TX=D5 @ 38400
Type a message + Enter to send it to the MCX.
----------------------------------------------
RX  {"code":"data","value":20.00}
    >> data value = 20.00
TX  {"code":"ack","of":"data","value":"20.00"}
```

### Try it both ways

- Type `hello there` in the Arduino serial monitor and press Enter →
  the MCX console prints `>> chat from Arduino: hello there`
- Press **SW3** on the FRDM board →
  the Arduino monitor prints `>> chat from MCX: hello from MCXA153`

That is a message transmitted from one board to the other, in both directions.

---

## 7. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Nothing on either side | No common ground | Wire GND. This is the most common failure. |
| MCX sends, Arduino silent | TX/RX not crossed | MCX `P1_5` goes to Arduino **D4**, not D5 |
| Arduino sends, MCX silent | Divider wired wrong, or `P1_4` not muxed | Check the divider outputs 3.3 V; re-check Pins tool |
| Garbage characters | Baud mismatch | `LINK_BAUD` must be 38400 in **both** `main.c` and the `.ino` |
| Occasional corrupt lines | SoftwareSerial at its limit | Drop both sides to 19200 |
| Works, then stops after a burst | RX overrun | Already handled — `LINK_Poll` clears the flag and resyncs on the next newline |
| Nothing on the MCX console at all | VCOM disabled | Open jumper **JP19** |
| MCX pin dead after wiring | 5 V reached `P1_4` | The divider was missing or miswired. `P1_4` is not 5 V tolerant. |

---

## 8. Where this fits in the vehicle project

This is **Stage 0** of [`../CONNECTING_THE_BOARDS.md`](../CONNECTING_THE_BOARDS.md). It
proves your toolchains, your framing and your grounding before any CAN hardware is
involved. When it works, keep the line protocol and swap the transport underneath —
`LINK_SendLine` / `LINK_Poll` become CAN frame TX/RX, and nothing above them changes.

That substitution is the whole point of putting a transport API between the application and
the wire.
