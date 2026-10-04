# NUCLEO-WL55JC + SX1262 shield: self-test firmware

Bring-up firmware for one board: a **NUCLEO-WL55JC** (board number MB1389) with the Semtech **SX1262MB2xAS** shield plugged into its Arduino header. It checks the radio, then hops as the FHSS leader so you can watch it.

> **Why the shield and not the WL55's own radio?** The STM32WL55 has a sub-GHz radio built in. But the **JC2** board is the low-band version: its antenna circuit is tuned for 433–510 MHz, not 902–928 MHz. So the WL55 is used only as the microcontroller here, and the 915 MHz shield does the radio work. Don't transmit on the Nucleo's own antenna connector. A **JC1** (high-band) board could use its built-in radio instead, which would need a different driver transport.

## Hardware setup

- Plug the shield straight onto the Nucleo's Arduino header. No jumper wires are needed.
- Keep the 915 MHz antenna on the shield's SMA connector whenever the board is powered.
- Connect the Nucleo's ST-LINK USB port to the computer.

| Shield signal | Arduino pin | WL55JC pin |
|---|---|---|
| SCK / MISO / MOSI | D13 / D12 / D11 | PA5 / PA6 / PA7 (SPI1) |
| NSS | D7 | PC1 |
| RESET | A0 | PB1 |
| BUSY | D3 | PB3 |
| Antenna switch power | D8 | PC2 |

The shield's signals come from Semtech's LoRaMac-node board files. The Arduino-to-STM32 mapping comes from the Zephyr `nucleo_wl55jc` board definition.

## Build, flash, watch

You only need the ARM compiler, with no STM32CubeIDE or ST account:

```sh
brew install arm-none-eabi-gcc arm-none-eabi-binutils
cd boards/nucleo_wl55jc
make            # builds build/selftest.bin (~9 KB)
make flash      # copies it onto the Nucleo's USB drive (/Volumes/NOD_...)
make monitor    # serial console at 115200 (quit: Ctrl-A, then K, then y)
```

Flashing is drag-and-drop: the Nucleo appears as a USB drive, and copying the `.bin` onto it programs the chip. If `make monitor` attaches after the board has already printed, press the black **RESET** button on the Nucleo to run the tests again.

## What it checks

| Check | Proves | Does **not** prove |
|---|---|---|
| Radio init | SPI wiring, BUSY handshake, reset, oscillator. It auto-detects crystal vs TCXO, so a shield with no sticker is fine | That the radio transmits or receives |
| PLL lock (50 channels) | The synthesizer can tune to every hop frequency | Output power or frequency accuracy |
| Noise floor | The receiver is on and reporting RSSI on each channel (informational) | That packets decode |
| TX timing | The modem sends a full 30-byte packet and reports done after the expected airtime (~72 ms ± 4) | That anything was heard over the air |
| Device errors | Nothing latched in the chip's error register afterwards | |
| CAN loopback | The MCP2515 answers on SPI and loops a frame back internally | The transceiver or the CAN bus (needs a second node) |

After the checks, it runs the real hopping link as the **leader** at 0 dBm, with a beacon every 400 ms dwell. It prints one line per 20 s cycle with the measured dwell range, which should be 400 ms (±1 ms), plus beacon and error counts. To confirm hops over the air, watch 902–928 MHz on a spectrum analyzer or SDR receiver.

**LEDs:**
- **Green:** every check passed.
- **Red:** a check failed or the firmware faulted.
- **Blue:** toggles on each hop.

## If radio init fails

The firmware retries every 3 s, so you can reseat the shield without reflashing.

| Error | Likely cause |
|---|---|
| -1 `NOT_FOUND` | No SPI reply: shield not fully seated, or NSS/SPI pins not making contact |
| -8 `BUSY` | BUSY never went low: check D3 (PB3), and that RESET on A0 (PB1) isn't held low |
| -6 `OSC` | Oscillator didn't start as either crystal or TCXO: shield fault |
| -7 `CALIB` | Calibration failed: check the 3.3 V supply |

## CAN hardware: HW-184 (MCP2515) + Adafruit TJA1051T/3

The STM32WL55 has no CAN controller, so the HW-184 board's **MCP2515** provides one over SPI. Its own **TJA1050** transceiver needs 5 V, which doesn't mix with the WL55's 3.3 V logic. So we remove it and use the **Adafruit TJA1051T/3** breakout instead. That board takes 3.3 V logic and makes its own 5 V for the bus.

### 1. Modify the HW-184

**No-solder bench option (for now):** snip only the TJA1050's **pin 4 (RXD)** with flush cutters, which stops it driving the MCP2515's receive line. Then attach jumper wires to **MCP2515 pins 1 (TXCAN) and 2 (RXCAN)** with micro-grabber clips, and run them to the Adafruit board's TX and RX. Check with a multimeter that the clips don't short the two pins together. Replace the clips with soldered wires once the setup is proven. The full steps below are the permanent version.


1. **Find the TJA1050:** the only 8-pin chip on the board (the 18-pin one is the MCP2515). Pin 1 is marked by a dot or notch.
2. **Check the connections before removing anything.** With a multimeter on continuity:
   - TJA1050 pin 1 (TXD) should beep to MCP2515 pin 1 (TXCAN).
   - TJA1050 pin 4 (RXD) should beep to MCP2515 pin 2 (RXCAN).
   - If not, stop and send a photo; this board revision is wired differently.
3. **Remove the TJA1050.** Use hot air, or cut its legs flush with the body and then desolder the stubs one at a time. Don't lift the pads.
4. **Solder two wires to the empty pads:**
   - Pad 1 (TXCAN) goes to the Adafruit board's **TX**.
   - Pad 4 (RXCAN) goes to the Adafruit board's **RX**.
   - If a pad is damaged, solder directly to MCP2515 pins 1 and 2 instead.
5. **Ignore the HW-184's terminal block and J1 jumper from now on.** They connected only to the removed chip.

### 2. Wire it up

| From | To (NUCLEO-WL55JC) | Notes |
|---|---|---|
| HW-184 VCC | 3.3 V rail (from CN7 pin 16) | **3.3 V, not 5 V.** The MCP2515 is fine at 3.3 V |
| HW-184 GND | GND | |
| HW-184 CS | CN10 pin 29 (PB5, Arduino D4) | MCP2515 chip-select. The shield doesn't use D4 |
| HW-184 SCK | CN10 pin 11 (PA5, D13) | Shared with the radio shield (SPI1) |
| HW-184 SI | CN10 pin 15 (PA7, D11) | Shared (MOSI) |
| HW-184 SO | CN10 pin 13 (PA6, D12) | Shared (MISO). The MCP2515 releases it when CS is high |
| HW-184 INT | not connected | The firmware polls |
| Adafruit Vcc | 3V3 | Sets its logic level to 3.3 V; it makes its own 5 V for the bus |
| Adafruit GND | GND | Also the bus common ground, per the embedded team's doc |
| Adafruit SLNT | GND | Normal mode (high would be listen-only) |
| Adafruit CANH / CANL | bus | Yellow to CANH, green to CANL, twisted (team convention) |
| Adafruit termination switch | | ON only if this board is at one physical end of the bus |

Physical pin orders, from the boards' silkscreen:
- **HW-184 header**, top to bottom: INT, SCK, SI, SO, CS, GND, VCC.
- **Adafruit TJA1051T/3 header**, left to right with the labels facing you: Vcc, GND, RX, TX, SLNT, CANH, CANL.

The LoRa shield occupies the Arduino sockets, so the CAN wires go on the Nucleo's long **ST morpho** pins. These are from Table 18 of ST's UM2592 (Rev 4):

| Signal | Morpho pin |
|---|---|
| SCK (PA5, D13) | CN10 pin 11 |
| MISO (PA6, D12) | CN10 pin 13 |
| MOSI (PA7, D11) | CN10 pin 15 |
| CAN CS (PB5, D4) | CN10 pin 29 |
| 3V3 | CN7 pin 16 |
| GND | CN10 pin 9 (also CN7 8, 19, 20, 22; CN10 20) |

CN10 is the right-hand header and CN7 the left, with the antenna at the top. Pin 1 is at the top end, odd and even pins are the two columns, and the numbers are printed along the headers. CN7 pin 18 is 5V: don't use it.

The CAN check runs whether or not the radio shield is fitted, so the CAN wiring can be tested on its own.

### 3. Settings to confirm

These are set in `selftest.c`:
- `CAN_OSC_HZ`: **confirmed 8 MHz** (the team's HW-184 crystal reads 8.000).
- `CAN_BITRATE`: the bus bit rate from the embedded team. 500 kbit/s is a placeholder.

### What the CAN check proves

Check 6 runs the MCP2515 in **loopback mode**. That proves:
- The SPI wiring and chip-select.
- The chip's reset and mode changes.
- That the controller sends and receives a frame internally.

It doesn't touch the transceiver or the bus. Testing those needs a second CAN node on the wires, such as the embedded team's board or a USB-CAN adapter.

## Status

**First hardware run: 2026-10-04, all checks passed.** NUCLEO-WL55JC2 + SX1262MB2xAS shield (crystal / CAS version) + HW-184 MCP2515 wired per this README:

```
=== FHSS self-test: NUCLEO-WL55JC + SX1262 shield ===
  PASS  radio init (clock: crystal)
        PLL locked on 50 / 50 channels
  PASS  PLL lock
  INFO  noise floor dBm: min -114, avg -111, max -109
        TX 30 B, expected 72 ms, measured: 75 74 74 74 74
  PASS  TX timing
  PASS  device errors
  PASS  CAN controller loopback (MCP2515)

RESULT: all checks passed

Hopping as leader (0 dBm, beacon every dwell). One line per 20 s cycle:
  cycle 1: dwell 400-400 ms, beacons sent 50, tx errors 0
  cycle 2: dwell 400-400 ms, beacons sent 100, tx errors 0
  ...
```

Measured send time is 2–3 ms longer than the calculated airtime. That's SPI and polling overhead at the 4 MHz system clock, within the link's 5 ms retune margin.

**Not yet verified on hardware:**
- **Over-the-air reception and leader/follower sync:** needs a second radio setup.
- **The hop pattern on a spectrum analyzer or SDR.**
- **The CAN transceiver and bus:** the loopback check stays inside the MCP2515, so this needs a second CAN node.
