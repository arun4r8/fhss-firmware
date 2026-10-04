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
make            # builds build/selftest.bin (~8 KB)
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

## Status

- **Verified on the laptop:** it cross-compiles with warnings treated as errors. CI builds it on every push. The vector table and memory layout have been checked in the binary. The driver logic is covered by the emulated-SX1262 tests (`make test` at the repo root).
- **Not yet run on hardware.** The register-level setup (clocks, SPI1, LPUART1, GPIO) follows ST's reference manual and headers, but this is its first time on a real board.
