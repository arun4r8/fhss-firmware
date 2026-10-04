# Nucleo-64 + Semtech SX1262MB2xAS shield port

This port puts the FHSS layer on an STM32 Nucleo-64 board (written for the NUCLEO-F446RE) with the Semtech **SX1262MB2xAS** mbed shield plugged into the Arduino headers. It uses ST's HAL.

> **Check which Nucleo you have.** The NUCLEO-F446RE's board number is MB1136. If yours says **MB1389**, it's probably a **NUCLEO-WL55JC**. That board already has an SX1262-class radio built into the STM32WL chip, so it doesn't need the shield. Tell the team which one you have before wiring anything.

| File | Purpose |
|---|---|
| `sx1262.c/.h` | Minimal SX1262 LoRa driver: init, retune, send, receive. Polls the radio over SPI, so DIO1 doesn't need to be wired |
| `fhss_link.c/.h` | Hopping packet link: hop clock, beacons, sync, TX slots, queue |
| `main_example.c` | Code blocks to paste into your CubeMX `main.c` |

## Shield pins (fixed by the shield layout)

| Shield signal | Arduino pin | NUCLEO-F446RE pin | CubeMX setting |
|---|---|---|---|
| SCK / MISO / MOSI | D13 / D12 / D11 | PA5 / PA6 / PA7 | SPI1 |
| NSS | D7 | PA8 | GPIO_Output, label `RADIO_NSS`, initial **High** |
| RESET | A0 | PA0 | GPIO_Output, label `RADIO_RST`, initial **High** |
| BUSY | D3 | PB3 | GPIO_Input, label `RADIO_BUSY`, no pull |
| ANT_SW power | D8 | PA9 | GPIO_Output, label `RADIO_ANT`, initial Low (the driver turns it on) |
| DIO1 | D5 | PB4 | not needed (optional interrupt) |

- **Pin map source.** These pins come from Semtech's reference code for this shield on Nucleo boards. Check them against the shield's silkscreen or schematic before powering up.
- **PB3 conflict.** PB3 is also the SWO debug pin. Make sure CubeMX's Debug setting is **Serial Wire**, not Trace Asynchronous SW.
- **Frequency band.** Only use a shield whose **Freq Band** box is marked 915. `sx1262_init` refuses frequencies outside 902–928 MHz.
- **Antenna.** Keep the antenna attached whenever the board is powered.

## Crystal or TCXO

SX1262 shields come in a crystal version (sticker ending **CAS**) and a TCXO version (sticker ending **DAS**). By default (`SX1262_CLOCK_AUTO`) the driver tries the crystal first and falls back to the TCXO if the oscillator doesn't start, so a shield with no sticker works too. After init, `radio.has_tcxo` says which one it found. You can force one with `SX1262_CLOCK_XTAL` or `SX1262_CLOCK_TCXO`; if a forced setting is wrong, init fails with error **-6** (`SX1262_ERR_OSC`).

## CubeMX / STM32CubeIDE setup

1. Create a new STM32 project for your Nucleo board and accept the default peripheral setup. That gives you USART2 on the ST-LINK virtual COM port at 115200 baud.
2. Set up **SPI1**:
   - Mode: Full-Duplex Master, hardware NSS **disabled**.
   - Frame format: 8-bit, MSB first.
   - CPOL Low, CPHA 1 Edge (SPI mode 0).
   - Prescaler: 16 or higher, which gives about 5 MHz. The SX1262 tops out at 16 MHz.
3. Set up the four GPIOs from the table above, with the labels exactly as shown.
4. Set **System Core → SYS → Debug** to Serial Wire.
5. Generate the code.
6. Copy `src/fhss.c`, `include/fhss.h`, `sx1262.c/.h` and `fhss_link.c/.h` into the project's `Core/Src` and `Core/Inc` folders.
7. Paste the blocks from `main_example.c` into the matching `USER CODE` sections of `main.c`.
8. Flash one board with `NODE_IS_LEADER 1` and the other with `0`.
9. Open the COM port at 115200 baud.

## Init error codes

| Code | Meaning | Check |
|---|---|---|
| -1 `NOT_FOUND` | SPI readback failed | Shield seated properly, SPI1 mode 0, NSS on PA8, board powered |
| -2 `BAD_CONFIG` | Invalid setting (e.g. power outside -9..22 dBm, or a frequency outside 902–928 MHz) | The config values in `main_example.c` |
| -6 `OSC` | Oscillator didn't start | Use `SX1262_CLOCK_AUTO`, or flip the forced clock setting |
| -7 `CALIB` | Chip calibration failed | Power supply; try another shield |
| -8 `BUSY` | BUSY pin never went low | BUSY on PB3 set as input, and the Debug setting isn't using SWO |
| -9 `PLL` | Synthesizer didn't lock on a frequency | Frequency in range; power supply; try another shield |

## Link parameters

| Parameter | Value |
|---|---|
| Channels | 50, 902.2–926.7 MHz, 500 kHz apart |
| Modulation | LoRa SF7, 125 kHz, CR 4/5, CRC on, sync word 0x12 |
| Dwell | 400 ms. Leader transmits in the first half, followers in the second |
| Max payload | 24 bytes (≈ 72 ms airtime with the 6-byte header) |
| Throughput | About 2 full packets per node per dwell (≈ 120 B/s each way) |
| Acquisition | ≤ 20.4 s. The follower parks on one channel until the leader passes through it |
| Lock loss | 3 dwells with no leader packet, then it searches again |

## Verified vs. not yet verified

**Verified** in the laptop test suite (`make test`, `tests/test_port_sx1262.c`). The real driver and link code run against emulated SX1262 radios. The emulator counts any command the datasheet forbids in the current state, such as retuning while receiving, SPI traffic while BUSY is high, or transmitting before calibration or before the RF switch is set up. The tests require zero violations.

The suite covers:
- Correct init sequence and register values, including Semtech errata 15.1 and 15.2.
- Detection of a missing chip, BUSY stuck high, and a wrong TCXO setting in either direction.
- A 3-minute leader/follower run with a 3 s outage, covering lock, relock, message delivery, exact 400 ms hops, and every transmission staying inside one dwell on the right channel.

**Not yet verified**: anything on real hardware. The emulator follows the datasheet, but it's only a model.
