# NUCLEO-F446RE + SX1276 / RFM95W port

This port puts the FHSS layer on a NUCLEO-F446RE with an SX1276-based 915 MHz LoRa radio (RFM95W module or Adafruit breakout), using ST's HAL.

| File | Purpose |
|---|---|
| `sx1276.c/.h` | Minimal LoRa driver: init, retune, send, receive. Polls the radio over SPI, so DIO0 doesn't need to be wired |
| `fhss_link.c/.h` | Hopping packet link: hop clock, beacons, sync, TX slots, queue |
| `main_example.c` | Code blocks to paste into your CubeMX `main.c` |

## Wiring (Nucleo Arduino header → RFM95W)

| RFM95W | Nucleo pin | STM32 pin |
|---|---|---|
| VIN / 3.3V | 3V3 | — |
| GND | GND | — |
| SCK | D13 | PA5 (SPI1_SCK) |
| MISO | D12 | PA6 (SPI1_MISO) |
| MOSI | D11 | PA7 (SPI1_MOSI) |
| CS / NSS | D10 | PB6 (GPIO output) |
| RST | D9 | PC7 (GPIO output) |
| G0 / DIO0 | not needed | — |

- **Power:** a bare RFM95W module runs on 3.3 V only, and 5 V will damage it. The Adafruit breakout has its own regulator.
- **LED:** PA5 is also the user LED LD2, so the LED will flicker during SPI traffic. That's expected, and the code doesn't use the LED.
- **Antenna:** attach a 915 MHz antenna before powering the radio. A 78 mm straight wire is a quarter-wave. Transmitting with no antenna can damage the power amplifier.

## CubeMX / STM32CubeIDE setup

1. Create a new STM32 project for the NUCLEO-F446RE board and accept the default peripheral setup. That gives you USART2 on the ST-LINK virtual COM port at 115200 baud.
2. Set up **SPI1**:
   - Mode: Full-Duplex Master, hardware NSS **disabled**.
   - Frame format: 8-bit, MSB first.
   - CPOL Low, CPHA 1 Edge (SPI mode 0).
   - Prescaler: 16 or higher, which gives about 5 MHz at an 84 MHz APB2 clock. The SX1276 tops out at 10 MHz.
3. Set **PB6** as GPIO_Output with user label `RFM_NSS` and initial level **High**.
4. Set **PC7** as GPIO_Output with user label `RFM_RST` and initial level **High**.
5. Generate the code.
6. Copy `src/fhss.c`, `include/fhss.h` and the `.c/.h` files from this folder (except `main_example.c`) into the project's `Core/Src` and `Core/Inc` folders.
7. Paste the blocks from `main_example.c` into the matching `USER CODE` sections of `main.c`.
8. Flash one board with `NODE_IS_LEADER 1` and the other with `0`.
9. Open both COM ports at 115200 baud:
   - The leader starts right away.
   - The follower prints `searching for leader`, then `locked to hop sequence` within about 20 s.
   - After that, each side prints the other's `L<n>` / `F<n>` messages.

## Link parameters

| Parameter | Value | Where to change it |
|---|---|---|
| Channels | 50, 902.2–926.7 MHz, 500 kHz apart | `fhss_link.c` |
| Modulation | LoRa SF7, 125 kHz, CR 4/5, CRC on, sync word 0x12 | `fhss_link.c` |
| Dwell | 400 ms. Leader transmits in the first half, followers in the second | `fhss_link.c` |
| Max payload | 24 bytes (≈ 72 ms airtime with the 6-byte header) | `fhss_link.h` |
| Throughput | About 2 full packets per node per dwell (≈ 120 B/s each way) | Raise it with a lower SF; it drops with a higher SF |
| Acquisition | ≤ 20.4 s. The follower parks on one channel until the leader passes through it | |
| Lock loss | 3 dwells with no leader packet, then it searches again | `max_missed_hops` |

Sync compensates for airtime on every packet: the follower works out when the leader started sending from the packet length. That means `rx_latency_ms` doesn't need measuring for this port.

## Verified vs. not yet verified

**Verified** in the laptop test suite (`make test`, `tests/test_port_sx1276.c`). The real driver and link code run against two emulated SX1276 radios for 3 simulated minutes, including a 3 s outage. The tests check:

- Register values, the FRF formula and airtime against the Semtech calculator.
- First lock and relock.
- Message delivery in both directions.
- The leader's radio hops exactly every 400 ms.
- Every transmission sits inside one dwell on the right channel.
- The follower never transmits before it's locked.
- Packets from another network are ignored.

**Not yet verified**: anything on real hardware. The emulator follows the datasheet, but it's only a model. When you first power up:

- [ ] `fhss_link_init` succeeds, meaning the chip version read back as 0x12. If it fails, check wiring, 3.3 V and SPI mode.
- [ ] Fixed-channel send/receive works before hopping is turned on (the prerequisite on your task sheet).
- [ ] An SDR or spectrum analyzer shows 400 ms dwells with random-looking hops across 902–928 MHz.
- [ ] The RF lead signs off on the 20 dB bandwidth (< 250 kHz) and output power before any testing on the air.
