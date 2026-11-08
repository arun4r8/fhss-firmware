# Test and bench logs

This folder holds every test result and bench capture to date, so you can see how the code and its tests changed from commit to commit. It covers 2026-09-27 to 2026-11-01.

| Folder | What's in it |
|---|---|
| `tests/history/` | `make test` output at **every commit**, re-run on 2026-11-01. The file name is order, commit date and commit hash. |
| `tests/` | Current tree on 2026-11-01: host tests, firmware builds (all 5 images with sizes), MATLAB tests |
| `hardware/` | Serial captures from the boards, one file per bench day, each with the exact command and its verbatim output |
| `plots/` | MATLAB plots: live two-board demos and the simulated-fault test plot |

## Host tests by commit

The suites are host C programs that run the real firmware code against emulated radios and CAN controllers (`make test`, also run in CI). Each count is the number of checks; every commit passes with 0 failures.

| # | Commit | Date | Change | fhss core | radio port | hub msg format | CAN bridge | MCP2515 |
|---|---|---|---|---|---|---|---|---|
| 1 | `2f73cbd` | 09-27 | FHSS layer: 400 ms dwell, 20 s no-revisit, uniform hopping | 64641 | | | | |
| 2 | `efff4e5` | 09-27 | First radio port (F446RE + SX1276) | 64641 | 1834 | | | |
| 3 | `1a49c67` | 09-27 | SX1262 driver replaces SX1276 (the shield we actually have) | 64641 | 1834 | | | |
| 4 | `ae03fa7` | 10-04 | WL55JC self-test firmware, SX1262 clock auto-detect | 64641 | 1859 | | | |
| 5 | `9da71e3` | 10-04 | CAN path: MCP2515 driver, hub message format, 31 B radio payload | 64641 | 1864 | 445 | | 75 |
| 6–10 | `3962dfc`…`4ab7dbd` | 10-04 | Wiring and README fixes (CS moved to D4, 8 MHz crystal) | same | same | same | | same |
| 11 | `5593e13` | 10-04 | First hardware run, all self-test checks pass | same | 1864 | 445 | | 75 |
| 12 | `c3e7d06` | 10-11 | CAN bridge: hub messages → hopping link → ground | 64641 | 1878 | 445 | 35 | 75 |
| 13 | `97b0a8b` | 10-18 | MATLAB hop tools (`make test` also checks the MATLAB reference sequences) | same | same | same | same | same |
| 14 | `97b9f15` | 10-25 | `flash-swd` target (OpenOCD) | same | same | same | same | same |
| 15–16 | `53a07a4`, `7b9cb03` | 11-01 | Real-bus test mode (`ROLE=ground HUB=1`), live MATLAB demo, logs | 64641 | 1878 | 445 | 35 | 75 |
| 17–18 | today's commits | 11-08 | 38-sensor tag-free frame (new `telem_frame` suite: 56 checks), 330/70 split, 250 kHz option | 64641 | 1944 | 445 | 54 | 75 |

What each suite change added, from diffing the test output between commits:

- **#2:**
  - Register math and airtime.
  - Init and error reporting.
  - Send, receive and retune.
  - A 3 min leader + follower run over emulated air with a 3 s outage. The follower locked at 20.06 s and relocked after the outage, and was never out of step.
  - Packets with another network ID are ignored.
- **#4:** init is checked for a legal command order on the real chip. Also checked: missing chip, stuck BUSY, clock mismatch, and crystal/TCXO auto-detect.
- **#5:**
  - Max-size 31 B payloads back to back: 900 packets, each inside one dwell.
  - Hub message format: the document's 10-sensor example (31 B, 4 CAN frames), bit layout, bad messages, 0–39 sensors, recovery from a lost frame.
  - MCP2515: bit timing, init, loopback, two-node bus, filters, rollover.
- **#17–18 (11-08):**
  - Frame format tests: 38 sensors → 77-byte frame, round trip, faults and unknown sensors, layout version and sequence.
  - Bridge frame mode: one frame from 4 hubs, sent only when something changed, newest data wins, missed frames counted.
  - Link: max-size packets at 200/200 and 330/70 (5 frames/s down, 9 B up), and at 250 kHz (200/200, and 340/60: 10 frames/s down, 30 B up).
  - 60 s end to end, 4 hubs at 10 Hz: 125 kHz + 330/70 gives 5.1 frames/s, oldest reading 249 ms; 250 kHz + 340/60 gives 10.2 frames/s, oldest 177 ms. 0 frames missed in both. Host emulation only.
- **#12:**
  - CAN bridge unit tests: config, 10- and 39-sensor messages, ID filter, newest message per sender when the radio is slow, interleaved hubs, slot reuse.
  - A 60 s end-to-end run in the port suite: 2 hubs, 2400 CAN messages, 300 delivered over the link, oldest reading 333 ms.

MATLAB (`tests/matlab_tests_2026-11-01.txt`): the MATLAB hop sequence matches the firmware on 27 reference sequences. The log parser and checks catch an injected off-sequence hop and a late packet.

## Hardware results

Hardware: two NUCLEO-WL55JC2 boards with SX1262 shields, plus HW-184 MCP2515 modules with TJA1051T/3 transceivers. Board serials: pod `0052003B…`, ground `00320026…`.

| Date | What was tested | Result | Log |
|---|---|---|---|
| 10-04 | First power-up, self-test on one board | All pass: radio init, PLL lock on 50/50 channels, TX timing (30 B in 74–75 ms vs 72 predicted), CAN loopback. Exact 400 ms dwell, 0 TX errors. | Read off the board's console at the time; summary in commit `5593e13` |
| 10-14 | Bridge firmware hopping on one board | Hopping and transmitting, status every 5 s | `hardware/2026-10-14.md` |
| 10-18 | Bridge demo, MCP2515 wiring diagnostics | CAN init failed (no SPI reply); diagnostics showed nothing driving MISO, traced to wiring | `hardware/2026-10-18.md` |
| 10-25 AM | Second board brought up; first two-board radio link | **67/67 beacons received, 0 missed hops, 0 CRC errors.** Ground on the pod's channel every hop. | `hardware/2026-10-25.md`, 11:03 |
| 10-25 AM | Pod CAN re-wire (MISO pull-up/down diagnostic) | SO/CS swap at CN10, then a half-powered MCP2515, found and fixed | `hardware/2026-10-25.md`, 11:20–12:48 |
| 10-25 PM | Pod full self-test after the fix | All checks passed, including CAN loopback | 19:29 |
| 10-25 PM | **End to end:** CAN (loopback) → bridge → hopping radio → ground decode | **232/232 messages decoded, 0 malformed, 0 CRC errors**, RSSI −21 dBm at 0 dBm TX | 19:30 |
| 10-25 PM | Ground board self-test (second HW-184) | All checks passed, including CAN loopback | 20:53 |
| 10-25 PM | Real CAN bus between the boards (ground as fake hub 5 → pod) | **Not working yet.** Ground TX gets no ACK (EFLG 0x15, then 0x0B after a re-wire); the pod sees 0 frames. Suspects: ground MCP2515↔transceiver TX/RX link, transceiver power/SLNT, H/L wiring. Radio link stayed locked throughout. | 20:54–21:14 |
| 10-25 PM | MATLAB live demo, 3 runs × 15 s, both boards | 38 hops each, **222 / 223 received (99.6%)**: 75/75, 73/74, 74/74; 0 off-sequence | `plots/hop_demo_*.png` (212054, 212758, 212929) |
| 11-08 | Self-test, both boards | Radios pass (crystal, PLL 50/50, 74–75 ms TX). **Both MCP2515s gave no SPI reply**: CS was on CN10 pin 27 (D5), following a pin photo that circled it one row high. The pod demo fed its fake hub frames to the bridge in software for the runs below. **After moving CS to pin 29 (D4): all checks pass on both boards, including CAN loopback** | `hardware/2026-11-08.md` |
| 11-08 | **38-sensor frame, 125 kHz, 330/70 split**, both directions, 76 s | **5.00 frames/s, 2 frames on all 183 hops**, 380/380 received, 0 missed, 0 CRC errors, none past the slot, −38 dBm. Ground → pod dummy commands (9 B, one per hop): **190/190**, 0 missed or corrupted | same |
| 11-08 | **38-sensor frame, 250 kHz, 340/60 split**, both directions, 76 s | **9.91 frames/s**, 4 frames on 175 of 183 hops (3 on the rest: the 4th frame's start is within ~2 ms of its deadline), 753/753 received, 0 missed, 0 CRC errors. Commands (30 B): **190/190** | same |
| 11-08 | Bugs found on hardware, fixed | (1) Blocking console output (26 ms per frame line) made the ground miss its 70 ms slot: only 10 commands in 76 s. Console output is now buffered and sent from the 1 kHz tick. (2) The pod's once-per-hop beacon could go out ahead of the first frame and push the second frame out of the slot (1 frame on 104 of 183 hops). A bare beacon now goes at the end of the slot. | same |
| 11-08 | Pod demo through the **real MCP2515** (CAN loopback), 125 kHz, both directions | CAN path clean (11,360 frames → 2,840 hub messages, 0 errors). First 3.74 frames/s: CAN draining at the 4 MHz clock pushed the second frame past its start deadline on half the hops. With the link asking for frames itself, back-to-back sends and a 5 ms pod end margin: **4.79 frames/s, 2 frames on 168 of 184 hops**, 363/363 received, commands 190/190. Raising the MCU clock is the remaining fix | `hardware/2026-11-08.md` |
| 11-08 | Real CAN bus between the boards (`HUB=1`) | **Still failing**: ground's frames never ACKed (EFLG 0x15), pod sees 0 frames. Controllers pass loopback, so it's the transceiver side or bus wiring | same |

Notes for reading the captures:

- Log lines are `CFG` / `HOP,ms,hop,ch,H|S` / `TX,ms,hop,ch,bytes` / `RX,ms,hop,ch`; the format is in `tools/matlab/README.md`.
- Each board's ms clock starts at its own power-up. Pod and ground logs line up by hop index.
- Long captures were trimmed when recorded, so some are excerpts.
- The 10-18 and 10-25 diagnostic builds were throwaway firmware and are not in the repo; only their output is kept.
- On a real bus, a frame that nobody acknowledges shows up as CAN send failures and EFLG 0x15 (TX error-passive). EFLG 0x0B means receive errors.

## For the embedded meeting

Values that are still placeholders in `boards/nucleo_wl55jc/bridge.c`, waiting on the embedded team:

- CAN bit rate: 500 kbit/s.
- Which CAN IDs carry hub messages: `HUB_ID_MASK 0` forwards everything.
- Resend ID base at the ground: 0x200 + hub.

Max radio payload is 31 B of hub message (one 10-sensor message) plus the 6 B link header. That is about 83 ms on air at SF7/125 kHz, roughly 5 messages/s per direction.
