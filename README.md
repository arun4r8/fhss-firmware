# fhss-firmware

A portable Frequency Hopping Spread Spectrum (FHSS) layer for the 902–928 MHz band, written in plain C99 so it runs on any microcontroller and radio.

## Requirements covered

| Requirement | How it's met | Test |
|---|---|---|
| 400 ms dwell per channel | Hop clock advances in whole 400 ms steps, so it never drifts | `test_leader_timing`, `test_late_ticks_keep_schedule` |
| Never revisit a channel within 20 s | Hops walk a shuffled list of all channels, so a channel returns every N × 400 ms. `fhss_init` rejects fewer than 50 channels | `test_leader_timing`, `test_sliding_window_occupancy` |
| Every channel equally likely | Unbiased shuffle (Fisher–Yates with rejection sampling), and each channel is used exactly once per cycle | `test_sequence_is_permutation`, `test_first_hop_uniform_across_seeds` |
| Stay in 902–928 MHz | `fhss_init` rejects out-of-band channel plans | `test_config_validation` |

These come from FCC 47 CFR 15.247(a)(1)(i). Have the team's RF lead check this against your final radio configuration before anything goes on air. In particular, confirm that the channel spacing is at least the radio's 20 dB bandwidth and that the 20 dB bandwidth is under 250 kHz.

## How it works

- **Shared sequence.** Every node builds the same hop sequence from a shared `seed`.
- **Leader.** One node is the leader. It owns the hop clock and puts a 4-byte sync header (`hop_index`, `ms_into_dwell`) on every packet.
- **Followers.** A follower starts **searching**: it parks on the first channel of the sequence and listens. The leader lands on that channel once per cycle, so a follower locks within about 20 s. Once locked, it adjusts its clock on every packet it receives. After `max_missed_hops` dwells with no packet, it goes back to searching.
- **Transmit rules.** `fhss_can_transmit()` returns false while a node is searching, because transmitting while parked on one channel would break the dwell limit. It also returns false within `guard_ms` of a hop, so no packet straddles a channel change.

## Ports

- **Nucleo-64 + Semtech SX1262MB2xAS shield**: [`ports/nucleo_sx1262`](ports/nucleo_sx1262/README.md) covers pins, CubeMX setup, init error codes and a ready-to-paste `main.c`.
- **NUCLEO-WL55JC + SX1262 shield (the team's current hardware)**: [`boards/nucleo_wl55jc`](boards/nucleo_wl55jc/README.md) has a ready-to-flash self-test and the CAN bridge firmware, built with only `arm-none-eabi-gcc` and flashed by drag-and-drop.

## CAN bridge

`src/can_bridge.c` carries the embedded team's sensor readings (hub messages, `src/can_msg.c`) from the pod's CAN bus over the hopping link to the ground station. It has no hardware dependencies, so the host tests run the same code as the board.

The board firmware uses **frame mode** (`src/telem_frame.c`):
- **Pod side:** reassembles each hub's CAN frames and copies every reading into a snapshot of all sensors. Each time its slot opens and something changed, it sends the whole snapshot as one frame.
- **Frame layout:** the frame has no per-sensor tags. Both ends share a fixed sensor order (the layout, or schema): byte 0 is the layout version and a 4-bit frame counter, then 2 bytes per sensor. 38 sensors make a 77-byte frame, 83 bytes on air with the 6-byte link header, 149 ms at SF7 / 125 kHz. `0xFFFF` means no valid reading (sensor error, or not heard yet).
- **Ground side:** checks the layout version and length, counts missed frames from the counter, and can rebuild each hub's CAN message.
- **Slot split:** the link gives the pod the first 330 ms of each 400 ms hop and the ground the last 70 ms (`leader_slot_ms`, the same on both nodes). That is 2 frames per hop = **5 full 38-sensor frames a second** down, and one payload of up to 9 bytes per hop up (2.5/s, for commands). In host emulation with 4 hubs at 10 Hz: 5.1 frames/s, none missed, the oldest reading 249 ms old on arrival.
- **Layout:** `telem_schema_placeholder` (hubs 1–3 with 10 sensors, hub 4 with 8) stands in until the embedded team sends the real order. Change the `version` whenever the layout changes, so a ground station with the old layout refuses the frames instead of mislabelling them.

**250 kHz option (host emulation only):** at SF7 / 250 kHz the frame takes 75 ms instead of 149, so a 340/60 split fits 4 frames per hop: **10 full frames a second**, matching the hubs' 10 Hz, with up to 30 bytes per hop going up. In a 60 s emulated run: 10.2 frames/s, none missed, the oldest reading 177 ms old. A 250 kHz signal falls under the FCC's "20 dB bandwidth of 250 kHz or more" hopping rule (at least 25 channels, at most 0.4 s per channel in any 10 s), which the 50-channel plan already meets; the RF lead should confirm the measured 20 dB bandwidth and the emissions at the 902 MHz band edge before it goes on air. Build with `BW250=1`.

Without a layout (`schema = NULL`) the bridge still forwards each hub message as it is, split into radio payloads on sensor boundaries. The 60 s host test of this mode is kept.

## Porting to other hardware

See `examples/port_template.c`. You provide two things:

1. A millisecond clock, passed into every call as `now_ms`. It may wrap around.
2. `set_frequency(hz)`, which retunes your transceiver.

Call `fhss_tick()` from the main loop or a timer running at 1 kHz or faster.

## Build and test (on your laptop)

```sh
make test
```

The tests simulate the radios millisecond by millisecond and check the rules above against every retune the library actually makes. They build with AddressSanitizer and UBSan enabled.

## Before flashing

- [ ] Transceivers are wired up and basic peer-to-peer comms work without hopping
- [ ] Choose `seed`, channel plan and `guard_ms` (at least the longest packet airtime plus the radio's retune time; the SX1262 link checks each packet against its slot instead)
- [ ] Measure `rx_latency_ms` on real hardware
- [ ] Confirm on a spectrum analyzer or SDR that the dwell is 400 ms and the hops look random
