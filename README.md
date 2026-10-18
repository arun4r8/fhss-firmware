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

`src/can_bridge.c` carries the embedded team's hub messages (`src/can_msg.c`) from the pod's CAN bus over the hopping link to the ground station. It has no hardware dependencies, so the host tests run the same code as the board:

- **Pod side:** reassembles each sender's CAN frames, then queues each complete message (`CAN_BRIDGE_QUEUE_LEN`, default 32) and hands them to the radio one at a time. Messages too big for one 31-byte radio payload are split on sensor-record boundaries, and every piece is a valid message on its own. If the radio refuses a payload partway through a message, the next poll resumes from that payload.
- **Ground side:** checks each payload and rebuilds its CAN frames.
- **Two modes:**
  - **Default, lossless:** every message is sent, in arrival order. The queue absorbs bursts; a message is only dropped if the queue is full, and that is counted (`queue_full`). This meets "buffer CAN packets without losing CAN data".
  - **`latest_only`:** a newer message from a sender replaces its queued, unsent one (`superseded`). For telemetry arriving faster than the link carries, where only the freshest reading matters.
- **Throughput:** the link carries about five 31-byte payloads a second each way. Lossless delivery only holds while the *average* message rate stays below that; the queue covers bursts, not a sustained overload. Above it, use `latest_only`: the ground then sees each hub's latest reading, at most about half a second old.

Per packet, the hopping layer adds 6 bytes (network ID, flags, 4-byte hop sync) and the hub format adds 1 byte plus 1 byte per sensor. A 10-sensor message is 37 bytes on air, 20 of them readings.

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
- [ ] Choose `seed`, channel plan and `guard_ms` (at least the longest packet airtime plus the radio's retune time)
- [ ] Measure `rx_latency_ms` on real hardware
- [ ] Confirm on a spectrum analyzer or SDR that the dwell is 400 ms and the hops look random
