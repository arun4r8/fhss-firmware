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
- **NUCLEO-WL55JC + SX1262 shield (the team's current hardware)**: [`boards/nucleo_wl55jc`](boards/nucleo_wl55jc/README.md) is a ready-to-flash self-test, built with only `arm-none-eabi-gcc` and flashed by drag-and-drop.

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
