# Range test

Measures the link at increasing distances, in both directions:

- **Packet error rate (PER):** how many packets fail to arrive or fail the CRC.
- **Bit error rate (BER):** wrong bits in the packets that did arrive, counted against a known test pattern.
- **RSSI and SNR** of every packet.
- **Link margin:** how far the worst SNR is above the level the radio can still decode.
- **Lock time:** the share of time the ground stayed locked to the pod's hopping.

The pod runs on a battery and sends a known pattern. The ground stays at the laptop and logs every packet. The pod also reports how many ground → pod commands it received and their RSSI/SNR, so the up direction is measured too.

## What you need

- Both NUCLEO-WL55JC2 + SX1262 shields with antennas. No CAN modules: range mode doesn't use them.
- A USB power bank and a micro-USB cable for the pod.
- The laptop with the ground board on USB.
- Something to measure distance: a phone map or GPS app, or a measuring wheel for short ranges.

## Flash both boards (once, at the bench)

From `boards/nucleo_wl55jc`, with both boards on USB:

```sh
make flash-swd APP=bridge ROLE=pod RANGE=1 TXDBM=14 SERIAL=0052003B3333511831363730
make flash-swd APP=bridge ROLE=ground RANGE=1 TXDBM=14 SERIAL=003200263433511437363934
```

- **Same settings on both boards.** Both need the same `TXDBM` and `BW250` settings.
- **Power:** `TXDBM` can be anywhere from −9 to 22 dBm. Start at 14 dBm, then repeat at 22 dBm to see the power headroom.
- **250 kHz:** add `BW250=1` to both commands to test the 250 kHz option.

## Setup in the field

1. **Ground:** put it at a fixed spot on the laptop. Keep the antenna vertical, about 1–1.5 m off the ground (on a box or table), and clear of the laptop and your body.
2. **Pod:** power it from the power bank. Its blue LED toggles every hop. Hold or mount it at the same height, antenna vertical.
3. **Path:** a straight line of sight (a field, a long road or a parking lot). Write down anything in the path.
4. **Start the logger:**
   ```sh
   python3 tools/range/range_log.py
   ```
   It asks for setup notes (antennas, heights, place, power), then prints a running summary every 5 s.
5. **First stop:** start next to the ground. Type `0` and press Enter, wait until it shows `locked` (up to 20 s), then let it run for 60 s.
6. **Walk the pod out in steps.** For example 25, 50, 100, 200, 400, 800 m and so on. At each stop:
   - type the distance (for example `200` or `200 behind trees`) and press Enter,
   - hold the pod still for about 60 s (150 hops),
   - watch the summary.
7. **Keep going until the PER passes ~50 % or the ground shows `SEARCHING`.** That stop is the edge of the range. Note it, then come back a step or two.
8. **Finish:** type `q` and press Enter. The log is saved in `logs/range/<date>/`.

Hold the pod the same way at every stop, since a body in the way costs several dB. Each 60 s stop is about 300 packets down and 150 up, enough to tell a few percent PER from zero.

## Report

```sh
python3 tools/range/range_report.py logs/range/<date>
```

This writes the following into that folder:

| File | Contents |
|---|---|
| `report.md` | Setup, link settings, a results table per stop with 95 % confidence intervals, and how each number is measured |
| `stops.csv` | The same numbers, one row per stop |
| `packets.csv` | Every received packet (stop, time, hop, channel, CRC, RSSI, SNR, bit errors, counters), for MATLAB or Excel |
| `range_plots.png` | PER, BER, RSSI and SNR vs distance. Needs matplotlib, and the stops labelled with numbers |

## Log format (ground board, RANGE=1)

```
RCFG,ground,<bandwidth_hz>,<pod_slot_ms>,<tx_dbm>,<payload_bytes>,<sf>       once at start
PKT,<ms>,<hop>,<ch>,<crc_ok>,<rssi_dbm>,<snr_q4>,<bit_errors>,<bits>,<seq>,<up_ok>,<up_rssi>,<up_snr_q4>,<cmds_sent>
STAT,<ms>,<locked>,<crc_errors>,<header_errors>,<cmds_sent>,<rx_ok>,<rx_bad>,<bit_errors>,<bits>   every second
```

- `snr_q4` is in quarter dB.
- `seq` and the `up_*` fields are `-1` when the CRC failed.
- `bits` is 568: the magic byte plus the 70-byte pattern. The pod's counters can't be checked, so they aren't counted.
- The logger puts the laptop's clock in front of each line and adds `MARK,<distance>` lines where you typed a stop.
