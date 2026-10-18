# MATLAB hop tools

A time vs channel plot of what the boards actually do, checked against what the hop sequence says they should do. These cover steps 1–3 of the SDR plan; later, the FPGA's FFT measurement becomes a third track on the same plot.

| File | What it does |
|---|---|
| `fhss_sequence.m` | The hop sequence from a seed, a bit-exact port of `src/fhss.c` |
| `read_hop_log.m` | Parses the bridge firmware's `CFG` / `HOP` / `TX` / `RX` log lines |
| `hop_plot.m` | Draws the plot and runs the checks |
| `hop_monitor.m` | Live plot from a board on USB; saves the capture |
| `lora_airtime_ms.m` | Packet time on air, same formula as the firmware |
| `test_fhss_sequence.m`, `test_hop_tools.m` | Tests (run both after any change) |
| `gen_ref_sequences.c`, `ref_sequences.csv` | Reference sequences from the real C code; `make test` fails if they go stale |

## Live plot from a board

1. Flash any bridge build (`make flash APP=bridge ROLE=pod DEMO=1` from `boards/nucleo_wl55jc`). They all print the log lines.
2. Close `make monitor` or any other serial window. Only one program can hold the port.
3. In MATLAB, `cd` to this folder and run:
   ```matlab
   hop_monitor
   ```
   It finds the Nucleo's port, shows the last 20 s, and saves everything to `hop_capture_<date>.txt`. Status lines print in the Command Window. Close the figure to stop.

To plot a saved capture, including the whole run:
```matlab
r = hop_plot(read_hop_log('hop_capture_20261018_120000.txt'))
```

## Reading the plot

- **x:** time in seconds on the board's clock. **y:** channel 0–49 (frequency = 902.2 + 0.5 × channel MHz).
- **Grey steps:** where `fhss_sequence(seed)` says the board should be.
- **Orange dots:** channel changes the board logged. Hollow means the follower is still searching.
- **Blue bars:** transmissions, drawn for their real time on air.
- **Red triangles:** packets received.
- **Red rings:** problems, meaning a hop on the wrong channel or a packet that ran past the next hop.

The title and the returned struct give the checks:

| Field | Healthy value |
|---|---|
| `mismatches` | 0 |
| `dwell_ms` | about 400–401 (logging adds a ms or two) |
| `min_revisit_s` | 20.0: each channel once per 50 × 400 ms cycle |
| `channel_uses` | all equal over whole cycles |
| `tx_straddles` | 0 |

## Log format (from `boards/nucleo_wl55jc/bridge.c`)

```
CFG,<role>,<seed>,<channels>,<base_hz>,<spacing_hz>,<dwell_ms>   once at start
HOP,<ms>,<hop_index>,<channel>,<H|S>   each channel change (S = searching)
TX,<ms>,<hop_index>,<channel>,<payload bytes>   ms = start of the transmission
RX,<ms>,<hop_index>,<channel>          a packet from our network arrived
```

`hop_index` is the position in the sequence, so a pod log and a ground log line up by hop even though each board's clock starts at its own power-up.
