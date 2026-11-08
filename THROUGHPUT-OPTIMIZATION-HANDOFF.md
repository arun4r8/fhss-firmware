# FHSS Link Throughput Optimization — Handoff Notes

**Context for the next chat:** Guadaloop pod↔ground telemetry link (repo: `~/Downloads/fhss-firmware`,
`arun4r8/fhss-firmware`). Goal: **maximize pod→ground telemetry throughput** within the TDD budget.
Arun talked to Brennen + the embedded team; there are **38 sensors total**. This doc captures a
scoping/constraints discussion so another chat can take over the actual design/build.

> Status (2026-11-08): **built in host emulation, uncommitted** — 38-sensor tag-free frame (77 B), 330/70 split,
> 5 frames/s; not yet on hardware. Next candidate: SF7 / 250 kHz at a 340/60 split (10 frames/s), pending Brennen.
> Original status: **scoping only — nothing built yet.** Several constraints still pending from the user and
> embedded (see "Open constraints"). Verify the repo facts below against the current code before acting.

---

## The optimization ideas on the table (from Arun)
1. **Drop the sensor tags** (~10 bytes) — either send all 38 sensors at once, or split into 4 packets
   with a hardcoded sensor→packet mapping.
2. **One big packet instead of 4**, "use the extra channels to split the data and resync on arrival"
   to fit more full packets in the budget.
3. **Lopsided dwell times** — ground→pod needs to send far less, so shorten the ground→pod slot and
   give that time to pod→ground.
4. Maximize throughput overall.

---

## Hardware/firmware constraints VERIFIED from the repo (2026-11-01)

### Radio: single 915 MHz chain — NO parallel channels possible
- Board is **NUCLEO-WL55JC2** = the **low-band** variant; its antenna/RF front end is tuned for
  **433–510 MHz, not 915**. Per `boards/nucleo_wl55jc/README.md`, the WL55's internal radio is
  unusable in-band ("don't transmit on the Nucleo's own antenna connector"); the WL55 is just the MCU.
- The only 915 MHz radio is the single **SX1262MB2xAS shield**, which **shares SPI1 with the MCP2515
  CAN controller** (`boards/nucleo_wl55jc/board.h`). One SPI bus, one radio, one TX/RX chain, half-duplex.
- **Implication for idea #2:** FHSS here is *sequential hopping on one radio* — you cannot be on two
  channels at once. Coalescing packets is still worthwhile, but the gain is **overhead reduction**
  (one preamble + one 6-byte hop header + one turnaround instead of four), **not** parallel bandwidth.
- *Theoretical only:* the WL55 internal radio could be a separate link on 433–510 MHz (real band
  isolation), but needs a 433 antenna + an unwritten SUBGHZ driver + restrictive 433 ISM duty-cycle
  rules. Not a practical high-throughput pipe.

### PHY: already near the regulatory throughput ceiling
- Current config (`boards/nucleo_wl55jc/selftest.c`): **LoRa, SF7 / BW 125 kHz / CR 4/5**, preamble 8,
  channel spacing 500 kHz. Driver (`ports/nucleo_sx1262/sx1262.c`) is **LoRa-only — no FSK path written.**
- Airtime at SF7/BW125 ≈ **~70 ms per 31-byte packet** (Semtech formula in `sx1262_airtime_ms`).
  README quotes ~**5× 31-byte payloads/sec each way** as a result.
- **FCC 15.247 path the repo targets:** 400 ms dwell/channel, ≥50 channels, never revisit within 20 s,
  20 dB BW < 250 kHz, channel spacing ≥ 20 dB BW (`README.md`, `src/fhss.c`). *(Correction 2026-11-08:
  this does NOT pin BW at 125 kHz. A 250 kHz signal falls under the ≥250 kHz rule — ≥25 channels,
  ≤0.4 s per 10 s — which the current 50 × 500 kHz / 400 ms plan already meets; see the SF7/250 analysis.)*
  The original note said it
  effectively **pins BW at 125 kHz**, and SF7 is already the fastest standard SF → **the LoRa PHY is
  near its throughput ceiling; byte-level tweaks won't change that much.**

### Payload structure (from `README.md` "CAN bridge")
- Max radio payload **31 bytes**. Per packet: hopping layer adds **6 bytes** (network ID, flags,
  4-byte hop sync); hub format adds **1 byte + 1 byte per sensor** (the "tags"); readings are
  **2 bytes per sensor**.
- Example: a 10-sensor message = 37 bytes on air (6 hop + 31 payload), of which 20 bytes are readings.
- Messages too big for 31 bytes are split on sensor-record boundaries; each piece is self-valid.
- CAN comes in via the MCP2515 (`src/can_bridge.c`); pod side keeps newest-unsent per sender,
  round-robins senders to the radio.

---

## Analysis / recommended levers

### On current hardware + LoRa PHY (incremental, ~1.5–2× stacked)
1. **Kill the per-sensor tags via a fixed schema.** At 31-byte payload, every byte freed = another
   reading. Requires both ends to commit to a static field order (confirm embedded can guarantee a
   fixed sensor set — no dynamic membership).
2. **Coalesce packets** to cut per-packet preamble + 6-byte hop header + turnaround overhead.
3. **Asymmetric TDD split** — give more of each dwell to pod→ground once we know the uplink size.

### The big lever (NOT yet on Arun's list): LoRa → GFSK
- SX1262 does GFSK up to ~300 kbps; even conservative ~50–100 kbps GFSK in a <250 kHz channel is
  FCC-FHSS-legal and **~10×+** the SF7 throughput.
- **Cost:** FSK driver must be written (doesn't exist), and GFSK is less sensitive than LoRa, so
  **range/link margin must be re-validated.** If throughput is the real goal, this dominates the
  byte-level optimizations. **This is the key decision for the next chat to resolve.**

### Dual-radio parallelism
- **Not available on current hardware** (JC2 band-lock, shared-SPI single radio). Would require new
  hardware (a second 915 radio with antenna isolation on both pod and ground).

---

## Open constraints still needed (blocking a real model)
1. **Range + link margin** (max pod↔ground distance) — decides whether GFSK / lower SF is viable
   without losing the link. *(from Arun)*
2. **Per-sensor update rates** — do all 38 need to go every frame, or can slow sensors round-robin
   across frames? Biggest payload-side lever; not in the firmware. *(from embedded)*
3. **Ground→pod payload + rate** — what actually goes up, to size how much dwell to steal for
   downlink. *(from embedded — the number Arun flagged)*
4. **Timing reconciliation** — firmware hops on a **400 ms dwell** (`src/fhss.c`), but Arun described
   a **200 ms** pod→ground / ground→pod budget. Are the 200 ms slots the two half-dwell TDD windows
   inside a 400 ms hop, or is the dwell being changed? *(from Arun)*
5. Confirm the "10 bytes of tags" figure vs the code's **1 byte/sensor** tag (38 sensors → ~38 tag
   bytes); clarify whether "10 bytes" was per-packet or a specific message.

---

## Suggested next step for the new chat
Once (1) and (4) and the **GFSK-vs-stay-LoRa** call are settled: produce a side-by-side throughput
model for LoRa-SF7 vs GFSK, then a concrete frame layout (fixed schema, coalesced payload, asymmetric
TDD split) with the resulting payloads/sec each way. Do not build until those constraints land.
