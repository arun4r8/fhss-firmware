## Summary

**First range test, 2026-11-14, Zilker Park (Austin).**

| | |
|---|---|
| Setup | Both boards about 1 m high, stock shield antennas vertical, 14 dBm, LoRa SF7 / 125 kHz, 330/70 slot split |
| Ground | On the laptop over USB, logging |
| Pod | On a USB power bank, carried out on foot |

| Range | Pod → ground (telemetry) | Ground → pod (commands) | Link |
|---|---|---|---|
| 0–50 m | **≤ 1 % packet loss**, BER ≤ 6e-5 | ≤ 1 % loss | Locked throughout |
| 75–100 m | **8–13 % loss**, BER 2–3e-3 | ≈ 9 % loss | Locked throughout |
| 150 m and beyond (distance not recorded after 150 m) | First minute 15 %, then **25–60 %** | 61 % loss over the stop | Lost lock 3 times in minutes 2–4, each regained within 4–18 s, then held |

**Findings**

1. **Range was limited by noise, not by signal level.**
   - At 150 m and beyond, packets arrived at about −94 dBm with SNR near 0 dB. The noise in the channel was therefore about −95 to −99 dBm at the ground (estimated in the table below).
   - The SX1262 should see about −117 dBm (thermal noise plus its noise figure) and decode down to about −124 dBm at SF7.
   - So the signal still had about 30 dB to spare, but noise about 20 dB above normal used it up.
   - The pod side was worse: its commands arrived at −88 dBm with SNR around −5 dB (noise about −89 dBm), which is why commands up were lost more often than telemetry down.
2. **No single bad channel.**
   - CRC failures were spread across all 50 channels: 5–33 % each, where chance alone gives about 10–31 %.
   - Failed packets had a median SNR of −0.5 dB, against +4.5 dB for good ones.
   - That fits fading plus a high noise floor, not one interferer or a timing problem.
3. **Bit errors only appear once packets start failing.**
   - BER was 0 in 377,152 bits checked up to 25 m (under 1e-5), and 18 errors in 714,544 bits up to 50 m (2.5e-5), all in the 5 packets that failed the CRC.
   - When a packet fails, the errors are few: median 6 bits wrong of 568.
   - So stronger error correction (coding rate 4/8 instead of 4/5) or a higher spreading factor would recover many of the failed packets, at the cost of airtime.
4. **The firmware held up at the edge of range.**
   - After each loss of lock the ground relocked within 4–18 s, against up to 20 s by design.
   - The pod kept sending 2 frames every hop (about 5/s) throughout.

**Likely noise sources, to check next**

The ground board was powered from the laptop's USB port during the test (team note, 11-15), so the laptop is the most likely source of the ground-side noise. It doesn't explain the pod side: the pod was on a power bank far from the laptop, yet its noise (about −89 dBm) was worse than the ground's (−95 to −99 dBm).

| Suspect | Side | Why | Test |
|---|---|---|---|
| **Laptop USB power and the laptop itself** | Ground | Ground radio was powered from, and sitting next to, the laptop: a common source of broadband noise | Power the ground board from a battery or a USB isolator, antenna on a cable 2 m from the laptop; repeat the noise scan |
| Power bank switching noise | Pod | Pod-side noise was the worst | Pod on a different battery, or a linear supply |
| MCU clock | Both | It went from 4 to 48 MHz just before this test. 19 × 48 MHz = 912 MHz, inside the band, and the worst channel was 912.7 MHz. The self-test noise scan on the bench was −102 dBm average at 48 MHz (11-14) against −111 dBm at 4 MHz (10-04). | Noise scan (self-test) at 4 and 48 MHz in the same place |
| Phones not in airplane mode (RF lead, 11-15) | Both | Phones carried next to both radios can desensitize the receivers; the pod was carried by a person | Repeat with every phone in airplane mode |
| Nearby cell-tower blockers (RF lead, 11-15) | Both | Strong out-of-band signals raise the receiver's effective noise | SDR sweep around the band at the site |
| Stock antennas with poor matching (RF lead, 11-15) | Both | Mismatch loses signal at both ends | VNA S11 on the stock antennas, or swap in matched antennas |
| Ambient 915 MHz traffic | Both | Smart meters, RFID, LoRaWAN | Brennen's SDR sweep of 902–928 MHz at the test site |

**Next tests**
- Retest under controlled conditions: ground on battery, phones in airplane mode, matched antennas.
- Then try 22 dBm, raise both antennas to about 2 m, and record the distance at every stop (a phone GPS pin works).
