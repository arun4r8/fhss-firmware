#!/usr/bin/env python3
"""Turn range-test logs into a report.

    python3 tools/range/range_report.py logs/range/2026-11-14

Reads every range_*.log in the folder (written by range_log.py) and writes,
in the same folder:
    report.md      setup, link configuration, results per stop, method
    stops.csv      the same numbers, one row per stop
    packets.csv    every received packet with its stop, for MATLAB/Excel
    range_plots.png  PER, BER, RSSI and SNR vs distance (needs matplotlib)

Standard library only, except the optional plot.
"""
import csv
import glob
import math
import os
import re
import sys

SENSITIVITY_DBM = {7: -124.0, 8: -127.0, 9: -129.5}   # SX1262, 125 kHz (datasheet)
SNR_LIMIT_DB = {5: -2.5, 6: -5.0, 7: -7.5, 8: -10.0, 9: -12.5, 10: -15.0, 11: -17.5, 12: -20.0}


def airtime_ms(payload, sf=7, bw=125000, cr=1, preamble=8):
    """Semtech formula, same as sx1262_airtime_ms (explicit header, CRC on)."""
    de = 1 if (2 ** sf) / bw * 1000 > 16 else 0
    num = 8 * payload - 4 * sf + 28 + 16
    blocks = max(math.ceil(num / (4 * (sf - 2 * de))), 0)
    syms = 8 + blocks * (cr + 4)
    return (preamble + 4.25 + syms) * (2 ** sf) / bw * 1000


# ---- confidence intervals (no scipy) --------------------------------------

def wilson(k, n, z=1.96):
    """95 % interval for a proportion k/n."""
    if n == 0:
        return (float('nan'), float('nan'))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def poisson_cdf(k, lam):
    """P(X <= k), summed in log space so large counts don't underflow."""
    if lam <= 0:
        return 1.0
    logs = [i * math.log(lam) - lam - math.lgamma(i + 1) for i in range(k + 1)]
    top = max(logs)
    return min(1.0, math.exp(top) * sum(math.exp(x - top) for x in logs))


def poisson_ci(k, z_lo=0.025, z_hi=0.975):
    """Exact 95 % interval for a Poisson count k (bisection)."""
    def solve(target, lo_count):
        lo, hi = 0.0, max(10.0, 2.0 * k + 10.0 * math.sqrt(k + 1) + 10.0)
        for _ in range(100):
            mid = (lo + hi) / 2
            v = poisson_cdf(lo_count, mid)
            if v > target:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2
    lower = 0.0 if k == 0 else solve(z_hi, k - 1)
    upper = solve(z_lo, k)
    return lower, upper


# ---- parsing ---------------------------------------------------------------

def parse_log(path):
    log = {'file': os.path.basename(path), 'notes': '', 'rcfg': {}, 'stops': []}
    stop = None

    def new_stop(label, t):
        s = {'label': label, 't0': t, 't1': t, 'pkts': [], 'stats': []}
        log['stops'].append(s)
        return s

    for raw in open(path, errors='replace'):
        raw = raw.rstrip('\n')
        if ' ' not in raw:
            continue
        t_s, line = raw.split(' ', 1)
        try:
            t = float(t_s)
        except ValueError:
            continue
        f = line.split(',')
        if f[0] == 'NOTE':
            log['notes'] = line[5:]
        elif f[0] == 'RCFG' and len(f) >= 7:
            log['rcfg'][f[1]] = {'bw': int(f[2]), 'slot': int(f[3]), 'tx_dbm': int(f[4]),
                                 'payload': int(f[5]), 'sf': int(f[6])}
        elif f[0] == 'MARK':
            stop = new_stop(line[5:], t)
        elif stop is None:
            continue
        elif f[0] == 'PKT' and len(f) >= 14:
            try:
                v = [int(x) for x in f[1:14]]
            except ValueError:
                continue
            stop['pkts'].append({'t': t, 'ms': v[0], 'hop': v[1], 'ch': v[2], 'ok': v[3] == 1,
                                 'rssi': v[4], 'snr': v[5] / 4, 'err': v[6], 'bits': v[7], 'seq': v[8],
                                 'up_ok': v[9], 'up_rssi': v[10], 'up_snr': v[11] / 4, 'cmds_sent': v[12]})
            stop['t1'] = t
        elif f[0] == 'STAT' and len(f) >= 10:
            try:
                v = [int(x) for x in f[1:10]]
            except ValueError:
                continue
            stop['stats'].append({'t': t, 'locked': v[1] == 1, 'crc': v[2], 'hdr': v[3], 'cmds': v[4]})
            stop['t1'] = t
    return log


def distance_m(label):
    m = re.match(r'\s*([0-9]+(?:\.[0-9]+)?)\s*(km|m|ft)?', label)
    if not m:
        return None
    v = float(m.group(1))
    return v * 1000 if m.group(2) == 'km' else v * 0.3048 if m.group(2) == 'ft' else v


def percentile(xs, q):
    if not xs:
        return float('nan')
    xs = sorted(xs)
    k = (len(xs) - 1) * q / 100
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def bins_over_time(stop, secs=60):
    """Per-minute numbers for a long stop: (start s, received, ok, sent, PER, RSSI, SNR, locked %)."""
    p = stop['pkts']
    if not p:
        return []
    t0 = stop['t0']
    out = []
    n = int((stop['t1'] - t0) // secs) + 1
    for b in range(n):
        q = [x for x in p if b * secs <= x['t'] - t0 < (b + 1) * secs]
        ok = [x for x in q if x['ok']]
        seqs = [x['seq'] for x in ok]
        sent = (max(seqs) - min(seqs) + 1) if len(seqs) > 1 and max(seqs) - min(seqs) < 30000 else len(seqs)
        st = [s for s in stop['stats'] if b * secs <= s['t'] - t0 < (b + 1) * secs]
        out.append({'start': b * secs, 'rx': len(q), 'ok': len(ok), 'sent': sent,
                    'per': 1 - len(ok) / sent if sent else float('nan'),
                    'rssi': mean([x['rssi'] for x in q]), 'snr': mean([x['snr'] for x in q]),
                    'locked': 100 * sum(s['locked'] for s in st) / len(st) if st else float('nan')})
    return out


def mean(xs):
    return sum(xs) / len(xs) if xs else float('nan')


def stdev(xs):
    if len(xs) < 2:
        return float('nan')
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def summarize(log, stop):
    p = stop['pkts']
    ok = [x for x in p if x['ok']]
    bad = [x for x in p if not x['ok']]
    r = {'file': log['file'], 'stop': stop['label'], 'distance_m': distance_m(stop['label']),
         'duration_s': stop['t1'] - stop['t0']}
    st = stop['stats']
    r['locked_pct'] = 100 * sum(s['locked'] for s in st) / len(st) if st else float('nan')
    r['header_errors'] = (st[-1]['hdr'] - st[0]['hdr']) if len(st) > 1 else 0

    # downlink: packets sent from the pod's counter (unwrapped), received ok / CRC-failed
    seqs = []
    off = 0
    for x in ok:
        s = x['seq'] + off
        if seqs and s < seqs[-1] - 30000:
            off += 65536
            s += 65536
        seqs.append(s)
    sent = (seqs[-1] - seqs[0] + 1) if seqs else 0
    r['down_sent'] = sent
    r['down_ok'] = len(ok)
    r['down_crc_fail'] = len(bad)
    r['down_lost'] = max(sent - len(ok) - len(bad), 0) if sent else float('nan')
    r['down_per'] = 1 - len(ok) / sent if sent else float('nan')
    r['down_per_lo'], r['down_per_hi'] = wilson(sent - len(ok), sent) if sent else (float('nan'),) * 2
    secs = r['duration_s'] or float('nan')
    r['down_ok_per_s'] = len(ok) / secs if secs == secs and secs > 0 else float('nan')

    # bit errors over every packet that arrived (CRC-passed ones have none)
    err = sum(x['err'] for x in p)
    bits = sum(x['bits'] for x in p)
    r['bit_errors'], r['bits_checked'] = err, bits
    r['ber'] = err / bits if bits else float('nan')
    lo, hi = poisson_ci(err) if bits else (float('nan'), float('nan'))
    r['ber_lo'] = lo / bits if bits else float('nan')
    r['ber_hi'] = hi / bits if bits else float('nan')

    rssi = [x['rssi'] for x in p]
    snr = [x['snr'] for x in p]
    r['rssi_mean'], r['rssi_min'], r['rssi_max'], r['rssi_std'] = (
        mean(rssi), min(rssi, default=float('nan')), max(rssi, default=float('nan')), stdev(rssi))
    r['snr_mean'], r['snr_min'], r['snr_max'] = mean(snr), min(snr, default=float('nan')), max(snr, default=float('nan'))
    r['snr_p10'] = percentile(snr, 10)
    # noise + interference in the channel: packet RSSI is signal + noise, SNR their ratio
    # (only from packets with SNR <= 8 dB: the SX1262's SNR reading saturates near +12 dB, so strong
    # packets say nothing about the noise)
    noise = [x['rssi'] - 10 * math.log10(1 + 10 ** (x['snr'] / 10)) for x in p if x['snr'] <= 8]
    if len(noise) < 10:
        noise = []
    r['noise_dbm'] = percentile(noise, 50)
    r['lock_losses'] = sum(1 for a, b in zip(st, st[1:]) if a['locked'] and not b['locked'])

    # uplink: commands the ground sent vs commands the pod reports receiving
    if len(ok) > 1:
        sent_up = ok[-1]['cmds_sent'] - ok[0]['cmds_sent']
        got_up = ok[-1]['up_ok'] - ok[0]['up_ok']
        r['up_sent'], r['up_ok'] = sent_up, got_up
        r['up_per'] = 1 - got_up / sent_up if sent_up > 0 else float('nan')
        lo, hi = wilson(max(sent_up - got_up, 0), sent_up) if sent_up > 0 else (float('nan'),) * 2
        r['up_per_lo'], r['up_per_hi'] = lo, hi
        seen = {}
        for x in ok:   # each command's RSSI/SNR once (repeated until the next command)
            seen.setdefault(x['up_ok'], (x['up_rssi'], x['up_snr']))
        vals = [v for k, v in seen.items() if k > 0]
        r['up_rssi_mean'] = mean([v[0] for v in vals])
        r['up_rssi_min'] = min((v[0] for v in vals), default=float('nan'))
        r['up_snr_mean'] = mean([v[1] for v in vals])
    else:
        for k in ('up_sent', 'up_ok', 'up_per', 'up_per_lo', 'up_per_hi', 'up_rssi_mean', 'up_rssi_min', 'up_snr_mean'):
            r[k] = float('nan')
    return r


# ---- output ------------------------------------------------------------------

def fmt(v, spec='.1f', none='–'):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return none
    return format(v, spec)


def pct(v, lo=None, hi=None):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return '–'
    s = f'{100 * v:.1f} %'
    if lo is not None and hi is not None and not math.isnan(lo):
        s += f' ({100 * lo:.1f}–{100 * hi:.1f})'
    return s


def ber_str(r):
    if not r['bits_checked']:
        return '–'
    if r['bit_errors'] == 0:
        return f"0 errors in {r['bits_checked']:,} bits (< {r['ber_hi']:.1e})"
    return f"{r['ber']:.1e} ({r['ber_lo']:.1e}–{r['ber_hi']:.1e})"


def write_report(folder, logs, rows):
    cfg = {}
    for lg in logs:
        cfg.update(lg['rcfg'].get('pod', {}) or lg['rcfg'].get('ground', {}))
    sf, bw = cfg.get('sf', 7), cfg.get('bw', 125000)
    payload = cfg.get('payload', 77)
    air = airtime_ms(payload + 6, sf, bw)
    sens = SENSITIVITY_DBM.get(sf) if bw == 125000 else None
    snr_lim = SNR_LIMIT_DB.get(sf)

    L = ['# Range test report', '']
    L.append('Texas Guadaloop RF comms, FHSS link between the pod and ground boards. '
             'Generated by `tools/range/range_report.py` from the logs listed at the end.\n')
    summary = os.path.join(folder, 'summary.md')
    if os.path.exists(summary):   # hand-written findings for this test
        L.append(open(summary).read().strip() + '\n')
    L.append('## Setup\n')
    for lg in logs:
        L.append(f"- `{lg['file']}`: {lg['notes'] or '(no notes)'}")
    L.append('')
    L.append('| Parameter | Value |\n|---|---|')
    L.append(f'| Radios | 2 × Semtech SX1262 (SX1262MB2xAS shield) on NUCLEO-WL55JC2 |')
    L.append(f'| Modulation | LoRa SF{sf}, {bw // 1000} kHz, CR 4/5, 8-symbol preamble, explicit header, CRC on |')
    L.append(f'| Hopping | 50 channels 902.2–926.7 MHz, 500 kHz apart, 400 ms per channel |')
    L.append(f"| TX power | {cfg.get('tx_dbm', '?')} dBm (both boards) |")
    L.append(f"| Pod → ground | {payload} B test payload + 6 B link header = {payload + 6} B, {air:.0f} ms on air, "
             f"pod slot {cfg.get('slot', '?')} ms of each 400 ms hop |")
    L.append(f"| Ground → pod | one test command per hop (fills the ground's slot) |")
    if sens:
        L.append(f'| SX1262 sensitivity (datasheet) | {sens:.0f} dBm; demodulation SNR limit {snr_lim} dB at SF{sf} |')
    L.append('')

    L.append('## Results\n')
    L.append('Pod → ground ("down") is the telemetry direction. Intervals in brackets are 95 % confidence.\n')
    L.append('| Stop | Time | Locked (lost) | Down: sent / ok / CRC fail | Down PER | BER (received packets) | RSSI mean (min) | SNR mean (min) | Noise floor | Margin | Up: sent / ok | Up PER | Up RSSI / SNR |')
    L.append('|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for r in rows:
        margin = r['snr_p10'] - snr_lim if snr_lim is not None and not math.isnan(r['snr_p10']) else float('nan')
        L.append(f"| {r['stop']} | {fmt(r['duration_s'], '.0f')} s | {fmt(r['locked_pct'], '.0f')} % ({r['lock_losses']}×) | "
                 f"{r['down_sent']} / {r['down_ok']} / {r['down_crc_fail']} | "
                 f"{pct(r['down_per'], r['down_per_lo'], r['down_per_hi'])} | {ber_str(r)} | "
                 f"{fmt(r['rssi_mean'], '.0f')} ({fmt(r['rssi_min'], '.0f')}) dBm | "
                 f"{fmt(r['snr_mean'])} ({fmt(r['snr_min'])}) dB | {fmt(r['noise_dbm'], '.0f')} dBm | {fmt(margin)} dB | "
                 f"{fmt(r['up_sent'], '.0f')} / {fmt(r['up_ok'], '.0f')} | "
                 f"{pct(r['up_per'], r['up_per_lo'], r['up_per_hi'])} | "
                 f"{fmt(r['up_rssi_mean'], '.0f')} dBm / {fmt(r['up_snr_mean'])} dB |")
    L.append('')
    if os.path.exists(os.path.join(folder, 'range_plots.png')):
        L.append('![PER, BER, RSSI and SNR vs distance](range_plots.png)\n')
    if os.path.exists(os.path.join(folder, 'range_timeline.png')):
        L.append('![The whole test over time](range_timeline.png)\n')

    long = [(lg, s) for lg in logs for s in lg['stops'] if s['t1'] - s['t0'] > 150 and s['pkts']]
    if long:
        L.append('## Long stops, minute by minute\n')
        for lg, s in long:
            L.append(f"**{s['label']}** ({s['t1'] - s['t0']:.0f} s):\n")
            L.append('| Minute | Down: sent / ok | Down PER | RSSI mean | SNR mean | Locked |\n|---|---|---|---|---|---|')
            for b in bins_over_time(s):
                L.append(f"| {b['start'] // 60 + 1} | {b['sent']} / {b['ok']} | {pct(b['per'])} | {fmt(b['rssi'], '.0f')} dBm | "
                         f"{fmt(b['snr'])} dB | {fmt(b['locked'], '.0f')} % |")
            L.append('')

    lossy = [s for lg in logs for s in lg['stops'] if s['pkts'] and
             sum(not x['ok'] for x in s['pkts']) >= 0.05 * len(s['pkts'])]
    if lossy:
        ch = {}
        for s in lossy:
            for x in s['pkts']:
                c = ch.setdefault(x['ch'], [0, 0])
                c[0 if x['ok'] else 1] += 1
        fails = sorted(((b / (a + b), k, a + b) for k, (a, b) in ch.items() if a + b >= 10), reverse=True)
        if fails:
            tot_bad = sum(b for a, b in ch.values())
            tot = sum(a + b for a, b in ch.values())
            avg = tot_bad / tot
            n_avg = tot / len(ch)
            spread = 1.96 * math.sqrt(avg * (1 - avg) / n_avg)
            L.append('## CRC failures by channel\n')
            L.append(f"Stops with 5 % or more CRC failures ({', '.join(s['label'] for s in lossy)}): {tot} packets received on "
                     f"{len(ch)} channels, {100 * avg:.0f} % failed the CRC. With about {n_avg:.0f} packets per channel, pure chance "
                     f"spreads that over roughly {100 * max(avg - spread, 0):.0f}–{100 * (avg + spread):.0f} %.\n")
            L.append('| | Channels (MHz, share failed) |\n|---|---|')
            fmtc = lambda xs: ', '.join(f"{902.2 + 0.5 * k:.1f} ({100 * f:.0f} %)" for f, k, n in xs)
            L.append(f'| Worst 5 | {fmtc(fails[:5])} |')
            L.append(f'| Best 5 | {fmtc(fails[-5:])} |')
            L.append('')

    L.append('## How each number is measured\n')
    L.append('- **Sent:** from the pod\'s transmit counter in the packets that passed the CRC (last − first + 1 at that stop), '
             'so packets lost before the first or after the last good one aren\'t counted.')
    L.append('- **Down PER:** 1 − (packets that passed the CRC ÷ sent). Covers both packets never detected and packets that failed the CRC. '
             '95 % interval: Wilson score.')
    L.append(f'- **BER:** the pod fills each packet with a known PRBS-9 pattern seeded by the hop index; the ground regenerates it and '
             f'counts wrong bits in every packet it received, including those that failed the CRC ({RANGE_BITS} bits checked per packet). '
             'Packets never detected aren\'t in the BER (they\'re in the PER). 95 % interval: exact Poisson; with 0 errors the upper bound is ~3 ÷ bits.')
    L.append('- **RSSI / SNR:** the SX1262\'s packet status for every received packet (SNR in 0.25 dB steps).')
    if snr_lim is not None:
        L.append(f'- **Margin:** 10th-percentile SNR at that stop minus the SF{sf} demodulation limit ({snr_lim} dB): how much more path loss '
                 'the link could take with 90 % of packets still above the limit.')
    L.append('- **Noise floor:** median over the stop of RSSI − 10·log10(1 + 10^(SNR/10)), the noise plus interference in the 125 kHz '
             'channel at the ground, from packets with SNR ≤ 8 dB (the SX1262’s SNR reading saturates near +12 dB, so stronger packets '
             'can’t show the noise; "–" means the signal was too strong to tell). Thermal noise plus the SX1262 noise figure is about '
             '−117 dBm, so a higher value means local noise or interference is setting the range rather than the signal level.')
    L.append('- **Up:** the ground sends one command per hop; the pod counts the ones it receives and reports the count, plus the last '
             'command\'s RSSI/SNR, inside its packets. Up PER = 1 − (received ÷ sent) over the stop (the pod\'s count lags by up to one hop).')
    L.append('- **Locked:** share of the ground\'s once-a-second status lines that showed it locked to the pod\'s hopping; '
             '(n×) is how many times it lost lock during the stop.')
    L.append('')
    L.append('Raw data: ' + ', '.join(f"`{lg['file']}`" for lg in logs) + '; per stop in `stops.csv`, every packet in `packets.csv`.')
    open(os.path.join(folder, 'report.md'), 'w').write('\n'.join(L) + '\n')


RANGE_BITS = 568


def write_csvs(folder, logs, rows):
    keys = list(rows[0].keys()) if rows else []
    with open(os.path.join(folder, 'stops.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: ('' if isinstance(v, float) and math.isnan(v) else v) for k, v in r.items()})
    with open(os.path.join(folder, 'packets.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['file', 'stop', 'distance_m', 'host_time', 'board_ms', 'hop', 'channel', 'crc_ok', 'rssi_dbm',
                    'snr_db', 'bit_errors', 'bits', 'seq', 'up_ok', 'up_rssi_dbm', 'up_snr_db', 'cmds_sent'])
        for lg in logs:
            for s in lg['stops']:
                d = distance_m(s['label'])
                for x in s['pkts']:
                    w.writerow([lg['file'], s['label'], '' if d is None else d, f"{x['t']:.3f}", x['ms'], x['hop'],
                                x['ch'], int(x['ok']), x['rssi'], x['snr'], x['err'], x['bits'],
                                x['seq'] if x['ok'] else '', x['up_ok'] if x['ok'] else '',
                                x['up_rssi'] if x['ok'] else '', x['up_snr'] if x['ok'] else '', x['cmds_sent']])


def write_plot(folder, rows):
    pts = [r for r in rows if r['distance_m'] is not None]
    if len(pts) < 2:
        return False
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    pts.sort(key=lambda r: r['distance_m'])
    d = [r['distance_m'] for r in pts]
    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    ax[0][0].plot(d, [100 * r['down_per'] for r in pts], 'o-', label='down (pod → ground)')
    ax[0][0].plot(d, [100 * r['up_per'] for r in pts], 's--', label='up (ground → pod)')
    ax[0][0].set_ylabel('PER (%)'); ax[0][0].legend(); ax[0][0].set_title('Packet error rate')
    ber = [max(r['ber'], 1e-7) if r['bits_checked'] else float('nan') for r in pts]
    ax[0][1].semilogy(d, ber, 'o-'); ax[0][1].set_title('Bit error rate (received packets; 0 drawn at 1e-7)')
    ax[1][0].plot(d, [r['rssi_mean'] for r in pts], 'o-', label='down mean')
    ax[1][0].plot(d, [r['rssi_min'] for r in pts], 'v:', label='down min')
    ax[1][0].plot(d, [r['up_rssi_mean'] for r in pts], 's--', label='up mean')
    ax[1][0].set_ylabel('RSSI (dBm)'); ax[1][0].legend(); ax[1][0].set_title('RSSI')
    ax[1][1].plot(d, [r['snr_mean'] for r in pts], 'o-', label='down mean')
    ax[1][1].plot(d, [r['snr_min'] for r in pts], 'v:', label='down min')
    ax[1][1].axhline(-7.5, color='r', lw=0.8, label='SF7 limit')
    ax[1][1].set_ylabel('SNR (dB)'); ax[1][1].legend(); ax[1][1].set_title('SNR')
    for a in ax.flat:
        a.set_xlabel('distance (m)'); a.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(folder, 'range_plots.png'), dpi=120)
    return True


def write_timeline(folder, logs):
    pk = [(x, s['label']) for lg in logs for s in lg['stops'] for x in s['pkts']]
    if len(pk) < 20:
        return False
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    t0 = min(x['t'] for x, _ in pk)
    step = 10
    nb = int((max(x['t'] for x, _ in pk) - t0) // step) + 1
    per, rssi, snr, tt = [], [], [], []
    for b in range(nb):
        q = [x for x, _ in pk if b * step <= x['t'] - t0 < (b + 1) * step]
        ok = [x['seq'] for x in q if x['ok']]
        sent = (max(ok) - min(ok) + 1) if len(ok) > 1 and max(ok) - min(ok) < 30000 else 0
        tt.append((b + 0.5) * step / 60)
        per.append(100 * (1 - len(ok) / sent) if sent else float('nan'))
        rssi.append(mean([x['rssi'] for x in q]))
        snr.append(mean([x['snr'] for x in q]))
    lost = [(s['t'] - t0) / 60 for lg in logs for st in lg['stops'] for s in st['stats'] if not s['locked']]
    fig, ax = plt.subplots(3, 1, figsize=(11, 7.5), sharex=True)
    ax[0].plot(tt, per, '-', color='#c0392b'); ax[0].set_ylabel('down PER (%)'); ax[0].set_ylim(-2, 102)
    if lost:
        ax[0].plot(lost, [100] * len(lost), '|', color='k', ms=10, label='ground not locked')
        ax[0].legend(loc='upper left')
    ax[1].plot(tt, rssi, '-', color='#2c6fbb'); ax[1].set_ylabel('RSSI (dBm)')
    ax[2].plot(tt, snr, '-', color='#2e8b57'); ax[2].axhline(-7.5, color='r', lw=0.8, ls='--')
    ax[2].set_ylabel('SNR (dB)'); ax[2].set_xlabel('minutes from start (10 s averages)')
    for lg in logs:
        for st in lg['stops']:
            x = (st['t0'] - t0) / 60
            for a in ax:
                a.axvline(x, color='0.6', lw=0.8, ls=':')
            ax[0].text(x, 104, st['label'].split(',')[0], rotation=30, fontsize=8, va='bottom')
    for a in ax:
        a.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(folder, 'range_timeline.png'), dpi=120)
    return True


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    folder = sys.argv[1]
    paths = sorted(glob.glob(os.path.join(folder, 'range_*.log')))
    if not paths:
        sys.exit(f'no range_*.log in {folder}')
    logs = [parse_log(p) for p in paths]
    rows = [summarize(lg, s) for lg in logs for s in lg['stops'] if s['pkts'] or s['stats']]
    write_csvs(folder, logs, rows)
    plotted = write_plot(folder, rows)
    write_timeline(folder, logs)
    write_report(folder, logs, rows)
    print(f"{len(rows)} stops from {len(paths)} log(s) -> {folder}/report.md, stops.csv, packets.csv"
          + (', range_plots.png' if plotted else ' (no plot: needs matplotlib and 2+ numeric distances)'))


if __name__ == '__main__':
    main()
