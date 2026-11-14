#!/usr/bin/env python3
"""Record a range test from the ground board.

Plug the ground board (RANGE=1 ground firmware) into the laptop and run:

    python3 tools/range/range_log.py            # finds the board's port
    python3 tools/range/range_log.py --port /dev/cu.usbmodem1403

At each stop, type the distance (for example "100" or "100 behind trees")
and press Enter; everything logged afterwards belongs to that stop until the
next one. Type "q" to finish. Every line from the board is saved with the
laptop's clock to logs/range/<date>/range_<time>.log; turn the logs into a
report with tools/range/range_report.py.

Standard library only (no pip installs).
"""
import argparse
import datetime
import glob
import os
import select
import sys
import termios
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def open_port(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0
    a[1] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] = 0
    a[4] = a[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


class Live:
    """Running numbers for the current stop, printed every few seconds."""

    def __init__(self):
        self.reset('-')

    def reset(self, mark):
        self.mark = mark
        self.ok = self.bad = self.err = self.bits = 0
        self.seq = []
        self.rssi = []
        self.snr = []
        self.up = []
        self.locked = None

    def line(self, f):
        if f[0] == 'PKT' and len(f) >= 14:
            ok = f[4] == '1'
            self.ok += ok
            self.bad += not ok
            self.err += int(f[7])
            self.bits += int(f[8])
            self.rssi.append(int(f[5]))
            self.snr.append(int(f[6]) / 4)
            if ok:
                self.seq.append(int(f[9]))
                self.up.append((int(f[10]), int(f[13])))
        elif f[0] == 'STAT' and len(f) >= 3:
            self.locked = f[2] == '1'

    def summary(self):
        sent = (self.seq[-1] - self.seq[0]) % 65536 + 1 if len(self.seq) > 1 else len(self.seq)
        per = 1 - self.ok / sent if sent else float('nan')
        ber = self.err / self.bits if self.bits else float('nan')
        up = ''
        if len(self.up) > 1:
            got = self.up[-1][0] - self.up[0][0]
            sent_up = self.up[-1][1] - self.up[0][1]
            if sent_up > 0:
                up = f' | up {got}/{sent_up} cmds'
        rssi = f'{sum(self.rssi) / len(self.rssi):.0f} dBm (min {min(self.rssi)})' if self.rssi else '-'
        snr = f'{sum(self.snr) / len(self.snr):.1f} dB' if self.snr else '-'
        lock = {None: '?', True: 'locked', False: 'SEARCHING'}[self.locked]
        return (f'[{self.mark}] {lock} | down ok {self.ok}, crc-fail {self.bad}, PER {per:.1%} | '
                f'BER {ber:.1e} | RSSI {rssi} | SNR {snr}{up}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', help='serial port of the ground board (default: the only /dev/cu.usbmodem*)')
    ap.add_argument('--out', help='output folder (default logs/range/<date>)')
    ap.add_argument('--notes', help='setup notes saved in the log (antennas, heights, place)')
    args = ap.parse_args()

    port = args.port
    if not port:
        ports = sorted(glob.glob('/dev/cu.usbmodem*'))
        if len(ports) != 1:
            sys.exit(f'found {len(ports)} boards ({", ".join(ports) or "none"}); plug in only the '
                     'ground board or pass --port')
        port = ports[0]
    now = datetime.datetime.now()
    out = args.out or os.path.join(REPO, 'logs', 'range', now.strftime('%Y-%m-%d'))
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, now.strftime('range_%H%M%S.log'))

    notes = args.notes
    if notes is None:
        print('Setup notes (antenna type/orientation, heights, place, TX power) then Enter:')
        notes = sys.stdin.readline().strip()

    fd = open_port(port)
    log = open(path, 'w', buffering=1)
    stamp = lambda: f'{time.time():.3f}'
    log.write(f'{stamp()} NOTE,{notes}\n')
    log.write(f'{stamp()} PORT,{port}\n')
    print(f'Logging {port} to {path}')
    print('Type a distance (e.g. "50" or "50 m, tree line") + Enter at each stop; "q" + Enter to finish.')

    live = Live()
    buf = b''
    next_print = time.time() + 5
    try:
        while True:
            r, _, _ = select.select([fd, sys.stdin], [], [], 0.2)
            if fd in r:
                try:
                    buf += os.read(fd, 4096)
                except BlockingIOError:
                    pass
                while b'\n' in buf:
                    raw, buf = buf.split(b'\n', 1)
                    line = raw.decode(errors='replace').strip()
                    if not line:
                        continue
                    log.write(f'{stamp()} {line}\n')
                    live.line(line.split(','))
                    if line.startswith(('===', 'RCFG', 'status')):
                        print(line)
            if sys.stdin in r:
                cmd = sys.stdin.readline().strip()
                if cmd.lower() in ('q', 'quit', 'exit'):
                    break
                if cmd:
                    log.write(f'{stamp()} MARK,{cmd}\n')
                    live.reset(cmd)
                    print(f'--- stop: {cmd}')
            if time.time() >= next_print:
                next_print = time.time() + 5
                print(live.summary())
    except KeyboardInterrupt:
        pass
    finally:
        log.write(f'{stamp()} END\n')
        log.close()
        os.close(fd)
    print(f'saved {path}\nreport: python3 tools/range/range_report.py {out}')


if __name__ == '__main__':
    main()
