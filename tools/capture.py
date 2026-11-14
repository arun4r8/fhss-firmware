#!/usr/bin/env python3
"""Capture every /dev/cu.usbmodem* serial port for N seconds.

    python3 tools/capture.py 30 out_dir

Writes out_dir/<port>.txt with the raw console output (115200 8N1) of each
board. Used for bench runs; start it just before flashing to catch the boot
messages.
"""
import glob
import os
import select
import sys
import termios
import time

secs, outdir = float(sys.argv[1]), sys.argv[2]
os.makedirs(outdir, exist_ok=True)
fds = {}
for p in sorted(glob.glob('/dev/cu.usbmodem*')):
    fd = os.open(p, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0
    a[1] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] = 0
    a[4] = a[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIFLUSH)
    fds[fd] = (p, open(os.path.join(outdir, os.path.basename(p) + '.txt'), 'wb'))
end = time.time() + secs
while time.time() < end:
    r, _, _ = select.select(list(fds), [], [], 0.2)
    for fd in r:
        try:
            fds[fd][1].write(os.read(fd, 4096))
            fds[fd][1].flush()
        except (BlockingIOError, OSError):
            pass
for fd, (p, f) in fds.items():
    f.close()
    os.close(fd)
    print(p, os.path.getsize(f.name), 'bytes')
