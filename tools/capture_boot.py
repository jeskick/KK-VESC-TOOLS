#!/usr/bin/env python3
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
print("open", port, flush=True)
s = serial.Serial(port, 115200, timeout=0.2)
# hardware reset via DTR
s.dtr = False
time.sleep(0.05)
s.dtr = True
time.sleep(0.05)
s.dtr = False
s.reset_input_buffer()
print("reset pulsed", flush=True)
buf = b""
t0 = time.time()
while time.time() - t0 < 10:
    chunk = s.read(4096)
    if chunk:
        buf += chunk
        sys.stdout.write(chunk.decode("utf-8", "replace"))
        sys.stdout.flush()
print("\n--- sending mcconf_get ---", flush=True)
s.write(b'{"cmd":"mcconf_get"}\n')
s.flush()
t0 = time.time()
while time.time() - t0 < 6:
    chunk = s.read(4096)
    if chunk:
        sys.stdout.write(chunk.decode("utf-8", "replace"))
        sys.stdout.flush()
s.close()
print("\ndone", flush=True)
