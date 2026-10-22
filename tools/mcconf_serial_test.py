#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import time

import serial


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
    print("opening", port, flush=True)
    ser = serial.Serial(port, 115200, timeout=0.5)
    print("opened", flush=True)
    time.sleep(4.0)
    pending = ser.read(ser.in_waiting or 1)
    # keep reading a few seconds
    t0 = time.time()
    while time.time() - t0 < 3.0:
        chunk = ser.read(1024)
        if chunk:
            pending += chunk
    text = pending.decode("utf-8", "replace")
    print("--- boot/telemetry ---", flush=True)
    print(text[-2500:], flush=True)

    cmd = '{"cmd":"mcconf_get"}\n'
    print("sending", cmd.strip(), flush=True)
    ser.write(cmd.encode("utf-8"))
    ser.flush()

    got = None
    t0 = time.time()
    while time.time() - t0 < 6.0:
        line = ser.readline()
        if not line:
            continue
        s = line.decode("utf-8", "replace").rstrip()
        print("RX:", s, flush=True)
        if '"type":"mcconf"' in s:
            try:
                got = json.loads(s)
            except json.JSONDecodeError:
                got = {"raw": s}
            break

    print("--- result ---", flush=True)
    print(json.dumps(got, indent=2) if got else "NO RESPONSE", flush=True)
    ser.close()
    ok = bool(got and got.get("ok"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
