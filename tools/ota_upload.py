#!/usr/bin/env python3
"""Upload firmware.bin to ESP32-S3 web OTA endpoint."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import requests
except ImportError as exc:
    raise SystemExit("需要 requests：pip install requests") from exc


def main() -> int:
    parser = argparse.ArgumentParser(description="ESP32-S3 HTTP OTA upload")
    parser.add_argument("ip", nargs="?", default="192.168.4.1", help="设备 IP，默认 192.168.4.1")
    parser.add_argument(
        "-f",
        "--firmware",
        default=".pio/build/esp32-s3-n16r8/firmware.bin",
        help="firmware.bin 路径",
    )
    args = parser.parse_args()

    fw = Path(args.firmware)
    if not fw.is_file():
        print(f"找不到固件: {fw}")
        return 1

    url = f"http://{args.ip}/update"
    info_url = f"http://{args.ip}/info"
    try:
        info = requests.get(info_url, timeout=5).json()
        print(f"设备: FW={info.get('fw')} IP={info.get('ip')} heap={info.get('heap')}")
    except Exception as err:
        print(f"警告: 无法读取 /info ({err})")

    print(f"上传 {fw} ({fw.stat().st_size} bytes) -> {url}")
    with fw.open("rb") as f:
        resp = requests.post(url, files={"firmware": f}, timeout=120)
    print(resp.status_code, resp.text.strip())
    return 0 if resp.status_code == 200 else 1


if __name__ == "__main__":
    raise SystemExit(main())
