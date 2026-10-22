#!/usr/bin/env python3
"""
Standalone VESC UART test via USB-TTL (NOT the ESP COM port).

Wire to VESC spare UART pads / COMM (Bluetooth unplugged):
  USB-TTL TX  →  VESC RX
  USB-TTL RX  ←  VESC TX
  USB-TTL GND →  VESC GND
  Do NOT connect USB-TTL 5V/3V3 unless required.

Usage:
  python tools/vesc_usb_ttl_test.py --list
  python tools/vesc_usb_ttl_test.py --port COM46
  python tools/vesc_usb_ttl_test.py --port COM46 --watch
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("需要 pyserial：pip install pyserial") from exc

from vesc_protocol import (
    COMM_FW_VERSION,
    COMM_GET_VALUES,
    PacketParser,
    build_packet,
    parse_fw_version,
    parse_get_values,
)


def list_ports_brief() -> list[str]:
    print("可用串口：")
    ports = list(list_ports.comports())
    if not ports:
        print("  (无) — 请插入 USB-TTL 后再试")
        return []
    names: list[str] = []
    for p in ports:
        hint = ""
        desc = (p.description or "").lower()
        if "ch343" in desc or "ch340" in desc or "esp" in desc:
            hint = "  << 可能是 ESP，不要用这个测 VESC"
        print(f"  {p.device:8}  {p.description}{hint}")
        names.append(p.device)
    return names


def hex_preview(data: bytes, n: int = 48) -> str:
    s = " ".join(f"{b:02X}" for b in data[:n])
    if len(data) > n:
        s += " ..."
    return s


def open_port(port: str, baud: int) -> serial.Serial | None:
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except serial.SerialException as err:
        print(f"打开失败: {err}")
        return None
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass
    return ser


def exchange(ser: serial.Serial, cmd: int, name: str, seconds: float) -> bool:
    parser = PacketParser()
    pkt = build_packet(cmd)
    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()
    print(f"TX {name}: {hex_preview(pkt)}")

    ok = False
    raw_total = 0
    deadline = time.time() + seconds
    while time.time() < deadline:
        chunk = ser.read(256)
        if not chunk:
            time.sleep(0.02)
            continue
        raw_total += len(chunk)
        print(f"  RX raw +{len(chunk)}: {hex_preview(chunk)}")
        for pl in parser.feed(chunk):
            print(f"  OK packet cmd={pl[0]} len={len(pl)}: {hex_preview(pl)}")
            if pl[0] == COMM_FW_VERSION:
                fw = parse_fw_version(pl)
                print(f"  => FW {fw or '?'}")
                ok = True
            elif pl[0] == COMM_GET_VALUES:
                tel = parse_get_values(pl)
                if tel:
                    print(
                        f"  => Vin={tel.v_in:.1f}V  Iin={tel.i_in:.2f}A  "
                        f"Imot={tel.i_motor:.2f}A  rpm={tel.rpm:.0f}  "
                        f"FET={tel.t_fet:.0f}C  fault={tel.fault}"
                    )
                    ok = True
                else:
                    print("  => GET_VALUES 解析失败（长度不够？）")
    if not ok:
        print(f"  (无有效回复，超时 {seconds:.1f}s，原始字节 {raw_total})")
    return ok


def try_once(port: str, baud: int, seconds: float) -> bool:
    print(f"\n打开 {port} @ {baud}")
    ser = open_port(port, baud)
    if ser is None:
        return False
    ok = False
    try:
        for cmd, name in (
            (COMM_FW_VERSION, "FW_VERSION"),
            (COMM_GET_VALUES, "GET_VALUES"),
        ):
            if exchange(ser, cmd, name, seconds):
                ok = True
    finally:
        ser.close()
    return ok


def watch(port: str, baud: int, period: float) -> int:
    print(f"\n打开 {port} @ {baud}  连续读取 Ctrl+C 结束")
    ser = open_port(port, baud)
    if ser is None:
        return 1
    try:
        while True:
            exchange(ser, COMM_GET_VALUES, "GET_VALUES", min(2.0, period))
            time.sleep(max(0.05, period - 0.2))
    except KeyboardInterrupt:
        print("\n停止")
    finally:
        ser.close()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="VESC spare/COMM UART test via USB-TTL")
    ap.add_argument("--port", "-p", default="", help="USB-TTL COM，例如 COM46（不要用 ESP COM6）")
    ap.add_argument("--baud", "-b", type=int, default=115200)
    ap.add_argument("--timeout", "-t", type=float, default=2.5, help="每条命令等待秒数")
    ap.add_argument("--list", action="store_true", help="只列出串口")
    ap.add_argument("--watch", action="store_true", help="连续 GET_VALUES")
    ap.add_argument(
        "--bauds",
        default="",
        help="逗号分隔多波特率尝试，例如 115200,9600,57600",
    )
    args = ap.parse_args()

    print("=== VESC USB-TTL 通信测试（独立脚本，不经过 ESP）===")
    print("接线：TTL TX→VESC RX，TTL RX←VESC TX，GND 共地；拔掉蓝牙。")
    print("备用焊盘同样三根。无回复就对调 TX/RX 再测。\n")
    names = list_ports_brief()

    if args.list:
        return 0

    port = args.port.strip()
    if not port:
        if not names:
            return 1
        print("\n请指定 USB-TTL 口，例如：")
        print(f"  python tools/vesc_usb_ttl_test.py --port {names[0]}")
        return 2

    bauds = [args.baud]
    if args.bauds.strip():
        bauds = [int(x.strip()) for x in args.bauds.split(",") if x.strip()]

    if args.watch:
        return watch(port, bauds[0], args.timeout)

    any_ok = False
    for baud in bauds:
        if try_once(port, baud, args.timeout):
            any_ok = True
            break
        print(f"波特率 {baud} 失败")

    if any_ok:
        print("\n结果：通信成功。备用脚/该 UART 可用，可把 ESP 接到同一组脚。")
        return 0

    print("\n结果：无有效数据。")
    print("排查：1) 对调 TX/RX  2) GND  3) 电调上电且 App 含 UART  4) 确认是 USB-TTL 口")
    print("对调后再跑：")
    print(f"  python tools/vesc_usb_ttl_test.py --port {port}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
