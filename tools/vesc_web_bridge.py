#!/usr/bin/env python3
"""Serve web/index.html and proxy /api/* to a VESC on USB (native CDC or USB-TTL).

  python tools/vesc_web_bridge.py
  python tools/vesc_web_bridge.py --port COM7
  python tools/vesc_web_bridge.py --list

Then open http://127.0.0.1:18765/  — same page as the board, live numbers from USB.
Windows 同一时刻只能有一个程序占用 COM 口。核对 VESC Tool 前，在页面点「释放串口」；关掉 VESC Tool 后再点「重新连接」。Ctrl+C 结束桥。
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

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
    COMM_GET_DECODED_PPM,
    COMM_GET_MCCONF,
    COMM_GET_VALUES,
    COMM_SET_MCCONF,
    PacketParser,
    build_packet,
    parse_decoded_ppm,
    parse_fw_version,
    parse_get_values,
    parse_mcconf_battery_limits,
    patch_mcconf_battery_limits,
)

ROOT = Path(__file__).resolve().parents[1]
HTML_PATH = ROOT / "web" / "index.html"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 18765
HTTP_PORT_FALLBACKS = (18765, 8765, 8088, 9080)


def list_serial_ports() -> list[tuple[str, str, int]]:
    """Return (device, description, score) high score first."""
    rows: list[tuple[str, str, int]] = []
    for p in list_ports.comports():
        desc = f"{p.description or ''} {p.manufacturer or ''} {p.hwid or ''}"
        low = desc.lower()
        score = 0
        if any(x in low for x in ("ch340", "ch343", "cp210", "esp32", "silicon labs")):
            score -= 20
        if "vesc" in low or "chibios" in low:
            score += 20
        if "0483" in low or "stmicro" in low or "stm32" in low:
            score += 12
        if "virtual com" in low or "usb serial device" in low:
            score += 4
        rows.append((p.device, (p.description or "").strip() or p.device, score))
    rows.sort(key=lambda r: r[2], reverse=True)
    return rows


class VescLink:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.ser: serial.Serial | None = None
        self.parser = PacketParser()
        self.lock = threading.Lock()
        self.fw = ""
        self.live: dict = {"vesc": False, "fw": "", "fault": 0}
        self.ppm_ok = True
        self.last_err = "未连接"
        self._stop = False
        self.held = True

    def open(self) -> None:
        print(f"打开串口 {self.port} @ {self.baud}", flush=True)
        ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=1.0)
        time.sleep(0.15)
        ser.reset_input_buffer()
        self.ser = ser
        self.parser.buf.clear()
        print("串口已打开，读取固件…", flush=True)
        pl = self._transact(COMM_FW_VERSION, timeout=1.8)
        if pl:
            self.fw = parse_fw_version(pl) or self.fw
        self.last_err = ""

    def close(self) -> None:
        self._stop = True
        self.held = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _mark_free(self, msg: str) -> None:
        self.last_err = msg
        self.live = {
            "type": "live",
            "vesc": False,
            "fw": self.fw or "?",
            "v_in": 0,
            "i_in": 0,
            "i_motor": 0,
            "rpm": 0,
            "duty": 0,
            "t_fet": 0,
            "t_motor": 0,
            "fault": 0,
            "ppm": 1500,
            "pct": 0,
            "mv": 0,
            "dir": "STOP",
            "mode": "USB",
            "link": "usb",
            "usb": "free",
            "port": self.port,
            "error": msg,
        }

    def release(self) -> dict:
        with self.lock:
            self.held = False
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            self._mark_free("USB 已释放，可开 VESC Tool")
        return {"ok": True, "usb": "free", "port": self.port}

    def reconnect(self) -> dict:
        with self.lock:
            self.held = True
            if self.ser is None:
                try:
                    self.open()
                except Exception as exc:
                    self._mark_free(str(exc))
                    return {"ok": False, "error": str(exc), "usb": "free", "port": self.port}
        self.poll_once()
        return {"ok": True, "usb": "held", "port": self.port, "fw": self.fw}

    def _transact(self, cmd: int, payload: bytes = b"", timeout: float = 1.5) -> bytes | None:
        if self.ser is None:
            return None
        pkt = build_packet(cmd, payload)
        self.ser.reset_input_buffer()
        self.parser.buf.clear()
        self.ser.write(pkt)
        self.ser.flush()
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = self.ser.read(4096)
            if not chunk:
                continue
            for pl in self.parser.feed(chunk):
                if pl and pl[0] == cmd:
                    return pl
        return None

    def poll_once(self) -> None:
        with self.lock:
            pl = self._transact(COMM_GET_VALUES, timeout=0.8)
            tel = parse_get_values(pl) if pl else None
            ppm = None
            if tel is not None and self.ppm_ok:
                ppl = self._transact(COMM_GET_DECODED_PPM, timeout=0.25)
                if ppl:
                    ppm = parse_decoded_ppm(ppl)
                else:
                    self.ppm_ok = False
        if tel is None:
            self.live = {
                "type": "live",
                "vesc": False,
                "fw": self.fw or "?",
                "v_in": 0,
                "i_in": 0,
                "i_motor": 0,
                "rpm": 0,
                "duty": 0,
                "t_fet": 0,
                "t_motor": 0,
                "fault": 0,
                "ppm": 1500,
                "pct": 0,
                "mv": 0,
                "dir": "STOP",
                "mode": "USB",
                "link": "usb",
                "usb": "held" if self.held and self.ser is not None else "free",
                "port": self.port,
                "error": self.last_err or "无数据",
            }
            self.last_err = "GET_VALUES 超时"
            return
        duty = tel.duty
        pct = int(round(duty * 100.0))
        if ppm is not None:
            pct = int(round(ppm * 100.0))
            ppm_us = int(round(1500 + ppm * 500))
        else:
            ppm_us = int(round(1500 + duty * 500))
        if pct > 2:
            direction = "FWD"
        elif pct < -2:
            direction = "REV"
        else:
            direction = "STOP"
        if self.fw:
            tel.fw = self.fw
        self.last_err = ""
        self.live = {
            "type": "live",
            "vesc": True,
            "fw": self.fw or tel.fw or "?",
            "v_in": tel.v_in,
            "i_in": tel.i_in,
            "i_motor": tel.i_motor,
            "rpm": tel.rpm,
            "duty": duty,
            "t_fet": tel.t_fet,
            "t_motor": tel.t_motor,
            "fault": tel.fault,
            "ppm": ppm_us,
            "pct": pct,
            "mv": 0,
            "dir": direction,
            "mode": "USB",
            "link": "usb",
            "usb": "held",
            "port": self.port,
        }

    def get_mcconf(self) -> dict:
        if self.ser is None:
            return {"ok": False, "error": "USB 已释放"}
        with self.lock:
            pl = self._transact(COMM_GET_MCCONF, timeout=4.0)
        if not pl or len(pl) < 6:
            return {"ok": False, "error": "GET_MCCONF 超时"}
        try:
            lim = parse_mcconf_battery_limits(pl[1:])
            lim["ok"] = True
            return lim
        except ValueError as exc:
            sig = 0
            if len(pl) >= 5:
                sig = (pl[1] << 24) | (pl[2] << 16) | (pl[3] << 8) | pl[4]
            return {"ok": False, "error": str(exc), "sig": sig}

    def set_mcconf(self, body: dict) -> dict:
        keys = (
            "battery_type",
            "cells",
            "battery_ah",
            "cut_start",
            "cut_end",
            "min_vin",
            "max_vin",
            "i_in_max",
            "watt_max",
            "motor_max",
            "motor_min",
            "invert",
        )
        patch = {k: body[k] for k in keys if k in body and body[k] is not None}
        if not patch:
            return {"ok": False, "error": "no fields"}
        if self.ser is None:
            return {"ok": False, "error": "USB 已释放"}
        with self.lock:
            pl = self._transact(COMM_GET_MCCONF, timeout=4.0)
            if not pl or len(pl) < 6:
                return {"ok": False, "error": "GET_MCCONF 超时"}
            blob = bytearray(pl[1:])
            try:
                patch_mcconf_battery_limits(blob, patch)
            except ValueError as exc:
                return {"ok": False, "error": str(exc)}
            ack = self._transact(COMM_SET_MCCONF, bytes(blob), timeout=4.0)
            verify = self._transact(COMM_GET_MCCONF, timeout=4.0)
        if verify and len(verify) >= 6:
            try:
                lim = parse_mcconf_battery_limits(verify[1:])
                lim["ok"] = True
                if ack is None:
                    lim["note"] = "SET 无 ACK，已用 GET 校验"
                return lim
            except ValueError as exc:
                return {"ok": False, "error": str(exc)}
        return {"ok": False, "error": "写入后读取失败"}

    def loop(self) -> None:
        fails = 0
        while not self._stop:
            if not self.held or self.ser is None:
                time.sleep(0.25)
                continue
            try:
                self.poll_once()
                fails = 0 if self.live.get("vesc") else fails + 1
            except serial.SerialException as exc:
                self.last_err = str(exc)
                fails += 1
                self.live["vesc"] = False
            if self.held and fails >= 8:
                try:
                    with self.lock:
                        if self.ser is not None:
                            self.ser.close()
                            self.ser = None
                    time.sleep(0.4)
                    with self.lock:
                        if self.held and self.ser is None:
                            self.open()
                    fails = 0
                except Exception as exc:
                    self.last_err = str(exc)
                    time.sleep(1.0)
            time.sleep(0.12)


LINK: VescLink | None = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args) -> None:
        path = self.path.split("?", 1)[0]
        if path in ("/api/live", "/api/gps"):
            return
        super().log_message(fmt, *args)

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store")

    def end_headers(self) -> None:
        self._cors()
        super().end_headers()

    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(204)
        self.end_headers()

    def _json(self, code: int, obj: dict) -> None:
        raw = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _bytes(self, code: int, data: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            html = HTML_PATH.read_bytes()
            self._bytes(200, html, "text/html; charset=utf-8")
            return
        if path == "/api/live":
            live = LINK.live if LINK else {"vesc": False, "error": "no link"}
            self._json(200, live)
            return
        if path == "/api/gps":
            self._json(
                200,
                {
                    "fix": False,
                    "sats": 0,
                    "time_valid": False,
                    "time": "",
                    "baud": 0,
                    "note": "usb",
                },
            )
            return
        if path == "/api/mcconf":
            if LINK is None:
                self._json(200, {"ok": False, "error": "no link"})
                return
            self._json(200, LINK.get_mcconf())
            return
        if path == "/api/usb":
            if LINK is None:
                self._json(200, {"ok": False, "usb": "free", "error": "no link"})
                return
            held = bool(LINK.held and LINK.ser is not None)
            self._json(
                200,
                {
                    "ok": True,
                    "usb": "held" if held else "free",
                    "port": LINK.port,
                    "fw": LINK.fw,
                },
            )
            return
        if path == "/api/ports":
            self._json(
                200,
                {
                    "ok": True,
                    "port": LINK.port if LINK else "",
                    "ports": [{"device": d, "desc": n, "score": s} for d, n, s in list_serial_ports()],
                },
            )
            return
        self._bytes(404, b"not found", "text/plain")

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length > 0 else b"{}"
        if path == "/api/usb":
            if LINK is None:
                self._json(200, {"ok": False, "error": "no link"})
                return
            try:
                body = json.loads(raw.decode("utf-8") or "{}")
            except json.JSONDecodeError:
                body = {}
            act = str(body.get("action") or "")
            if act == "release":
                self._json(200, LINK.release())
                return
            if act == "open":
                self._json(200, LINK.reconnect())
                return
            self._json(200, {"ok": False, "error": "action 用 release 或 open"})
            return
        if path != "/api/mcconf":
            self._bytes(404, b"not found", "text/plain")
            return
        if LINK is None:
            self._json(200, {"ok": False, "error": "no link"})
            return
        try:
            body = json.loads(raw.decode("utf-8") or "{}")
        except json.JSONDecodeError:
            self._json(200, {"ok": False, "error": "bad json"})
            return
        self._json(200, LINK.set_mcconf(body))


def probe_port(port: str, baud: int) -> str:
    link = VescLink(port, baud)
    try:
        link.open()
        fw = link.fw
        link.close()
        return fw
    except Exception:
        try:
            link.close()
        except Exception:
            pass
        return ""


def pick_port(requested: str, baud: int) -> str:
    if requested:
        return requested
    ports = list_serial_ports()
    if not ports:
        raise SystemExit("没有串口。用 USB 线把电调接到电脑，并关掉 VESC Tool。")
    print("可用串口：")
    for dev, desc, score in ports:
        mark = "  << 优先" if score >= 8 else ("  << 可能是 ESP，跳过探测" if score < 0 else "")
        print(f"  {dev:8}  {desc}{mark}")
    candidates = [p for p in ports if p[2] >= 0]
    if not candidates:
        candidates = ports
    for dev, desc, score in candidates:
        print(f"探测 {dev} ({desc}) …")
        fw = probe_port(dev, baud)
        if fw:
            print(f"  找到 VESC FW {fw}")
            return dev
        print("  无回复")
    raise SystemExit(
        "没有电调应答。关掉 VESC Tool 后重试，或指定口：python tools/vesc_web_bridge.py --port COM7"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="USB VESC → 静态网页实时数据")
    ap.add_argument("--port", "-p", default="", help="COM 口，省略则自动探测")
    ap.add_argument("--baud", "-b", type=int, default=115200)
    ap.add_argument("--listen", default=DEFAULT_HOST)
    ap.add_argument("--http-port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    if not HTML_PATH.exists():
        raise SystemExit(f"找不到页面 {HTML_PATH}")

    if args.list:
        rows = list_serial_ports()
        if not rows:
            print("无串口")
            return 1
        for dev, desc, score in rows:
            print(f"{dev:8}  {desc}  score={score}")
        return 0

    global LINK
    com = pick_port(args.port.strip(), args.baud)
    LINK = VescLink(com, args.baud)
    ports = [args.http_port] if args.http_port != DEFAULT_PORT else list(HTTP_PORT_FALLBACKS)
    httpd = None
    bound = 0
    last_err: Exception | None = None
    for p in ports:
        try:
            httpd = ThreadingHTTPServer((args.listen, p), Handler)
            bound = p
            break
        except OSError as exc:
            last_err = exc
            print(f"端口 {p} 不可用：{exc}", flush=True)
    if httpd is None:
        raise SystemExit(f"无法监听 HTTP：{last_err}")
    url = f"http://{args.listen}:{bound}/"
    print(f"HTTP {url}", flush=True)

    LINK.open()
    LINK.poll_once()
    print(f"固件 {LINK.fw or '未知'}  — 打开 {url}", flush=True)
    threading.Thread(target=LINK.loop, daemon=True).start()
    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    print("Ctrl+C 结束。核对 VESC Tool 前先在页面点「释放串口」。", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n停止")
    finally:
        LINK.close()
        httpd.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
