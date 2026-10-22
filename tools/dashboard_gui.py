#!/usr/bin/env python3
"""
2.4\" ST7789 (240×320) dashboard simulator.

Data (wired VESC):
  - ESP COM/CDC -> throttle (type:"tel") + VESC (type:"vesc")

Usage:
  pip install pyserial
  python tools/dashboard_gui.py
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("需要 pyserial：pip install pyserial") from exc

from dashboard_theme import (
    BAR_H,
    COL_ACCENT,
    COL_BG,
    COL_BLE_OFF,
    COL_BLE_ON,
    COL_CYAN,
    COL_DIM,
    COL_LINE,
    COL_ORANGE,
    COL_PANEL,
    COL_RED,
    COL_REV,
    COL_TEXT,
    LCD_DIAG_IN,
    LCD_H,
    LCD_W,
    MARGIN,
    ZONE_FOOTER,
    ZONE_HEADER,
    ZONE_METRICS,
    ZONE_RPM,
    ZONE_TEMP,
    ZONE_THROTTLE,
)

DEFAULT_PORT = "COM6"
BAUD = 115200
LIVE_URL = "http://192.168.4.1/api/live"


def list_com_ports() -> list[str]:
    ports = [p.device for p in list_ports.comports()]
    if DEFAULT_PORT not in ports:
        ports.insert(0, DEFAULT_PORT)
    return ports or [DEFAULT_PORT]


class HttpLiveReader(threading.Thread):
    def __init__(self, url: str, out: queue.Queue):
        super().__init__(daemon=True)
        self.url = url
        self.out = out
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        try:
            import urllib.request
        except ImportError:
            self.out.put({"_event": "error", "msg": "urllib missing"})
            return
        self.out.put({"_event": "connected", "port": self.url})
        while not self._stop.is_set():
            try:
                with urllib.request.urlopen(self.url, timeout=1.5) as resp:
                    text = resp.read().decode("utf-8", errors="ignore")
                    payload = json.loads(text)
                    if payload.get("type") == "live":
                        # Fan out as tel + vesc for existing renderer
                        self.out.put(
                            {
                                "type": "tel",
                                "ppm": payload.get("ppm", 1500),
                                "pct": payload.get("pct", 0),
                                "dir": payload.get("dir", "STOP"),
                                "mode": payload.get("mode", "PWM"),
                                "mv": payload.get("mv", 0),
                            }
                        )
                        if payload.get("vesc"):
                            self.out.put(
                                {
                                    "type": "vesc",
                                    "connected": True,
                                    "fw": payload.get("fw", ""),
                                    "v_in": payload.get("v_in", 0),
                                    "i_in": payload.get("i_in", 0),
                                    "i_motor": payload.get("i_motor", 0),
                                    "rpm": payload.get("rpm", 0),
                                    "duty": payload.get("duty", 0),
                                    "t_fet": payload.get("t_fet", 0),
                                    "t_motor": payload.get("t_motor", 0),
                                    "fault": payload.get("fault", 0),
                                }
                            )
            except Exception as err:
                self.out.put({"_event": "error", "msg": f"WiFi live: {err}"})
                time.sleep(1.0)
                continue
            time.sleep(0.2)


class SerialReader(threading.Thread):
    def __init__(self, port: str, out: queue.Queue):
        super().__init__(daemon=True)
        self.port_name = port
        self.out = out
        self._stop = threading.Event()
        self._ser: serial.Serial | None = None
        self._write_lock = threading.Lock()

    def stop(self) -> None:
        self._stop.set()

    def write_line(self, text: str) -> bool:
        with self._write_lock:
            if self._ser is None or not self._ser.is_open:
                return False
            try:
                data = text.strip() + "\n"
                self._ser.write(data.encode("utf-8"))
                self._ser.flush()
                return True
            except serial.SerialException:
                return False

    def run(self) -> None:
        while not self._stop.is_set():
            try:
                ser = serial.Serial(self.port_name, BAUD, timeout=0.2)
                ser.dtr = False
                ser.rts = False
                self._ser = ser
                self.out.put({"_src": "com", "_event": "connected", "port": self.port_name})
                buf = b""
                while not self._stop.is_set():
                    chunk = ser.read(256)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        text = line.decode("utf-8", errors="ignore").strip()
                        if not text:
                            continue
                        if text.startswith("#") or "vesc" in text.lower():
                            self.out.put({"_event": "log", "msg": text[:300]})
                        if not text.startswith("{"):
                            continue
                        try:
                            payload = json.loads(text)
                        except json.JSONDecodeError:
                            continue
                        if payload.get("type") in ("tel", "vesc"):
                            payload["_src"] = payload.get("type")
                            self.out.put(payload)
                self._ser = None
                ser.close()
            except serial.SerialException as err:
                self._ser = None
                self.out.put({"_src": "com", "_event": "error", "msg": str(err)})
                time.sleep(1.2)


class DashboardState:
    def __init__(self) -> None:
        self.ble_ok = False  # UART link from ESP type:vesc
        self.com_ok = False
        self.mode = "PWM"
        self.dir = "STOP"
        self.ppm = 1500
        self.pct = 0
        self.mv = 0.0
        self.v_in = 0.0
        self.i_in = 0.0
        self.i_motor = 0.0
        self.rpm = 0.0
        self.duty = 0.0
        self.t_fet = 0.0
        self.t_motor = 0.0
        self.fault = 0
        self.fw = ""
        self.ppm_vesc: float | None = None
        self.last_tel = 0.0
        self.last_vesc = 0.0


class DashboardRenderer:
    """Draw dashboard in logical 240×320 coordinates."""

    def __init__(self, canvas: tk.Canvas, scale: float) -> None:
        self.c = canvas
        self.scale = scale

    def _xy(self, x: float, y: float) -> tuple[int, int]:
        s = self.scale
        return int(x * s), int(y * s)

    def _rect(self, x: float, y: float, w: float, h: float, **kw) -> None:
        s = self.scale
        self.c.create_rectangle(x * s, y * s, (x + w) * s, (y + h) * s, **kw)

    def _line(self, x1: float, y1: float, x2: float, y2: float, **kw) -> None:
        s = self.scale
        w = kw.pop("width", 1)
        if isinstance(w, int) and w > 1:
            kw["width"] = max(1, int(w * s / 2))
        self.c.create_line(x1 * s, y1 * s, x2 * s, y2 * s, width=w, **kw)

    def _text(self, x: float, y: float, text: str, *, fill=COL_TEXT, size=12, anchor="nw",
              bold=False) -> None:
        font = ("Consolas", max(8, int(size * self.scale * 0.42)))
        if bold:
            font = ("Consolas", max(8, int(size * self.scale * 0.42)), "bold")
        self.c.create_text(*self._xy(x, y), text=text, fill=fill, font=font, anchor=anchor)

    def _hbar(self, x: float, y: float, w: float, h: float, ratio: float, color: str) -> None:
        self._rect(x, y, w, h, fill=COL_PANEL, outline=COL_LINE)
        fill_w = max(0.0, min(1.0, ratio)) * (w - 2)
        if fill_w > 0:
            self._rect(x + 1, y + 1, fill_w, h - 2, fill=color, outline="")

    def _bipolar_bar(self, x: float, y: float, w: float, h: float, ppm: int) -> None:
        self._rect(x, y, w, h, fill=COL_PANEL, outline=COL_LINE)
        mid = x + w / 2
        self._line(mid, y, mid, y + h, fill=COL_LINE, width=max(1, self.scale // 2))
        pct = max(-100, min(100, (ppm - 1500) // 5))
        half = (w - 4) / 2
        if pct > 0:
            self._rect(mid, y + 1, half * (pct / 100.0), h - 2, fill=COL_ACCENT, outline="")
        elif pct < 0:
            wneg = half * (-pct / 100.0)
            self._rect(mid - wneg, y + 1, wneg, h - 2, fill=COL_REV, outline="")

    def draw(self, st: DashboardState) -> None:
        self.c.delete("all")
        s = self.scale
        self.c.configure(width=LCD_W * s, height=LCD_H * s)
        self._rect(0, 0, LCD_W, LCD_H, fill=COL_BG, outline=COL_LINE)

        # --- Header: battery voltage ---
        vx, vy, vw, vh = ZONE_HEADER
        vin = st.v_in if st.last_vesc > 0 else 0.0
        vcolor = COL_ACCENT if vin > 20 else COL_ORANGE if vin > 18 else COL_DIM
        if vin <= 0:
            vcolor = COL_DIM
        self._text(vx + MARGIN, vy + 6, "BAT", fill=COL_DIM, size=11)
        if vin > 0:
            self._text(vx + MARGIN, vy + 20, f"{vin:.1f}", fill=vcolor, size=34, bold=True)
            self._text(vx + MARGIN + 88, vy + 38, "V", fill=COL_DIM, size=14)
        else:
            self._text(vx + MARGIN, vy + 24, "--", fill=COL_DIM, size=28, bold=True)

        ble_c = COL_BLE_ON if st.ble_ok else COL_BLE_OFF
        self._rect(LCD_W - MARGIN - 44, vy + 10, 44, 20, fill=COL_PANEL, outline=COL_LINE)
        self.c.create_oval(
            (LCD_W - MARGIN - 38) * s, (vy + 16) * s,
            (LCD_W - MARGIN - 28) * s, (vy + 26) * s,
            fill=ble_c, outline="",
        )
        self._text(LCD_W - MARGIN - 24, vy + 14, "UART", fill=COL_TEXT, size=10)

        com_c = COL_BLE_ON if st.com_ok else COL_BLE_OFF
        self.c.create_oval(
            (LCD_W - MARGIN - 38) * s, (vy + 30) * s,
            (LCD_W - MARGIN - 28) * s, (vy + 40) * s,
            fill=com_c, outline="",
        )
        self._text(LCD_W - MARGIN - 24, vy + 28, "COM", fill=COL_TEXT, size=10)

        self._line(MARGIN, vy + vh - 1, LCD_W - MARGIN, vy + vh - 1, fill=COL_LINE)

        # --- RPM hero ---
        rx, ry, rw, rh = ZONE_RPM
        rpm_abs = abs(st.rpm)
        self._text(rx + MARGIN, ry + 4, "ERPM", fill=COL_DIM, size=11)
        rpm_color = COL_CYAN if rpm_abs < 8000 else COL_ORANGE
        if st.last_vesc <= 0:
            rpm_txt, rpm_color = "-----", COL_DIM
        else:
            rpm_txt = f"{st.rpm:.0f}"
        self._text(rx + MARGIN, ry + 22, rpm_txt, fill=rpm_color, size=30, bold=True)

        # RPM arc-style bar (0..12000)
        self._hbar(MARGIN, ry + 58, LCD_W - 2 * MARGIN, BAR_H, min(1.0, rpm_abs / 12000.0), COL_CYAN)

        # --- Metrics: current / duty ---
        mx, my, mw, mh = ZONE_METRICS
        col_w = (mw - 3 * MARGIN) / 2
        self._rect(mx + MARGIN, my + 4, col_w, 52, fill=COL_PANEL, outline=COL_LINE)
        self._rect(mx + MARGIN + col_w + MARGIN, my + 4, col_w, 52, fill=COL_PANEL, outline=COL_LINE)
        self._text(mx + MARGIN + 6, my + 8, "I in", fill=COL_DIM, size=10)
        self._text(mx + MARGIN + col_w + MARGIN + 6, my + 8, "I motor", fill=COL_DIM, size=10)
        iin = f"{st.i_in:.1f}A" if st.last_vesc > 0 else "--"
        imot = f"{st.i_motor:.1f}A" if st.last_vesc > 0 else "--"
        self._text(mx + MARGIN + 6, my + 24, iin, fill=COL_ORANGE, size=18, bold=True)
        self._text(mx + MARGIN + col_w + MARGIN + 6, my + 24, imot, fill=COL_ORANGE, size=18, bold=True)

        self._text(mx + MARGIN, my + 58, "Duty", fill=COL_DIM, size=10)
        duty = abs(st.duty) if st.last_vesc > 0 else 0.0
        self._text(mx + MARGIN + 36, my + 56, f"{duty:.2f}" if st.last_vesc > 0 else "--",
                   fill=COL_TEXT, size=12)
        self._hbar(MARGIN, my + 72, LCD_W - 2 * MARGIN, BAR_H - 2, min(1.0, duty), COL_ACCENT)

        # --- Throttle ---
        tx, ty, tw, th = ZONE_THROTTLE
        self._line(MARGIN, ty, LCD_W - MARGIN, ty, fill=COL_LINE)
        ppm_show = int(st.ppm_vesc) if st.ppm_vesc is not None else st.ppm
        dir_map = {"FWD": ("前进", COL_ACCENT), "REV": ("后退", COL_REV), "STOP": ("停止", COL_DIM)}
        dir_lbl, dir_c = dir_map.get(st.dir, (st.dir, COL_TEXT))
        self._text(tx + MARGIN, ty + 8, "THR", fill=COL_DIM, size=11)
        self._text(tx + MARGIN + 34, ty + 4, f"{ppm_show}", fill=COL_TEXT, size=22, bold=True)
        self._text(tx + MARGIN + 100, ty + 10, "us", fill=COL_DIM, size=11)
        self._text(tx + MARGIN + 130, ty + 6, dir_lbl, fill=dir_c, size=14, bold=True)

        src = "VESC" if st.ppm_vesc is not None else "ESP"
        self._text(tx + MARGIN + 190, ty + 10, src, fill=COL_DIM, size=9)

        self._bipolar_bar(MARGIN, ty + 36, LCD_W - 2 * MARGIN, 22, ppm_show)
        pct = max(-100, min(100, (ppm_show - 1500) // 5))
        pct_s = f"+{pct}%" if pct > 0 else f"{pct}%"
        self._text(tx + MARGIN, ty + 62, f"{pct_s}   {st.mv / 1000:.2f}V ADC", fill=COL_DIM, size=10)

        # --- Temperature ---
        zx, zy, zw, zh = ZONE_TEMP
        self._line(MARGIN, zy, LCD_W - MARGIN, zy, fill=COL_LINE)
        tfet = st.t_fet if st.last_vesc > 0 else 0.0
        tmot = st.t_motor if st.last_vesc > 0 else 0.0
        fc = COL_RED if tfet > 85 else COL_ORANGE if tfet > 70 else COL_TEXT
        mc = COL_RED if tmot > 100 else COL_ORANGE if tmot > 80 else COL_TEXT
        self._text(zx + MARGIN, zy + 6, "FET", fill=COL_DIM, size=10)
        self._text(zx + MARGIN + 36, zy + 4,
                   f"{tfet:.0f}C" if st.last_vesc > 0 else "--", fill=fc, size=14, bold=True)
        self._text(zx + MARGIN + 110, zy + 6, "MOT", fill=COL_DIM, size=10)
        self._text(zx + MARGIN + 146, zy + 4,
                   f"{tmot:.0f}C" if st.last_vesc > 0 else "--", fill=mc, size=14, bold=True)

        # --- Footer ---
        fx, fy, fw, fh = ZONE_FOOTER
        self._line(MARGIN, fy, LCD_W - MARGIN, fy, fill=COL_LINE)
        mode_txt = st.mode if st.last_tel > 0 else "--"
        self._text(fx + MARGIN, fy + 8, mode_txt, fill=COL_CYAN, size=13, bold=True)
        self._text(fx + MARGIN + 56, fy + 10, "GPIO2", fill=COL_DIM, size=10)
        fault_c = COL_RED if st.fault else COL_DIM
        self._text(fx + MARGIN + 110, fy + 8, f"F{st.fault}", fill=fault_c, size=12, bold=True)
        fw = st.fw or "--"
        self._text(fx + MARGIN, fy + 26, f"VESC {fw}", fill=COL_DIM, size=9)
        age_v = time.time() - st.last_vesc if st.last_vesc else 999
        age_t = time.time() - st.last_tel if st.last_tel else 999
        self._text(fx + MARGIN + 110, fy + 26,
                   f"V{age_v:.0f}s T{age_t:.0f}s", fill=COL_DIM, size=9)


class DashboardApp:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("2.4\" ST7789 仪表盘  240×320  (UART VESC)")
        self.root.configure(bg="#1a1a1a")
        self.q: queue.Queue = queue.Queue()
        self.com_reader: SerialReader | None = None
        self.http_reader: HttpLiveReader | None = None
        self.state = DashboardState()
        self.scale_var = tk.StringVar(value="2× 预览")
        self._build()
        self._auto_connect()
        self.root.after(50, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="2.4\" ST7789  240×320  竖屏仪表盘", font=("Segoe UI", 12, "bold")).pack(
            side="left"
        )
        ttk.Label(top, textvariable=self.scale_var).pack(side="right", padx=8)
        self.scale_box = ttk.Combobox(
            top, values=["物理 2.4\"", "1× 像素", "2× 预览", "3× 预览"],
            width=10, state="readonly",
        )
        self.scale_box.set("2× 预览")
        self.scale_box.pack(side="right")
        self.scale_box.bind("<<ComboboxSelected>>", self._on_scale)

        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        ttk.Combobox(top, textvariable=self.port_var, values=list_com_ports(), width=8,
                     state="readonly").pack(side="right", padx=4)
        ttk.Button(top, text="COM", command=self._connect_com).pack(side="right", padx=2)
        ttk.Button(top, text="WiFi Live", command=self._connect_wifi).pack(side="right", padx=2)

        frame = ttk.Frame(self.root, padding=8)
        frame.pack()
        self.scale = 2
        self.canvas = tk.Canvas(frame, bg=COL_BG, highlightthickness=2, highlightbackground="#333")
        self.canvas.pack()
        self.renderer = DashboardRenderer(self.canvas, self.scale)

        cmd_row = ttk.Frame(self.root, padding=(8, 0, 8, 4))
        cmd_row.pack(fill="x")
        ttk.Label(cmd_row, text="串口命令").pack(side="left")
        self.cmd_var = tk.StringVar(value='{"cmd":"uart_reconnect"}')
        self.cmd_entry = ttk.Entry(cmd_row, textvariable=self.cmd_var)
        self.cmd_entry.pack(side="left", fill="x", expand=True, padx=6)
        self.cmd_entry.bind("<Return>", lambda _e: self._send_cmd())
        ttk.Button(cmd_row, text="发送", command=self._send_cmd).pack(side="left")
        ttk.Button(
            cmd_row, text="重连UART", command=lambda: self._send_preset("uart_reconnect")
        ).pack(side="left", padx=2)

        self.log = tk.Text(self.root, height=5, wrap="word", bg="#111", fg="#c8c8c8",
                           insertbackground="#fff", font=("Consolas", 9))
        self.log.pack(fill="x", padx=8, pady=(0, 4))
        self.log.insert("end", "COM 连接后可看 VESC 遥测；需要时可点「重连UART」。\n")
        self.log.configure(state="disabled")

        hint = ttk.Label(
            self.root,
            text="推荐：电脑连热点 ESP32S3-OTA / 12345678，再点 WiFi Live → http://192.168.4.1/",
            padding=(8, 0, 8, 8),
        )
        hint.pack()

        self.status = ttk.Label(self.root, text="初始化…", padding=8)
        self.status.pack(fill="x")

    def _scale_factor(self, name: str) -> int | float:
        if name == "1× 像素":
            return 1
        if name == "3× 预览":
            return 3
        if name == "物理 2.4\"":
            dpi = self.root.winfo_fpixels("1i")
            w_in = LCD_DIAG_IN * LCD_W / math.hypot(LCD_W, LCD_H)
            return max(1.0, (w_in * dpi) / LCD_W)
        return 2

    def _on_scale(self, event: tk.Event | None = None) -> None:
        name = self.scale_box.get() if event else "2× 预览"
        self.scale_var.set(name)
        self.scale = self._scale_factor(name)
        self.renderer = DashboardRenderer(self.canvas, self.scale)
        self.renderer.draw(self.state)

    def _stop_readers(self) -> None:
        if self.com_reader is not None:
            self.com_reader.stop()
            self.com_reader = None
        if self.http_reader is not None:
            self.http_reader.stop()
            self.http_reader = None

    def _connect_com(self) -> None:
        self._stop_readers()
        self.com_reader = SerialReader(self.port_var.get(), self.q)
        self.com_reader.start()

    def _connect_wifi(self) -> None:
        self._stop_readers()
        self.http_reader = HttpLiveReader(LIVE_URL, self.q)
        self.http_reader.start()
        self.status.configure(text="WiFi Live: 请先连接 ESP32S3-OTA …")

    def _append_log(self, msg: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _send_preset(self, cmd: str) -> None:
        self.cmd_var.set(json.dumps({"cmd": cmd}, separators=(",", ":")))
        self._send_cmd()

    def _send_cmd(self) -> None:
        text = self.cmd_var.get().strip()
        if not text:
            return
        if self.com_reader is None:
            self.status.configure(text="请先点 COM 连接")
            return
        ok = self.com_reader.write_line(text)
        self._append_log((">> " if ok else "!! 发送失败 ") + text)
        if not ok:
            self.status.configure(text="发送失败：COM 未连接")

    def _auto_connect(self) -> None:
        self._connect_com()
        self._on_scale()

    def _poll(self) -> None:
        dirty = False
        while True:
            try:
                item = self.q.get_nowait()
            except queue.Empty:
                break
            if item.get("_event") == "connected" and item.get("port"):
                self.state.com_ok = True
                self.status.configure(text=f"已连接 {item['port']}")
                dirty = True
            elif item.get("_event") == "error":
                self.status.configure(text=str(item.get("msg", ""))[:100])
            elif item.get("_event") == "log":
                self._append_log(str(item.get("msg", "")))
            elif item.get("type") == "vesc":
                self.state.ble_ok = bool(item.get("connected", True))
                self.state.v_in = float(item.get("v_in", 0))
                self.state.i_in = float(item.get("i_in", 0))
                self.state.i_motor = float(item.get("i_motor", 0))
                self.state.rpm = float(item.get("rpm", 0))
                self.state.duty = float(item.get("duty", 0))
                self.state.t_fet = float(item.get("t_fet", 0))
                self.state.t_motor = float(item.get("t_motor", 0))
                self.state.fault = int(item.get("fault", 0))
                if item.get("fw"):
                    self.state.fw = str(item["fw"])
                self.state.last_vesc = time.time()
                self.status.configure(
                    text=f"VESC  Vin={self.state.v_in:.1f}V  I={self.state.i_in:.1f}A"
                    + (f"  {item.get('speed', '')}km/h" if item.get("speed") is not None else "")
                )
                dirty = True
            elif item.get("type") == "tel":
                self.state.com_ok = True
                self.state.ppm = int(item.get("ppm", 1500))
                self.state.pct = int(item.get("pct", 0))
                self.state.dir = str(item.get("dir", "STOP"))
                self.state.mode = str(item.get("mode", "PWM"))
                self.state.mv = float(item.get("mv", 0))
                self.state.last_tel = time.time()
                dirty = True

        if dirty:
            self.renderer.draw(self.state)
        self.root.after(50, self._poll)

    def _on_close(self) -> None:
        self._stop_readers()
        self.root.destroy()

    def run(self) -> None:
        self.renderer.draw(self.state)
        self.root.mainloop()


if __name__ == "__main__":
    DashboardApp().run()
