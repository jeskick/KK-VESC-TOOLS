"""VESC UART packet helpers (USB-TTL / ESP wired link)."""

from __future__ import annotations

import struct
from dataclasses import dataclass

COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_MCCONF = 13
COMM_GET_MCCONF = 14
COMM_GET_DECODED_PPM = 55

MCCONF_SIG_602 = 776184161
MCCONF_SIG_605 = 1065524471
MCCONF_SIG_701 = 2466726157
OFF602_CURRENT_MAX = 8
OFF602_CURRENT_MIN = 12
OFF602_IN_CURRENT_MAX = 16
OFF602_MIN_VIN = 46
OFF602_MAX_VIN = 50
OFF602_CUT_START = 54
OFF602_CUT_END = 58
OFF602_WATT_MAX = 77
OFF602_INVERT = 420
OFF602_TYPE = 460
OFF602_CELLS = 461
OFF602_AH = 462
MAX_PACKET = 4096
CHEM_LIMITS = (
    {"full": 4.20, "cut_s": 3.50, "cut_e": 3.30, "min": 3.00, "max": 4.30},
    {"full": 3.65, "cut_s": 3.00, "cut_e": 2.80, "min": 2.60, "max": 3.70},
    {"full": 2.40, "cut_s": 2.15, "cut_e": 2.10, "min": 2.00, "max": 2.45},
)


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_packet(cmd: int, payload: bytes = b"") -> bytes:
    pl = bytes([cmd]) + payload
    c = crc16(pl)
    if len(pl) < 256:
        return bytes([0x02, len(pl)]) + pl + bytes([c >> 8, c & 0xFF, 0x03])
    return (
        bytes([0x03, (len(pl) >> 8) & 0xFF, len(pl) & 0xFF])
        + pl
        + bytes([c >> 8, c & 0xFF, 0x03])
    )


def _get_u32(buf: bytes, i: int) -> tuple[int, int]:
    return ((buf[i] << 24) | (buf[i + 1] << 16) | (buf[i + 2] << 8) | buf[i + 3], i + 4)


def _put_u32(buf: bytearray, i: int, v: int) -> int:
    buf[i] = (v >> 24) & 0xFF
    buf[i + 1] = (v >> 16) & 0xFF
    buf[i + 2] = (v >> 8) & 0xFF
    buf[i + 3] = v & 0xFF
    return i + 4


def get_float32_auto(buf: bytes, i: int) -> tuple[float, int]:
    import math

    res, i = _get_u32(buf, i)
    e = (res >> 23) & 0xFF
    sig_i = res & 0x7FFFFF
    neg = (res & (1 << 31)) != 0
    sig = 0.0
    if e != 0 or sig_i != 0:
        sig = sig_i / (8388608.0 * 2.0) + 0.5
        e -= 126
    if neg:
        sig = -sig
    return math.ldexp(sig, e), i


def put_float32_auto(buf: bytearray, i: int, number: float) -> int:
    import math

    if abs(number) < 1.5e-38:
        number = 0.0
    sig, e = math.frexp(number)
    sig_abs = abs(sig)
    sig_i = 0
    if sig_abs >= 0.5:
        sig_i = int((sig_abs - 0.5) * 2.0 * 8388608.0)
        e += 126
    res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF)
    if sig < 0:
        res |= 1 << 31
    return _put_u32(buf, i, res)


def get_float16(buf: bytes, i: int, scale: float) -> tuple[float, int]:
    v = struct.unpack(">h", buf[i : i + 2])[0]
    return v / scale, i + 2


def put_float16(buf: bytearray, i: int, number: float, scale: float) -> int:
    v = int(number * scale)
    v = max(-32768, min(32767, v))
    buf[i] = (v >> 8) & 0xFF
    buf[i + 1] = v & 0xFF
    return i + 2


def _limits_sane(lim: dict) -> bool:
    return 5.0 < lim["min_vin"] < 130.0 and 5.0 < lim["max_vin"] < 140.0 and lim["min_vin"] < lim["max_vin"]


def _parse_602(conf: bytes) -> dict:
    if len(conf) < 466:
        raise ValueError(f"mcconf too short: {len(conf)}")

    def f32(off: int) -> float:
        v, _ = get_float32_auto(conf, off)
        return v

    sig, _ = _get_u32(conf, 0)
    return {
        "ok": True,
        "sig": sig,
        "battery_type": conf[OFF602_TYPE],
        "cells": conf[OFF602_CELLS],
        "battery_ah": f32(OFF602_AH),
        "cut_start": f32(OFF602_CUT_START),
        "cut_end": f32(OFF602_CUT_END),
        "min_vin": f32(OFF602_MIN_VIN),
        "max_vin": f32(OFF602_MAX_VIN),
        "i_in_max": f32(OFF602_IN_CURRENT_MAX),
        "motor_max": f32(OFF602_CURRENT_MAX),
        "motor_min": f32(OFF602_CURRENT_MIN),
        "watt_max": f32(OFF602_WATT_MAX),
        "invert": 1 if conf[OFF602_INVERT] else 0,
        "layout": "602",
    }


def _walk_605_early(conf: bytes, sig: int) -> tuple[dict, dict]:
    """Same early-field walk as src/vesc_mcconf.cpp parse605Early. Returns (limits, offsets)."""
    if len(conf) < 80:
        raise ValueError(f"mcconf too short: {len(conf)}")
    i = 4
    i += 4
    off_mot = i
    motor_max, i = get_float32_auto(conf, i)
    _, i = get_float32_auto(conf, i)
    off_in = i
    i_in_max, i = get_float32_auto(conf, i)
    _, i = get_float32_auto(conf, i)
    _, i = get_float16(conf, i, 10000)
    _, i = get_float16(conf, i, 10000)
    _, i = get_float32_auto(conf, i)
    _, i = get_float32_auto(conf, i)
    _, i = get_float32_auto(conf, i)
    _, i = get_float16(conf, i, 10000)
    _, i = get_float32_auto(conf, i)
    _, i = get_float32_auto(conf, i)
    off_min = i
    min_vin, i = get_float16(conf, i, 10)
    off_max = i
    max_vin, i = get_float16(conf, i, 10)
    off_cs = i
    cut_start, i = get_float16(conf, i, 10)
    off_ce = i
    cut_end, i = get_float16(conf, i, 10)
    _, i = get_float16(conf, i, 10)
    _, i = get_float16(conf, i, 10)
    i += 1
    i += 4
    _, i = get_float16(conf, i, 10000)
    _, i = get_float16(conf, i, 10000)
    _, i = get_float16(conf, i, 10000)
    off_w = i
    watt_max, i = get_float32_auto(conf, i)
    lim = {
        "ok": True,
        "sig": sig,
        "battery_type": 0,
        "cells": 0,
        "battery_ah": 0,
        "cut_start": cut_start,
        "cut_end": cut_end,
        "min_vin": min_vin,
        "max_vin": max_vin,
        "i_in_max": i_in_max,
        "motor_max": motor_max,
        "motor_min": 0,
        "watt_max": watt_max,
        "invert": 0,
        "layout": "605",
    }
    offs = {
        "mot": off_mot,
        "in": off_in,
        "min": off_min,
        "max": off_max,
        "cs": off_cs,
        "ce": off_ce,
        "w": off_w,
    }
    return lim, offs


def parse_mcconf_battery_limits(conf: bytes) -> dict:
    if len(conf) < 8:
        raise ValueError(f"mcconf too short: {len(conf)}")
    sig, _ = _get_u32(conf, 0)
    if sig == MCCONF_SIG_602:
        return _parse_602(conf)
    try:
        lim, _ = _walk_605_early(conf, sig)
        if _limits_sane(lim):
            return lim
    except (ValueError, struct.error, IndexError):
        pass
    if len(conf) >= 466:
        lim = _parse_602(conf)
        lim["sig"] = sig
        if _limits_sane(lim):
            return lim
    raise ValueError(f"unsupported signature {sig}")


def sanitize_write_patch(patch: dict) -> dict:
    """Clamp related MCCONF fields the way docs/vesc-safety-params.md describes."""
    out = {k: v for k, v in patch.items() if v is not None}
    try:
        btype = int(float(out["battery_type"])) if "battery_type" in out else 0
    except (TypeError, ValueError):
        btype = 0
    if btype < 0 or btype > 2:
        btype = 0
    if "battery_type" in out:
        out["battery_type"] = btype
    ch = CHEM_LIMITS[btype]
    try:
        cells = int(float(out.get("cells") or 0))
    except (TypeError, ValueError):
        cells = 0
    if cells < 1 or cells > 24:
        cells = 0
    else:
        out["cells"] = cells

    def fget(key: str):
        if key not in out:
            return None
        try:
            return float(out[key])
        except (TypeError, ValueError):
            return None

    if cells:
        max_vin = fget("max_vin")
        if max_vin is None or max_vin < cells * ch["full"]:
            out["max_vin"] = round(cells * ch["max"], 1)
        min_vin = fget("min_vin")
        cut_end = fget("cut_end")
        cut_start = fget("cut_start")
        if cut_start is not None and cut_end is not None and cut_end >= cut_start:
            out["cut_start"] = round(cells * ch["cut_s"], 1)
            out["cut_end"] = round(cells * ch["cut_e"], 1)
            cut_end = out["cut_end"]
        if min_vin is None or (cut_end is not None and min_vin >= cut_end):
            out["min_vin"] = round(cells * ch["min"], 1)

    motor = fget("motor_max")
    if motor is not None:
        motor = max(1.0, min(250.0, motor))
        out["motor_max"] = motor
    brake = fget("motor_min")
    if brake is not None:
        if brake > 0:
            brake = -brake
        cap = motor if motor is not None else 250.0
        out["motor_min"] = max(-min(120.0, cap), min(0.0, brake))
    elif motor is not None:
        out["motor_min"] = -min(40.0, motor * 0.5)
    if "invert" in out:
        try:
            out["invert"] = 1 if int(float(out["invert"])) else 0
        except (TypeError, ValueError):
            out["invert"] = 0

    iin = fget("i_in_max")
    if iin is not None:
        out["i_in_max"] = max(1.0, min(250.0, iin))
    watt = fget("watt_max")
    if watt is not None:
        out["watt_max"] = max(50.0, min(20000.0, watt))
    ah = fget("battery_ah")
    if ah is not None:
        out["battery_ah"] = max(0.1, min(200.0, ah))
    return out


def patch_mcconf_battery_limits(conf: bytearray, patch: dict) -> dict:
    patch = sanitize_write_patch(patch)
    sig, _ = _get_u32(bytes(conf), 0)
    if sig == MCCONF_SIG_602:
        if "motor_max" in patch:
            put_float32_auto(conf, OFF602_CURRENT_MAX, float(patch["motor_max"]))
        if "motor_min" in patch:
            put_float32_auto(conf, OFF602_CURRENT_MIN, float(patch["motor_min"]))
        if "i_in_max" in patch:
            put_float32_auto(conf, OFF602_IN_CURRENT_MAX, float(patch["i_in_max"]))
        if "min_vin" in patch:
            put_float32_auto(conf, OFF602_MIN_VIN, float(patch["min_vin"]))
        if "max_vin" in patch:
            put_float32_auto(conf, OFF602_MAX_VIN, float(patch["max_vin"]))
        if "cut_start" in patch:
            put_float32_auto(conf, OFF602_CUT_START, float(patch["cut_start"]))
        if "cut_end" in patch:
            put_float32_auto(conf, OFF602_CUT_END, float(patch["cut_end"]))
        if "watt_max" in patch:
            put_float32_auto(conf, OFF602_WATT_MAX, float(patch["watt_max"]))
        if "battery_type" in patch:
            conf[OFF602_TYPE] = int(patch["battery_type"]) & 0xFF
        if "cells" in patch:
            conf[OFF602_CELLS] = int(patch["cells"]) & 0xFF
        if "battery_ah" in patch:
            put_float32_auto(conf, OFF602_AH, float(patch["battery_ah"]))
        if "invert" in patch:
            conf[OFF602_INVERT] = 1 if int(patch["invert"]) else 0
        return parse_mcconf_battery_limits(bytes(conf))

    lim, offs = _walk_605_early(bytes(conf), sig)
    if not _limits_sane(lim):
        raise ValueError(f"cannot patch signature {sig}")
    if "motor_max" in patch:
        put_float32_auto(conf, offs["mot"], float(patch["motor_max"]))
    if "i_in_max" in patch:
        put_float32_auto(conf, offs["in"], float(patch["i_in_max"]))
    if "min_vin" in patch:
        put_float16(conf, offs["min"], float(patch["min_vin"]), 10.0)
    if "max_vin" in patch:
        put_float16(conf, offs["max"], float(patch["max_vin"]), 10.0)
    if "cut_start" in patch:
        put_float16(conf, offs["cs"], float(patch["cut_start"]), 10.0)
    if "cut_end" in patch:
        put_float16(conf, offs["ce"], float(patch["cut_end"]), 10.0)
    if "watt_max" in patch:
        put_float32_auto(conf, offs["w"], float(patch["watt_max"]))
    return parse_mcconf_battery_limits(bytes(conf))


class PacketParser:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self.buf.extend(data)
        out: list[bytes] = []
        while True:
            if len(self.buf) < 5:
                break
            start = self.buf[0]
            if start == 0x02:
                ln = self.buf[1]
                need = 2 + ln + 3
                hdr = 2
            elif start == 0x03:
                if len(self.buf) < 6:
                    break
                ln = (self.buf[1] << 8) | self.buf[2]
                need = 3 + ln + 3
                hdr = 3
            else:
                self.buf.pop(0)
                continue
            if ln == 0 or ln > MAX_PACKET:
                self.buf.pop(0)
                continue
            if len(self.buf) < need:
                break
            frame = bytes(self.buf[:need])
            del self.buf[:need]
            pl = frame[hdr : hdr + ln]
            rx_crc = (frame[-3] << 8) | frame[-2]
            if frame[-1] != 0x03 or crc16(pl) != rx_crc:
                continue
            out.append(pl)
        return out


@dataclass
class VescTelemetry:
    fw: str = ""
    v_in: float = 0.0
    i_in: float = 0.0
    i_motor: float = 0.0
    rpm: float = 0.0
    duty: float = 0.0
    t_fet: float = 0.0
    t_motor: float = 0.0
    fault: int = 0
    ppm_us: float | None = None


def parse_fw_version(payload: bytes) -> str:
    if len(payload) < 3 or payload[0] != COMM_FW_VERSION:
        return ""
    patch = payload[3] if len(payload) >= 4 else 0
    return f"{payload[1]}.{payload[2]}.{patch}"


def parse_get_values(payload: bytes) -> VescTelemetry | None:
    if len(payload) < 56 or payload[0] != COMM_GET_VALUES:
        return None
    tel = VescTelemetry()
    tel.t_fet = struct.unpack(">h", payload[1:3])[0] / 10.0
    tel.t_motor = struct.unpack(">h", payload[3:5])[0] / 10.0
    tel.i_motor = struct.unpack(">i", payload[5:9])[0] / 100.0
    tel.i_in = struct.unpack(">i", payload[9:13])[0] / 100.0
    tel.duty = struct.unpack(">h", payload[21:23])[0] / 1000.0
    tel.rpm = float(struct.unpack(">i", payload[23:27])[0])
    tel.v_in = struct.unpack(">h", payload[27:29])[0] / 10.0
    tel.fault = payload[53]
    return tel


def parse_decoded_ppm(payload: bytes) -> float | None:
    if len(payload) < 5 or payload[0] != COMM_GET_DECODED_PPM:
        return None
    return float(struct.unpack(">f", payload[1:5])[0])
