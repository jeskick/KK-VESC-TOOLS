"""Render pinmap.png from src/pins.h so the wiring picture cannot drift from the code.

The physical header order is a property of the board, not the firmware, so it lives here;
every pin *assignment* is parsed out of pins.h. Run: python tools/make_pinmap.py
"""

import os
import re
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Silkscreen labels in physical order, top to bottom, as printed on the board.
LEFT_SILK = ["3V3", "EN", "4", "5", "6", "7", "15", "16", "17", "18",
             "8", "3", "46", "9", "10", "11", "12", "13", "14", "5V"]
RIGHT_SILK = ["TX", "RX", "1", "2", "42", "41", "40", "39", "38", "37",
              "36", "35", "0", "45", "48", "47", "21", "20", "19", "GND"]

# Pins that are never available, with the reason shown on the map.
RESERVED = {
    "EN": ("blocked", "板复位"),
    "TX": ("blocked", "CH343 串口"),
    "RX": ("blocked", "CH343 串口"),
    "37": ("blocked", "PSRAM"),
    "36": ("blocked", "PSRAM"),
    "35": ("blocked", "PSRAM"),
    "0": ("blocked", "BOOT 按键"),
    "45": ("blocked", "启动脚 VDD_SPI"),
    "46": ("blocked", "禁用：内部下拉 + 启动脚"),
    "20": ("blocked", "USB D−"),
    "19": ("blocked", "USB D+"),
    "48": ("spare", "板载 RGB 灯"),
}
RAILS = {
    "3V3": ("rail", "→ 屏 VCC"),
    "5V": ("rail", "→ GPS VCC"),
    "GND": ("gnd", "→ 公共地"),
}

# How each pins.h macro is presented on the map.
MACROS = {
    "TFT_PIN_SCK":  ("tft", "屏 SCL / CLK"),
    "TFT_PIN_MOSI": ("tft", "屏 SDA / MOSI"),
    "TFT_PIN_RST":  ("tft", "屏 RST"),
    "TFT_PIN_DC":   ("tft", "屏 DC"),
    "TFT_PIN_CS":   ("tft", "屏 CS"),
    "TFT_PIN_BL":   ("tft", "屏 BL 背光"),
    "KEY_PIN_UP":    ("key", "按键 UP"),
    "KEY_PIN_DOWN":  ("key", "按键 DOWN"),
    "KEY_PIN_LEFT":  ("key", "按键 LEFT"),
    "KEY_PIN_RIGHT": ("key", "按键 RIGHT"),
    "KEY_PIN_OK":    ("key", "按键 OK"),
    "PIN_THROTTLE": ("io", "油门电位器 (ADC1)"),
    "PIN_PPM":      ("io", "PPM / PWM 输出"),
    "VESC_UART_RX": ("vesc", "VESC  RX ← 对方 TX"),
    "VESC_UART_TX": ("vesc", "VESC  TX → 对方 RX"),
    "GPS_UART_TX": ("gps", "GPS  TX → 模块 RX"),
    "GPS_UART_RX": ("gps", "GPS  RX ← 模块 TX"),
    "SD_MMC_CLK": ("sd", "SD CLK  (板载)"),
    "SD_MMC_CMD": ("sd", "SD CMD  (板载)"),
    "SD_MMC_D0":  ("sd", "SD DATA (板载)"),
}

PALETTE = {
    "tft":     ((37, 99, 235), (255, 255, 255)),
    "key":     ((22, 163, 74), (255, 255, 255)),
    "gps":     ((234, 88, 12), (255, 255, 255)),
    "vesc":    ((147, 51, 234), (255, 255, 255)),
    "io":      ((13, 148, 136), (255, 255, 255)),
    "sd":      ((100, 116, 139), (255, 255, 255)),
    "rail":    ((220, 38, 38), (255, 255, 255)),
    "gnd":     ((30, 41, 59), (255, 255, 255)),
    "spare":   ((226, 232, 240), (71, 85, 105)),
    "blocked": ((245, 246, 248), (148, 163, 184)),
}
LEGEND = [("tft", "屏幕"), ("key", "按键"), ("gps", "GPS"), ("vesc", "VESC"),
          ("io", "油门/输出"), ("sd", "SD 卡"), ("rail", "电源"), ("gnd", "地"),
          ("spare", "空闲"), ("blocked", "禁用")]


def parse_pins_h():
    """gpio number (as string) -> (kind, text), from the #define lines only."""
    path = os.path.join(ROOT, "src", "pins.h")
    with open(path, encoding="utf-8") as f:
        src = f.read()
    out = {}
    for name, val in re.findall(r"^#define\s+(\w+)\s+(\d+)\b", src, re.M):
        if name in MACROS:
            out[val] = MACROS[name]
    return out


def font(size, bold=False):
    for name in (("msyhbd.ttc", "msyh.ttc") if bold else ("msyh.ttc",)):
        p = os.path.join("C:\\Windows\\Fonts", name)
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def main():
    assigned = parse_pins_h()

    def resolve(silk):
        if silk in RAILS:
            return RAILS[silk]
        if silk in RESERVED:
            return RESERVED[silk]
        if silk in assigned:
            return assigned[silk]
        return ("spare", "空闲")

    ROW_H = 46
    TOP = 150
    BOARD_L, BOARD_R = 600, 880
    BOX_W = 372
    W, H = 1480, TOP + 20 * ROW_H + 150

    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    f_title = font(34, True)
    f_sub = font(17)
    f_row = font(20)
    f_silk = font(19, True)
    f_small = font(16)

    d.text((40, 38), "ESP32-S3 N16R8  接线对照图", font=f_title, fill=(15, 23, 42))
    d.text((40, 84), "丝印编号与实物一一对应。每个模块占一段连续排针，并靠近它需要的电源轨。"
                     "由 src/pins.h 自动生成。", font=f_sub, fill=(100, 116, 139))

    # board body
    d.rounded_rectangle([BOARD_L, TOP - 26, BOARD_R, TOP + 20 * ROW_H - 18], 10,
                        fill=(24, 24, 27))
    d.text(((BOARD_L + BOARD_R) // 2, TOP + 10 * ROW_H - 22), "ESP32-S3\nN16R8",
           font=f_silk, fill=(161, 161, 170), anchor="mm", align="center")

    for i in range(20):
        y = TOP + i * ROW_H
        for side, silk in (("L", LEFT_SILK[i]), ("R", RIGHT_SILK[i])):
            kind, text = resolve(silk)
            bg, fg = PALETTE[kind]
            pad_x = BOARD_L + 22 if side == "L" else BOARD_R - 22

            # pad + silkscreen number inside the board outline
            d.ellipse([pad_x - 9, y - 9, pad_x + 9, y + 9], fill=(250, 204, 21)
                      if kind not in ("blocked", "spare") else (82, 82, 91))
            d.text((pad_x + (26 if side == "L" else -26), y), silk, font=f_silk,
                   fill=(244, 244, 245), anchor="lm" if side == "L" else "rm")

            # label block, and a lead line back to the pad
            if side == "L":
                x1, x0 = BOARD_L - 40, BOARD_L - 40 - BOX_W
                d.line([x1, y, pad_x, y], fill=bg if kind != "blocked" else (203, 213, 225),
                       width=2)
            else:
                x0, x1 = BOARD_R + 40, BOARD_R + 40 + BOX_W
                d.line([pad_x, y, x0, y], fill=bg if kind != "blocked" else (203, 213, 225),
                       width=2)
            d.rounded_rectangle([x0, y - 17, x1, y + 17], 7, fill=bg)
            # EN / TX / RX / 3V3 / 5V / GND are not GPIO numbers
            gpio = "GPIO%s" % silk if silk.isdigit() else silk if silk not in RAILS else ""
            d.text((x0 + 14, y), gpio, font=f_row, fill=fg, anchor="lm")
            d.text((x1 - 14, y), text, font=f_row, fill=fg, anchor="rm")

    # legend
    ly = TOP + 20 * ROW_H + 26
    lx = 40
    for kind, name in LEGEND:
        bg, _ = PALETTE[kind]
        d.rounded_rectangle([lx, ly, lx + 26, ly + 18], 4, fill=bg)
        d.text((lx + 34, ly + 9), name, font=f_small, fill=(71, 85, 105), anchor="lm")
        lx += 34 + int(d.textlength(name, font=f_small)) + 26

    d.text((40, ly + 44),
           "GPIO21(GPS 接收) 与 GPIO1(油门) 为实测确认脚，保持不变。"
           "GPIO38/39/40 是板载 SD，GPIO35–37 为 PSRAM，均不可改。"
           "油门必须留在 ADC1(GPIO1–10)：开启 WiFi 后 ADC2 读数不可用。",
           font=f_small, fill=(100, 116, 139))

    out = os.path.join(ROOT, "pinmap.png")
    img.save(out)
    print("wrote %s  (%dx%d)" % (out, W, H))

    # A pin assigned in pins.h but absent from the headers would be dropped from the
    # picture without a trace, so fail loudly instead.
    header = set(LEFT_SILK) | set(RIGHT_SILK)
    orphans = sorted(g for g in assigned if g not in header)
    if orphans:
        print("ERROR: pins.h assigns GPIO not present on either header: %s"
              % ", ".join(orphans), file=sys.stderr)
        return 1
    print("placed %d/%d assignments from pins.h" % (len(assigned), len(MACROS)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
