"""2.4\" ST7789 240×320 dashboard layout — mirror these in ESP firmware later."""

from __future__ import annotations

# Physical panel
LCD_W = 240
LCD_H = 320
LCD_DIAG_IN = 2.4

# RGB565-friendly palette (also fine on PC preview)
COL_BG = "#0c1218"
COL_PANEL = "#152028"
COL_LINE = "#243040"
COL_TEXT = "#e8eef4"
COL_DIM = "#6a7f92"
COL_ACCENT = "#3ddc84"
COL_CYAN = "#4fc3f7"
COL_ORANGE = "#ffb74d"
COL_RED = "#ef5350"
COL_REV = "#ff8a80"
COL_BLE_ON = "#69f0ae"
COL_BLE_OFF = "#546e7a"

# Layout rectangles (x, y, w, h) in logical pixels
ZONE_HEADER = (0, 0, 240, 54)
ZONE_RPM = (0, 54, 240, 118)
ZONE_METRICS = (0, 118, 240, 178)
ZONE_THROTTLE = (0, 178, 240, 248)
ZONE_TEMP = (0, 248, 240, 286)
ZONE_FOOTER = (0, 286, 240, 320)

MARGIN = 8
BAR_H = 14
