#include "tft_dashboard.h"

#include "pins.h"

#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define LGFX_USE_V1

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI _bus;
  lgfx::Light_PWM _light;

public:
  LGFX() {
    {
      auto cfg = _bus.config();
      // SPI3: SPI2's native pins are GPIO9-13 — the keypad. Stay off that bus.
      cfg.spi_host = SPI3_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = true;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = TFT_PIN_SCK;
      cfg.pin_mosi = TFT_PIN_MOSI;
      cfg.pin_miso = -1;
      cfg.pin_dc = TFT_PIN_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs = TFT_PIN_CS;
      cfg.pin_rst = TFT_PIN_RST;
      cfg.pin_busy = -1;
      cfg.panel_width = 240;
      cfg.panel_height = 320;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 0;
      cfg.dummy_read_bits = 0;
      cfg.readable = false;
      cfg.invert = true;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      _panel.config(cfg);
    }
    {
      auto cfg = _light.config();
      cfg.pin_bl = TFT_PIN_BL;
      cfg.invert = false;
      cfg.freq = 12000;
      cfg.pwm_channel = 7;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

static LGFX tft;
static lgfx::LGFX_Sprite canvas(&tft);
static bool ready = false;

static const int SCREEN_W = 240;
static const int EDGE = 2;  // hug left/right edges

static const uint16_t C_BG = 0x0000;
// Outdoor high-contrast (black bg)
static const uint16_t C_TEXT = 0xFFFF;
static const uint16_t C_MUTE = 0x9CF3;
static const uint16_t C_DIM = 0x8410;
static const uint16_t C_SPEED = 0x07FF;
static const uint16_t C_VOLT = 0x07E0;
static const uint16_t C_GREEN = 0x07E0;
static const uint16_t C_YELLOW = 0xFFE0;
static const uint16_t C_DOT = 0xFFE0;
static const uint16_t C_ORANGE = 0xFD20;
static const uint16_t C_RED = 0xF800;
static const uint16_t C_REV = 0xF81F;
static const uint16_t C_BAR_BG = 0x2945;
static const uint16_t C_AMP = 0xFD60;
static const uint16_t C_WATT = 0xFFFF;
static const uint16_t C_SEC = 0x07FF;
static const uint16_t C_TEMP = 0xFFFF;
static const uint16_t C_WH = 0xFFE0;

static int glyphW(int textSize) {
  return 6 * textSize;
}

static int glyphH(int textSize) {
  return 8 * textSize;
}

static int unitW(const char *unit) {
  if (unit == nullptr || unit[0] == '\0') {
    return 0;
  }
  return 2 + (int)strlen(unit) * glyphW(1);
}

static void drawUnitAt(int x, int yValue, int valueSize, const char *unit) {
  if (unit == nullptr || unit[0] == '\0') {
    return;
  }
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextSize(1);
  canvas.setTextColor(C_MUTE);
  canvas.drawString(unit, x, yValue + glyphH(valueSize) - glyphH(1) - 2);
}

static int intBlockW(const char *digits, const char *unit, int valueSize) {
  return (int)strlen(digits) * glyphW(valueSize) + unitW(unit);
}

/** Left-edge: integer + unit */
static void drawIntLeft(int y, int value, int maxDigits, const char *unit, int valueSize,
                        uint16_t color, int padDigits = 1) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%0*d", padDigits, value);
  if ((int)strlen(buf) > maxDigits) {
    buf[maxDigits] = '\0';
  }
  const int x = EDGE;
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextSize(valueSize);
  canvas.setTextColor(color);
  canvas.drawString(buf, x, y);
  drawUnitAt(x + (int)strlen(buf) * glyphW(valueSize), y, valueSize, unit);
}

/** Right-align integer+unit so block's right edge = rightEdge */
static void drawIntAlignRight(int rightEdge, int y, int value, int maxDigits, const char *unit,
                              int valueSize, uint16_t color, int padDigits = 1) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%0*d", padDigits, value);
  if ((int)strlen(buf) > maxDigits) {
    buf[maxDigits] = '\0';
  }
  const int block = intBlockW(buf, unit, valueSize);
  const int x = rightEdge - block;
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextSize(valueSize);
  canvas.setTextColor(color);
  canvas.drawString(buf, x, y);
  drawUnitAt(x + (int)strlen(buf) * glyphW(valueSize), y, valueSize, unit);
}

/** Right-edge: integer + unit (block flush to screen right) */
static void drawIntRight(int y, int value, int maxDigits, const char *unit, int valueSize,
                         uint16_t color, int padDigits = 1) {
  drawIntAlignRight(SCREEN_W - EDGE, y, value, maxDigits, unit, valueSize, color, padDigits);
}

/** One-decimal number with a tiny pixel-dot (not a full glyph-width '.').
 *  Returns total drawn width. */
static int drawDec1At(int x, int y, float value, float maxV, int valueSize, uint16_t color,
                      int padDigits = 1) {
  if (value < 0.0f) {
    value = 0.0f;
  }
  if (value > maxV) {
    value = maxV;
  }
  int whole = (int)value;
  int frac = (int)((value - (float)whole) * 10.0f + 0.5f);
  if (frac >= 10) {
    whole += 1;
    frac = 0;
  }

  char wb[8];
  char fb[2];
  snprintf(wb, sizeof(wb), "%0*d", padDigits, whole);
  snprintf(fb, sizeof(fb), "%d", frac);

  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextSize(valueSize);
  canvas.setTextColor(color);
  canvas.drawString(wb, x, y);
  int cx = x + (int)strlen(wb) * glyphW(valueSize);

  // Compact but visible decimal: bright yellow 3×3 px
  const int dotSize = 3;
  const int gap = 2;
  const int dotX = cx + gap;
  const int dotY = y + glyphH(valueSize) - dotSize - (valueSize >= 5 ? 6 : 4);
  canvas.fillRect(dotX, dotY, dotSize, dotSize, C_DOT);
  cx = dotX + dotSize + gap;

  canvas.drawString(fb, cx, y);
  cx += glyphW(valueSize);
  return cx - x;
}

/** Left-edge voltage with tight decimal + V unit. Fixed 2 whole digits: 25.2 / 00.0 */
static void drawVoltLeft(int y, float volts, int valueSize, uint16_t color) {
  const int w = drawDec1At(EDGE, y, volts, 99.9f, valueSize, color, 2);
  drawUnitAt(EDGE + w, y, valueSize, "V");
}

/** Centered one-decimal (or int if >=100) for Wh */
static void drawWhCentered(int y, float wh, int valueSize, uint16_t color) {
  if (wh < 0.0f) {
    wh = 0.0f;
  }
  if (wh > 9999.0f) {
    wh = 9999.0f;
  }
  canvas.setTextDatum(textdatum_t::top_left);
  if (wh >= 100.0f) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%.0f", wh);
    const int tw = (int)strlen(buf) * glyphW(valueSize);
    canvas.setTextSize(valueSize);
    canvas.setTextColor(color);
    canvas.drawString(buf, (SCREEN_W - tw) / 2, y);
  } else {
    // measure: whole digits + tiny dot + 1 frac digit
    int whole = (int)wh;
    char wb[8];
    snprintf(wb, sizeof(wb), "%d", whole);
    const int tw = (int)strlen(wb) * glyphW(valueSize) + 7 + glyphW(valueSize);
    drawDec1At((SCREEN_W - tw) / 2, y, wh, 99.9f, valueSize, color);
  }
}

/** Right-edge battery % */
static void drawPctRight(int y, int pct, int valueSize, uint16_t color) {
  drawIntRight(y, pct, 2, "%", valueSize, color, 2);
}

static void drawThrottleBar(int x, int y, int w, int h, int ppm) {
  canvas.fillRoundRect(x, y, w, h, 3, C_BAR_BG);
  const int mid = x + w / 2;
  canvas.drawFastVLine(mid, y + 2, h - 4, C_MUTE);
  const int pct = constrain((ppm - 1500) / 5, -100, 100);
  const int half = (w / 2) - 4;
  if (pct > 0) {
    const int bw = half * pct / 100;
    if (bw > 0) {
      canvas.fillRoundRect(mid + 1, y + 2, bw, h - 4, 2, C_GREEN);
    }
  } else if (pct < 0) {
    const int bw = half * (-pct) / 100;
    if (bw > 0) {
      canvas.fillRoundRect(mid - bw, y + 2, bw, h - 4, 2, C_REV);
    }
  }
}

static const float kGaugeMaxKmh = 160.0f;
static const float kGaugeStart = 135.0f;  // 0 deg = 3 o'clock, growing clockwise
static const float kGaugeSweep = 270.0f;
static const float kDeg2Rad = 0.01745329f;

/** fillArc for a range that crosses 0 deg, which LovyanGFX would normalize away. */
static void arcSeg(int cx, int cy, int rIn, int rOut, float a0, float a1, uint16_t color) {
  if (a1 <= a0) {
    return;
  }
  while (a0 >= 360.0f) {
    a0 -= 360.0f;
    a1 -= 360.0f;
  }
  if (a1 > 360.0f) {
    canvas.fillArc(cx, cy, rIn, rOut, a0, 359.99f, color);
    canvas.fillArc(cx, cy, rIn, rOut, 0.0f, a1 - 360.0f, color);
    return;
  }
  canvas.fillArc(cx, cy, rIn, rOut, a0, (a1 >= 360.0f) ? 359.99f : a1, color);
}

static uint16_t gaugeBandColor(float kmh) {
  if (kmh >= 120.0f) {
    return C_RED;
  }
  if (kmh >= 90.0f) {
    return C_ORANGE;
  }
  if (kmh >= 60.0f) {
    return C_YELLOW;
  }
  if (kmh >= 25.0f) {
    return C_SPEED;
  }
  return C_GREEN;
}

static const uint8_t kPageCount = 3;

/** Which main-screen page is live. Sits in the gap above the throttle strip. */
static void drawPageDots(uint8_t page) {
  const int cx = SCREEN_W / 2;
  for (int i = 0; i < (int)kPageCount; i++) {
    const int x = cx - 5 * ((int)kPageCount - 1) + i * 10;
    if (i == (int)page) {
      canvas.fillCircle(x, 287, 2, C_SPEED);
    } else {
      canvas.drawCircle(x, 287, 2, C_DIM);
    }
  }
}

/** Bottom throttle strip — identical on every page. */
static void drawThrottleRow(const DashboardInput *in) {
  const int y = 292;
  const char *dir = in->throttleDir ? in->throttleDir : "STOP";
  uint16_t dirC = C_MUTE;
  if (strcmp(dir, "FWD") == 0) {
    dirC = C_GREEN;
  } else if (strcmp(dir, "REV") == 0) {
    dirC = C_REV;
  }
  char buf[12];
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextSize(2);
  canvas.setTextColor(dirC);
  canvas.drawString(dir, EDGE, y);
  canvas.setTextColor(C_TEXT);
  snprintf(buf, sizeof(buf), "%+d%%", in->throttlePct);
  canvas.drawString(buf, EDGE + 60, y);
  drawThrottleBar(128, y, SCREEN_W - EDGE - 128, 16, in->throttlePpm);
}

static bool tempValid(float t) {
  return t > -20.0f && t < 200.0f;
}

enum VoltWarn { VOLT_OK = 0, VOLT_WARN, VOLT_CUT };

static VoltWarn classifyVolt(const DashboardInput *in) {
  if (!in->vescValid) {
    return VOLT_OK;
  }
  if (in->cutEndV > 1.0f && in->voltageV <= in->cutEndV) {
    return VOLT_CUT;
  }
  if (in->cutStartV > 1.0f && in->voltageV <= in->cutStartV) {
    return VOLT_WARN;
  }
  if (in->batteryPct >= 0.0f) {
    if (in->batteryPct < 10.0f) {
      return VOLT_CUT;
    }
    if (in->batteryPct < 25.0f) {
      return VOLT_WARN;
    }
  }
  return VOLT_OK;
}

static uint16_t voltColor(VoltWarn w) {
  if (w == VOLT_CUT) {
    return C_RED;
  }
  if (w == VOLT_WARN) {
    return C_YELLOW;
  }
  return C_VOLT;
}

static uint16_t battColor(float pct) {
  if (pct < 10.0f) {
    return C_RED;
  }
  if (pct < 25.0f) {
    return C_YELLOW;
  }
  return C_VOLT;
}

static uint16_t tempColor(float t) {
  if (t >= 85.0f) {
    return C_RED;
  }
  if (t >= 70.0f) {
    return C_ORANGE;
  }
  return C_TEMP;
}

void tftDashboardInit() {
  tft.init();
  tft.setRotation(0);
  tft.setBrightness(255);
  canvas.setColorDepth(16);
  canvas.createSprite(240, 320);
  ready = true;
  canvas.fillScreen(C_BG);
  canvas.pushSprite(0, 0);
}

static void drawIdleScreen(const DashboardInput *in) {
  const uint32_t t = millis();
  const float phase = t * 0.0018f;
  const float breathe = 0.55f + 0.45f * sinf(t * 0.0045f);

  canvas.fillScreen(C_BG);

  // Top GPS block
  {
    canvas.setTextDatum(textdatum_t::top_left);
    canvas.setTextSize(2);
    char line[28];

    // Stable GPS link: don't flip UBX/NMEA/RX every frame
    const char *link = "NO";
    uint16_t linkC = C_RED;
    if (in->gpsFixOk) {
      link = "FIX";
      linkC = C_GREEN;
    } else if (in->gpsUbxAlive || in->gpsNmeaAlive) {
      link = "OK";  // decoded (UBX and/or NMEA) — single label
      linkC = C_SPEED;
    } else if (in->gpsRxAlive || in->gpsRawAlive) {
      link = in->gpsRxAlive ? "RX" : "RAW";
      linkC = in->gpsRxAlive ? C_ORANGE : C_YELLOW;
    }

    static const char *holdLink = "NO";
    static uint16_t holdC = C_RED;
    static uint8_t holdRank = 0;
    static uint32_t holdMs = 0;
    const uint8_t rank = in->gpsFixOk                         ? 4
                         : (in->gpsUbxAlive || in->gpsNmeaAlive) ? 3
                         : in->gpsRxAlive                        ? 2
                         : in->gpsRawAlive                       ? 1
                                                                 : 0;
    if (rank > holdRank) {
      holdLink = link;
      holdC = linkC;
      holdRank = rank;
      holdMs = t;
    } else if (rank == holdRank && rank > 0) {
      holdLink = link;
      holdC = linkC;
      holdMs = t;
    } else if (rank < holdRank && (t - holdMs) > 4000) {
      // only downgrade after 4s without equal/better
      holdLink = link;
      holdC = linkC;
      holdRank = rank;
      holdMs = t;
    }

    canvas.setTextColor(holdC);
    snprintf(line, sizeof(line), "GPS %s", holdLink);
    canvas.drawString(line, EDGE, 4);

    canvas.setTextDatum(textdatum_t::top_right);
    canvas.setTextColor(in->gpsNmeaAlive || in->gpsUbxAlive || in->gpsFixOk ? C_SPEED : C_DIM);
    snprintf(line, sizeof(line), "SAT %02u", (unsigned)in->gpsSats);
    canvas.drawString(line, SCREEN_W - EDGE, 4);

    canvas.setTextDatum(textdatum_t::top_center);
    if (in->gpsTimeValid) {
      uint8_t h8 = (uint8_t)((in->gpsHour + 8) % 24);
      canvas.setTextColor(in->gpsFresh ? C_GREEN : C_DIM);
      snprintf(line, sizeof(line), "%02u:%02u:%02u", h8, in->gpsMinute, in->gpsSecond);
    } else {
      canvas.setTextColor(C_DIM);
      snprintf(line, sizeof(line), "00:00:00");
    }
    canvas.drawString(line, SCREEN_W / 2, 28);

    // Speed centered — show live GPS speed (bright when valid)
    const bool spdOk = in->gpsSpeedValid || in->gpsFixOk || in->gpsNmeaAlive || in->gpsUbxAlive;
    canvas.setTextColor(in->gpsSpeedValid ? C_TEXT : (spdOk ? C_MUTE : C_DIM));
    snprintf(line, sizeof(line), "%05.1f", (double)in->gpsSpeedKmh);
    canvas.setTextSize(4);
    canvas.drawString(line, SCREEN_W / 2, 50);
    canvas.setTextSize(2);
    canvas.setTextColor(C_MUTE);
    canvas.drawString("km/h", SCREEN_W / 2, 86);
    canvas.setTextDatum(textdatum_t::top_left);
  }

  // Animation ring moved down; no outer text ring
  const int cx = SCREEN_W / 2;
  const int cy = 168;
  const float ringMax = 56.0f;

  for (int i = 0; i < 3; i++) {
    const float r = 22.0f + i * 14.0f + 3.0f * sinf(phase + i * 0.7f);
    uint16_t c;
    if (i == 0) {
      c = C_SPEED;
    } else if (i == 1) {
      c = 0x0497;
    } else {
      c = 0x02B0;
    }
    canvas.drawCircle(cx, cy, (int)r, c);
  }

  const int nDots = 10;
  for (int i = 0; i < nDots; i++) {
    const float a = phase * 1.4f + (float)i * (6.2831853f / (float)nDots);
    const float rr = 46.0f + 3.0f * sinf(phase * 2.0f + i);
    const int x = cx + (int)(cosf(a) * rr);
    const int y = cy + (int)(sinf(a) * rr);
    const int rad = (i % 3 == 0) ? 4 : 2;
    canvas.fillCircle(x, y, rad, (i % 2) ? C_SPEED : C_YELLOW);
  }

  const int coreR = (int)(10.0f + 4.0f * breathe);
  canvas.fillCircle(cx, cy, coreR, 0x0230);
  canvas.drawCircle(cx, cy, coreR + 2, C_SPEED);

  canvas.setTextDatum(textdatum_t::middle_center);
  canvas.setTextSize(4);
  canvas.setTextColor(breathe > 0.7f ? C_TEXT : C_SPEED);
  canvas.drawString("VESC", cx, cy);

  const int yBelow = cy + (int)ringMax + 12;

  canvas.setTextSize(2);
  canvas.setTextColor(C_MUTE);
  const int dots = (int)((t / 350) % 4);
  char status[20];
  if (!in->vescLinkOk) {
    if (in->vescLinkBle) {
      snprintf(status, sizeof(status), "NO BLE%.*s", dots, "...");
    } else {
      snprintf(status, sizeof(status), "NO WIRE%.*s", dots, "...");
    }
  } else {
    snprintf(status, sizeof(status), "WAITING%.*s", dots, "...");
  }
  canvas.drawString(status, cx, yBelow);

  {
    char line[28];
    auto fmtHms = [](char *out, size_t n, uint32_t sec) {
      const uint32_t h = sec / 3600u;
      const uint32_t m = (sec % 3600u) / 60u;
      const uint32_t s = sec % 60u;
      snprintf(out, n, "%02lu:%02lu:%02lu", (unsigned long)h, (unsigned long)m,
               (unsigned long)s);
    };
    char hms[12];
    canvas.setTextSize(2);
    canvas.setTextColor(C_MUTE);
    fmtHms(hms, sizeof(hms), in->totalRunSec);
    snprintf(line, sizeof(line), "TOTAL %s", hms);
    canvas.drawString(line, cx, yBelow + 22);
    fmtHms(hms, sizeof(hms), in->sessionSec);
    snprintf(line, sizeof(line), "SESS  %s", hms);
    canvas.setTextColor(in->vescLinkOk ? C_SPEED : C_DIM);
    canvas.drawString(line, cx, yBelow + 44);
  }

  drawThrottleRow(in);
  canvas.pushSprite(0, 0);
}

static void drawGpsScreen(const DashboardInput *in) {
  const uint32_t t = millis();
  canvas.fillScreen(C_BG);

  // The arc sweeps toward the reading instead of snapping to it. Time-based easing so the
  // motion is the same whatever the frame interval turns out to be.
  static float shownKmh = 0.0f;
  static uint32_t lastAnimMs = 0;
  const uint32_t dt = (lastAnimMs == 0) ? 0 : (t - lastAnimMs);
  lastAnimMs = t;
  float target = (in->gpsSpeedKmh >= 0.0f) ? in->gpsSpeedKmh : 0.0f;
  if (target > kGaugeMaxKmh) {
    target = kGaugeMaxKmh;
  }
  if (dt > 0 && dt < 1000) {
    shownKmh += (target - shownKmh) * (1.0f - expf(-(float)dt / 220.0f));
  } else {
    shownKmh = target;
  }
  if (fabsf(shownKmh - target) < 0.05f) {
    shownKmh = target;
  }

  char line[40];
  const int cx = SCREEN_W / 2;
  const int cy = 112;
  const int rOut = 68;
  const int rIn = 52;
  const float breathe = 0.5f + 0.5f * sinf((float)t * 0.0042f);

  // ——— Header: sats left, clock right ———
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextSize(1);
  canvas.setTextColor(C_MUTE);
  canvas.drawString("SAT", EDGE, 8);
  uint16_t satC = C_DIM;
  if (in->gpsFixOk) {
    satC = C_GREEN;
  } else if (in->gpsNmeaAlive || in->gpsUbxAlive) {
    satC = C_SPEED;
  }
  snprintf(line, sizeof(line), "%02u", (unsigned)in->gpsSats);
  canvas.setTextSize(2);
  canvas.setTextColor(satC);
  canvas.drawString(line, EDGE + 22, 2);

  canvas.setTextDatum(textdatum_t::top_right);
  canvas.setTextSize(2);
  if (in->gpsTimeValid) {
    const uint8_t h8 = (uint8_t)((in->gpsHour + 8) % 24);
    snprintf(line, sizeof(line), "%02u:%02u:%02u", h8, in->gpsMinute, in->gpsSecond);
    canvas.setTextColor(in->gpsFresh ? C_TEXT : C_DIM);
  } else {
    snprintf(line, sizeof(line), "00:00:00");
    canvas.setTextColor(C_DIM);
  }
  canvas.drawString(line, SCREEN_W - EDGE, 2);

  // ——— Ring gauge, 0..160 km/h over a 270 deg sweep ———
  arcSeg(cx, cy, rIn, rOut, kGaugeStart, kGaugeStart + kGaugeSweep, C_BAR_BG);
  arcSeg(cx, cy, rOut + 1, rOut + 2, kGaugeStart, kGaugeStart + kGaugeSweep, 0x2124);
  arcSeg(cx, cy, rIn - 2, rIn - 1, kGaugeStart, kGaugeStart + kGaugeSweep, 0x2124);

  // 5 km/h slices so the filled part colour-grades as it grows
  for (float v = 0.0f; v < shownKmh; v += 5.0f) {
    const float vEnd = (v + 5.0f > shownKmh) ? shownKmh : (v + 5.0f);
    const float a0 = kGaugeStart + kGaugeSweep * (v / kGaugeMaxKmh);
    const float a1 = kGaugeStart + kGaugeSweep * (vEnd / kGaugeMaxKmh) + 0.8f;
    arcSeg(cx, cy, rIn, rOut, a0, a1, gaugeBandColor(v));
  }

  // Ticks every 20 km/h, lit once the needle has passed them
  for (int i = 0; i <= 8; i++) {
    const float v = (float)i * 20.0f;
    const float a = (kGaugeStart + kGaugeSweep * (v / kGaugeMaxKmh)) * kDeg2Rad;
    const float ca = cosf(a);
    const float sa = sinf(a);
    const int r0 = rOut + 3;
    const int r1 = rOut + ((i % 2 == 0) ? 9 : 6);
    const uint16_t c = (v <= shownKmh + 0.01f) ? gaugeBandColor(v) : 0x39E7;
    canvas.drawLine(cx + (int)(ca * (float)r0), cy + (int)(sa * (float)r0),
                    cx + (int)(ca * (float)r1), cy + (int)(sa * (float)r1), c);
  }

  canvas.setTextSize(1);
  canvas.setTextColor(C_MUTE);
  canvas.setTextDatum(textdatum_t::middle_center);
  {
    const float a0 = kGaugeStart * kDeg2Rad;
    const float a1 = (kGaugeStart + kGaugeSweep) * kDeg2Rad;
    const int rl = rOut + 15;
    canvas.drawString("0", cx + (int)(cosf(a0) * (float)rl), cy + (int)(sinf(a0) * (float)rl));
    canvas.drawString("160", cx + (int)(cosf(a1) * (float)rl), cy + (int)(sinf(a1) * (float)rl));
  }

  // Leading-edge marker, breathing so a steady speed still looks live
  {
    const float a = (kGaugeStart + kGaugeSweep * (shownKmh / kGaugeMaxKmh)) * kDeg2Rad;
    const int rm = (rIn + rOut) / 2;
    const int mx = cx + (int)(cosf(a) * (float)rm);
    const int my = cy + (int)(sinf(a) * (float)rm);
    canvas.fillCircle(mx, my, 4 + (int)(2.0f * breathe), gaugeBandColor(shownKmh));
    canvas.fillCircle(mx, my, 2, C_TEXT);
  }

  // ——— Centre readout ———
  {
    const int iv = (int)(shownKmh + 0.5f);
    snprintf(line, sizeof(line), "%d", iv);
    canvas.setTextDatum(textdatum_t::middle_center);
    canvas.setTextSize((iv >= 100) ? 5 : 6);
    canvas.setTextColor(in->gpsSpeedValid ? C_TEXT : C_DIM);
    canvas.drawString(line, cx, cy - 6);
    canvas.setTextSize(1);
    canvas.setTextColor(C_MUTE);
    canvas.drawString("km/h", cx, cy + 26);
  }

  // ——— Three stat rows ———
  auto fmtHms = [](char *out, size_t n, uint32_t sec) {
    snprintf(out, n, "%lu:%02lu:%02lu", (unsigned long)(sec / 3600u),
             (unsigned long)((sec % 3600u) / 60u), (unsigned long)(sec % 60u));
  };
  auto statRow = [&](int y, const char *label, uint16_t labelC, float kmh, const char *extra) {
    canvas.drawFastHLine(EDGE, y - 5, SCREEN_W - 2 * EDGE, 0x2124);
    canvas.setTextDatum(textdatum_t::top_left);
    canvas.setTextSize(1);
    canvas.setTextColor(labelC);
    canvas.drawString(label, EDGE, y + 5);

    char v[12];
    snprintf(v, sizeof(v), "%.1f", (double)kmh);
    canvas.setTextDatum(textdatum_t::top_right);
    canvas.setTextSize(2);
    canvas.setTextColor(kmh > 0.05f ? C_TEXT : C_DIM);
    canvas.drawString(v, 104, y);
    canvas.setTextDatum(textdatum_t::top_left);
    canvas.setTextSize(1);
    canvas.setTextColor(C_MUTE);
    canvas.drawString("km/h", 108, y + 9);

    canvas.setTextDatum(textdatum_t::top_right);
    canvas.setTextColor(C_SEC);
    canvas.drawString(extra, SCREEN_W - EDGE, y + 5);
  };

  char extra[32];
  char hms[16];
  if (in->tripMaxY > 0) {
    snprintf(extra, sizeof(extra), "%02u-%02u %02u:%02u", in->tripMaxMo, in->tripMaxD,
             in->tripMaxH, in->tripMaxMi);
  } else {
    snprintf(extra, sizeof(extra), "--");
  }
  statRow(188, "MAX", C_YELLOW, in->tripMaxKmh, extra);

  fmtHms(hms, sizeof(hms), in->tripSessionSec);
  snprintf(extra, sizeof(extra), "%s", hms);
  statRow(221, "TRIP", C_SPEED, in->tripSessionAvgKmh, extra);

  fmtHms(hms, sizeof(hms), in->tripTotalSec);
  snprintf(extra, sizeof(extra), "%.1fkm %s", (double)in->tripTotalKm, hms);
  statRow(254, "ALL", C_GREEN, in->tripTotalAvgKmh, extra);

  drawPageDots(in->dataPage);
  drawThrottleRow(in);
  canvas.pushSprite(0, 0);
}

static const char *cardinal16(float deg) {
  static const char *names[16] = {"N",  "NNE", "NE", "ENE", "E",  "ESE", "SE", "SSE",
                                  "S",  "SSW", "SW", "WSW", "W",  "WNW", "NW", "NNW"};
  int i = (int)((deg + 11.25f) / 22.5f);
  return names[i & 15];
}

/** Heading-up compass rose. Magnetometer heading is used when the 5883 is alive so
 *  the rose works at a standstill; GPS course-over-ground is the moving fallback. */
static void drawCompassScreen(const DashboardInput *in) {
  const uint32_t t = millis();
  canvas.fillScreen(C_BG);

  const bool magLive = in->magOk;
  const bool gpsLive = in->gpsHeadValid;
  const bool live = magLive || gpsLive;
  const float headDeg = magLive ? in->magHeadDeg : in->gpsHeadDeg;

  static float shownHead = 0.0f;
  static uint32_t lastHeadMs = 0;
  const uint32_t dt = (lastHeadMs == 0) ? 0 : (t - lastHeadMs);
  lastHeadMs = t;
  if (live) {
    float d = headDeg - shownHead;
    while (d > 180.0f) {
      d -= 360.0f;
    }
    while (d < -180.0f) {
      d += 360.0f;
    }
    if (dt > 0 && dt < 1000) {
      shownHead += d * (1.0f - expf(-(float)dt / 200.0f));
    } else {
      shownHead = headDeg;
    }
    while (shownHead < 0.0f) {
      shownHead += 360.0f;
    }
    while (shownHead >= 360.0f) {
      shownHead -= 360.0f;
    }
  }

  char line[48];
  const int cx = SCREEN_W / 2;
  const int cy = 128;
  const int rOut = 84;

  // ——— Header: sats left, clock right ———
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextSize(1);
  canvas.setTextColor(C_MUTE);
  canvas.drawString("SAT", EDGE, 8);
  uint16_t satC = C_DIM;
  if (in->gpsFixOk) {
    satC = C_GREEN;
  } else if (in->gpsNmeaAlive || in->gpsUbxAlive) {
    satC = C_SPEED;
  }
  snprintf(line, sizeof(line), "%02u", (unsigned)in->gpsSats);
  canvas.setTextSize(2);
  canvas.setTextColor(satC);
  canvas.drawString(line, EDGE + 22, 2);

  canvas.setTextDatum(textdatum_t::top_right);
  canvas.setTextSize(2);
  if (in->gpsTimeValid) {
    const uint8_t h8 = (uint8_t)((in->gpsHour + 8) % 24);
    snprintf(line, sizeof(line), "%02u:%02u:%02u", h8, in->gpsMinute, in->gpsSecond);
    canvas.setTextColor(in->gpsFresh ? C_TEXT : C_DIM);
  } else {
    snprintf(line, sizeof(line), "00:00:00");
    canvas.setTextColor(C_DIM);
  }
  canvas.drawString(line, SCREEN_W - EDGE, 2);

  // ——— Rose ———
  canvas.drawCircle(cx, cy, rOut, live ? 0x39E7 : 0x2124);
  canvas.drawCircle(cx, cy, rOut - 1, live ? 0x2124 : C_BG);

  for (int b = 0; b < 360; b += 15) {
    const float a = ((float)b - shownHead - 90.0f) * kDeg2Rad;
    const float ca = cosf(a);
    const float sa = sinf(a);
    const bool principal = (b % 45) == 0;
    const int r0 = rOut - (principal ? 12 : 6);
    canvas.drawLine(cx + (int)(ca * (float)r0), cy + (int)(sa * (float)r0),
                    cx + (int)(ca * (float)(rOut - 2)), cy + (int)(sa * (float)(rOut - 2)),
                    live ? (principal ? C_MUTE : 0x39E7) : 0x2124);
  }

  canvas.setTextDatum(textdatum_t::middle_center);
  const char *marks[4] = {"N", "E", "S", "W"};
  for (int i = 0; i < 4; i++) {
    const float a = ((float)(i * 90) - shownHead - 90.0f) * kDeg2Rad;
    const int r = rOut - 26;
    const int x = cx + (int)(cosf(a) * (float)r);
    const int y = cy + (int)(sinf(a) * (float)r);
    canvas.setTextSize(2);
    canvas.setTextColor(!live ? C_DIM : (i == 0) ? C_RED : C_MUTE);
    canvas.drawString(marks[i], x, y);
  }

  {
    const int yTip = cy - rOut + 2;
    canvas.fillTriangle(cx, yTip + 12, cx - 7, yTip - 2, cx + 7, yTip - 2,
                        live ? C_YELLOW : C_DIM);
  }

  canvas.setTextDatum(textdatum_t::middle_center);
  if (live) {
    snprintf(line, sizeof(line), "%03u", (unsigned)((int)(headDeg + 0.5f) % 360));
    canvas.setTextSize(4);
    canvas.setTextColor(C_TEXT);
    canvas.drawString(line, cx, cy - 10);
    canvas.setTextSize(2);
    canvas.setTextColor(magLive ? C_GREEN : C_SPEED);
    canvas.drawString(cardinal16(headDeg), cx, cy + 22);
  } else {
    canvas.setTextSize(4);
    canvas.setTextColor(C_DIM);
    canvas.drawString("---", cx, cy - 10);
  }

  canvas.setTextDatum(textdatum_t::top_center);
  canvas.setTextSize(2);
  if (in->gpsSpeedValid) {
    snprintf(line, sizeof(line), "%.1f km/h", (double)in->gpsSpeedKmh);
    canvas.setTextColor(C_TEXT);
  } else {
    snprintf(line, sizeof(line), "00.0 km/h");
    canvas.setTextColor(C_DIM);
  }
  canvas.drawString(line, cx, 220);

  const char *ts = in->trackState ? in->trackState : "off";
  if (strcmp(ts, "rec") == 0) {
    canvas.setTextSize(1);
    canvas.fillCircle(cx - 34, 250, 3, C_RED);
    snprintf(line, sizeof(line), "REC %lu", (unsigned long)in->trackPoints);
    canvas.setTextColor(C_TEXT);
    canvas.drawString(line, cx, 246);
  }

  drawPageDots(in->dataPage);
  drawThrottleRow(in);
  canvas.pushSprite(0, 0);
}

static void drawStatsScreen(const DashboardInput *in) {
  canvas.fillScreen(C_BG);
  char line[36];

  canvas.setTextDatum(textdatum_t::top_center);
  canvas.setTextSize(2);
  canvas.setTextColor(C_SPEED);
  canvas.drawString("TRIP STATS", SCREEN_W / 2, 8);

  canvas.setTextDatum(textdatum_t::top_left);
  const int baseY = 44;
  const int rowH = 72;
  const int nItems = 3;
  const int sel = (in->statsSel < 0) ? 0 : (in->statsSel >= nItems ? nItems - 1 : in->statsSel);

  auto drawRow = [&](int idx, const char *title, const char *value, const char *when) {
    const int y = baseY + idx * rowH;
    const bool on = (idx == sel);
    if (on) {
      canvas.fillRect(EDGE - 2, y - 4, SCREEN_W - 2 * EDGE + 4, rowH - 8, 0x2124);
      canvas.drawRect(EDGE - 2, y - 4, SCREEN_W - 2 * EDGE + 4, rowH - 8, C_SPEED);
    }
    canvas.setTextSize(2);
    canvas.setTextColor(on ? C_YELLOW : C_MUTE);
    canvas.drawString(title, EDGE, y);
    canvas.setTextColor(on ? C_TEXT : C_SPEED);
    canvas.drawString(value, EDGE, y + 22);
    canvas.setTextSize(1);
    canvas.setTextColor(C_MUTE);
    canvas.drawString(when, EDGE, y + 46);
  };

  snprintf(line, sizeof(line), "%05.1f km/h", (double)in->tripMaxKmh);
  char when[28];
  if (in->tripMaxY > 0) {
    snprintf(when, sizeof(when), "%04u-%02u-%02u %02u:%02u:%02u", in->tripMaxY, in->tripMaxMo,
             in->tripMaxD, in->tripMaxH, in->tripMaxMi, in->tripMaxS);
  } else {
    snprintf(when, sizeof(when), "time --");
  }
  drawRow(0, "MAX SPEED", line, when);

  snprintf(line, sizeof(line), "%.2f km", (double)in->tripDayKm);
  if (in->tripDayY > 0) {
    snprintf(when, sizeof(when), "day %04u-%02u-%02u", in->tripDayY, in->tripDayMo, in->tripDayD);
  } else {
    snprintf(when, sizeof(when), "day --");
  }
  drawRow(1, "TODAY", line, when);

  snprintf(line, sizeof(line), "%.2f km", (double)in->tripTotalKm);
  if (in->tripTotY > 0) {
    snprintf(when, sizeof(when), "upd %04u-%02u-%02u %02u:%02u", in->tripTotY, in->tripTotMo,
             in->tripTotD, in->tripTotH, in->tripTotMi);
  } else {
    snprintf(when, sizeof(when), "upd --");
  }
  drawRow(2, "TOTAL", line, when);

  canvas.pushSprite(0, 0);
}

static void drawSettingsScreen(const DashboardInput *in) {
  canvas.fillScreen(C_BG);
  canvas.setTextDatum(textdatum_t::top_center);
  canvas.setTextSize(2);
  canvas.setTextColor(C_SPEED);
  canvas.drawString("SETTINGS", SCREEN_W / 2, 6);

  char linkLine[28];
  char modeLine[28];
  char chLine[28];
  char dataLine[28];
  char trackLine[28];
  char revLine[28];
  char magLine[28];
  snprintf(linkLine, sizeof(linkLine), "VESC %s", in->vescLinkBle ? "无线" : "有线");
  snprintf(modeLine, sizeof(modeLine), "OUT %s", in->outModeLabel ? in->outModeLabel : "?");
  snprintf(chLine, sizeof(chLine), "CPPM CH %u", (unsigned)in->outThrottleCh);
  snprintf(dataLine, sizeof(dataLine), "DATA VIEW %s", in->forceData ? "ON" : "AUTO");
  snprintf(trackLine, sizeof(trackLine), "TRACK LOG %s", in->trackLogOn ? "ON" : "OFF");
  snprintf(revLine, sizeof(revLine), "THR REVERSE %s", in->thrReverse ? "ON" : "OFF");
  snprintf(magLine, sizeof(magLine), "MAG ROT %u", (unsigned)in->magYawDeg);

  const char *items[11] = {
      linkLine,  "TRIP STATS", "THROTTLE CAL", revLine,  modeLine,
      chLine,    "SD TEST",    trackLine,      dataLine, magLine,
      "BACK",
  };
  const int n = 11;
  const int sel = (in->statsSel < 0) ? 0 : (in->statsSel >= n ? n - 1 : in->statsSel);
  const int rowH = 22;
  const int baseY = 26;
  canvas.setTextDatum(textdatum_t::top_left);
  for (int i = 0; i < n; i++) {
    const int y = baseY + i * rowH;
    const bool on = (i == sel);
    if (on) {
      canvas.fillRect(EDGE, y - 4, SCREEN_W - 2 * EDGE, rowH - 4, 0x2124);
      canvas.drawRect(EDGE, y - 4, SCREEN_W - 2 * EDGE, rowH - 4, C_SPEED);
    }
    canvas.setTextSize(2);
    canvas.setTextColor(on ? C_YELLOW : C_TEXT);
    canvas.drawString(items[i], EDGE + 10, y + 4);
  }

  canvas.setTextSize(1);
  canvas.setTextDatum(textdatum_t::top_center);
  if (sel == 0) {
    canvas.setTextColor(C_MUTE);
    canvas.drawString(in->vescLinkStatus ? in->vescLinkStatus : "", SCREEN_W / 2, 288);
  } else if (in->thrCalReady) {
    canvas.setTextColor(C_GREEN);
    char line[40];
    snprintf(line, sizeof(line), "CAL C%d F%d R%d", in->thrCalCenter, in->thrCalFwd, in->thrCalRev);
    canvas.drawString(line, SCREEN_W / 2, 288);
  } else {
    canvas.setTextColor(C_MUTE);
    canvas.drawString("CAL not set", SCREEN_W / 2, 288);
  }
  canvas.pushSprite(0, 0);
}

static void drawThrCalScreen(const DashboardInput *in) {
  canvas.fillScreen(C_BG);
  char line[48];
  const int step = in->statsSel;  // 0 center 1 fwd 2 rev

  canvas.setTextDatum(textdatum_t::top_center);
  canvas.setTextSize(2);
  canvas.setTextColor(C_SPEED);
  canvas.drawString("THR CAL", SCREEN_W / 2, 8);

  canvas.setTextSize(1);
  canvas.setTextColor(C_YELLOW);
  const char *how = "release stick, press OK";
  if (step == 1) {
    how = "hold FULL FWD, press OK";
  } else if (step >= 2) {
    how = "hold FULL REV, press OK";
  }
  canvas.drawString(how, SCREEN_W / 2, 34);

  canvas.setTextSize(3);
  canvas.setTextColor(C_TEXT);
  snprintf(line, sizeof(line), "%d", in->thrLiveMv);
  canvas.drawString(line, SCREEN_W / 2, 58);
  canvas.setTextSize(1);
  canvas.setTextColor(C_MUTE);
  canvas.drawString("mV live", SCREEN_W / 2, 92);

  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextSize(2);
  auto row = [&](int y, const char *lab, int v, bool focus) {
    canvas.setTextColor(focus ? C_YELLOW : C_MUTE);
    canvas.drawString(lab, EDGE, y);
    canvas.setTextColor(focus ? C_TEXT : C_SPEED);
    if (v >= 0) {
      snprintf(line, sizeof(line), "%d mV", v);
    } else {
      snprintf(line, sizeof(line), "----");
    }
    canvas.drawString(line, 110, y);
  };
  row(120, "1 CEN", in->thrCalCenter, step == 0);
  row(152, "2 FWD", in->thrCalFwd, step == 1);
  row(184, "3 REV", in->thrCalRev, step == 2);

  canvas.setTextDatum(textdatum_t::top_center);
  canvas.setTextSize(1);
  if (in->thrCalHint && in->thrCalHint[0] != '\0') {
    canvas.setTextColor(C_ORANGE);
    canvas.drawString(in->thrCalHint, SCREEN_W / 2, 230);
  } else {
    canvas.setTextColor(C_MUTE);
    canvas.drawString("OK capture   LEFT back", SCREEN_W / 2, 230);
  }

  canvas.setTextColor(C_MUTE);
  snprintf(line, sizeof(line), "out %+d%%  %dus", in->thrOutPct, in->thrOutPpm);
  canvas.drawString(line, SCREEN_W / 2, 260);
  if (in->thrCalReady) {
    canvas.setTextColor(C_GREEN);
    canvas.drawString("using saved CAL", SCREEN_W / 2, 280);
  } else {
    canvas.setTextColor(C_MUTE);
    canvas.drawString("not saved yet", SCREEN_W / 2, 280);
  }
  canvas.pushSprite(0, 0);
}

static void drawSdTestScreen(const DashboardInput *in) {
  canvas.fillScreen(C_BG);
  char line[48];
  canvas.setTextDatum(textdatum_t::top_center);
  canvas.setTextSize(2);
  canvas.setTextColor(C_SPEED);
  canvas.drawString("SD TEST", SCREEN_W / 2, 10);

  canvas.setTextSize(3);
  const char *st = in->sdStatus ? in->sdStatus : "none";
  uint16_t c = C_ORANGE;
  if (strcmp(st, "ok") == 0) {
    c = C_GREEN;
  } else if (strcmp(st, "no_card") == 0) {
    c = C_YELLOW;
  } else if (strcmp(st, "error") == 0) {
    c = C_RED;
  }
  canvas.setTextColor(c);
  canvas.drawString(st, SCREEN_W / 2, 80);

  canvas.setTextSize(2);
  canvas.setTextColor(C_TEXT);
  canvas.drawString(in->sdMsg ? in->sdMsg : "-", SCREEN_W / 2, 140);

  const char *ts = in->trackState ? in->trackState : "off";
  canvas.setTextSize(2);
  canvas.setTextColor((strcmp(ts, "rec") == 0) ? C_GREEN
                      : (strcmp(ts, "error") == 0) ? C_RED
                                                   : C_MUTE);
  snprintf(line, sizeof(line), "TRACK %s", ts);
  canvas.drawString(line, SCREEN_W / 2, 196);

  if (in->trackFile != nullptr && in->trackFile[0] != '\0') {
    canvas.setTextSize(1);
    canvas.setTextColor(C_MUTE);
    canvas.drawString(in->trackFile, SCREEN_W / 2, 226);
    snprintf(line, sizeof(line), "%lu pts", (unsigned long)in->trackPoints);
    canvas.drawString(line, SCREEN_W / 2, 242);
  }

  canvas.pushSprite(0, 0);
}

void tftDashboardUpdate(const DashboardInput *in) {
  if (!ready || in == nullptr) {
    return;
  }

  if (in->uiScreen == 1) {
    drawSettingsScreen(in);
    return;
  }
  if (in->uiScreen == 2) {
    drawStatsScreen(in);
    return;
  }
  if (in->uiScreen == 3) {
    drawThrCalScreen(in);
    return;
  }
  if (in->uiScreen == 4) {
    drawSdTestScreen(in);
    return;
  }

  // GPS pages stand on their own — they must not depend on the VESC link.
  if (in->dataPage == 1) {
    drawGpsScreen(in);
    return;
  }
  if (in->dataPage == 2) {
    drawCompassScreen(in);
    return;
  }

  if (!in->forceData && (!in->vescLinkOk || !in->vescValid)) {
    drawIdleScreen(in);
    return;
  }

  canvas.fillScreen(C_BG);
  char buf[28];
  const VoltWarn vw = classifyVolt(in);
  // Larger metrics; speed/throttle sizes unchanged
  const int numSize = 6;
  const int rowH = glyphH(numSize) + 6;  // tight row gap
  const int row1Y = 100;
  const int row2Y = row1Y + rowH;

  // ——— GPS sats: tucked beside the big speed, no row of its own ———
  {
    canvas.setTextDatum(textdatum_t::top_left);
    canvas.setTextSize(1);
    canvas.setTextColor(C_MUTE);
    canvas.drawString("SAT", EDGE, 4);
    uint16_t satC = C_DIM;
    if (in->gpsFixOk) {
      satC = C_GREEN;
    } else if (in->gpsNmeaAlive || in->gpsUbxAlive) {
      satC = C_SPEED;
    }
    snprintf(buf, sizeof(buf), "%02u", (unsigned)in->gpsSats);
    canvas.setTextSize(2);
    canvas.setTextColor(satC);
    canvas.drawString(buf, EDGE, 14);
  }

  // ——— Speed (keep size 9) ———
  {
    canvas.setTextDatum(textdatum_t::top_left);
    int spd = in->vescValid ? (int)(fabsf(in->speedKmh) + 0.5f) : 0;
    if (spd > 99) {
      spd = 99;
    }
    snprintf(buf, sizeof(buf), "%02d", spd);
    canvas.setTextColor(in->vescValid ? C_SPEED : C_DIM);
    const int spdSize = 9;
    const int sw = (int)strlen(buf) * glyphW(spdSize);
    const int sx = (SCREEN_W - sw) / 2;
    const int sy = 4;
    canvas.setTextSize(spdSize);
    canvas.drawString(buf, sx, sy);
    canvas.setTextSize(1);
    canvas.setTextColor(C_MUTE);
    canvas.drawString("km/h", sx + sw + 2, sy + glyphH(spdSize) - glyphH(1) - 4);
  }

  // ——— Clock: centered in the band between the big speed and the voltage row ———
  {
    canvas.setTextDatum(textdatum_t::top_center);
    canvas.setTextSize(2);
    if (in->gpsTimeValid) {
      const uint8_t h8 = (uint8_t)((in->gpsHour + 8) % 24);
      snprintf(buf, sizeof(buf), "%02u:%02u:%02u", h8, in->gpsMinute, in->gpsSecond);
      canvas.setTextColor(in->gpsFresh ? C_TEXT : C_DIM);
    } else {
      snprintf(buf, sizeof(buf), "00:00:00");
      canvas.setTextColor(C_DIM);
    }
    canvas.drawString(buf, SCREEN_W / 2, 80);
  }

  // ——— Temp top-right overlay (2x previous size) ———
  {
    const bool tOk = in->vescValid && tempValid(in->tempFetC);
    int t = tOk ? (int)(in->tempFetC + 0.5f) : 0;
    if (t > 199) {
      t = 199;
    }
    snprintf(buf, sizeof(buf), "%02d", t);
    const int tSize = 4;
    const int tw = (int)strlen(buf) * glyphW(tSize);
    const int tx = SCREEN_W - EDGE - tw - unitW("C");
    canvas.setTextDatum(textdatum_t::top_left);
    canvas.setTextSize(tSize);
    canvas.setTextColor(tOk ? tempColor(in->tempFetC) : C_DIM);
    canvas.drawString(buf, tx, 2);
    drawUnitAt(tx + tw, 2, tSize, "C");
  }

  // ——— Row: Voltage LEFT | Battery RIGHT ———
  drawVoltLeft(row1Y, in->vescValid ? in->voltageV : 0.0f, numSize,
               in->vescValid ? voltColor(vw) : C_DIM);

  {
    const bool battOk = in->vescValid && in->batteryPct >= 0.0f;
    int pct = battOk ? (int)(in->batteryPct + 0.5f) : 0;
    pct = constrain(pct, 0, 99);
    drawPctRight(row1Y, pct, numSize, battOk ? battColor((float)pct) : C_DIM);
  }

  // ——— Row: Current | Power = 4:6 — current right-align in left 40%, power flush right ———
  {
    // 3-digit A plus 4-digit W needs 268px at numSize — wider than the 240px panel, which
    // pushed the current column off the left edge. One size down fits with a real gap:
    // A = 2..98, W = 110..236. Anchored to both edges so neither can drift into the other.
    const int pwrSize = 5;
    const int y = row2Y + (glyphH(numSize) - glyphH(pwrSize)) / 2;
    int a = in->vescValid ? (int)lroundf(fabsf(in->currentInA)) : 0;
    int w = in->vescValid ? (int)lroundf(fabsf(in->powerW)) : 0;
    a = constrain(a, 0, 999);
    w = constrain(w, 0, 9999);
    drawIntLeft(y, a, 3, "A", pwrSize, in->vescValid ? C_AMP : C_DIM, 3);
    drawIntRight(y, w, 4, "W", pwrSize, in->vescValid ? C_WATT : C_DIM, 4);
  }

  // ——— Secondary: eRPM left | Wh center (slightly lower, larger) | DUTY right ———
  {
    const int secY = row2Y + rowH + 2;
    canvas.setTextDatum(textdatum_t::top_left);
    const int erpmSize = 3;
    const int whSize = 5;
    const int dutySize = 3;
    const int valY = secY + 11;
    const uint16_t secC = in->vescValid ? C_SEC : C_DIM;

    // Labels on same band as eRPM/DUTY
    canvas.setTextSize(1);
    canvas.setTextColor(C_MUTE);
    canvas.drawString("eRPM", EDGE, secY);
    canvas.drawString("DUTY", SCREEN_W - EDGE - 4 * glyphW(1), secY);
    canvas.setTextDatum(textdatum_t::top_center);
    canvas.drawString("Wh", SCREEN_W / 2, secY);
    canvas.setTextDatum(textdatum_t::top_left);

    // eRPM — flush left. Padded to 4 so the placeholder cannot reach the centered Wh.
    long erpm = in->vescValid ? (long)lroundf(fabsf(in->rpm)) : 0;
    if (erpm > 999999) {
      erpm = 999999;
    }
    snprintf(buf, sizeof(buf), "%04ld", erpm);
    canvas.setTextSize(erpmSize);
    canvas.setTextColor(secC);
    canvas.drawString(buf, EDGE, valY);

    // DUTY — flush right
    int dutyPct = in->vescValid ? (int)lroundf(fabsf(in->duty) * 100.0f) : 0;
    dutyPct = constrain(dutyPct, 0, 99);
    snprintf(buf, sizeof(buf), "%02d%%", dutyPct);
    const int dutyW = (int)strlen(buf) * glyphW(dutySize);
    canvas.setTextSize(dutySize);
    canvas.setTextColor(secC);
    canvas.drawString(buf, SCREEN_W - EDGE - dutyW, valY);

    // Wh — centered, slight nudge down; tight decimal (no fat '.')
    drawWhCentered(valY + 4, in->vescValid ? in->whUsed : 0.0f, whSize,
                   in->vescValid ? C_WH : C_DIM);
  }

  // ——— Warn ———
  canvas.setTextDatum(textdatum_t::top_center);
  canvas.setTextSize(2);
  const int warnY = 268;
  if (in->fault) {
    canvas.setTextColor(C_RED);
    snprintf(buf, sizeof(buf), "FAULT %u", (unsigned)in->fault);
    canvas.drawString(buf, 120, warnY);
  } else if (vw == VOLT_CUT) {
    canvas.setTextColor(C_RED);
    canvas.drawString("切断", 120, warnY);
  } else if (vw == VOLT_WARN) {
    canvas.setTextColor(C_YELLOW);
    canvas.drawString("低压", 120, warnY);
  } else if (!in->vescLinkOk) {
    canvas.setTextColor(C_RED);
    canvas.drawString("断联", 120, warnY);
  }

  drawPageDots(in->dataPage);
  drawThrottleRow(in);
  canvas.pushSprite(0, 0);
}
