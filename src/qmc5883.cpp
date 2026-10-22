#include "qmc5883.h"

#include "pins.h"

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

// M10G-5883 magnetometer on I2C. Pins are GPIO17/GPIO18; SDA vs SCL is probed.
// SDA vs SCL is what we probe. Three chips show up on these GPS+compass boards.
static const uint8_t kAddrL = 0x0D;  // QMC5883L
static const uint8_t kAddrP = 0x2C;  // QMC5883P
static const uint8_t kAddrH = 0x1E;  // HMC5883L / QMC5883 clones

enum ChipKind : uint8_t { CHIP_NONE = 0, CHIP_L = 1, CHIP_P = 2, CHIP_H = 3 };

static MagStatus st = {};
static ChipKind chip = CHIP_NONE;
static Preferences prefs;
static bool nvsSaved = false;
static uint32_t lastReadMs = 0;
static uint32_t lastScanMs = 0;
static uint8_t scanMap = 0;
static int16_t xmin = 32767, xmax = -32768;
static int16_t ymin = 32767, ymax = -32768;
static uint16_t yawOff = 0;  // MAG ROT 0/90/180/270 — default 0 after Y-axis fix

static const char *chipName(ChipKind k) {
  switch (k) {
    case CHIP_L:
      return "QMC5883L";
    case CHIP_P:
      return "QMC5883P";
    case CHIP_H:
      return "HMC5883L";
    default:
      return "-";
  }
}

static void logLine(const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
}

static void busBegin(int sda, int scl) {
  Wire.end();
  delay(5);
  Wire.begin(sda, scl);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  gpio_set_pull_mode((gpio_num_t)sda, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)scl, GPIO_PULLUP_ONLY);
  delay(15);
}

static bool i2cAck(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static int scanBus() {
  int n = 0;
  char line[96];
  size_t pos = 0;
  line[0] = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (!i2cAck(a)) {
      continue;
    }
    n++;
    if (pos + 5 < sizeof(line)) {
      pos += (size_t)snprintf(line + pos, sizeof(line) - pos, "%s%02X", pos ? "," : "",
                              (unsigned)a);
    }
  }
  if (n == 0) {
    logLine("# MAG bus empty\n");
  } else {
    logLine("# MAG bus ACK [%s]\n", line);
  }
  return n;
}

static bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  const uint8_t got = (uint8_t)Wire.requestFrom((int)addr, (int)n);
  if (got != n) {
    return false;
  }
  for (uint8_t i = 0; i < n; i++) {
    dst[i] = (uint8_t)Wire.read();
  }
  return true;
}

static bool cfgL(uint8_t addr) {
  // SET/RESET period must be programmed or the L part never DRDY's.
  if (!i2cWrite(addr, 0x0B, 0x01)) {
    return false;
  }
  if (!i2cWrite(addr, 0x0A, 0x00)) {
    return false;
  }
  // OSR=512, RNG=2G, ODR=50Hz, continuous.
  return i2cWrite(addr, 0x09, 0x09);
}

static bool cfgP(uint8_t addr) {
  if (!i2cWrite(addr, 0x29, 0x06)) {
    return false;
  }
  if (!i2cWrite(addr, 0x0B, 0x08)) {
    return false;
  }
  return i2cWrite(addr, 0x0A, 0xCF);
}

static bool cfgH(uint8_t addr) {
  if (!i2cWrite(addr, 0x00, 0x70)) {
    return false;
  }
  if (!i2cWrite(addr, 0x01, 0x20)) {
    return false;
  }
  return i2cWrite(addr, 0x02, 0x00);
}

static bool readL(uint8_t addr, int16_t *x, int16_t *y, int16_t *z) {
  uint8_t d[6];
  if (!i2cRead(addr, 0x00, d, 6)) {
    return false;
  }
  *x = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));
  *y = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8));
  *z = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8));
  // Clones often leave DRDY stuck; reject only the "bus pulled high" pattern.
  return !(*x == -1 && *y == -1 && *z == -1);
}

static bool readP(uint8_t addr, int16_t *x, int16_t *y, int16_t *z) {
  uint8_t d[6];
  if (!i2cRead(addr, 0x01, d, 6)) {
    return false;
  }
  *x = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));
  *y = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8));
  *z = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8));
  return *x != 0 || *y != 0 || *z != 0;
}

static bool readH(uint8_t addr, int16_t *x, int16_t *y, int16_t *z) {
  uint8_t d[6];
  if (!i2cRead(addr, 0x03, d, 6)) {
    return false;
  }
  *x = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
  *z = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
  *y = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
  return true;
}

static bool tryChip(uint8_t addr, ChipKind kind) {
  if (!i2cAck(addr)) {
    return false;
  }
  bool cfgOk = false;
  if (kind == CHIP_L) {
    cfgOk = cfgL(addr);
  } else if (kind == CHIP_P) {
    cfgOk = cfgP(addr);
  } else {
    cfgOk = cfgH(addr);
  }
  if (!cfgOk) {
    logLine("# MAG ack 0x%02X cfg fail\n", (unsigned)addr);
    return false;
  }
  delay(30);
  int16_t x = 0, y = 0, z = 0;
  bool got = false;
  for (int i = 0; i < 8 && !got; i++) {
    delay(20);
    if (kind == CHIP_L) {
      got = readL(addr, &x, &y, &z);
    } else if (kind == CHIP_P) {
      got = readP(addr, &x, &y, &z);
    } else {
      got = readH(addr, &x, &y, &z);
    }
  }
  if (!got) {
    logLine("# MAG ack 0x%02X no data\n", (unsigned)addr);
    return false;
  }
  chip = kind;
  st.ok = true;
  st.addr = addr;
  st.chip = chipName(kind);
  st.phase = "ok";
  st.x = x;
  st.y = y;
  st.z = z;
  return true;
}

static void saveMap(int sda, int scl, uint8_t addr);

static bool probeMap(int sda, int scl) {
  logLine("# MAG probe SDA=GPIO%d SCL=GPIO%d\n", sda, scl);
  busBegin(sda, scl);
  st.sda = (int8_t)sda;
  st.scl = (int8_t)scl;
  if (scanBus() == 0) {
    logLine("# MAG none on SDA=%d SCL=%d\n", sda, scl);
    return false;
  }
  if (tryChip(kAddrL, CHIP_L) || tryChip(kAddrP, CHIP_P) || tryChip(kAddrH, CHIP_H)) {
    logLine("# MAG %s @0x%02X SDA=%d SCL=%d x=%d y=%d z=%d\n", st.chip, (unsigned)st.addr, sda,
            scl, (int)st.x, (int)st.y, (int)st.z);
    return true;
  }
  logLine("# MAG ACK but unknown chip on SDA=%d SCL=%d\n", sda, scl);
  return false;
}

static bool probeAllMaps() {
  const int maps[][2] = {{MAG_PIN_A, MAG_PIN_B}, {MAG_PIN_B, MAG_PIN_A}};
  for (unsigned i = 0; i < sizeof(maps) / sizeof(maps[0]); i++) {
    if (probeMap(maps[i][0], maps[i][1])) {
      saveMap(maps[i][0], maps[i][1], st.addr);
      return true;
    }
  }
  return false;
}

static void saveMap(int sda, int scl, uint8_t addr) {
  if (nvsSaved) {
    return;
  }
  if (!prefs.begin("mag", false)) {
    return;
  }
  prefs.putChar("sda", (int8_t)sda);
  prefs.putChar("scl", (int8_t)scl);
  prefs.putUChar("addr", addr);
  prefs.putUShort("yaw", yawOff);
  prefs.end();
  nvsSaved = true;
  logLine("# MAG NVS save SDA=%d SCL=%d addr=0x%02X\n", sda, scl, (unsigned)addr);
}

static bool loadMap(int *sda, int *scl) {
  *sda = MAG_PIN_A;
  *scl = MAG_PIN_B;
  if (!prefs.begin("mag", true)) {
    return false;
  }
  const bool have = prefs.isKey("sda") && prefs.isKey("scl");
  if (have) {
    *sda = (int)prefs.getChar("sda", (int8_t)MAG_PIN_A);
    *scl = (int)prefs.getChar("scl", (int8_t)MAG_PIN_B);
  }
  const bool yawMigrated = prefs.isKey("yawv");
  yawOff = prefs.getUShort("yaw", 0);
  prefs.end();
  if (!yawMigrated) {
    // Previous firmware defaulted MAG ROT to 180. That offset was wrong once
    // Y was flipped; start at 0. User can still pick 180 after this.
    yawOff = 0;
    if (prefs.begin("mag", false)) {
      prefs.putUShort("yaw", 0);
      prefs.putUChar("yawv", 1);
      prefs.end();
    }
  } else if (yawOff != 0 && yawOff != 90 && yawOff != 180 && yawOff != 270) {
    yawOff = 0;
  }
  const bool onMagPins =
      (*sda == MAG_PIN_A || *sda == MAG_PIN_B) && (*scl == MAG_PIN_A || *scl == MAG_PIN_B) &&
      *sda != *scl;
  return have && onMagPins;
}

static void updateHeading(int16_t x, int16_t y) {
  if (x < xmin) {
    xmin = x;
  }
  if (x > xmax) {
    xmax = x;
  }
  if (y < ymin) {
    ymin = y;
  }
  if (y > ymax) {
    ymax = y;
  }
  float cx = 0.0f;
  float cy = 0.0f;
  const int rx = (int)xmax - (int)xmin;
  const int ry = (int)ymax - (int)ymin;
  if (rx > 200 && ry > 200) {
    cx = 0.5f * ((float)xmax + (float)xmin);
    cy = 0.5f * ((float)ymax + (float)ymin);
  }
  // Phone compass: clockwise increases heading. This QMC5883's Y is opposite
  // that convention, so negate Y. MAG ROT is an extra 0/90/180/270 mount offset.
  float deg = atan2f(-((float)y - cy), (float)x - cx) * 57.2957795f;
  if (deg < 0.0f) {
    deg += 360.0f;
  }
  deg += (float)yawOff;
  while (deg >= 360.0f) {
    deg -= 360.0f;
  }
  st.headingDeg = deg;
  st.yawDeg = yawOff;
}

static bool readOnce() {
  int16_t x = 0, y = 0, z = 0;
  bool ok = false;
  if (chip == CHIP_L) {
    ok = readL(st.addr, &x, &y, &z);
  } else if (chip == CHIP_P) {
    ok = readP(st.addr, &x, &y, &z);
  } else if (chip == CHIP_H) {
    ok = readH(st.addr, &x, &y, &z);
  }
  if (!ok) {
    st.fails++;
    return false;
  }
  st.x = x;
  st.y = y;
  st.z = z;
  st.reads++;
  st.fails = 0;
  updateHeading(x, y);
  return true;
}

void magInit() {
  st = {};
  st.chip = "-";
  st.phase = "scan";
  st.sda = MAG_PIN_A;
  st.scl = MAG_PIN_B;
  chip = CHIP_NONE;
  nvsSaved = false;
  xmin = 32767;
  xmax = -32768;
  ymin = 32767;
  ymax = -32768;

  int sda = MAG_PIN_A;
  int scl = MAG_PIN_B;
  const bool remembered = loadMap(&sda, &scl);
  st.yawDeg = yawOff;
  if (remembered && probeMap(sda, scl)) {
    saveMap(sda, scl, st.addr);
    return;
  }
  if (probeAllMaps()) {
    return;
  }
  st.ok = false;
  st.phase = "none";
  st.chip = "-";
  lastScanMs = millis();
  scanMap = 0;
  logLine("# MAG not found — wire SDA/SCL to GPIO17/GPIO18 (not GPIO3)\n");
}

void magLoop() {
  if (st.ok) {
    const uint32_t now = millis();
    if (now - lastReadMs < 50) {
      return;
    }
    lastReadMs = now;
    if (!readOnce()) {
      if (st.fails >= 20) {
        logLine("# MAG lost, rescan\n");
        st.ok = false;
        st.phase = "scan";
        chip = CHIP_NONE;
        lastScanMs = now;
      }
    }
    return;
  }

  const uint32_t now = millis();
  if (now - lastScanMs < 2500) {
    return;
  }
  lastScanMs = now;
  st.phase = "scan";
  if (probeAllMaps()) {
    return;
  }
  st.phase = "none";
}

const MagStatus *magStatus() {
  return &st;
}

void magCycleYaw() {
  yawOff = (uint16_t)((yawOff + 90u) % 360u);
  if (prefs.begin("mag", false)) {
    prefs.putUShort("yaw", yawOff);
    prefs.putUChar("yawv", 1);
    prefs.end();
  }
  st.yawDeg = yawOff;
  if (chip != CHIP_NONE) {
    updateHeading(st.x, st.y);
  }
  logLine("# MAG yaw %u\n", (unsigned)yawOff);
}

uint16_t magYawDeg() {
  return yawOff;
}

size_t magDiagJson(char *buf, size_t bufLen) {
  if (buf == nullptr || bufLen < 64) {
    return 0;
  }
  return (size_t)snprintf(
      buf, bufLen,
      "{\"ok\":%s,\"chip\":\"%s\",\"phase\":\"%s\",\"addr\":%u,\"sda\":%d,\"scl\":%d,"
      "\"x\":%d,\"y\":%d,\"z\":%d,\"head\":%.1f,\"yaw\":%u,\"reads\":%lu,\"fails\":%lu}",
      st.ok ? "true" : "false", st.chip ? st.chip : "-", st.phase ? st.phase : "-",
      (unsigned)st.addr, (int)st.sda, (int)st.scl, (int)st.x, (int)st.y, (int)st.z,
      (double)st.headingDeg, (unsigned)st.yawDeg, (unsigned long)st.reads,
      (unsigned long)st.fails);
}
