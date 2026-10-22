#include "gps_nmea.h"

#include "pins.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

// M10G-5883: factory NMEA @ 38400 even without fix.
// UART2 = VESC. UART0 default pins = CH343 debug. GPS = UART1.

static HardwareSerial GpsSerial(1);
static Preferences gpsPrefs;
static GpsFix fix = {};
static uint32_t lastByteMs = 0;
static uint32_t lastFramedMs = 0;
static uint32_t lastSentenceMs = 0;
static uint32_t lastUbxMs = 0;
static uint32_t lastPvtMs = 0;
static uint32_t lastProbeMs = 0;
static uint32_t lastDiagMs = 0;
static uint32_t lastSpeedMs = 0;
static uint8_t prevRx = 0;
static uint32_t activeBaud = GPS_UART_BAUD;
static bool activeInvert = false;
static int8_t activeRxPin = GPS_UART_RX;
static int8_t activeTxPin = GPS_UART_TX;
static uint32_t uartOpenMs = 0;
static bool baudLocked = false;

// Auto scan is a fallback for an unknown module, not a boot ritual.
// Factory M10 talks 38400; a flight-controller flash may have left 230400/115200.
static const uint32_t kScanBauds[] = {38400u, 230400u, 115200u, 57600u, 9600u,
                                     19200u,  460800u, 4800u};
static const uint8_t kScanBaudCount = sizeof(kScanBauds) / sizeof(kScanBauds[0]);
static uint32_t scanOrder[sizeof(kScanBauds) / sizeof(kScanBauds[0])];
static uint8_t scanOrderCount = 0;
static uint8_t scanIndex = 0;  // 0..(orderCount*2-1); first pass normal, second invert
static uint32_t scanSlotStartMs = 0;
static uint32_t listenStartMs = 0;
static uint32_t scanSlotBytes = 0;
static uint16_t scanSlotDollar = 0;
static uint16_t scanSlotUbx = 0;
static uint16_t scanSlotTalker = 0;
static uint16_t scanSlotPrint = 0;
static int32_t bestScore = -1;
static uint32_t bestBaud = GPS_UART_BAUD;
static bool bestInvert = false;
static bool scanDone = false;
static bool baudSaved = false;
// 1 Hz NMEA arrives in a burst; the slot must outlast one silent gap plus the burst.
static const uint32_t kScanSlotMs = 1100;
static const uint32_t kListenGiveUpMs = 2000;

static void glog(const char *fmt, ...) {
  char buf[220];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
}

static char lineBuf[160];
static size_t lineLen = 0;
static uint8_t peekPos = 0;

static uint32_t windowStartMs = 0;
static uint32_t windowBytes = 0;
static uint32_t windowNmea = 0;

static uint8_t hexRing[48];
static uint8_t hexCount = 0;
static uint16_t dollarWindow = 0;
static uint16_t ubxWindow = 0;
static uint16_t printWindow = 0;

static void noteHexSample(uint8_t b) {
  if (hexCount < sizeof(hexRing)) {
    hexRing[hexCount++] = b;
  }
  if (b == '$') {
    dollarWindow++;
  }
  if (b >= 32 && b < 127) {
    printWindow++;
  }
}

static void dumpHexSample() {
  size_t pos = 0;
  fix.hexSample[0] = 0;
  for (uint8_t i = 0; i < hexCount && pos + 3 < sizeof(fix.hexSample); i++) {
    if (i) {
      fix.hexSample[pos++] = ' ';
    }
    pos += (size_t)snprintf(fix.hexSample + pos, sizeof(fix.hexSample) - pos, "%02X", hexRing[i]);
  }

  glog("# GPS HEX %s | $=%u ubx=%u ascii=%u baud=%lu inv=%d\n", fix.hexSample,
       (unsigned)dollarWindow, (unsigned)ubxWindow, (unsigned)printWindow,
       (unsigned long)activeBaud, (int)activeInvert);
  fix.dollarCount = dollarWindow;
  fix.ubxSyncCount = ubxWindow;
  hexCount = 0;
  dollarWindow = 0;
  ubxWindow = 0;
  printWindow = 0;
}

// UBX frame parser
enum UbxState : uint8_t { U_SYNC1, U_SYNC2, U_CLASS, U_ID, U_LEN1, U_LEN2, U_PAYLOAD, U_CKA, U_CKB };
static UbxState ubxSt = U_SYNC1;
static uint8_t ubxCls = 0, ubxId = 0;
static uint16_t ubxLen = 0, ubxIdx = 0;
static uint8_t ubxPay[128];
static uint8_t ubxCka = 0, ubxCkb = 0, ubxCkaRx = 0;

static void ubxChecksum(const uint8_t *body, size_t len, uint8_t *cka, uint8_t *ckb) {
  uint8_t a = 0, b = 0;
  for (size_t i = 0; i < len; i++) {
    a = (uint8_t)(a + body[i]);
    b = (uint8_t)(b + a);
  }
  *cka = a;
  *ckb = b;
}

static void sendUbx(const uint8_t *body, size_t bodyLen) {
  uint8_t cka, ckb;
  ubxChecksum(body, bodyLen, &cka, &ckb);
  GpsSerial.write((uint8_t)0xB5);
  GpsSerial.write((uint8_t)0x62);
  GpsSerial.write(body, bodyLen);
  GpsSerial.write(cka);
  GpsSerial.write(ckb);
}

static void sendUbxNavPvtPoll() {
  const uint8_t body[] = {0x01, 0x07, 0x00, 0x00};  // NAV-PVT poll
  sendUbx(body, sizeof(body));
}

static void sendUbxEnablePvtRate() {
  // Legacy CFG-MSG: NAV-PVT on UART1 rate 1
  const uint8_t body[] = {0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
  sendUbx(body, sizeof(body));
}

static void sendUbxEnableNmeaLegacy() {
  const uint8_t gga[] = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
  const uint8_t rmc[] = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
  const uint8_t vtg[] = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
  sendUbx(gga, sizeof(gga));
  sendUbx(rmc, sizeof(rmc));
  sendUbx(vtg, sizeof(vtg));
}

/** M10 CFG-VALSET: enable NMEA out + GGA/RMC UART1 + NAV-PVT UART1 (RAM layer). */
static void sendUbxM10ValSet() {
  // Keys (u-blox M10):
  // CFG-UART1OUTPROT-NMEA 0x10740002 = 1
  // CFG-MSGOUT-NMEA_ID_GGA_UART1 0x209100bb = 1
  // CFG-MSGOUT-NMEA_ID_RMC_UART1 0x209100ac = 1
  // CFG-MSGOUT-UBX_NAV_PVT_UART1 0x20910007 = 1
  auto appendKV_L = [](uint8_t *dst, size_t *n, uint32_t key, uint8_t v) {
    dst[(*n)++] = (uint8_t)(key & 0xFF);
    dst[(*n)++] = (uint8_t)((key >> 8) & 0xFF);
    dst[(*n)++] = (uint8_t)((key >> 16) & 0xFF);
    dst[(*n)++] = (uint8_t)((key >> 24) & 0xFF);
    dst[(*n)++] = v;
  };
  auto appendKV_U1 = appendKV_L;

  uint8_t body[64];
  size_t n = 0;
  body[n++] = 0x06;
  body[n++] = 0x8A;  // CFG-VALSET
  body[n++] = 0;     // len lo placeholder
  body[n++] = 0;
  body[n++] = 0x00;  // version
  body[n++] = 0x01;  // layers = RAM
  body[n++] = 0x00;
  body[n++] = 0x00;
  appendKV_L(body, &n, 0x10740002u, 1);  // NMEA out
  appendKV_U1(body, &n, 0x209100bbu, 1); // GGA
  appendKV_U1(body, &n, 0x209100acu, 1); // RMC
  appendKV_U1(body, &n, 0x209100b1u, 1); // VTG
  appendKV_U1(body, &n, 0x20910007u, 1); // NAV-PVT
  const uint16_t payLen = (uint16_t)(n - 4);
  body[2] = (uint8_t)(payLen & 0xFF);
  body[3] = (uint8_t)(payLen >> 8);
  sendUbx(body, n);
}

/** Motion/navigation tuning, kept in its own VALSET because VALSET is atomic and a rejected
 *  key here must not drag the working NMEA/PVT setup down with it.
 *
 *  CFG-NAVSPG-DYNMODEL  automotive, so the receiver's velocity filter assumes ground-vehicle
 *                       dynamics rather than the default "portable".
 *  CFG-MOT-GNSSSPEED_THRS  static hold. Below this speed the receiver freezes position and
 *                       reports velocity as exactly zero until it sees evidence of movement.
 *                       This is what makes a flight controller read a clean 0 at a standstill.
 *  CFG-MOT-GNSSDIST_THRS   how far the position must drift before static hold is released. */
static void sendUbxM10Motion() {
  uint8_t body[32];
  size_t n = 0;
  body[n++] = 0x06;
  body[n++] = 0x8A;  // CFG-VALSET
  body[n++] = 0;     // len placeholder
  body[n++] = 0;
  body[n++] = 0x00;  // version
  body[n++] = 0x01;  // layers = RAM
  body[n++] = 0x00;
  body[n++] = 0x00;
  auto key = [&](uint32_t k) {
    body[n++] = (uint8_t)(k & 0xFF);
    body[n++] = (uint8_t)((k >> 8) & 0xFF);
    body[n++] = (uint8_t)((k >> 16) & 0xFF);
    body[n++] = (uint8_t)((k >> 24) & 0xFF);
  };
  key(0x20110021u);  // DYNMODEL, E1
  body[n++] = 4;     // automotive
  key(0x20250038u);  // GNSSSPEED_THRS, U1, cm/s
  body[n++] = 60;    // 0.60 m/s = 2.2 km/h
  key(0x3025003bu);  // GNSSDIST_THRS, U2, m
  body[n++] = 5;
  body[n++] = 0;
  const uint16_t payLen = (uint16_t)(n - 4);
  body[2] = (uint8_t)(payLen & 0xFF);
  body[3] = (uint8_t)(payLen >> 8);
  sendUbx(body, n);
}

static int32_t rdI4(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
                   ((uint32_t)p[3] << 24));
}

static uint32_t rdU4(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint32_t lastGoodSpeedMs = 0;
static uint32_t lastPvtSpeedMs = 0;
static float kalmanX = 0.0f;  // filtered speed, km/h
static float kalmanP = 4.0f;  // its variance, (km/h)^2
static bool kalmanInit = false;
static uint32_t lastFiltMs = 0;
static uint8_t spikeRejects = 0;
static bool movingState = false;
static uint8_t moveVotes = 0;
static uint8_t stopVotes = 0;
static bool gpsConfiguredOnce = false;

static const uint32_t kSpeedStaleMs = 3000;  // no sample at all -> drop the held number
// NAV-PVT is the receiver's Doppler solution; the NMEA copies lag it and disagree.
static const uint32_t kPvtPreferMs = 2500;
// 2 m/s: above this the receiver is telling us its own speed is guesswork.
static const float kSpeedAccLimitKmh = 7.2f;
static const uint8_t kMaxSpikeRejects = 3;  // never stonewall a genuine jump

// Standing still, Doppler speed still reads 0.4..1.8 km/h — that is the sensor's own noise
// floor, not motion. A single threshold lets spikes through and chatters when the reading
// sits on it, so moving/stopped is a latched state with hysteresis and vote confirmation.
static const float kEnterMovingKmh = 3.0f;  // and never below 2 sigma of the reported error
static const float kExitMovingKmh = 1.5f;
static const uint8_t kEnterVotes = 2;  // consecutive samples, ~1 s each
static const uint8_t kExitVotes = 3;
static const float kNoiseSigmaMult = 2.0f;
static const float kAccFloorKmh = 1.2f;  // assumed sigma when a source reports none (NMEA)
// Kalman tuning. kAccelVarPerS is how much speed may drift in a second without being told;
// kMeasVarFloor stops a flattering sAcc from disabling the filter. Raise the floor or lower
// the accel variance for a calmer needle, do the opposite for a snappier one.
static const float kAccelVarPerS = 1.0f;   // (km/h)^2 added per second of prediction
static const float kMeasVarFloor = 4.0f;   // (km/h)^2, i.e. never trust better than 2 km/h
static const float kInnovGateSigma = 4.0f;

/** Feed one speed sample through source arbitration, a noise deadband and a Kalman filter. */
static void applySpeedKmh(float kmh, uint32_t src, float accKmh) {
  if (!(kmh >= 0.0f) || kmh > 400.0f) {
    return;
  }
  const uint32_t ts = millis();

  // Three sources writing one field made the display hop between disagreeing estimates.
  // Let NMEA in only while NAV-PVT is silent.
  if (src == 3) {
    lastPvtSpeedMs = ts;
  } else if (lastPvtSpeedMs != 0 && (ts - lastPvtSpeedMs) < kPvtPreferMs) {
    return;
  }

  fix.rawSpeedKmh = kmh;
  fix.speedAccKmh = accKmh;
  fix.speedSrc = src;
  fix.speedValid = true;
  lastSpeedMs = ts;

  if (accKmh > 0.0f && accKmh > kSpeedAccLimitKmh) {
    fix.speedRejects++;
    return;
  }

  const float sigma = (accKmh > 0.0f) ? accKmh : kAccFloorKmh;
  const float r = fmaxf(sigma * sigma, kMeasVarFloor);

  const bool wasMoving = movingState;
  if (!movingState) {
    if (kmh >= fmaxf(kEnterMovingKmh, kNoiseSigmaMult * sigma)) {
      if (++moveVotes >= kEnterVotes) {
        movingState = true;
        stopVotes = 0;
      }
    } else {
      moveVotes = 0;
    }
  } else {
    if (kmh < kExitMovingKmh) {
      if (++stopVotes >= kExitVotes) {
        movingState = false;
        moveVotes = 0;
      }
    } else {
      stopVotes = 0;
    }
  }

  const float z = movingState ? kmh : 0.0f;

  if (!kalmanInit || (!wasMoving && movingState)) {
    // Pulling away from a stop: snap to the reading instead of slewing up from zero.
    kalmanX = z;
    kalmanP = r;
    kalmanInit = true;
  } else {
    const float dtS = (float)(ts - lastFiltMs) * 0.001f;
    kalmanP += kAccelVarPerS * dtS;
    const float innov = z - kalmanX;
    if (fabsf(innov) > kInnovGateSigma * sqrtf(kalmanP + r) && spikeRejects < kMaxSpikeRejects) {
      spikeRejects++;
      fix.speedRejects++;
      lastFiltMs = ts;
      return;
    }
    spikeRejects = 0;
    const float k = kalmanP / (kalmanP + r);
    kalmanX += k * innov;
    kalmanP *= (1.0f - k);
  }
  lastFiltMs = ts;

  if (kalmanX < 0.0f) {
    kalmanX = 0.0f;
  }
  fix.speedKmh = movingState ? kalmanX : 0.0f;
  if (fix.speedKmh > 0.0f) {
    lastGoodSpeedMs = ts;
  }
}

static void handleNavPvt(const uint8_t *p, uint16_t len) {
  // NAV-PVT length is 92 bytes on M10
  if (len < 92) {
    return;
  }
  lastUbxMs = millis();
  lastPvtMs = millis();
  lastSentenceMs = lastPvtMs;
  strncpy(fix.lastTalker, "NAVPVT", sizeof(fix.lastTalker) - 1);

  const uint8_t valid = p[11];
  const uint8_t fixType = p[20];
  const uint8_t flags = p[21];
  const uint8_t numSV = p[23];
  // Dummy PVT after boot (year 2021, validTime=0) used to overwrite a good NMEA clock.
  if ((valid & 0x02) != 0) {
    fix.hour = p[8];
    fix.minute = p[9];
    fix.second = p[10];
    fix.timeValid = true;
  }
  int32_t gSpeed = rdI4(p + 60);        // mm/s, receiver's 2D ground speed
  const uint32_t sAcc = rdU4(p + 68);   // mm/s, its own accuracy estimate
  if (gSpeed < 0) {
    gSpeed = -gSpeed;
  }

  fix.satellites = numSV;
  fix.year = (uint16_t)(p[4] | ((uint16_t)p[5] << 8));
  fix.month = p[6];
  fix.day = p[7];
  fix.dateValid = (valid & 0x01) != 0 && fix.year >= 2000 && fix.month >= 1 && fix.month <= 12 &&
                  fix.day >= 1 && fix.day <= 31;
  // 2D/3D is enough; gnssFixOK can flap on M10
  fix.valid = (fixType >= 2) || ((flags & 0x01) != 0 && numSV >= 4);

  // A 2D fix has no usable velocity solution; taking it is what let walking read 30 km/h.
  if (fixType >= 3) {
    applySpeedKmh((float)gSpeed * 0.0036f, 3, (float)sAcc * 0.0036f);
  }

  if (fixType >= 2) {
    fix.lonE7 = rdI4(p + 24);
    fix.latE7 = rdI4(p + 28);
    fix.altMmMsl = rdI4(p + 36);
    fix.accMm = rdU4(p + 40);
    fix.posValid = true;
    // headMot is derived from velocity, so it is only a real bearing once actually moving.
    const uint32_t headAcc = rdU4(p + 72);
    fix.headDeg = (float)rdI4(p + 64) * 1e-5f;
    fix.headAccDeg = (float)headAcc * 1e-5f;
    fix.headValid = movingState && fix.headAccDeg < 45.0f;
  } else {
    fix.posValid = false;
    fix.headValid = false;
  }
  fix.totalNmea++;
  windowNmea++;
}

static void ubxFeed(uint8_t b) {
  switch (ubxSt) {
  case U_SYNC1:
    if (b == 0xB5) {
      ubxSt = U_SYNC2;
    }
    break;
  case U_SYNC2:
    if (b == 0x62) {
      lastFramedMs = millis();
      ubxSt = U_CLASS;
    } else {
      ubxSt = U_SYNC1;
    }
    break;
  case U_CLASS:
    ubxCls = b;
    ubxCka = b;
    ubxCkb = b;
    ubxSt = U_ID;
    break;
  case U_ID:
    ubxId = b;
    ubxCka = (uint8_t)(ubxCka + b);
    ubxCkb = (uint8_t)(ubxCkb + ubxCka);
    ubxSt = U_LEN1;
    break;
  case U_LEN1:
    ubxLen = b;
    ubxCka = (uint8_t)(ubxCka + b);
    ubxCkb = (uint8_t)(ubxCkb + ubxCka);
    ubxSt = U_LEN2;
    break;
  case U_LEN2:
    ubxLen |= (uint16_t)b << 8;
    ubxCka = (uint8_t)(ubxCka + b);
    ubxCkb = (uint8_t)(ubxCkb + ubxCka);
    ubxIdx = 0;
    if (ubxLen > sizeof(ubxPay)) {
      ubxSt = U_SYNC1;  // skip oversized
    } else if (ubxLen == 0) {
      ubxSt = U_CKA;
    } else {
      ubxSt = U_PAYLOAD;
    }
    break;
  case U_PAYLOAD:
    ubxPay[ubxIdx++] = b;
    ubxCka = (uint8_t)(ubxCka + b);
    ubxCkb = (uint8_t)(ubxCkb + ubxCka);
    if (ubxIdx >= ubxLen) {
      ubxSt = U_CKA;
    }
    break;
  case U_CKA:
    ubxCkaRx = b;
    ubxSt = U_CKB;
    break;
  case U_CKB:
    if (ubxCkaRx == ubxCka && b == ubxCkb) {
      lastUbxMs = millis();
      if (ubxCls == 0x01 && ubxId == 0x07) {
        handleNavPvt(ubxPay, ubxLen);
      } else if (ubxCls == 0x05 && ubxLen >= 2) {
        // ACK/NAK of a config message. Worth surfacing: a NAKed VALSET means the receiver
        // never enabled static hold and the speed we show is software-filtered only.
        Serial.printf("# GPS %s cls=%02X id=%02X\n", (ubxId == 0x01) ? "ACK" : "NAK", ubxPay[0],
                      ubxPay[1]);
      }
    }
    ubxSt = U_SYNC1;
    break;
  }
}

static bool parseUtcTime(const char *hhmmss, uint8_t *h, uint8_t *m, uint8_t *s) {
  if (hhmmss == nullptr || strlen(hhmmss) < 6) {
    return false;
  }
  char tmp[3] = {0, 0, 0};
  tmp[0] = hhmmss[0];
  tmp[1] = hhmmss[1];
  *h = (uint8_t)atoi(tmp);
  tmp[0] = hhmmss[2];
  tmp[1] = hhmmss[3];
  *m = (uint8_t)atoi(tmp);
  tmp[0] = hhmmss[4];
  tmp[1] = hhmmss[5];
  *s = (uint8_t)atoi(tmp);
  return *h < 24 && *m < 60 && *s < 60;
}

static int splitFields(char *s, char *fields[], int maxFields) {
  int n = 0;
  fields[n++] = s;
  for (char *p = s; *p && n < maxFields; p++) {
    if (*p == ',') {
      *p = 0;
      fields[n++] = p + 1;
    } else if (*p == '*') {
      *p = 0;
      break;
    }
  }
  return n;
}

static void notePeek(uint8_t b) {
  char c = (b >= 32 && b < 127) ? (char)b : '.';
  if (peekPos + 1 >= sizeof(fix.rawPeek)) {
    memmove(fix.rawPeek, fix.rawPeek + 1, sizeof(fix.rawPeek) - 2);
    peekPos = sizeof(fix.rawPeek) - 2;
  }
  fix.rawPeek[peekPos++] = c;
  fix.rawPeek[peekPos] = 0;
}

static bool nmeaChecksumOk(const char *s) {
  // "$....*HH"
  if (s[0] != '$') {
    return false;
  }
  const char *star = strrchr(s, '*');
  if (star == nullptr || star <= s + 1 || strlen(star) < 3) {
    return false;
  }
  uint8_t cs = 0;
  for (const char *p = s + 1; p < star; p++) {
    cs ^= (uint8_t)(*p);
  }
  char hex[3] = {star[1], star[2], 0};
  const int expect = (int)strtol(hex, nullptr, 16);
  return expect == (int)cs;
}

static bool nmeaTalkerOk(const char *s) {
  // $GPxxx / $GNxxx / $GBxxx / $GAxxx / $GQxxx / $GLxxx
  return s[0] == '$' && s[1] == 'G' &&
         (s[2] == 'P' || s[2] == 'N' || s[2] == 'B' || s[2] == 'A' || s[2] == 'Q' ||
          s[2] == 'L');
}

static void finishNmeaLine() {
  if (lineLen < 8 || lineBuf[0] != '$') {
    lineLen = 0;
    return;
  }
  lineBuf[lineLen] = 0;
  if (!nmeaTalkerOk(lineBuf) || !nmeaChecksumOk(lineBuf)) {
    lineLen = 0;
    return;
  }
  lastSentenceMs = millis();
  fix.totalNmea++;
  windowNmea++;

  char talk[8] = {};
  size_t i = 0;
  for (const char *p = lineBuf + 1; *p && *p != ',' && *p != '*' && i + 1 < sizeof(talk); p++) {
    talk[i++] = *p;
  }
  strncpy(fix.lastTalker, talk, sizeof(fix.lastTalker) - 1);

  char *fields[24];
  const int n = splitFields(lineBuf, fields, 24);
  if (n < 2) {
    lineLen = 0;
    return;
  }
  const bool isRmc = strstr(fields[0], "RMC") != nullptr;
  const bool isGga = strstr(fields[0], "GGA") != nullptr;
  const bool isVtg = strstr(fields[0], "VTG") != nullptr;
  const bool isGll = strstr(fields[0], "GLL") != nullptr;
  // M10 often emits GLL (with UTC) long before RMC/GGA. Status V is fine for the clock.
  if (isGll && n >= 6) {
    uint8_t h, m, s;
    if (parseUtcTime(fields[5], &h, &m, &s)) {
      fix.hour = h;
      fix.minute = m;
      fix.second = s;
      fix.timeValid = true;
    }
  }
  if (isRmc && n >= 8) {
    uint8_t h, m, s;
    if (parseUtcTime(fields[1], &h, &m, &s)) {
      fix.hour = h;
      fix.minute = m;
      fix.second = s;
      fix.timeValid = true;
    }
    const bool navOk = fields[2][0] == 'A';
    if (fields[7][0]) {
      const float knots = strtof(fields[7], nullptr);
      if (knots >= 0.0f && knots < 200.0f && navOk) {
        applySpeedKmh(knots * 1.852f, 1, 0.0f);
      }
    }
    if (navOk) {
      fix.valid = true;
    } else {
      fix.valid = false;
    }
    // RMC date field usually index 9: ddmmyy
    if (n >= 10 && fields[9][0] && strlen(fields[9]) >= 6) {
      const int dd = (fields[9][0] - '0') * 10 + (fields[9][1] - '0');
      const int mo = (fields[9][2] - '0') * 10 + (fields[9][3] - '0');
      const int yy = (fields[9][4] - '0') * 10 + (fields[9][5] - '0');
      if (dd >= 1 && dd <= 31 && mo >= 1 && mo <= 12) {
        fix.day = (uint8_t)dd;
        fix.month = (uint8_t)mo;
        fix.year = (uint16_t)(2000 + yy);
        fix.dateValid = true;
      }
    }
  }
  if (isGga && n >= 8) {
    uint8_t h, m, s;
    if (parseUtcTime(fields[1], &h, &m, &s)) {
      fix.hour = h;
      fix.minute = m;
      fix.second = s;
      fix.timeValid = true;
    }
    if (fields[7][0]) {
      const int sats = atoi(fields[7]);
      if (sats >= 0 && sats <= 64) {
        fix.satellites = (uint8_t)sats;
      }
    }
    if (fields[6][0] && fields[6][0] != '0') {
      fix.valid = true;
    }
  }
  // VTG: field7 = km/h (K), field5 = knots (N)
  if (isVtg && n >= 8) {
    float kmh = -1.0f;
    if (n >= 9 && fields[7][0] && (fields[8][0] == 'K' || fields[8][0] == 'k')) {
      kmh = strtof(fields[7], nullptr);
    } else if (n >= 7 && fields[5][0] && (fields[6][0] == 'N' || fields[6][0] == 'n')) {
      kmh = strtof(fields[5], nullptr) * 1.852f;
    }
    if (kmh >= 0.0f && kmh < 200.0f) {
      applySpeedKmh(kmh, 2, 0.0f);
    }
  }
  lineLen = 0;
}

static void reopenGpsUart(uint32_t baud, int8_t rxPin, int8_t txPin, bool invert) {
  GpsSerial.end();
  delay(20);
  gpio_reset_pin((gpio_num_t)rxPin);
  gpio_reset_pin((gpio_num_t)txPin);
  delay(5);
  // Do NOT pinMode() UART pins after begin — detaches peripheral on ESP32.
  // 230400 baud = 23 KB/s; 2048 bytes overflowed within one slow UI frame and chopped
  // sentences into checksum failures.
  GpsSerial.setRxBufferSize(4096);
  GpsSerial.setTxBufferSize(256);
  GpsSerial.begin(baud, SERIAL_8N1, rxPin, txPin, invert);
  // Weak pull-up on RX keeps idle high without pinMode() remapping.
  gpio_set_pull_mode((gpio_num_t)rxPin, GPIO_PULLUP_ONLY);
  activeBaud = baud;
  activeInvert = invert;
  activeRxPin = rxPin;
  activeTxPin = txPin;
  fix.baud = baud;
  fix.inverted = invert;
  fix.rxPin = rxPin;
  fix.txPin = txPin;
  lineLen = 0;
  ubxSt = U_SYNC1;
  prevRx = 0;
  uartOpenMs = millis();
  // Do not flush RX — the first GLL often already has UTC, and setup() used to
  // throw those sentences away then sit in TFT/WiFi for many seconds.
  glog("# GPS UART1 RX=GPIO%d TX=GPIO%d @ %lu inv=%d\n", (int)rxPin, (int)txPin,
       (unsigned long)baud, (int)invert);
}

static int32_t scoreSlot(uint32_t bytes, uint16_t dollars, uint16_t ubx, uint16_t printable) {
  // Prefer real protocol sync; printable alone is weak (noise can look printable).
  int32_t s = (int32_t)dollars * 100 + (int32_t)ubx * 200;
  if (bytes >= 20 && printable * 2 >= bytes) {
    s += (int32_t)(printable / 4);
  }
  if (bytes > 0 && dollars == 0 && ubx == 0) {
    s -= 5;  // garbage traffic
  }
  return s;
}

static void loadBaudPrefs(uint32_t *baud, bool *invert) {
  *baud = GPS_UART_BAUD;
  *invert = false;
  if (!gpsPrefs.begin("gps", true)) {
    return;
  }
  *baud = gpsPrefs.getULong("baud", GPS_UART_BAUD);
  *invert = gpsPrefs.getBool("inv", false);
  gpsPrefs.end();
}

static void saveBaudPrefs(uint32_t baud, bool invert) {
  if (baudSaved) {
    return;
  }
  if (!gpsPrefs.begin("gps", false)) {
    return;
  }
  gpsPrefs.putULong("baud", baud);
  gpsPrefs.putBool("inv", invert);
  gpsPrefs.end();
  baudSaved = true;
  glog("# GPS NVS save baud=%lu inv=%d\n", (unsigned long)baud, (int)invert);
}

static void buildScanOrder(uint32_t preferBaud) {
  scanOrderCount = 0;
  auto pushUnique = [&](uint32_t b) {
    for (uint8_t i = 0; i < scanOrderCount; i++) {
      if (scanOrder[i] == b) {
        return;
      }
    }
    if (scanOrderCount < kScanBaudCount) {
      scanOrder[scanOrderCount++] = b;
    }
  };
  pushUnique(preferBaud);
  for (uint8_t i = 0; i < kScanBaudCount; i++) {
    pushUnique(kScanBauds[i]);
  }
}

static void beginScanSlot(uint8_t idx) {
  const bool invert = (idx >= scanOrderCount);
  const uint8_t baudIdx = invert ? (uint8_t)(idx - scanOrderCount) : idx;
  const uint32_t baud = scanOrder[baudIdx];
  scanSlotStartMs = millis();
  scanSlotBytes = 0;
  scanSlotDollar = 0;
  scanSlotUbx = 0;
  scanSlotTalker = 0;
  scanSlotPrint = 0;
  hexCount = 0;
  dollarWindow = 0;
  ubxWindow = 0;
  printWindow = 0;
  strncpy(fix.scanPhase, "scan", sizeof(fix.scanPhase) - 1);
  fix.score = bestScore;
  reopenGpsUart(baud, GPS_UART_RX, GPS_UART_TX, invert);
  glog("# GPS SCAN %u/%u baud=%lu inv=%d\n", (unsigned)(idx + 1),
       (unsigned)(scanOrderCount * 2), (unsigned long)baud, (int)invert);
}

static void finishScanSlot() {
  const int32_t s = scoreSlot(scanSlotBytes, scanSlotDollar, scanSlotUbx, scanSlotPrint);
  glog("# GPS SLOT score=%ld $=%u ubx=%u bytes=%lu baud=%lu inv=%d\n", (long)s,
       (unsigned)scanSlotDollar, (unsigned)scanSlotUbx, (unsigned long)scanSlotBytes,
       (unsigned long)activeBaud, (int)activeInvert);
  if (s > bestScore) {
    bestScore = s;
    bestBaud = activeBaud;
    bestInvert = activeInvert;
  }
  // One talker ($G/$B/$P) or one UBX sync is enough — waiting for a second '$' just
  // burns a 1 Hz cycle, and reopening the port would flush the sentences we just used.
  if (scanSlotTalker >= 1 || scanSlotUbx >= 1) {
    baudLocked = true;
    scanDone = true;
    strncpy(fix.scanPhase, "locked", sizeof(fix.scanPhase) - 1);
    fix.score = s;
    bestBaud = activeBaud;
    bestInvert = activeInvert;
    bestScore = s;
    saveBaudPrefs(activeBaud, activeInvert);
    return;
  }

  scanIndex++;
  if (scanIndex >= scanOrderCount * 2) {
    scanDone = true;
    baudLocked = true;
    strncpy(fix.scanPhase, bestScore > 0 ? "locked" : "listen", sizeof(fix.scanPhase) - 1);
    fix.score = bestScore;
    reopenGpsUart(bestBaud, GPS_UART_RX, GPS_UART_TX, bestInvert);
    if (bestScore > 0) {
      saveBaudPrefs(bestBaud, bestInvert);
    }
    glog("# GPS SCAN DONE best baud=%lu inv=%d score=%ld\n", (unsigned long)bestBaud,
         (int)bestInvert, (long)bestScore);
    return;
  }
  beginScanSlot(scanIndex);
}

static void gpsConfigureM10() {
  sendUbxM10ValSet();
  sendUbxM10Motion();
  sendUbxEnablePvtRate();
  sendUbxEnableNmeaLegacy();
  sendUbxNavPvtPoll();
}

static void startScanFromListen() {
  scanDone = false;
  baudLocked = false;
  bestScore = -1;
  strncpy(fix.scanPhase, "scan", sizeof(fix.scanPhase) - 1);
  scanIndex = 0;
  glog("# GPS listen gave up baud=%lu inv=%d — scan\n", (unsigned long)activeBaud,
       (int)activeInvert);
  beginScanSlot(scanIndex);
}

void gpsNmeaInit() {
  fix = {};
  lineLen = 0;
  peekPos = 0;
  ubxSt = U_SYNC1;
  prevRx = 0;
  baudLocked = false;
  scanDone = false;
  scanIndex = 0;
  bestScore = -1;
  baudSaved = false;
  lastByteMs = lastFramedMs = lastSentenceMs = lastUbxMs = lastPvtMs = 0;
  lastSpeedMs = 0;
  lastGoodSpeedMs = 0;
  lastPvtSpeedMs = 0;
  kalmanX = 0.0f;
  kalmanP = kMeasVarFloor;
  kalmanInit = false;
  lastFiltMs = 0;
  spikeRejects = 0;
  movingState = false;
  moveVotes = 0;
  stopVotes = 0;
  gpsConfiguredOnce = false;
  fix.rawSpeedKmh = 0;
  fix.speedAccKmh = 0;
  fix.speedRejects = 0;
  fix.speedSrc = 0;
  fix.speedKmh = 0;
  fix.speedValid = false;
  windowStartMs = millis();
  windowBytes = windowNmea = 0;
  lastProbeMs = lastDiagMs = 0;
  hexCount = 0;
  fix.hexSample[0] = 0;
  strncpy(fix.scanPhase, "scan", sizeof(fix.scanPhase) - 1);

  uint32_t preferBaud = GPS_UART_BAUD;
  bool preferInv = false;
  loadBaudPrefs(&preferBaud, &preferInv);
  bestBaud = preferBaud;
  bestInvert = preferInv;
  buildScanOrder(preferBaud);

  // Open the remembered (or factory) baud and parse immediately. Scanning every boot
  // was the delay: 8 bauds × invert × 2 s. Invert is a last-resort pass, not a default.
  scanDone = true;
  baudLocked = false;
  listenStartMs = 0;
  strncpy(fix.scanPhase, "listen", sizeof(fix.scanPhase) - 1);
  reopenGpsUart(preferBaud, GPS_UART_RX, GPS_UART_TX, preferInv);
  glog("# GPS listen baud=%lu inv=%d (scan only if silent)\n", (unsigned long)preferBaud,
       (int)preferInv);
}

void gpsNmeaLoop() {
  uint32_t now = millis();

  while (GpsSerial.available() > 0) {
    const int v = GpsSerial.read();
    if (v < 0) {
      break;
    }
    const uint8_t b = (uint8_t)v;
    lastByteMs = now;
    fix.totalBytes++;
    windowBytes++;
    if (!scanDone) {
      scanSlotBytes++;
    }
    notePeek(b);
    noteHexSample(b);

    if (prevRx == 0xB5 && b == 0x62) {
      lastFramedMs = now;
      lastUbxMs = now;  // keep UBX status sticky while sync keeps arriving
      ubxWindow++;
      if (!scanDone) {
        scanSlotUbx++;
      }
    }
    if (b == '$') {
      lastFramedMs = now;
      if (!scanDone) {
        scanSlotDollar++;
      }
    }
    if (prevRx == '$' && (b == 'G' || b == 'B' || b == 'P')) {
      lastFramedMs = now;
      baudLocked = true;
      if (!scanDone) {
        scanSlotTalker++;
      }
    }
    prevRx = b;

    if (b >= 32 && b < 127 && !scanDone) {
      scanSlotPrint++;
    }

    ubxFeed(b);

    if (b == '\n' || b == '\r') {
      finishNmeaLine();
      if (fix.totalNmea > 0) {
        baudLocked = true;
      }
    } else if (b == '$') {
      lineLen = 0;
      lineBuf[lineLen++] = '$';
    } else if (lineLen > 0 && lineLen + 1 < sizeof(lineBuf)) {
      lineBuf[lineLen++] = (char)b;
    } else if (lineLen + 1 >= sizeof(lineBuf)) {
      lineLen = 0;
    }
  }

  // Draining the RX buffer takes milliseconds, and the parsers stamp themselves with
  // millis(). Re-read the clock so no timestamp is ever ahead of `now` — an unsigned
  // `now - stamp` on a future stamp wraps to ~4.3e9 and trips every age/timeout below.
  now = millis();

  if (listenStartMs == 0) {
    listenStartMs = now;
  }

  if (scanDone) {
    if (baudLocked || lastUbxMs != 0) {
      baudLocked = true;
      strncpy(fix.scanPhase, "locked", sizeof(fix.scanPhase) - 1);
      saveBaudPrefs(activeBaud, activeInvert);
    } else if (now - listenStartMs >= kListenGiveUpMs) {
      startScanFromListen();
    }
  } else if (scanSlotTalker >= 1 || scanSlotUbx >= 1 ||
             (now - scanSlotStartMs >= kScanSlotMs)) {
    finishScanSlot();
  }

  if (now - windowStartMs >= 1000) {
    fix.bytesPerSec = (uint16_t)windowBytes;
    fix.nmeaPerSec = (uint16_t)windowNmea;
    windowBytes = 0;
    windowNmea = 0;
    windowStartMs = now;
  }

  fix.rawAlive = (lastByteMs != 0) && (now - lastByteMs < 2000);
  fix.rxAlive = (lastFramedMs != 0) && (now - lastFramedMs < 3000);
  fix.ubxAlive = (lastUbxMs != 0) && (now - lastUbxMs < 5000);
  const bool pvtAlive = (lastPvtMs != 0) && (now - lastPvtMs < 8000);
  const bool asciiNmea = (lastSentenceMs != 0) && (now - lastSentenceMs < 8000);
  fix.nmeaAlive = pvtAlive || asciiNmea;
  fix.ageMs = (lastSentenceMs != 0) ? (now - lastSentenceMs) : 0xFFFFFFFFu;
  fix.rxPin = activeRxPin;
  fix.txPin = activeTxPin;
  fix.baud = activeBaud;
  fix.inverted = activeInvert;

  // Soften fix flag: don't yank valid every frame if PVT briefly missing
  if (!fix.nmeaAlive && !fix.ubxAlive) {
    fix.valid = false;
  }

  // A real stop is reported by applySpeedKmh once near-zero persists, so this only
  // covers samples drying up completely.
  if (lastSpeedMs != 0 && (now - lastSpeedMs) >= kSpeedStaleMs) {
    fix.speedKmh = 0.0f;
  }
  if (!pvtAlive && !asciiNmea && lastSpeedMs != 0 && (now - lastSpeedMs > 8000)) {
    fix.speedKmh = 0.0f;
    fix.rawSpeedKmh = 0.0f;
    fix.speedValid = false;
    lastGoodSpeedMs = 0;
    kalmanInit = false;
    kalmanX = 0.0f;
    movingState = false;
    moveVotes = 0;
    stopVotes = 0;
    fix.posValid = false;
    fix.headValid = false;
  }

  if (now - lastDiagMs >= 1000) {
    lastDiagMs = now;
    if (hexCount > 0) {
      dumpHexSample();
    }
    Serial.printf(
        "# gps spd=%.2f raw=%.2f acc=%.2f mv=%d rej=%lu src=%lu ok=%d fix=%d sats=%u talk=%s $\n",
        (double)fix.speedKmh, (double)fix.rawSpeedKmh, (double)fix.speedAccKmh, (int)movingState,
        (unsigned long)fix.speedRejects, (unsigned long)fix.speedSrc, (int)fix.speedValid,
        (int)fix.valid, (unsigned)fix.satellites, fix.lastTalker[0] ? fix.lastTalker : "-");
  }

  // Configure once after lock (periodic CFG was killing NMEA — dollar count collapsed).
  if (scanDone && !gpsConfiguredOnce && (fix.nmeaAlive || fix.ubxAlive || fix.rxAlive)) {
    gpsConfiguredOnce = true;
    lastProbeMs = now;
    gpsConfigureM10();
    Serial.println("# gps configure once (enable RMC/VTG/PVT)");
  } else if (gpsConfiguredOnce && (now - lastProbeMs >= 60000)) {
    lastProbeMs = now;
    sendUbxEnableNmeaLegacy();  // light NMEA refresh only
  } else if (scanDone && !fix.rxAlive && (now - lastProbeMs >= 5000)) {
    lastProbeMs = now;
    sendUbxEnableNmeaLegacy();
    sendUbxNavPvtPoll();
  }
}

const GpsFix *gpsNmeaFix() {
  return &fix;
}

/** 1e-7 degrees to text without routing through a float. The sign lives on the whole part,
 *  which is 0 for anything inside +/-1 degree, so it is applied to the string instead. */
void gpsFormatE7(char *out, size_t n, int32_t e7) {
  const long long a = (e7 < 0) ? -(long long)e7 : (long long)e7;
  snprintf(out, n, "%s%lld.%07lld", (e7 < 0) ? "-" : "", a / 10000000LL, a % 10000000LL);
}

size_t gpsNmeaDiagJson(char *buf, size_t bufLen) {
  if (buf == nullptr || bufLen < 64) {
    return 0;
  }
  char latS[20];
  char lonS[20];
  gpsFormatE7(latS, sizeof(latS), fix.latE7);
  gpsFormatE7(lonS, sizeof(lonS), fix.lonE7);
  return (size_t)snprintf(
      buf, bufLen,
      "{\"ok\":true,\"raw\":%s,\"framed\":%s,\"nmea\":%s,\"ubx\":%s,\"fix\":%s,"
      "\"baud\":%lu,\"inv\":%s,\"rx_pin\":%d,\"tx_pin\":%d,\"bps\":%u,\"total\":%lu,"
      "\"dollar\":%u,\"ubx_sync\":%u,\"score\":%ld,\"scan\":\"%s\","
      "\"peek\":\"%s\",\"hex\":\"%s\",\"talker\":\"%s\",\"sats\":%u,"
      "\"time_valid\":%s,\"time\":\"%02u:%02u:%02u\","
      "\"spd\":%.2f,\"spd_raw\":%.2f,\"spd_acc\":%.2f,\"spd_rej\":%lu,"
      "\"moving\":%s,\"spd_src\":%lu,\"spd_ok\":%s,"
      "\"pos_ok\":%s,\"lat\":%s,\"lon\":%s,\"alt_m\":%.1f,\"acc_m\":%.1f,"
      "\"head_ok\":%s,\"head\":%.1f}",
      fix.rawAlive ? "true" : "false", fix.rxAlive ? "true" : "false",
      fix.nmeaAlive ? "true" : "false", fix.ubxAlive ? "true" : "false",
      fix.valid ? "true" : "false", (unsigned long)fix.baud, fix.inverted ? "true" : "false",
      (int)fix.rxPin, (int)fix.txPin, (unsigned)fix.bytesPerSec, (unsigned long)fix.totalBytes,
      (unsigned)fix.dollarCount, (unsigned)fix.ubxSyncCount, (long)fix.score,
      fix.scanPhase[0] ? fix.scanPhase : "?", fix.rawPeek[0] ? fix.rawPeek : "",
      fix.hexSample[0] ? fix.hexSample : "", fix.lastTalker[0] ? fix.lastTalker : "",
      (unsigned)fix.satellites, fix.timeValid ? "true" : "false", (unsigned)fix.hour,
      (unsigned)fix.minute, (unsigned)fix.second, (double)fix.speedKmh, (double)fix.rawSpeedKmh,
      (double)fix.speedAccKmh, (unsigned long)fix.speedRejects, movingState ? "true" : "false",
      (unsigned long)fix.speedSrc, fix.speedValid ? "true" : "false",
      fix.posValid ? "true" : "false", latS, lonS, (double)fix.altMmMsl * 0.001,
      (double)fix.accMm * 0.001, fix.headValid ? "true" : "false", (double)fix.headDeg);
}
