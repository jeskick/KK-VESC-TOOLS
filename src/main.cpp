#include <Arduino.h>
#include <Preferences.h>
#include <string.h>

#include "ota_wifi.h"
#include "tft_dashboard.h"
#include "vesc_uart.h"
#include "vesc_now.h"
#include "live_status.h"
#include "keypad.h"
#include "gps_log.h"
#include "gps_nmea.h"
#include "gps_trip.h"
#include "sd_probe.h"
#include "qmc5883.h"
#include "pins.h"

// PIN_THROTTLE and PIN_PPM live in pins.h with the rest of the map.
// Output mode / CPPM channel: change in SETTINGS menu only (BOOT unused).
static const int PPM_CHANNELS = 4;
static const int PPM_PERIOD_US = 20000;
static const int PPM_PULSE_US = 300;

enum OutMode : uint8_t { MODE_PWM = 0, MODE_CPPM = 1 };

static int adcMinSeen = 4095;
static int adcMaxSeen = 0;
static int mvMinSeen = 5000;
static int mvMaxSeen = 0;
static int adcRest = -1;
static int mvRest = -1;
static int calCenter = -1;
static int calFwd = -1;
static int calRev = -1;
static bool calReady = false;
static Preferences prefs;
static volatile uint16_t ppmCh[PPM_CHANNELS] = {1500, 1500, 1500, 1500};
static volatile uint16_t throttleUs = 1500;
static volatile OutMode outMode = MODE_PWM;
static volatile uint8_t throttleCh = 3;  // 1-based, used in CPPM
static bool forceDataView = false;       // show data screen without a live VESC link
static bool thrReverse = false;          // mirror output about neutral, for a swapped pot

// VESC link runtime: TOTAL = cumulative linked time (NVS); SESS = this/last session
static uint64_t totalRunMs = 0;
static uint32_t sessionStartMs = 0;
static uint32_t lastSessionMs = 0;
static bool runtimeLinked = false;
static uint32_t runtimeLastSampleMs = 0;
static uint32_t runtimeLastSaveMs = 0;

static uint8_t uiScreen = 0;  // 0 main, 1 settings, 2 trip, 3 thr cal
static int8_t statsSel = 0;
static uint8_t dataPage = 0;  // main screen: 0 VESC, 1 GPS, 2 compass (LEFT/RIGHT)
static const uint8_t kDataPages = 4;
static int draftCalC = -1, draftCalF = -1, draftCalR = -1;
static float mvFilt = 0.0f;
static bool mvFiltInit = false;
static int ppmSmoothed = 1500;
static int mvCenter = 1500;
static int mvFwdHi = 0;
static int mvRevLo = 5000;

static void loadTotalRuntime() {
  prefs.begin("run", true);
  totalRunMs = prefs.getULong64("total_ms", 0);
  prefs.end();
}

static void saveTotalRuntime() {
  prefs.begin("run", false);
  prefs.putULong64("total_ms", totalRunMs);
  prefs.end();
  runtimeLastSaveMs = millis();
}

static void tickLinkRuntime() {
  const bool linked = vescUartConnected();
  const uint32_t now = millis();

  if (linked) {
    if (!runtimeLinked) {
      sessionStartMs = now;
      runtimeLastSampleMs = now;
      runtimeLinked = true;
    } else if (now != runtimeLastSampleMs) {
      totalRunMs += (uint64_t)(now - runtimeLastSampleMs);
      runtimeLastSampleMs = now;
    }
    if (now - runtimeLastSaveMs >= 30000u) {
      saveTotalRuntime();
    }
  } else if (runtimeLinked) {
    if (now != runtimeLastSampleMs) {
      totalRunMs += (uint64_t)(now - runtimeLastSampleMs);
      runtimeLastSampleMs = now;
    }
    lastSessionMs = now - sessionStartMs;
    runtimeLinked = false;
    saveTotalRuntime();
  }
}

static uint32_t runtimeTotalSec() {
  return (uint32_t)(totalRunMs / 1000ull);
}

static uint32_t runtimeSessionSec() {
  if (runtimeLinked) {
    return (millis() - sessionStartMs) / 1000u;
  }
  return lastSessionMs / 1000u;
}

static const int kCalMinSpanMv = 150;
static const char *calHint = nullptr;

static bool calPointsUsable(int c, int f, int r) {
  if (c < 400 || c > 3700 || f < 0 || r < 0) {
    return false;
  }
  if (abs(f - c) < kCalMinSpanMv || abs(r - c) < kCalMinSpanMv) {
    return false;
  }
  // FWD and REV must sit on opposite sides of center.
  return (int)(f - c) * (int)(r - c) < 0;
}

static void saveCalPoints(int center, int fwd, int rev) {
  calCenter = center;
  calFwd = fwd;
  calRev = rev;
  calReady = calPointsUsable(calCenter, calFwd, calRev);
  if (calReady) {
    mvCenter = calCenter;
    mvFwdHi = calFwd;
    mvRevLo = calRev;
    // FWD/REV points already encode pot polarity. A leftover THR REVERSE
    // would flip them again, which is why cal still needed a separate invert.
    thrReverse = false;
  }
  prefs.begin("thr", false);
  prefs.putInt("center", calCenter);
  prefs.putInt("fwd", calFwd);
  prefs.putInt("rev", calRev);
  prefs.putBool("revout", thrReverse);
  prefs.end();
  Serial.printf("# cal saved c=%d f=%d r=%d ready=%d rev=%d\n", calCenter, calFwd, calRev,
                (int)calReady, (int)thrReverse);
}

static void toggleOutMode();
static void toggleThrottleReverse();
static void cycleThrottleCh();
static void saveOutMode();
static void toggleVescLink();

static void handleUiKeys() {
  KeyId id = KEY_NONE;
  KeyAction act = KEY_ACT_NONE;

  auto wrapDec = [](int sel, int n) -> int {
    if (n <= 0) {
      return 0;
    }
    return (sel <= 0) ? (n - 1) : (sel - 1);
  };
  auto wrapInc = [](int sel, int n) -> int {
    if (n <= 0) {
      return 0;
    }
    return (sel >= n - 1) ? 0 : (sel + 1);
  };

  while (keypadPollAction(&id, &act)) {
    Serial.printf("# key id=%u act=%u held=%s scr=%u sel=%d\n", (unsigned)id, (unsigned)act,
                  keypadHeldLabel(), (unsigned)uiScreen, statsSel);

    if (uiScreen == 0) {
      if (id == KEY_OK && act == KEY_ACT_CLICK) {
        uiScreen = 1;
        statsSel = 0;
        Serial.println("# ui → settings");
      } else if (act == KEY_ACT_CLICK && id == KEY_RIGHT) {
        dataPage = (uint8_t)((dataPage + 1) % kDataPages);
        Serial.printf("# page → %u\n", (unsigned)dataPage);
      } else if (act == KEY_ACT_CLICK && id == KEY_LEFT) {
        dataPage = (uint8_t)((dataPage + kDataPages - 1) % kDataPages);
        Serial.printf("# page → %u\n", (unsigned)dataPage);
      }
      continue;
    }

    if (uiScreen == 1) {
      // 0 vesc link, 1 trip, 2 thr cal, 3 thr rev, 4 out mode, 5 cppm ch,
      // 6 sd, 7 track log, 8 data view, 9 mag yaw, 10 back
      const int n = 11;
      if (act == KEY_ACT_CLICK && (id == KEY_UP || id == KEY_RIGHT)) {
        statsSel = wrapDec(statsSel, n);
      } else if (act == KEY_ACT_CLICK && id == KEY_DOWN) {
        statsSel = wrapInc(statsSel, n);
      } else if (act == KEY_ACT_CLICK && id == KEY_LEFT) {
        uiScreen = 0;
      } else if (act == KEY_ACT_CLICK && id == KEY_OK) {
        if (statsSel == 0) {
          toggleVescLink();
        } else if (statsSel == 1) {
          uiScreen = 2;
          statsSel = 0;
        } else if (statsSel == 2) {
          uiScreen = 3;
          statsSel = 0;
          draftCalC = -1;
          draftCalF = -1;
          draftCalR = -1;
          calHint = nullptr;
        } else if (statsSel == 3) {
          toggleThrottleReverse();
        } else if (statsSel == 4) {
          toggleOutMode();
        } else if (statsSel == 5) {
          cycleThrottleCh();
        } else if (statsSel == 6) {
          sdProbeRun();
          uiScreen = 4;
        } else if (statsSel == 7) {
          gpsLogSetEnabled(!gpsLogEnabled());
        } else if (statsSel == 8) {
          forceDataView = !forceDataView;
          saveOutMode();
          Serial.printf("# data view -> %s\n", forceDataView ? "ON" : "AUTO");
        } else if (statsSel == 9) {
          magCycleYaw();
        } else {
          uiScreen = 0;
        }
      }
      continue;
    }

    if (uiScreen == 2) {
      const int n = 3;
      if (act == KEY_ACT_CLICK && (id == KEY_UP || id == KEY_RIGHT)) {
        statsSel = wrapDec(statsSel, n);
      } else if (act == KEY_ACT_CLICK && id == KEY_DOWN) {
        statsSel = wrapInc(statsSel, n);
      } else if (act == KEY_ACT_CLICK && (id == KEY_LEFT || id == KEY_OK)) {
        uiScreen = 1;
        statsSel = 1;
        tripSaveNow();
      }
      continue;
    }

    if (uiScreen == 3) {
      // 0 center, 1 full fwd, 2 full rev (OK captures; LEFT back)
      if (act == KEY_ACT_CLICK && id == KEY_LEFT) {
        calHint = nullptr;
        if (statsSel <= 0) {
          uiScreen = 1;
          statsSel = 2;
        } else {
          statsSel--;
        }
      } else if (act == KEY_ACT_CLICK && id == KEY_OK) {
        const int live = (int)(mvFilt + 0.5f);
        if (statsSel == 0) {
          draftCalC = live;
          draftCalF = -1;
          draftCalR = -1;
          statsSel = 1;
          calHint = nullptr;
        } else if (statsSel == 1) {
          if (draftCalC < 0 || abs(live - draftCalC) < kCalMinSpanMv) {
            calHint = "hold FULL FWD, then OK";
          } else {
            draftCalF = live;
            statsSel = 2;
            calHint = nullptr;
          }
        } else if (statsSel == 2) {
          if (draftCalC < 0 || abs(live - draftCalC) < kCalMinSpanMv) {
            calHint = "hold FULL REV, then OK";
          } else if ((live - draftCalC) * (draftCalF - draftCalC) >= 0) {
            calHint = "must be the opposite end";
          } else {
            draftCalR = live;
            saveCalPoints(draftCalC, draftCalF, draftCalR);
            if (calReady) {
              calHint = nullptr;
              uiScreen = 1;
              statsSel = 2;
              Serial.println("# ui cal saved → settings");
            } else {
              calHint = "cal rejected — try again";
            }
          }
        }
      }
      continue;
    }

    if (uiScreen == 4) {
      if (act == KEY_ACT_CLICK && id == KEY_LEFT) {
        uiScreen = 1;
        statsSel = 6;
      } else if (act == KEY_ACT_CLICK && id == KEY_OK) {
        sdProbeRun();
      }
      continue;
    }
  }
}

static void applyThrottleChannels(int us) {
  us = constrain(us, 1000, 2000);
  throttleUs = (uint16_t)us;
  for (int i = 0; i < PPM_CHANNELS; i++) {
    ppmCh[i] = 1500;
  }
  const int idx = constrain((int)throttleCh, 1, PPM_CHANNELS) - 1;
  ppmCh[idx] = (uint16_t)us;
}

static void saveVescLink(VescLinkMode mode) {
  prefs.begin("vesc", false);
  prefs.putUChar("link", (uint8_t)mode);
  prefs.end();
}

static VescLinkMode loadVescLink() {
  prefs.begin("vesc", true);
  const uint8_t m = prefs.getUChar("link", (uint8_t)VESC_LINK_WIRE);
  prefs.end();
  return (m == (uint8_t)VESC_LINK_NOW) ? VESC_LINK_NOW : VESC_LINK_WIRE;
}

static void toggleVescLink() {
  const VescLinkMode next =
      (vescUartLinkMode() == VESC_LINK_NOW) ? VESC_LINK_WIRE : VESC_LINK_NOW;
  saveVescLink(next);
  vescUartSetLinkMode(next);
  Serial.printf("# vesc link -> %s\n", next == VESC_LINK_NOW ? "NOW" : "WIRE");
}

static void saveOutMode() {
  prefs.begin("thr", false);
  prefs.putUChar("mode", (uint8_t)outMode);
  prefs.putUChar("tch", throttleCh);
  prefs.putBool("dataview", forceDataView);
  prefs.putBool("revout", thrReverse);
  prefs.end();
}

static void loadOutMode() {
  prefs.begin("thr", true);
  const uint8_t m = prefs.getUChar("mode", MODE_PWM);
  const uint8_t ch = prefs.getUChar("tch", 3);
  forceDataView = false;
  // "rev" is already taken by the reverse calibration point, hence "revout".
  thrReverse = prefs.getBool("revout", false);
  prefs.end();
  outMode = (m == MODE_CPPM) ? MODE_CPPM : MODE_PWM;
  throttleCh = (ch >= 1 && ch <= PPM_CHANNELS) ? ch : 3;
}

static void ppmOutTask(void *arg) {
  (void)arg;
  pinMode(PIN_PPM, OUTPUT);
  for (;;) {
    const OutMode mode = outMode;
    uint16_t ch[PPM_CHANNELS];
    for (int i = 0; i < PPM_CHANNELS; i++) {
      ch[i] = ppmCh[i];
    }
    const int us = (int)throttleUs;

    if (mode == MODE_PWM) {
      digitalWrite(PIN_PPM, HIGH);
      delayMicroseconds(us);
      digitalWrite(PIN_PPM, LOW);
      int low = PPM_PERIOD_US - us;
      if (low < 1000) {
        low = 1000;
      }
      delayMicroseconds(low);
    } else {
      // JR-style CPPM: idle HIGH, 300us LOW separators, 20ms frame
      digitalWrite(PIN_PPM, HIGH);
      const uint32_t start = micros();
      for (int i = 0; i < PPM_CHANNELS; i++) {
        digitalWrite(PIN_PPM, LOW);
        delayMicroseconds(PPM_PULSE_US);
        digitalWrite(PIN_PPM, HIGH);
        int body = (int)ch[i] - PPM_PULSE_US;
        if (body < 200) {
          body = 200;
        }
        delayMicroseconds(body);
      }
      digitalWrite(PIN_PPM, LOW);
      delayMicroseconds(PPM_PULSE_US);
      digitalWrite(PIN_PPM, HIGH);
      const int used = (int)(micros() - start);
      int sync = PPM_PERIOD_US - used;
      if (sync < 4500) {
        sync = 4500;
      }
      delayMicroseconds(sync);
    }
  }
}

static void startPpmOutput() {
  applyThrottleChannels(1500);
  xTaskCreatePinnedToCore(ppmOutTask, "ppmout", 3072, nullptr, 1, nullptr, 0);
  disableCore0WDT();
}

static void toggleOutMode() {
  outMode = (outMode == MODE_PWM) ? MODE_CPPM : MODE_PWM;
  saveOutMode();
  Serial.printf("# mode -> %s\n", outMode == MODE_PWM ? "PWM" : "CPPM");
}

static void toggleThrottleReverse() {
  thrReverse = !thrReverse;
  saveOutMode();
  Serial.printf("# throttle reverse -> %s\n", thrReverse ? "ON" : "OFF");
}

static void cycleThrottleCh() {
  if (outMode != MODE_CPPM) {
    toggleOutMode();
    return;
  }
  throttleCh = (uint8_t)(throttleCh % PPM_CHANNELS) + 1;
  applyThrottleChannels((int)throttleUs);
  saveOutMode();
  Serial.printf("# CPPM throttle CH%d\n", (int)throttleCh);
}

static int effectiveCenterMv() {
  return calReady ? calCenter : mvCenter;
}

static int effectiveFwdMv() {
  if (calReady) {
    return calFwd;
  }
  int hi = mvFwdHi;
  const int center = effectiveCenterMv();
  if (hi < center + 200) {
    hi = constrain(center + 1400, center + 200, 3300);
  }
  return hi;
}

static int effectiveRevMv() {
  if (calReady) {
    return calRev;
  }
  int lo = mvRevLo;
  const int center = effectiveCenterMv();
  if (lo > center - 200) {
    lo = constrain(center - 1400, 0, center - 200);
  }
  return lo;
}

static int median5(int a, int b, int c, int d, int e) {
  int v[5] = {a, b, c, d, e};
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (v[j] < v[i]) {
        const int t = v[i];
        v[i] = v[j];
        v[j] = t;
      }
    }
  }
  return v[2];
}

static int readThrottleMilliVoltsFiltered() {
  const int m = median5(analogReadMilliVolts(PIN_THROTTLE), analogReadMilliVolts(PIN_THROTTLE),
                        analogReadMilliVolts(PIN_THROTTLE), analogReadMilliVolts(PIN_THROTTLE),
                        analogReadMilliVolts(PIN_THROTTLE));
  if (!mvFiltInit) {
    mvFilt = (float)m;
    mvFiltInit = true;
  } else {
    mvFilt = mvFilt * 0.78f + (float)m * 0.22f;
  }
  return (int)(mvFilt + 0.5f);
}

static int readThrottleRaw() {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(PIN_THROTTLE);
  }
  return (int)(sum / 8);
}

static void sampleRestCenterMv() {
  mvFiltInit = false;
  for (int i = 0; i < 50; i++) {
    readThrottleMilliVoltsFiltered();
    delay(8);
  }
  mvCenter = (int)(mvFilt + 0.5f);
  mvRest = mvCenter;
  if (mvCenter < 800 || mvCenter > 2500) {
    mvCenter = 1500;
  }
  mvFwdHi = mvCenter;
  mvRevLo = mvCenter;
}

static void loadCal() {
  prefs.begin("thr", true);
  calCenter = prefs.getInt("center", -1);
  calFwd = prefs.getInt("fwd", -1);
  calRev = prefs.getInt("rev", -1);
  prefs.end();
  calReady = calPointsUsable(calCenter, calFwd, calRev);
  if (calReady) {
    mvCenter = calCenter;
    mvFwdHi = calFwd;
    mvRevLo = calRev;
    // Old setups stored polarity twice (cal points + THR REVERSE). Fold the
    // switch into the FWD/REV points so direction comes from calibration only.
    if (thrReverse) {
      const int t = calFwd;
      calFwd = calRev;
      calRev = t;
      mvFwdHi = calFwd;
      mvRevLo = calRev;
      thrReverse = false;
      prefs.begin("thr", false);
      prefs.putInt("fwd", calFwd);
      prefs.putInt("rev", calRev);
      prefs.putBool("revout", false);
      prefs.end();
      Serial.println("# cal absorbed THR REVERSE into FWD/REV points");
    }
  }
}

static int mvToPpmUs(int mv) {
  const int center = effectiveCenterMv();
  int lo = effectiveRevMv();
  int hi = effectiveFwdMv();
  // Support either polarity: "fwd" may be lower or higher than center.
  if (hi < center && lo > center) {
    const int t = hi;
    hi = lo;
    lo = t;
    // If calibrated with fwd=low side, invert mapping sense below via swap flag
  }
  bool fwdIsHigh = true;
  if (calReady) {
    fwdIsHigh = (calFwd >= calCenter);
    if (fwdIsHigh) {
      hi = calFwd;
      lo = calRev;
    } else {
      hi = calRev;  // high physical end
      lo = calFwd;  // low physical end
    }
  }
  if (!calReady) {
    if (lo > center - 80) {
      lo = constrain(center - 200, 0, center - 80);
    }
    if (hi < center + 80) {
      hi = constrain(center + 200, center + 80, 3300);
    }
  }

  const int dead = 70;
  int us;
  if (mv >= center - dead && mv <= center + dead) {
    us = 1500;
  } else if (mv > center) {
    const int span = hi - center - dead;
    float t = span > 0 ? (float)(mv - center - dead) / (float)span : 0.0f;
    if (t < 0.0f) {
      t = 0.0f;
    }
    if (t > 1.0f) {
      t = 1.0f;
    }
    // high side → FWD if fwdIsHigh else REV
    us = fwdIsHigh ? (1500 + (int)(t * 500.0f + 0.5f)) : (1500 - (int)(t * 500.0f + 0.5f));
  } else {
    const int span = center - dead - lo;
    float t = span > 0 ? (float)(center - dead - mv) / (float)span : 0.0f;
    if (t < 0.0f) {
      t = 0.0f;
    }
    if (t > 1.0f) {
      t = 1.0f;
    }
    us = fwdIsHigh ? (1500 - (int)(t * 500.0f + 0.5f)) : (1500 + (int)(t * 500.0f + 0.5f));
  }
  if (thrReverse) {
    // Mirror about neutral. Must happen before the slew limiter, which tracks ppmSmoothed:
    // flipping afterwards would make every sample look like a full-scale jump.
    us = 3000 - us;
  }
  if (ppmSmoothed >= 1485 && ppmSmoothed <= 1515 && us > 1470 && us < 1530) {
    us = 1500;
  }
  const int maxStep = 80;
  if (us > ppmSmoothed + maxStep) {
    us = ppmSmoothed + maxStep;
  } else if (us < ppmSmoothed - maxStep) {
    us = ppmSmoothed - maxStep;
  }
  ppmSmoothed = constrain(us, 1000, 2000);
  return ppmSmoothed;
}

static int ppmToThrottlePct(int ppm) {
  return constrain((ppm - 1500) / 5, -100, 100);
}

static void noteRange(int raw, int mv) {
  const int center = effectiveCenterMv();
  if (raw < adcMinSeen) {
    adcMinSeen = raw;
  }
  if (raw > adcMaxSeen) {
    adcMaxSeen = raw;
  }
  if (mv < mvMinSeen) {
    mvMinSeen = mv;
  }
  if (mv > mvMaxSeen) {
    mvMaxSeen = mv;
  }
  if (mv > center + 60 && mv > mvFwdHi) {
    mvFwdHi = mv;
  }
  if (mv < center - 60 && mv < mvRevLo) {
    mvRevLo = mv;
  }
}

static bool extractInt(const char *line, const char *key, int *out) {
  const char *p = strstr(line, key);
  if (p == nullptr) {
    return false;
  }
  *out = atoi(p + strlen(key));
  return true;
}

static void handleCommand(char *line) {
  if (vescUartHandleCommand(line)) {
    return;
  }
  if (otaWifiHandleCommand(line)) {
    return;
  }
  if (strstr(line, "\"cmd\":\"ota_status\"") != nullptr || strstr(line, "\"cmd\": \"ota_status\"") != nullptr) {
    Serial.printf("{\"type\":\"ota\",\"fw\":\"%s\",\"wifi\":%s,\"ip\":\"%s\",\"msg\":\"%s\"}\n", FW_VERSION,
                  otaWifiConnected() ? "true" : "false", otaWifiConnected() ? otaWifiIp() : "",
                  otaWifiStatusText());
    return;
  }
  if (strstr(line, "\"cmd\":\"cal\"") == nullptr && strstr(line, "\"cmd\": \"cal\"") == nullptr) {
    if (strstr(line, "reset_range") != nullptr) {
      adcMinSeen = 4095;
      adcMaxSeen = 0;
      mvMinSeen = 5000;
      mvMaxSeen = 0;
      mvCenter = 1450;
      mvFwdHi = mvCenter;
      mvRevLo = mvCenter;
      Serial.println("# range reset");
    }
    return;
  }
  int center = -1, fwd = -1, rev = -1;
  extractInt(line, "\"center\":", &center);
  extractInt(line, "\"fwd\":", &fwd);
  extractInt(line, "\"rev\":", &rev);
  if (center < 0 || fwd < 0 || rev < 0) {
    Serial.println("# cal missing fields");
    return;
  }
  saveCalPoints(center, fwd, rev);
}

static void pollSerial() {
  static char buf[160];
  static size_t n = 0;
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) {
        buf[n] = 0;
        handleCommand(buf);
        n = 0;
      }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      n = 0;
    }
  }
}

static void emitTelemetry(int raw, int mv, int pct, int ppm) {
  const char *dir = "STOP";
  if (pct > 3) {
    dir = "FWD";
  } else if (pct < -3) {
    dir = "REV";
  }
  const char *mode = outMode == MODE_PWM ? "PWM" : "CPPM";
  liveStatusSetThrottle(ppm, pct, (float)mv, dir, mode);

  const VescValues *v = vescUartValues();
  const bool vescOk = vescUartConnected() && v != nullptr && v->valid;
  if (vescOk) {
    liveStatusSetVesc(true, vescUartFwVersion(), v->voltageInV, v->currentInA, v->currentMotorA,
                      v->rpm, v->duty, v->tempFetC, v->tempMotorC, v->fault);
  } else {
    liveStatusSetVesc(false, vescUartFwVersion(), 0, 0, 0, 0, 0, 0, 0, 0);
  }

  char line[320];
  snprintf(
      line, sizeof(line),
      "{\"type\":\"tel\",\"adc\":%d,\"mv\":%.1f,\"pct\":%d,\"ppm\":%d,\"dir\":\"%s\","
      "\"center\":%d,\"mode\":\"%s\",\"tch\":%u,"
      "\"ch1\":%u,\"ch2\":%u,\"ch3\":%u,\"ch4\":%u,\"pin\":%d,"
      "\"min\":%d,\"max\":%d,\"vmin\":%d,\"vmax\":%d,\"revlo\":%d,\"fwdhi\":%d}",
      raw, (float)mv, pct, ppm, dir, mvCenter, mode, (unsigned)throttleCh,
      (unsigned)ppmCh[0], (unsigned)ppmCh[1], (unsigned)ppmCh[2], (unsigned)ppmCh[3],
      PIN_PPM, adcMinSeen, adcMaxSeen, mvMinSeen, mvMaxSeen, mvRevLo, mvFwdHi);
  Serial.println(line);
  Serial.printf("# keys %s held=%s scr=%u\n", keypadRawLabel(), keypadHeldLabel(),
                (unsigned)uiScreen);
}

static const char *dirFromPct(int pct) {
  if (pct > 3) {
    return "FWD";
  }
  if (pct < -3) {
    return "REV";
  }
  return "STOP";
}

static void updateTftDashboard(int localPpm, int pct) {
  const VescValues *v = vescUartValues();
  const VescSetupValues *s = vescUartSetupValues();
  const bool vescOk = vescUartConnected() && v != nullptr && v->valid;
  const float vescPpm = vescUartDecodedPpm();
  const int ppmShow = vescPpm > 0.0f ? (int)(vescPpm + 0.5f) : localPpm;
  const int pctShow = (ppmShow - 1500) / 5;
  const VescBatteryLimits *lim = vescUartCachedBatteryLimits();

  DashboardInput d = {};
  d.vescLinkOk = vescUartConnected();
  d.vescLinkBle = (vescUartLinkMode() == VESC_LINK_NOW);
  d.vescLinkStatus = vescUartLinkStatus();
  d.vescBlePhase = vescNowState();
  d.vescBleHint = vescNowHint();
  d.nowRssiOk = vescNowRssiOk();
  d.nowRssi = vescNowRssi();
  d.vescValid = vescOk;
  if (vescOk) {
    d.voltageV = v->voltageInV;
    d.currentInA = v->currentInA;
    d.currentMotorA = v->currentMotorA;
    d.tempFetC = v->tempFetC;
    d.tempMotorC = v->tempMotorC;
    d.powerW = v->voltageInV * v->currentInA;
    d.fault = v->fault;
    d.rpm = v->rpm;
    d.whUsed = v->wattHours;
    d.duty = v->duty;
  }
  if (s != nullptr && s->valid) {
    d.speedKmh = s->speedMps * 3.6f;
    d.batteryPct = s->batteryLevel * 100.0f;
    if (s->voltageInV > 0.0f) {
      d.voltageV = s->voltageInV;
    }
    if (s->wattHours > 0.0f) {
      d.whUsed = s->wattHours;
    }
    if (s->rpm != 0.0f || d.rpm == 0.0f) {
      d.rpm = s->rpm;
    }
  } else if (vescOk) {
    d.speedKmh = vescErpmToKmh(v->rpm);
    d.batteryPct = -1.0f;
  } else {
    d.batteryPct = -1.0f;
  }
  if (lim != nullptr) {
    d.cutStartV = lim->cutStartV;
    d.cutEndV = lim->cutEndV;
  }
  d.throttlePpm = ppmShow;
  d.throttlePct = constrain(pctShow, -100, 100);
  d.throttleDir = dirFromPct(d.throttlePct);
  d.totalRunSec = runtimeTotalSec();
  d.sessionSec = runtimeSessionSec();

  d.keyLabel = keypadHeldLabel();
  d.keyRawLabel = keypadRawLabel();
  const GpsFix *g = gpsNmeaFix();
  d.gpsTimeValid = g->timeValid;
  d.gpsHour = g->hour;
  d.gpsMinute = g->minute;
  d.gpsSecond = g->second;
  d.gpsSats = g->satellites;
  d.gpsSpeedValid = g->speedValid;
  d.gpsSpeedKmh = g->speedKmh;
  d.gpsFresh = g->nmeaAlive;
  d.gpsRawAlive = g->rawAlive;
  d.gpsRxAlive = g->rxAlive;
  d.gpsNmeaAlive = g->nmeaAlive;
  d.gpsUbxAlive = g->ubxAlive;
  d.gpsFixOk = g->valid;
  d.gpsHeadValid = g->headValid;
  d.gpsHeadDeg = g->headDeg;
  {
    const MagStatus *m = magStatus();
    d.magOk = m->ok;
    d.magHeadDeg = m->headingDeg;
    d.magSda = m->sda;
    d.magScl = m->scl;
    d.magX = m->x;
    d.magY = m->y;
    d.magZ = m->z;
    d.magChip = m->chip;
    d.magPhase = m->phase;
  }
  d.gpsPosValid = g->posValid;
  // 5 decimals is ~1 m, which is all the screen can usefully show. The exact integer
  // degrees go to the track file; a double is plenty for this many digits.
  static char latStr[16];
  static char lonStr[16];
  snprintf(latStr, sizeof(latStr), "%.5f", (double)g->latE7 * 1e-7);
  snprintf(lonStr, sizeof(lonStr), "%.5f", (double)g->lonE7 * 1e-7);
  d.gpsLatStr = latStr;
  d.gpsLonStr = lonStr;
  d.gpsBaud = g->baud;
  d.gpsRxPin = g->rxPin;
  d.gpsBytesPerSec = g->bytesPerSec;
  d.gpsNmeaPerSec = g->nmeaPerSec;
  d.gpsLastTalker = g->lastTalker[0] ? g->lastTalker : "-";
  d.gpsRawPeek = g->rawPeek[0] ? g->rawPeek : "-";
  d.gpsHexSample = g->hexSample[0] ? g->hexSample : "";

  d.uiScreen = uiScreen;
  d.statsSel = statsSel;
  d.dataPage = dataPage;
  d.forceData = forceDataView;
  d.outModeLabel = (outMode == MODE_PWM) ? "PWM" : "CPPM";
  d.outThrottleCh = throttleCh;
  d.thrReverse = thrReverse;
  d.magYawDeg = magYawDeg();
  const TripSnapshot *tr = tripSnapshot();
  d.tripMaxKmh = tr->maxSpeedKmh;
  d.tripMaxY = tr->maxYear;
  d.tripMaxMo = tr->maxMonth;
  d.tripMaxD = tr->maxDay;
  d.tripMaxH = tr->maxHour;
  d.tripMaxMi = tr->maxMinute;
  d.tripMaxS = tr->maxSecond;
  d.tripDayKm = tr->dayKm;
  d.tripDayY = tr->dayYear;
  d.tripDayMo = tr->dayMonth;
  d.tripDayD = tr->dayDay;
  d.tripTotalKm = tr->totalKm;
  d.tripTotY = tr->totalYear;
  d.tripTotMo = tr->totalMonth;
  d.tripTotD = tr->totalDay;
  d.tripTotH = tr->totalHour;
  d.tripTotMi = tr->totalMinute;
  d.tripTotS = tr->totalSecond;
  d.tripSessionKm = tr->sessionKm;
  d.tripSessionSec = tr->sessionMoveSec;
  d.tripTotalSec = tr->totalMoveSec;
  d.tripSessionAvgKmh = tripAvgKmh(tr->sessionKm, tr->sessionMoveSec);
  d.tripTotalAvgKmh = tripAvgKmh(tr->totalKm, tr->totalMoveSec);

  d.thrLiveMv = (int)(mvFilt + 0.5f);
  if (uiScreen == 3) {
    d.thrCalCenter = draftCalC;
    d.thrCalFwd = draftCalF;
    d.thrCalRev = draftCalR;
  } else {
    d.thrCalCenter = calCenter;
    d.thrCalFwd = calFwd;
    d.thrCalRev = calRev;
  }
  d.thrCalReady = calReady;
  d.thrCalHint = calHint;
  d.thrOutPpm = localPpm;
  d.thrOutPct = pct;

  {
    const SdProbeResult *sd = sdProbeLast();
    const char *st = "none";
    if (sd->status == SD_PROBE_NO_CARD) {
      st = "no_card";
    } else if (sd->status == SD_PROBE_OK) {
      st = "ok";
    } else if (sd->status == SD_PROBE_ERROR) {
      st = "error";
    }
    d.sdStatus = st;
    d.sdMsg = sd->msg;
    d.sdClk = sd->clk;
    d.sdCmd = sd->cmd;
    d.sdD0 = sd->d0;
  }

  const GpsLogStatus *tl = gpsLogStatus();
  d.trackLogOn = tl->enabled;
  d.trackState = tl->state;
  d.trackFile = tl->file;
  d.trackPoints = tl->points;

  tftDashboardUpdate(&d);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("ESP32-S3 N16R8 boot");
  Serial.println("OUT=GPIO2  mode/CH only via SETTINGS menu");

  loadOutMode();
  loadTotalRuntime();
  const VescLinkMode vescLink = loadVescLink();
  runtimeLastSaveMs = millis();
  gpsNmeaInit();  // open UART before the rest of setup so GLL time is not dropped

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  adcAttachPin(PIN_THROTTLE);
  analogSetPinAttenuation(PIN_THROTTLE, ADC_11db);
  adcMinSeen = 4095;
  adcMaxSeen = 0;
  mvMinSeen = 5000;
  mvMaxSeen = 0;
  loadCal();
  if (calReady) {
    Serial.printf("# using saved throttle cal C=%d F=%d R=%d mV\n", calCenter, calFwd, calRev);
    mvFiltInit = false;
    mvCenter = calCenter;
  } else {
    Serial.println("# sampling rest center — keep stick released");
    sampleRestCenterMv();
  }
  mvMinSeen = mvCenter;
  mvMaxSeen = mvCenter;
  ppmSmoothed = 1500;
  Serial.printf("# center=%d mV -> PPM 1500\n", effectiveCenterMv());

  startPpmOutput();
  vescUartInit(vescLink);
  gpsNmeaLoop();
  magInit();
  gpsNmeaLoop();
  tripInit();
  sdProbeInit();
  sdProbeRun();  // no card => no_card expected
  gpsLogInit();  // after the probe, which is what mounts the card
  gpsNmeaLoop();
  Serial.printf("# out mode=%s throttleCH=%u  map FWD/REV center=1500\n",
                outMode == MODE_PWM ? "PWM" : "CPPM", (unsigned)throttleCh);
  tftDashboardInit();
  Serial.println("# TFT ST7789 240x320 ready");
  gpsNmeaLoop();
  otaWifiInit();
  keypadInit();  // after TFT/WiFi — SPI2 native pins are the keypad
  Serial.printf("# keypad UP=%d DN=%d LT=%d RT=%d OK=%d (active-LOW)\n", KEY_PIN_UP,
                KEY_PIN_DOWN, KEY_PIN_LEFT, KEY_PIN_RIGHT, KEY_PIN_OK);
  Serial.printf("# FW %s | OTA SoftAP ESP32S3-OTA / 12345678 -> http://192.168.4.1/ (60s auto-off)\n",
                FW_VERSION);
  gpsNmeaLoop();
}

void loop() {
  pollSerial();

  const int mv = readThrottleMilliVoltsFiltered();
  const int raw = readThrottleRaw();
  noteRange(raw, mv);
  int ppm;
  if (millis() < 3000) {
    ppm = 1500;
    ppmSmoothed = 1500;
  } else {
    ppm = mvToPpmUs(mv);
  }
  const int pct = ppmToThrottlePct(ppm);
  applyThrottleChannels(ppm);

  // GPIO2 PPM/PWM is unchanged. UART duty is extra, and only in ESP-NOW
  // so a wired PPM cable to the VESC does not fight SET_DUTY.
  if (vescUartLinkMode() == VESC_LINK_NOW && vescNowLinked()) {
    static uint32_t lastDutyMs = 0;
    const uint32_t nowDuty = millis();
    if (nowDuty - lastDutyMs >= 20) {
      lastDutyMs = nowDuty;
      float duty = 0.0f;
      if (ppm >= 1520 || ppm <= 1480) {
        duty = (float)(ppm - 1500) / 500.0f;
      }
      vescUartSetDuty(duty);
    }
  }

  vescUartLoop();
  otaWifiLoop();
  keypadLoop();
  handleUiKeys();
  gpsNmeaLoop();
  magLoop();
  tripLoop();
  gpsLogLoop();
  tickLinkRuntime();

  static uint32_t lastDraw = 0;
  // Faster refresh when idle / stats / GPS gauge so the UI and its animation stay smooth
  const uint32_t drawMs =
      (uiScreen != 0 || dataPage != 0) ? 50
      : ((vescUartConnected() && vescUartValues()->valid) ? 150 : 50);
  if (millis() - lastDraw >= drawMs) {
    lastDraw = millis();
    updateTftDashboard(ppm, pct);
  }

  static uint32_t lastLog = 0;
  // Slow telemetry while debugging GPS so # GPS HEX lines are readable
  if (millis() - lastLog >= 1000) {
    lastLog = millis();
    emitTelemetry(raw, mv, pct, ppm);
  }
  delay(5);
}
