#include "gps_trip.h"

#include "gps_nmea.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

static Preferences tripPrefs;
static TripSnapshot snap = {};
static uint32_t lastSampleMs = 0;
static uint32_t lastSaveMs = 0;
static bool dirty = false;
// Millisecond accumulators: seconds alone would drop the remainder every sample.
static uint64_t sessionMoveMs = 0;
static uint64_t totalMoveMs = 0;
static const float kMoveKmh = 0.4f;

static void toUtc8(uint8_t uh, uint8_t um, uint8_t us, uint16_t uy, uint8_t umo, uint8_t ud,
                   uint8_t *h8, uint8_t *m8, uint8_t *s8, uint16_t *y8, uint8_t *mo8,
                   uint8_t *d8) {
  int h = (int)uh + 8;
  int d = (int)ud;
  int mo = (int)umo;
  int y = (int)uy;
  if (h >= 24) {
    h -= 24;
    d += 1;
    static const uint8_t mdays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int dim = 31;
    if (mo >= 1 && mo <= 12) {
      dim = mdays[mo - 1];
      if (mo == 2 && ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0))) {
        dim = 29;
      }
    }
    if (d > dim) {
      d = 1;
      mo += 1;
      if (mo > 12) {
        mo = 1;
        y += 1;
      }
    }
  }
  *h8 = (uint8_t)h;
  *m8 = um;
  *s8 = us;
  *y8 = (uint16_t)y;
  *mo8 = (uint8_t)mo;
  *d8 = (uint8_t)d;
}

static void loadTrip() {
  if (!tripPrefs.begin("trip", true)) {
    return;
  }
  snap.maxSpeedKmh = tripPrefs.getFloat("max_spd", 0.0f);
  snap.maxYear = (uint16_t)tripPrefs.getUShort("max_y", 0);
  snap.maxMonth = tripPrefs.getUChar("max_mo", 0);
  snap.maxDay = tripPrefs.getUChar("max_d", 0);
  snap.maxHour = tripPrefs.getUChar("max_h", 0);
  snap.maxMinute = tripPrefs.getUChar("max_mi", 0);
  snap.maxSecond = tripPrefs.getUChar("max_s", 0);

  snap.dayKm = tripPrefs.getFloat("day_km", 0.0f);
  snap.dayYear = (uint16_t)tripPrefs.getUShort("day_y", 0);
  snap.dayMonth = tripPrefs.getUChar("day_mo", 0);
  snap.dayDay = tripPrefs.getUChar("day_d", 0);

  snap.totalKm = tripPrefs.getFloat("tot_km", 0.0f);
  snap.totalYear = (uint16_t)tripPrefs.getUShort("tot_y", 0);
  snap.totalMonth = tripPrefs.getUChar("tot_mo", 0);
  snap.totalDay = tripPrefs.getUChar("tot_d", 0);
  snap.totalHour = tripPrefs.getUChar("tot_h", 0);
  snap.totalMinute = tripPrefs.getUChar("tot_mi", 0);
  snap.totalSecond = tripPrefs.getUChar("tot_s", 0);
  totalMoveMs = tripPrefs.getULong64("tot_ms", 0);
  snap.totalMoveSec = (uint32_t)(totalMoveMs / 1000u);
  tripPrefs.end();
}

void tripSaveNow() {
  if (!tripPrefs.begin("trip", false)) {
    return;
  }
  tripPrefs.putFloat("max_spd", snap.maxSpeedKmh);
  tripPrefs.putUShort("max_y", snap.maxYear);
  tripPrefs.putUChar("max_mo", snap.maxMonth);
  tripPrefs.putUChar("max_d", snap.maxDay);
  tripPrefs.putUChar("max_h", snap.maxHour);
  tripPrefs.putUChar("max_mi", snap.maxMinute);
  tripPrefs.putUChar("max_s", snap.maxSecond);

  tripPrefs.putFloat("day_km", snap.dayKm);
  tripPrefs.putUShort("day_y", snap.dayYear);
  tripPrefs.putUChar("day_mo", snap.dayMonth);
  tripPrefs.putUChar("day_d", snap.dayDay);

  tripPrefs.putFloat("tot_km", snap.totalKm);
  tripPrefs.putUShort("tot_y", snap.totalYear);
  tripPrefs.putUChar("tot_mo", snap.totalMonth);
  tripPrefs.putUChar("tot_d", snap.totalDay);
  tripPrefs.putUChar("tot_h", snap.totalHour);
  tripPrefs.putUChar("tot_mi", snap.totalMinute);
  tripPrefs.putUChar("tot_s", snap.totalSecond);
  tripPrefs.putULong64("tot_ms", totalMoveMs);
  tripPrefs.end();
  dirty = false;
  lastSaveMs = millis();
}

void tripInit() {
  snap = {};
  sessionMoveMs = 0;
  totalMoveMs = 0;
  loadTrip();
  lastSampleMs = 0;
  lastSaveMs = millis();
  dirty = false;
}

float tripAvgKmh(float km, uint32_t moveSec) {
  if (moveSec < 5u || !(km > 0.0f)) {
    return 0.0f;
  }
  return km / ((float)moveSec / 3600.0f);
}

void tripLoop() {
  const GpsFix *g = gpsNmeaFix();
  const uint32_t now = millis();

  uint8_t h8 = 0, m8 = 0, s8 = 0, mo8 = 0, d8 = 0;
  uint16_t y8 = 0;
  bool haveStamp = false;
  if (g->timeValid && g->dateValid) {
    toUtc8(g->hour, g->minute, g->second, g->year, g->month, g->day, &h8, &m8, &s8, &y8, &mo8,
           &d8);
    haveStamp = true;
  }

  // New calendar day (UTC+8) → reset day odometer
  if (haveStamp) {
    if (snap.dayYear == 0 || y8 != snap.dayYear || mo8 != snap.dayMonth || d8 != snap.dayDay) {
      snap.dayKm = 0.0f;
      snap.dayYear = y8;
      snap.dayMonth = mo8;
      snap.dayDay = d8;
      dirty = true;
    }
  }

  if (lastSampleMs != 0 && g->speedValid && g->speedKmh >= kMoveKmh) {
    const uint32_t dtMs = now - lastSampleMs;
    const float dtH = (float)dtMs / 3600000.0f;
    if (dtH > 0.0f && dtH < 0.05f) {  // ignore huge gaps (>3min)
      const float dkm = g->speedKmh * dtH;
      snap.dayKm += dkm;
      snap.totalKm += dkm;
      snap.sessionKm += dkm;
      sessionMoveMs += dtMs;
      totalMoveMs += dtMs;
      snap.sessionMoveSec = (uint32_t)(sessionMoveMs / 1000u);
      snap.totalMoveSec = (uint32_t)(totalMoveMs / 1000u);
      if (haveStamp) {
        snap.totalYear = y8;
        snap.totalMonth = mo8;
        snap.totalDay = d8;
        snap.totalHour = h8;
        snap.totalMinute = m8;
        snap.totalSecond = s8;
      }
      dirty = true;
    }
  }
  lastSampleMs = now;

  if (g->speedValid && g->speedKmh > snap.maxSpeedKmh + 0.05f) {
    snap.maxSpeedKmh = g->speedKmh;
    if (haveStamp) {
      snap.maxYear = y8;
      snap.maxMonth = mo8;
      snap.maxDay = d8;
      snap.maxHour = h8;
      snap.maxMinute = m8;
      snap.maxSecond = s8;
    }
    dirty = true;
  }

  if (dirty && (now - lastSaveMs >= 15000u)) {
    tripSaveNow();
  }
}

const TripSnapshot *tripSnapshot() {
  return &snap;
}
