#include "gps_log.h"

#include "gps_nmea.h"

#include <Arduino.h>
#include <FS.h>
#include <Preferences.h>
#include <SD_MMC.h>
#include <string.h>

// Rows are held in RAM and appended in blocks. At 1 Hz this buffer covers ~20 s, so the card
// is touched rarely and a stall cannot line up with a frame.
static const size_t kBufCap = 2048;
static const size_t kFlushAt = 1536;
static const uint32_t kFlushMs = 10000;

static const char kHeader[] = "utc,lat,lon,alt_m,speed_kmh,head_deg,sats,acc_m\n";

static bool enabled = true;
static bool recording = false;
static fs::File logFile;
static char fileName[24];
static uint32_t points = 0;
static uint32_t bytesWritten = 0;
static char buf[kBufCap];
static size_t bufLen = 0;
static uint32_t lastFlushMs = 0;
static uint8_t lastSec = 0xFF;
static bool cardMissing = false;
static bool openFailed = false;
static GpsLogStatus st = {};
static Preferences logPrefs;

static void flushBuf() {
  if (bufLen == 0 || !recording) {
    return;
  }
  bytesWritten += (uint32_t)logFile.write((const uint8_t *)buf, bufLen);
  // Power is cut by yanking the plug, so there is never a clean unmount. Pushing each block
  // out now is what bounds a crash to the few rows still sitting in the buffer.
  logFile.flush();
  bufLen = 0;
  lastFlushMs = millis();
}

static bool tryOpen(const GpsFix *g) {
  if (SD_MMC.cardType() == CARD_NONE) {
    cardMissing = true;
    return false;
  }
  cardMissing = false;

  if (!SD_MMC.exists("/tracks") && !SD_MMC.mkdir("/tracks")) {
    openFailed = true;
    return false;
  }

  // Named from GPS UTC rather than a counter, so tracks sort chronologically and survive
  // a wiped NVS.
  snprintf(fileName, sizeof(fileName), "%02u%02u%02u_%02u%02u%02u.csv",
           (unsigned)(g->year % 100), (unsigned)g->month, (unsigned)g->day, (unsigned)g->hour,
           (unsigned)g->minute, (unsigned)g->second);
  char path[40];
  snprintf(path, sizeof(path), "/tracks/%s", fileName);

  logFile = SD_MMC.open(path, FILE_WRITE);
  if (!logFile) {
    openFailed = true;
    fileName[0] = '\0';
    return false;
  }
  openFailed = false;
  logFile.write((const uint8_t *)kHeader, sizeof(kHeader) - 1);
  logFile.flush();

  recording = true;
  points = 0;
  bytesWritten = sizeof(kHeader) - 1;
  bufLen = 0;
  lastSec = 0xFF;
  lastFlushMs = millis();
  Serial.printf("# track rec %s\n", path);
  return true;
}

static void appendRow(const GpsFix *g) {
  char lat[20];
  char lon[20];
  gpsFormatE7(lat, sizeof(lat), g->latE7);
  gpsFormatE7(lon, sizeof(lon), g->lonE7);

  char row[128];
  const int n = snprintf(row, sizeof(row),
                         "%04u-%02u-%02uT%02u:%02u:%02uZ,%s,%s,%.1f,%.1f,%.1f,%u,%.1f\n",
                         (unsigned)g->year, (unsigned)g->month, (unsigned)g->day,
                         (unsigned)g->hour, (unsigned)g->minute, (unsigned)g->second, lat, lon,
                         (double)g->altMmMsl * 0.001, (double)g->speedKmh, (double)g->headDeg,
                         (unsigned)g->satellites, (double)g->accMm * 0.001);
  if (n <= 0 || (size_t)n >= sizeof(row)) {
    return;
  }
  if (bufLen + (size_t)n > kBufCap) {
    flushBuf();
  }
  if (bufLen + (size_t)n > kBufCap) {
    return;  // card is not taking writes; drop the row rather than run off the buffer
  }
  memcpy(buf + bufLen, row, (size_t)n);
  bufLen += (size_t)n;
  points++;
}

void gpsLogInit() {
  logPrefs.begin("gpslog", false);
  enabled = logPrefs.getBool("en", true);
  logPrefs.end();
  recording = false;
  points = 0;
  bytesWritten = 0;
  bufLen = 0;
  fileName[0] = '\0';
  Serial.printf("# track log %s\n", enabled ? "ON" : "OFF");
}

void gpsLogLoop() {
  if (!enabled) {
    if (recording) {
      gpsLogStop();
    }
    return;
  }

  const GpsFix *g = gpsNmeaFix();

  if (!recording) {
    // The filename comes from GPS UTC, so hold off until date, time and position are all real.
    if (g->posValid && g->dateValid && g->timeValid) {
      tryOpen(g);
    }
    return;
  }

  // One row per GPS epoch, and only while moving. Logging at a standstill would just pile up
  // a cloud of noise points on one spot, which is exactly what makes a track look furry.
  if (g->posValid && g->speedKmh > 0.0f && g->second != lastSec) {
    lastSec = g->second;
    appendRow(g);
  }

  if (bufLen >= kFlushAt || (bufLen > 0 && (millis() - lastFlushMs) >= kFlushMs)) {
    flushBuf();
  }
}

void gpsLogStop() {
  if (!recording) {
    return;
  }
  flushBuf();
  logFile.close();
  recording = false;
  Serial.printf("# track stop %s pts=%lu bytes=%lu\n", fileName, (unsigned long)points,
                (unsigned long)bytesWritten);
}

bool gpsLogEnabled() {
  return enabled;
}

void gpsLogSetEnabled(bool on) {
  if (on == enabled) {
    return;
  }
  enabled = on;
  if (!enabled) {
    gpsLogStop();
  }
  logPrefs.begin("gpslog", false);
  logPrefs.putBool("en", enabled);
  logPrefs.end();
  Serial.printf("# track log -> %s\n", enabled ? "ON" : "OFF");
}

const GpsLogStatus *gpsLogStatus() {
  st.enabled = enabled;
  st.recording = recording;
  strncpy(st.file, fileName, sizeof(st.file) - 1);
  st.file[sizeof(st.file) - 1] = '\0';
  st.points = points;
  st.bytes = bytesWritten + (uint32_t)bufLen;
  if (!enabled) {
    st.state = "off";
  } else if (recording) {
    st.state = "rec";
  } else if (cardMissing) {
    st.state = "no card";
  } else if (openFailed) {
    st.state = "error";
  } else {
    st.state = "wait fix";
  }
  return &st;
}

size_t gpsLogDiagJson(char *buf_, size_t bufLen_) {
  if (buf_ == nullptr || bufLen_ < 32) {
    return 0;
  }
  const GpsLogStatus *s = gpsLogStatus();
  return (size_t)snprintf(buf_, bufLen_,
                          "{\"ok\":true,\"enabled\":%s,\"recording\":%s,\"state\":\"%s\","
                          "\"file\":\"%s\",\"points\":%lu,\"bytes\":%lu}",
                          s->enabled ? "true" : "false", s->recording ? "true" : "false",
                          s->state, s->file, (unsigned long)s->points, (unsigned long)s->bytes);
}
