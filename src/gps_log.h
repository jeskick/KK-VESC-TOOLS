#pragma once

#include <stddef.h>
#include <stdint.h>

struct GpsLogStatus {
  bool enabled;    // user setting, persisted
  bool recording;  // a file is open
  char file[24];   // basename, empty when not recording
  uint32_t points;
  uint32_t bytes;
  const char *state;  // "off" | "wait fix" | "no card" | "error" | "rec"
};

void gpsLogInit();

/** Call from the main loop only. Writing to SD can stall for tens of milliseconds while the
 *  card does its own housekeeping, which would show up as a hitch in the gauge animation if
 *  it happened on the draw path. */
void gpsLogLoop();

/** Flush and close. Must be called before anything unmounts the card. */
void gpsLogStop();

bool gpsLogEnabled();
void gpsLogSetEnabled(bool on);

const GpsLogStatus *gpsLogStatus();
size_t gpsLogDiagJson(char *buf, size_t bufLen);
