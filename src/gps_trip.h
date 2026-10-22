#pragma once

#include <stdint.h>

struct TripSnapshot {
  float maxSpeedKmh;
  uint16_t maxYear;
  uint8_t maxMonth;
  uint8_t maxDay;
  uint8_t maxHour;  // UTC+8 stored for display convenience
  uint8_t maxMinute;
  uint8_t maxSecond;

  float dayKm;
  uint16_t dayYear;
  uint8_t dayMonth;
  uint8_t dayDay;

  // Since boot. Moving time only (speed above the accumulate threshold), so the
  // average is a riding average and not diluted by standing still.
  float sessionKm;
  uint32_t sessionMoveSec;
  uint32_t totalMoveSec;

  float totalKm;
  uint16_t totalYear;  // last accumulate stamp (UTC+8 date/time)
  uint8_t totalMonth;
  uint8_t totalDay;
  uint8_t totalHour;
  uint8_t totalMinute;
  uint8_t totalSecond;
};

void tripInit();
void tripLoop();  // call after gpsNmeaLoop
void tripSaveNow();
const TripSnapshot *tripSnapshot();

/** km / h over moving time; 0 when there is not enough time logged to be meaningful. */
float tripAvgKmh(float km, uint32_t moveSec);
