#pragma once

#include <stddef.h>
#include <stdint.h>

struct GpsFix {
  bool valid;
  bool timeValid;
  bool dateValid;
  bool speedValid;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t satellites;
  float speedKmh;
  uint32_t ageMs;

  bool rawAlive;       // any UART bytes (even without decode)
  bool rxAlive;        // saw UBX/$G frame header
  bool nmeaAlive;
  bool ubxAlive;
  uint32_t baud;
  bool inverted;       // UART RX invert
  int8_t rxPin;
  int8_t txPin;
  uint16_t bytesPerSec;
  uint16_t nmeaPerSec;
  uint32_t totalBytes;
  uint32_t totalNmea;
  uint16_t dollarCount;  // '$' in last window / scan slot
  uint16_t ubxSyncCount; // B5 62 pairs in last window / scan slot
  int32_t score;         // active / best scan score
  char scanPhase[12];    // "scan" | "locked" | "listen"
  char lastTalker[8];
  char rawPeek[12];
  char hexSample[97];  // "AA BB .." up to 32 bytes
  float rawSpeedKmh;   // last instantaneous sample before hold filter
  float speedAccKmh;   // receiver speed accuracy estimate (0 = unknown, NMEA sources)
  uint32_t speedSrc;   // 1=RMC 2=VTG 3=PVT 4=POS
  uint32_t speedRejects;  // samples dropped by accuracy / spike gates

  // Position, kept as the receiver's raw 1e-7 degrees. A float cannot hold a degree value
  // at that resolution (~7 significant digits), so converting here would throw away metres.
  bool posValid;
  int32_t latE7;
  int32_t lonE7;
  int32_t altMmMsl;    // mm above mean sea level
  uint32_t accMm;      // horizontal accuracy estimate
  bool headValid;
  float headDeg;       // course over ground 0..360, only meaningful while moving
  float headAccDeg;
};

void gpsNmeaInit();
void gpsNmeaLoop();

const GpsFix *gpsNmeaFix();
size_t gpsNmeaDiagJson(char *buf, size_t bufLen);

/** Format 1e-7 degrees as plain decimal text. Kept next to the representation it belongs to
 *  so callers never have to route a coordinate through a float to print it. */
void gpsFormatE7(char *out, size_t n, int32_t e7);
