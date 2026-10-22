#pragma once

#include <stddef.h>
#include <stdint.h>

struct MagStatus {
  bool ok;
  uint8_t addr;
  int8_t sda;
  int8_t scl;
  const char *chip;  // "QMC5883L" / "QMC5883P" / "HMC5883L" / "-"
  const char *phase; // "scan" / "ok" / "none"
  int16_t x;
  int16_t y;
  int16_t z;
  float headingDeg;  // 0..360, magnetic, not true north
  uint16_t yawDeg;   // 0/90/180/270, added after atan2 for mount rotation
  uint32_t reads;
  uint32_t fails;
};

void magInit();
void magLoop();
void magCycleYaw();
uint16_t magYawDeg();
const MagStatus *magStatus();
size_t magDiagJson(char *buf, size_t bufLen);
