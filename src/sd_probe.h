#pragma once

#include <stddef.h>
#include <stdint.h>

enum SdProbeStatus : uint8_t {
  SD_PROBE_NONE = 0,
  SD_PROBE_NO_CARD,   // mount failed — expected without card / wrong pins
  SD_PROBE_OK,        // card mounted
  SD_PROBE_ERROR,     // unexpected error
};

struct SdProbeResult {
  SdProbeStatus status;
  uint8_t cardType;  // 0 none, 1 MMC, 2 SDSC, 3 SDHC
  uint64_t sizeBytes;
  char msg[48];
  int clk;
  int cmd;
  int d0;
};

void sdProbeInit();
/** Re-run mount test. Safe with or without card. */
const SdProbeResult *sdProbeRun();
const SdProbeResult *sdProbeLast();
size_t sdProbeDiagJson(char *buf, size_t bufLen);
