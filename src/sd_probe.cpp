#include "sd_probe.h"

#include "pins.h"

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <string.h>

static SdProbeResult last = {};

void sdProbeInit() {
  last = {};
  last.status = SD_PROBE_NONE;
  last.clk = SD_MMC_CLK;
  last.cmd = SD_MMC_CMD;
  last.d0 = SD_MMC_D0;
  strncpy(last.msg, "not tested", sizeof(last.msg) - 1);
}

/** Describe the currently mounted card into `last`. Assumes a card is open. */
static void fillCardInfo() {
  const uint8_t t = SD_MMC.cardType();
  last.cardType = t;
  last.sizeBytes = SD_MMC.cardSize();
  last.status = SD_PROBE_OK;
  const char *tn = (t == CARD_MMC)    ? "MMC"
                   : (t == CARD_SD)   ? "SDSC"
                   : (t == CARD_SDHC) ? "SDHC"
                                      : "?";
  snprintf(last.msg, sizeof(last.msg), "OK %s %lluMB", tn,
           (unsigned long long)(last.sizeBytes / (1024ull * 1024ull)));
}

const SdProbeResult *sdProbeRun() {
  last = {};
  last.clk = SD_MMC_CLK;
  last.cmd = SD_MMC_CMD;
  last.d0 = SD_MMC_D0;

  // Already mounted by an earlier probe: report the live card and touch nothing else.
  // setPins refuses to run while a card is open — that, not the wiring, is what made a
  // retest report "setPins failed" — and unmounting to work around it would cut the track
  // logger's open file out from under it.
  if (SD_MMC.cardType() != CARD_NONE) {
    fillCardInfo();
    return &last;
  }

  // 1-bit SDMMC on common N16R8 onboard slot pins
  if (!SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0)) {
    last.status = SD_PROBE_ERROR;
    strncpy(last.msg, "setPins failed", sizeof(last.msg) - 1);
    Serial.printf("# SD setPins CLK=%d CMD=%d D0=%d FAIL\n", SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
    return &last;
  }

  // mode1bit=true, format_if_mount_failed=false
  const bool ok = SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT, 5);
  if (!ok) {
    last.status = SD_PROBE_NO_CARD;
    strncpy(last.msg, "mount fail (no card or bad pins)", sizeof(last.msg) - 1);
    Serial.printf("# SD mount FAIL CLK=%d CMD=%d D0=%d (no card is normal)\n", SD_MMC_CLK,
                  SD_MMC_CMD, SD_MMC_D0);
    SD_MMC.end();
    return &last;
  }

  if (SD_MMC.cardType() == CARD_NONE) {
    last.status = SD_PROBE_NO_CARD;
    strncpy(last.msg, "CARD_NONE", sizeof(last.msg) - 1);
    SD_MMC.end();
    return &last;
  }

  fillCardInfo();
  Serial.printf("# SD %s\n", last.msg);
  // Keep mounted if OK so later features can use it; end() on failure path only.
  return &last;
}

const SdProbeResult *sdProbeLast() {
  return &last;
}

size_t sdProbeDiagJson(char *buf, size_t bufLen) {
  if (buf == nullptr || bufLen < 32) {
    return 0;
  }
  const char *st = "none";
  if (last.status == SD_PROBE_NO_CARD) {
    st = "no_card";
  } else if (last.status == SD_PROBE_OK) {
    st = "ok";
  } else if (last.status == SD_PROBE_ERROR) {
    st = "error";
  }
  return (size_t)snprintf(
      buf, bufLen,
      "{\"ok\":true,\"status\":\"%s\",\"msg\":\"%s\",\"clk\":%d,\"cmd\":%d,\"d0\":%d,"
      "\"type\":%u,\"mb\":%llu}",
      st, last.msg[0] ? last.msg : "", last.clk, last.cmd, last.d0, (unsigned)last.cardType,
      (unsigned long long)(last.sizeBytes / (1024ull * 1024ull)));
}
