#include "live_status.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

static int gPpm = 1500;
static int gPct = 0;
static float gMv = 0;
static char gDir[8] = "STOP";
static char gMode[8] = "PWM";

static bool gVescOk = false;
static char gFw[24] = "";
static float gVin = 0, gIin = 0, gImot = 0, gRpm = 0, gDuty = 0, gTfet = 0, gTmot = 0;
static uint8_t gFault = 0;

void liveStatusSetThrottle(int ppm, int pct, float mv, const char *dir, const char *mode) {
  gPpm = ppm;
  gPct = pct;
  gMv = mv;
  snprintf(gDir, sizeof(gDir), "%s", dir ? dir : "STOP");
  snprintf(gMode, sizeof(gMode), "%s", mode ? mode : "PWM");
}

void liveStatusSetVesc(bool connected, const char *fw, float vIn, float iIn, float iMotor, float rpm,
                       float duty, float tFet, float tMotor, uint8_t fault) {
  gVescOk = connected;
  if (fw != nullptr && fw[0]) {
    snprintf(gFw, sizeof(gFw), "%s", fw);
  }
  gVin = vIn;
  gIin = iIn;
  gImot = iMotor;
  gRpm = rpm;
  gDuty = duty;
  gTfet = tFet;
  gTmot = tMotor;
  gFault = fault;
}

size_t liveStatusToJson(char *buf, size_t bufLen) {
  if (buf == nullptr || bufLen < 32) {
    return 0;
  }
  return (size_t)snprintf(
      buf, bufLen,
      "{\"type\":\"live\",\"vesc\":%s,\"fw\":\"%s\","
      "\"v_in\":%.2f,\"i_in\":%.2f,\"i_motor\":%.2f,\"rpm\":%.0f,\"duty\":%.3f,"
      "\"t_fet\":%.1f,\"t_motor\":%.1f,\"fault\":%u,"
      "\"ppm\":%d,\"pct\":%d,\"mv\":%.1f,\"dir\":\"%s\",\"mode\":\"%s\"}",
      gVescOk ? "true" : "false", gFw[0] ? gFw : "?", gVin, gIin, gImot, gRpm, gDuty, gTfet, gTmot,
      (unsigned)gFault, gPpm, gPct, gMv, gDir, gMode);
}
