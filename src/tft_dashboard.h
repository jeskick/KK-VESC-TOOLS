#pragma once

#include <stdint.h>

struct DashboardInput {
  bool vescLinkOk;
  bool vescLinkBle;
  const char *vescLinkStatus;
  bool vescValid;
  float voltageV;
  float currentInA;
  float powerW;
  float speedKmh;
  float batteryPct;  // <0 if unknown — from VESC SETUP battery_level
  float tempFetC;
  float cutStartV;
  float cutEndV;
  float rpm;     // eRPM from VESC GET_VALUES
  float whUsed;  // Wh consumed (VESC coulomb/energy counters)
  float duty;    // 0..1
  int throttlePpm;
  int throttlePct;
  const char *throttleDir;
  uint8_t fault;
  uint32_t totalRunSec;   // cumulative linked time (NVS)
  uint32_t sessionSec;    // this link session, or last session if disconnected
  // Idle/boot test: 5-way key + GPS
  const char *keyLabel;   // "-", "UP", "DN", "LT", "RT", "OK"
  const char *keyRawLabel;  // "U1 D1 L1 R1 O1" (0=pressed)
  bool gpsTimeValid;
  uint8_t gpsHour;        // UTC
  uint8_t gpsMinute;
  uint8_t gpsSecond;
  uint8_t gpsSats;
  bool gpsSpeedValid;
  float gpsSpeedKmh;
  bool gpsFresh;          // NMEA within a few seconds
  bool gpsRawAlive;       // any UART bytes
  bool gpsRxAlive;        // framed header
  bool gpsNmeaAlive;      // parsed $NMEA lines
  bool gpsUbxAlive;
  bool gpsFixOk;
  bool gpsHeadValid;  // course over ground is only a bearing while actually moving
  float gpsHeadDeg;
  bool magOk;
  float magHeadDeg;
  int magSda;
  int magScl;
  int16_t magX;
  int16_t magY;
  int16_t magZ;
  const char *magChip;
  const char *magPhase;  // "scan" / "ok" / "none"
  bool gpsPosValid;
  const char *gpsLatStr;
  const char *gpsLonStr;
  uint32_t gpsBaud;
  int gpsRxPin;
  uint16_t gpsBytesPerSec;
  uint16_t gpsNmeaPerSec;
  const char *gpsLastTalker;
  const char *gpsRawPeek;
  const char *gpsHexSample;

  // 0=main 1=settings 2=trip 3=throttle cal 4=sd test
  uint8_t uiScreen;
  int8_t statsSel;  // settings / trip / cal step select
  uint8_t dataPage;  // main screen page: 0 VESC, 1 GPS (LEFT/RIGHT)
  bool forceData;   // show the data screen even without a VESC link
  const char *outModeLabel;  // "PWM" / "CPPM"
  uint8_t outThrottleCh;     // 1..4 when CPPM
  bool thrReverse;           // output mirrored about neutral
  uint16_t magYawDeg;        // 0/90/180/270 mount correction
  float tripMaxKmh;
  uint16_t tripMaxY;
  uint8_t tripMaxMo, tripMaxD, tripMaxH, tripMaxMi, tripMaxS;
  float tripDayKm;
  uint16_t tripDayY;
  uint8_t tripDayMo, tripDayD;
  float tripTotalKm;
  uint16_t tripTotY;
  uint8_t tripTotMo, tripTotD, tripTotH, tripTotMi, tripTotS;
  float tripSessionKm;
  uint32_t tripSessionSec;  // moving time since boot
  uint32_t tripTotalSec;    // moving time all-time
  float tripSessionAvgKmh;
  float tripTotalAvgKmh;

  // throttle cal live
  int thrLiveMv;
  int thrCalCenter;
  int thrCalFwd;
  int thrCalRev;
  bool thrCalReady;
  int thrOutPpm;
  int thrOutPct;
  const char *thrCalHint;  // wizard prompt / error

  // SD probe
  const char *sdStatus;  // "ok" / "no_card" / "error" / "none"
  const char *sdMsg;
  int sdClk;
  int sdCmd;
  int sdD0;

  // GPS track logging
  bool trackLogOn;
  const char *trackState;  // "off" / "wait fix" / "no card" / "error" / "rec"
  const char *trackFile;
  uint32_t trackPoints;
};

void tftDashboardInit();
void tftDashboardUpdate(const DashboardInput *in);
