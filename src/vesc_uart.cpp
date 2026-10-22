#include "vesc_uart.h"

#include "pins.h"
#include "vesc_ble.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <driver/gpio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static HardwareSerial VescSerial(2);

static VescValues lastValues = {};
static VescSetupValues lastSetup = {};
static char fwVersion[24] = "";
static uint8_t rxPayload[512];
static size_t rxPayloadLen = 0;
static float decodedPpmUs = -1.0f;

static uint32_t lastInfoMs = 0;
static uint32_t lastRxMs = 0;
static uint32_t lastPollMs = 0;
static uint32_t lastFwMs = 0;
static uint8_t pollPhase = 0;
static bool pollPaused = false;

static uint8_t syncExpectCmd = 0;
static uint8_t syncBuf[512];
static size_t syncLen = 0;
static bool syncGot = false;

static uint8_t mcconfBlob[512];
static size_t mcconfBlobLen = 0;
static VescBatteryLimits cachedLimits = {};
static bool limitsFetchTried = false;
static uint32_t limitsFetchAtMs = 0;

static VescLinkMode linkMode = VESC_LINK_WIRE;
static bool linkStarted = false;

static uint32_t linkTimeoutMs() {
  return (linkMode == VESC_LINK_BLE) ? 4000u : 2500u;
}

static uint32_t syncTimeoutMs() {
  return (linkMode == VESC_LINK_BLE) ? 4000u : 2500u;
}

static const char *linkTag() {
  return (linkMode == VESC_LINK_BLE) ? "ble" : "uart";
}

static size_t linkWrite(const uint8_t *data, size_t len) {
  if (linkMode == VESC_LINK_BLE) {
    return vescBleWrite(data, len) ? len : 0;
  }
  return VescSerial.write(data, len);
}

static int linkAvailable() {
  if (linkMode == VESC_LINK_BLE) {
    return vescBleAvailable();
  }
  return VescSerial.available();
}

static int linkRead() {
  if (linkMode == VESC_LINK_BLE) {
    return vescBleRead();
  }
  return VescSerial.read();
}

static void sendPacket(uint8_t cmd, const uint8_t *data, size_t dataLen) {
  uint8_t frame[520];
  const size_t n = vescBuildPacket(cmd, data, dataLen, frame, sizeof(frame));
  if (n > 0) {
    linkWrite(frame, n);
  }
}

static void sendCommand(uint8_t cmd) {
  sendPacket(cmd, nullptr, 0);
}

static void handlePayload(const uint8_t *payload, size_t len) {
  if (len < 1) {
    return;
  }
  lastRxMs = millis();

  if (syncExpectCmd != 0 && payload[0] == syncExpectCmd) {
    if (len <= sizeof(syncBuf)) {
      memcpy(syncBuf, payload, len);
      syncLen = len;
      syncGot = true;
    }
    return;
  }

  if (payload[0] == COMM_GET_VALUES) {
    vescParseGetValues(payload, len, &lastValues);
    return;
  }
  if (payload[0] == COMM_GET_VALUES_SETUP) {
    vescParseGetValuesSetup(payload, len, &lastSetup);
    return;
  }
  if (payload[0] == COMM_GET_DECODED_PPM) {
    float ppm = 0.0f;
    if (vescParseDecodedPpm(payload, len, &ppm)) {
      decodedPpmUs = ppm;
    }
    return;
  }
  if (payload[0] == COMM_FW_VERSION && len >= 3) {
    snprintf(fwVersion, sizeof(fwVersion), "%u.%u.%u", payload[1], payload[2],
             len >= 4 ? payload[3] : 0);
    Serial.printf("# vesc FW %s Vin=%.1f\n", fwVersion,
                  lastValues.valid ? lastValues.voltageInV : 0.0f);
  }
}

static void pollRx() {
  while (linkAvailable() > 0) {
    const int v = linkRead();
    if (v < 0) {
      break;
    }
    if (vescPacketFeed((uint8_t)v, rxPayload, sizeof(rxPayload), &rxPayloadLen)) {
      handlePayload(rxPayload, rxPayloadLen);
    }
  }
}

static void drainRx() {
  while (linkAvailable() > 0) {
    linkRead();
  }
  vescPacketParserReset();
}

static bool awaitCmd(uint8_t cmd, uint32_t timeoutMs) {
  syncExpectCmd = cmd;
  syncGot = false;
  syncLen = 0;
  const uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    pollRx();
    if (syncGot) {
      syncExpectCmd = 0;
      return true;
    }
    delay(1);
  }
  syncExpectCmd = 0;
  return false;
}

static void setErr(char *err, size_t errLen, const char *msg) {
  if (err != nullptr && errLen > 0) {
    snprintf(err, errLen, "%s", msg);
  }
}

static bool fetchMcconfInternal(VescBatteryLimits *limits, char *err, size_t errLen) {
  pollPaused = true;
  delay(30);
  drainRx();

  sendCommand(COMM_GET_MCCONF);
  if (!awaitCmd(COMM_GET_MCCONF, syncTimeoutMs())) {
    pollPaused = false;
    setErr(err, errLen, "GET_MCCONF timeout");
    return false;
  }

  if (syncLen < 5) {
    pollPaused = false;
    setErr(err, errLen, "GET_MCCONF short");
    return false;
  }

  const size_t confLen = syncLen - 1;
  if (confLen > sizeof(mcconfBlob)) {
    pollPaused = false;
    setErr(err, errLen, "GET_MCCONF too large");
    return false;
  }
  memcpy(mcconfBlob, &syncBuf[1], confLen);
  mcconfBlobLen = confLen;

  if (!vescMcconfParseLimits(mcconfBlob, mcconfBlobLen, limits) || !limits->valid) {
    pollPaused = false;
    char tmp[64];
    snprintf(tmp, sizeof(tmp), "unsupported signature %lu", (unsigned long)limits->signature);
    setErr(err, errLen, tmp);
    return false;
  }

  cachedLimits = *limits;
  pollPaused = false;
  if (err != nullptr && errLen > 0) {
    err[0] = '\0';
  }
  return true;
}

const VescBatteryLimits *vescUartCachedBatteryLimits() {
  return cachedLimits.valid ? &cachedLimits : nullptr;
}

bool vescUartFetchMcconf(VescBatteryLimits *limits, char *err, size_t errLen) {
  if (limits == nullptr) {
    setErr(err, errLen, "null limits");
    return false;
  }
  return fetchMcconfInternal(limits, err, errLen);
}

bool vescUartApplyBatteryLimits(const VescBatteryPatch *patch, VescBatteryLimits *outLimits,
                                char *err, size_t errLen) {
  if (patch == nullptr) {
    setErr(err, errLen, "null patch");
    return false;
  }

  VescBatteryLimits cur = {};
  if (!fetchMcconfInternal(&cur, err, errLen)) {
    return false;
  }

  if (!vescMcconfPatchLimits(mcconfBlob, mcconfBlobLen, patch)) {
    setErr(err, errLen, "patch failed (signature/layout)");
    return false;
  }

  pollPaused = true;
  delay(20);
  drainRx();
  sendPacket(COMM_SET_MCCONF, mcconfBlob, mcconfBlobLen);
  // VESC typically replies with COMM_SET_MCCONF (cmd only)
  if (!awaitCmd(COMM_SET_MCCONF, syncTimeoutMs())) {
    // Some firmwares may not ACK; continue to verify via GET
    Serial.println("# vesc SET_MCCONF no ACK, verifying via GET");
  }

  VescBatteryLimits verify = {};
  if (!fetchMcconfInternal(&verify, err, errLen)) {
    return false;
  }
  if (outLimits != nullptr) {
    *outLimits = verify;
  }
  return true;
}

static void printMcconfJson(const VescBatteryLimits *l, bool ok, const char *err) {
  if (!ok) {
    Serial.printf("{\"type\":\"mcconf\",\"ok\":false,\"error\":\"%s\"}\n",
                  err != nullptr ? err : "fail");
    return;
  }
  Serial.printf(
      "{\"type\":\"mcconf\",\"ok\":true,\"sig\":%lu,\"battery_type\":%u,\"cells\":%u,\"battery_ah\":%.2f,"
      "\"cut_start\":%.2f,\"cut_end\":%.2f,\"min_vin\":%.2f,\"max_vin\":%.2f,"
      "\"i_in_max\":%.2f,\"motor_max\":%.2f,\"motor_min\":%.2f,\"watt_max\":%.1f,\"invert\":%u}\n",
      (unsigned long)l->signature, (unsigned)l->batteryType, (unsigned)l->batteryCells, l->batteryAh, l->cutStartV,
      l->cutEndV, l->minVinV, l->maxVinV, l->inCurrentMaxA, l->motorCurrentMaxA, l->motorCurrentMinA, l->wattMaxW,
      (unsigned)l->invertDirection);
}

static bool jsonHasKey(const char *line, const char *key) {
  char pat[48];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  return strstr(line, pat) != nullptr;
}

static bool jsonGetFloat(const char *line, const char *key, float *out) {
  char pat[48];
  snprintf(pat, sizeof(pat), "\"%s\":", key);
  const char *p = strstr(line, pat);
  if (p == nullptr) {
    snprintf(pat, sizeof(pat), "\"%s\": ", key);
    p = strstr(line, pat);
  }
  if (p == nullptr) {
    return false;
  }
  p = strchr(p, ':');
  if (p == nullptr) {
    return false;
  }
  p++;
  while (*p == ' ' || *p == '\t') {
    p++;
  }
  char *end = nullptr;
  const float v = strtof(p, &end);
  if (end == p) {
    return false;
  }
  *out = v;
  return true;
}

static bool jsonGetU8(const char *line, const char *key, uint8_t *out) {
  float v = 0;
  if (!jsonGetFloat(line, key, &v)) {
    return false;
  }
  if (v < 0 || v > 255) {
    return false;
  }
  *out = (uint8_t)(v + 0.5f);
  return true;
}

static void beginUart() {
  VescSerial.end();
  delay(5);
  gpio_reset_pin((gpio_num_t)VESC_UART_RX);
  gpio_reset_pin((gpio_num_t)VESC_UART_TX);
  VescSerial.setRxBufferSize(1024);
  VescSerial.setTxBufferSize(1024);
  VescSerial.begin(VESC_UART_BAUD, SERIAL_8N1, VESC_UART_RX, VESC_UART_TX);
  vescPacketParserReset();
  lastRxMs = 0;
  lastValues = {};
  lastSetup = {};
  mcconfBlobLen = 0;
  cachedLimits = {};
  limitsFetchTried = false;
  Serial.printf("# VESC UART TX=GPIO%d RX=GPIO%d @ %d\n", VESC_UART_TX, VESC_UART_RX,
                VESC_UART_BAUD);
  delay(20);
  sendCommand(COMM_FW_VERSION);
}

static void applyLink() {
  lastRxMs = 0;
  lastValues = {};
  lastSetup = {};
  mcconfBlobLen = 0;
  cachedLimits = {};
  limitsFetchTried = false;
  fwVersion[0] = '\0';
  vescPacketParserReset();

  if (linkMode == VESC_LINK_BLE) {
    VescSerial.end();
    vescBleBegin();
    Serial.printf("# VESC link BLE → %s\n", VESC_BLE_NAME);
  } else {
    vescBleEnd();
    beginUart();
  }
  linkStarted = true;
}

void vescUartInit(VescLinkMode mode) {
  linkMode = mode;
  applyLink();
}

void vescUartSetLinkMode(VescLinkMode mode) {
  if (linkStarted && mode == linkMode) {
    return;
  }
  linkMode = mode;
  applyLink();
}

VescLinkMode vescUartLinkMode() {
  return linkMode;
}

const char *vescUartLinkStatus() {
  if (linkMode == VESC_LINK_WIRE) {
    return vescUartConnected() ? "有线 已连接" : "有线 UART";
  }
  const char *st = vescBleState();
  if (strcmp(st, "ok") == 0) {
    return vescUartConnected() ? "无线 已连接" : "无线 等待电调";
  }
  if (strcmp(st, "scan") == 0) {
    return "无线 扫描 KK_VESC_BLE";
  }
  if (strcmp(st, "conn") == 0) {
    return "无线 正在连接";
  }
  if (strcmp(st, "fail") == 0) {
    return "无线 未找到";
  }
  return "无线 BLE";
}

void vescUartLoop() {
  if (linkMode == VESC_LINK_BLE) {
    vescBleLoop();
  }
  pollRx();
  const uint32_t now = millis();

  if (!pollPaused && now - lastPollMs >= 200) {
    lastPollMs = now;
    switch (pollPhase++ % 5) {
    case 0:
    case 2:
    case 4:
      sendCommand(COMM_GET_VALUES);
      break;
    case 1:
      sendCommand(COMM_GET_VALUES_SETUP);
      break;
    case 3:
      sendCommand(COMM_GET_DECODED_PPM);
      break;
    }
  }

  if (!pollPaused && fwVersion[0] == '\0' && now - lastFwMs >= 5000) {
    lastFwMs = now;
    sendCommand(COMM_FW_VERSION);
  }

  // One-shot MCCONF cache for TFT voltage thresholds (after link is stable)
  if (!pollPaused && !cachedLimits.valid && !limitsFetchTried && vescUartConnected() &&
      lastValues.valid && now - lastRxMs < 500) {
    if (limitsFetchAtMs == 0) {
      limitsFetchAtMs = now;
    } else if (now - limitsFetchAtMs >= 1500) {
      limitsFetchTried = true;
      VescBatteryLimits lim = {};
      char err[64];
      if (fetchMcconfInternal(&lim, err, sizeof(err))) {
        Serial.printf("# mcconf cache cells=%u cut=%.1f/%.1f\n", (unsigned)lim.batteryCells,
                      lim.cutStartV, lim.cutEndV);
      } else {
        Serial.printf("# mcconf cache fail: %s\n", err);
      }
    }
  }
  if (!vescUartConnected()) {
    limitsFetchTried = false;
    limitsFetchAtMs = 0;
  }

  pollRx();

  if (pollPaused || now - lastInfoMs < 400) {
    return;
  }
  lastInfoMs = now;

  if (!vescUartConnected() || !lastValues.valid) {
    Serial.printf("{\"type\":\"vesc\",\"connected\":false,\"link\":\"%s\"}\n", linkTag());
    return;
  }

  const float speedKmh =
      lastSetup.valid ? (lastSetup.speedMps * 3.6f) : vescErpmToKmh(lastValues.rpm);
  const float battPct = lastSetup.valid ? (lastSetup.batteryLevel * 100.0f) : -1.0f;

  Serial.printf(
      "{\"type\":\"vesc\",\"connected\":true,\"link\":\"%s\",\"fw\":\"%s\","
      "\"v_in\":%.1f,\"i_in\":%.2f,\"i_motor\":%.2f,\"rpm\":%.0f,\"speed\":%.1f,"
      "\"duty\":%.3f,\"t_fet\":%.1f,\"t_motor\":%.1f,\"wh\":%.2f,\"ah\":%.3f,"
      "\"batt_pct\":%.0f,\"fault\":%u}\n",
      linkTag(), fwVersion[0] ? fwVersion : "?", lastValues.voltageInV, lastValues.currentInA,
      lastValues.currentMotorA, lastValues.rpm, speedKmh, lastValues.duty, lastValues.tempFetC,
      lastValues.tempMotorC, lastValues.wattHours, lastValues.ampHours, battPct,
      (unsigned)lastValues.fault);
}

bool vescUartConnected() {
  return lastRxMs != 0 && (millis() - lastRxMs) < linkTimeoutMs();
}

const VescValues *vescUartValues() {
  return &lastValues;
}

const VescSetupValues *vescUartSetupValues() {
  return &lastSetup;
}

const char *vescUartFwVersion() {
  return fwVersion;
}

float vescUartDecodedPpm() {
  return decodedPpmUs;
}

float vescErpmToKmh(float erpm) {
  if (fabsf(erpm) < 1.0f) {
    return 0.0f;
  }
  const float mechRpm = erpm / (float)MOTOR_POLE_PAIRS;
  return mechRpm * WHEEL_DIAMETER_M * 3.14159265f / 60.0f * 3.6f;
}

bool vescUartHandleCommand(const char *line) {
  if (line == nullptr) {
    return false;
  }
  if (strstr(line, "uart_reconnect") != nullptr) {
    applyLink();
    return true;
  }

  const bool isGet = strstr(line, "\"cmd\":\"mcconf_get\"") != nullptr ||
                     strstr(line, "\"cmd\": \"mcconf_get\"") != nullptr;
  const bool isSet = strstr(line, "\"cmd\":\"mcconf_set\"") != nullptr ||
                     strstr(line, "\"cmd\": \"mcconf_set\"") != nullptr;
  if (!isGet && !isSet) {
    return false;
  }

  char err[80] = "";
  VescBatteryLimits lim = {};

  if (isGet) {
    const bool ok = vescUartFetchMcconf(&lim, err, sizeof(err));
    printMcconfJson(&lim, ok, err);
    return true;
  }

  VescBatteryPatch patch = {};
  float f = 0;
  uint8_t u8 = 0;
  if (jsonGetU8(line, "battery_type", &u8) && u8 <= 2) {
    patch.setBatteryType = true;
    patch.batteryType = u8;
  }
  if (jsonGetU8(line, "cells", &u8)) {
    patch.setCells = true;
    patch.batteryCells = u8;
  }
  if (jsonGetFloat(line, "cut_start", &f)) {
    patch.setCutStart = true;
    patch.cutStartV = f;
  }
  if (jsonGetFloat(line, "cut_end", &f)) {
    patch.setCutEnd = true;
    patch.cutEndV = f;
  }
  if (jsonGetFloat(line, "min_vin", &f)) {
    patch.setMinVin = true;
    patch.minVinV = f;
  }
  if (jsonGetFloat(line, "max_vin", &f)) {
    patch.setMaxVin = true;
    patch.maxVinV = f;
  }
  if (jsonGetFloat(line, "i_in_max", &f)) {
    patch.setInCurrentMax = true;
    patch.inCurrentMaxA = f;
  }
  if (jsonGetFloat(line, "watt_max", &f)) {
    patch.setWattMax = true;
    patch.wattMaxW = f;
  }
  if (jsonGetFloat(line, "motor_max", &f)) {
    patch.setMotorCurrentMax = true;
    patch.motorCurrentMaxA = f;
    if (!jsonHasKey(line, "motor_min")) {
      patch.setMotorCurrentMin = true;
      patch.motorCurrentMinA = (f > 0.0f) ? -fminf(40.0f, f * 0.5f) : -20.0f;
    }
  }
  if (jsonGetFloat(line, "motor_min", &f)) {
    patch.setMotorCurrentMin = true;
    patch.motorCurrentMinA = (f > 0.0f) ? -f : f;
  }
  if (jsonGetU8(line, "invert", &u8)) {
    patch.setInvert = true;
    patch.invertDirection = u8 ? 1 : 0;
  }
  if (jsonGetFloat(line, "battery_ah", &f)) {
    patch.setBatteryAh = true;
    patch.batteryAh = f;
  }

  if (!(patch.setCells || patch.setCutStart || patch.setCutEnd || patch.setMinVin ||
        patch.setMaxVin || patch.setInCurrentMax || patch.setWattMax || patch.setMotorCurrentMax ||
        patch.setBatteryAh || patch.setBatteryType || patch.setInvert || patch.setMotorCurrentMin)) {
    printMcconfJson(nullptr, false, "no fields");
    return true;
  }

  if (patch.setCells && jsonHasKey(line, "auto_cuts") &&
      (strstr(line, "\"auto_cuts\":true") != nullptr ||
       strstr(line, "\"auto_cuts\": true") != nullptr ||
       strstr(line, "\"auto_cuts\":1") != nullptr)) {
    if (!patch.setCutStart) {
      patch.setCutStart = true;
      patch.cutStartV = patch.batteryCells * 3.5f;
    }
    if (!patch.setCutEnd) {
      patch.setCutEnd = true;
      patch.cutEndV = patch.batteryCells * 3.3f;
    }
    if (!patch.setMinVin) {
      patch.setMinVin = true;
      patch.minVinV = patch.batteryCells * 3.0f;
    }
    if (!patch.setMaxVin) {
      patch.setMaxVin = true;
      patch.maxVinV = patch.batteryCells * 4.3f;
    }
  }

  const bool ok = vescUartApplyBatteryLimits(&patch, &lim, err, sizeof(err));
  printMcconfJson(&lim, ok, err);
  return true;
}
