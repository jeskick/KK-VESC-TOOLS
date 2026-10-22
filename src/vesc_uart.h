#pragma once

#include "vesc_protocol.h"
#include "vesc_mcconf.h"

enum VescLinkMode : uint8_t { VESC_LINK_WIRE = 0, VESC_LINK_BLE = 1 };

void vescUartInit(VescLinkMode mode);
void vescUartSetLinkMode(VescLinkMode mode);
VescLinkMode vescUartLinkMode();
const char *vescUartLinkStatus();
void vescUartLoop();
bool vescUartConnected();
const VescValues *vescUartValues();
const VescSetupValues *vescUartSetupValues();
const char *vescUartFwVersion();
float vescUartDecodedPpm();
float vescErpmToKmh(float erpm);
bool vescUartHandleCommand(const char *line);

// Synchronous MCCONF read/write (pauses telemetry polling)
bool vescUartFetchMcconf(VescBatteryLimits *limits, char *err, size_t errLen);
bool vescUartApplyBatteryLimits(const VescBatteryPatch *patch, VescBatteryLimits *outLimits,
                                char *err, size_t errLen);

// Last known battery limits from MCCONF (for TFT voltage warn); may be invalid
const VescBatteryLimits *vescUartCachedBatteryLimits();
