#pragma once

#include <stddef.h>
#include <stdint.h>

void vescNowBegin();
void vescNowEnd();
void vescNowLoop();
// SoftAP came up or went down — keep ESP-NOW on the same channel.
void vescNowOnWifiChanged();
void vescNowKeepChannel();
bool vescNowWrite(const uint8_t *data, size_t len);
int vescNowAvailable();
int vescNowRead();
bool vescNowLinked();
// Latest ESP-NOW RSSI from the paired bridge. False if WIRE / unpaired / stale.
bool vescNowRssiOk();
int8_t vescNowRssi();
// "off" / "wait" / "ok"
const char *vescNowState();
const char *vescNowHint();
