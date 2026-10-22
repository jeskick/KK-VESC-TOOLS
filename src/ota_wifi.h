#pragma once

#include <stdint.h>

#ifndef FW_VERSION
#define FW_VERSION "1.0.0"
#endif

// SoftAP: SSID ESP32S3-OTA, pass 12345678, IP 192.168.4.1
// Auto-off if no client connects within 60s after boot.
void otaWifiInit();
void otaWifiLoop();
bool otaWifiConnected();
const char *otaWifiIp();
const char *otaWifiStatusText();

// Serial: {"cmd":"wifi_on"} / {"cmd":"wifi_off"} / {"cmd":"ota_status"}
bool otaWifiHandleCommand(const char *line);
