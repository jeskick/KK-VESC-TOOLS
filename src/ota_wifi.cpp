#include "ota_wifi.h"

#include "live_status.h"
#include "vesc_uart.h"
#include "gps_log.h"
#include "gps_nmea.h"
#include "sd_probe.h"
#include "qmc5883.h"
#include "web_page.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <SD_MMC.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static constexpr const char *kApSsid = "ESP32S3-OTA";
static constexpr const char *kApPass = "12345678";
static constexpr uint32_t kApIdleTimeoutMs = 600000;  // 10 min while debugging GPS

static WebServer otaServer(80);
static bool apActive = false;
static bool otaReady = false;
static bool clientSeen = false;
static bool wifiClosed = false;
static uint32_t apStartMs = 0;
static char statusText[96] = "WiFi off";
static char ipCache[16] = "192.168.4.1";

static void setStatus(const char *text) {
  snprintf(statusText, sizeof(statusText), "%s", text);
  Serial.printf("# ota: %s\n", text);
}

static void handleInfo() {
  char buf[192];
  snprintf(buf, sizeof(buf),
           "{\"fw\":\"%s\",\"ip\":\"%s\",\"heap\":%u,\"wifi\":%s,\"ota\":%s,\"clients\":%u}",
           FW_VERSION, ipCache, (unsigned)ESP.getFreeHeap(), apActive ? "true" : "false",
           otaReady ? "true" : "false", (unsigned)WiFi.softAPgetStationNum());
  otaServer.send(200, "application/json", buf);
}

static void handleLive() {
  char buf[384];
  liveStatusToJson(buf, sizeof(buf));
  otaServer.send(200, "application/json", buf);
}

static void handleGpsDiag() {
  char buf[512];
  gpsNmeaDiagJson(buf, sizeof(buf));
  otaServer.send(200, "application/json", buf);
}

static void handleMagDiag() {
  char buf[256];
  magDiagJson(buf, sizeof(buf));
  otaServer.send(200, "application/json", buf);
}

static void handleSdDiag() {
  // Fresh probe each request
  sdProbeRun();
  char buf[256];
  sdProbeDiagJson(buf, sizeof(buf));
  otaServer.send(200, "application/json", buf);
}

static void sendMcconfJson(bool ok, const VescBatteryLimits *l, const char *err) {
  char buf[560];
  if (!ok || l == nullptr) {
    snprintf(buf, sizeof(buf), "{\"ok\":false,\"error\":\"%s\"}", err != nullptr ? err : "fail");
    otaServer.send(200, "application/json", buf);
    return;
  }
  snprintf(buf, sizeof(buf),
           "{\"ok\":true,\"sig\":%lu,\"battery_type\":%u,\"cells\":%u,\"battery_ah\":%.2f,"
           "\"cut_start\":%.2f,\"cut_end\":%.2f,\"min_vin\":%.2f,\"max_vin\":%.2f,"
           "\"i_in_max\":%.2f,\"motor_max\":%.2f,\"motor_min\":%.2f,\"watt_max\":%.1f,\"invert\":%u}",
           (unsigned long)l->signature, (unsigned)l->batteryType, (unsigned)l->batteryCells, l->batteryAh, l->cutStartV,
           l->cutEndV, l->minVinV, l->maxVinV, l->inCurrentMaxA, l->motorCurrentMaxA, l->motorCurrentMinA, l->wattMaxW,
           (unsigned)l->invertDirection);
  otaServer.send(200, "application/json", buf);
}

static void handleMcconfGet() {
  char err[80] = "";
  VescBatteryLimits lim = {};
  const bool ok = vescUartFetchMcconf(&lim, err, sizeof(err));
  sendMcconfJson(ok, &lim, err);
}

static bool bodyGetFloat(const String &body, const char *key, float *out) {
  char pat[48];
  snprintf(pat, sizeof(pat), "\"%s\":", key);
  int idx = body.indexOf(pat);
  if (idx < 0) {
    return false;
  }
  idx += (int)strlen(pat);
  while (idx < (int)body.length() && (body[idx] == ' ' || body[idx] == '\t')) {
    idx++;
  }
  *out = body.substring(idx).toFloat();
  return true;
}

static void handleMcconfPost() {
  if (!otaServer.hasArg("plain")) {
    sendMcconfJson(false, nullptr, "missing body");
    return;
  }
  const String body = otaServer.arg("plain");
  VescBatteryPatch patch = {};
  float f = 0;

  if (bodyGetFloat(body, "battery_type", &f)) {
    const uint8_t t = (uint8_t)(f + 0.5f);
    if (t <= 2) {
      patch.setBatteryType = true;
      patch.batteryType = t;
    }
  }
  if (bodyGetFloat(body, "cells", &f)) {
    patch.setCells = true;
    patch.batteryCells = (uint8_t)(f + 0.5f);
  }
  if (bodyGetFloat(body, "cut_start", &f)) {
    patch.setCutStart = true;
    patch.cutStartV = f;
  }
  if (bodyGetFloat(body, "cut_end", &f)) {
    patch.setCutEnd = true;
    patch.cutEndV = f;
  }
  if (bodyGetFloat(body, "min_vin", &f)) {
    patch.setMinVin = true;
    patch.minVinV = f;
  }
  if (bodyGetFloat(body, "max_vin", &f)) {
    patch.setMaxVin = true;
    patch.maxVinV = f;
  }
  if (bodyGetFloat(body, "i_in_max", &f)) {
    patch.setInCurrentMax = true;
    patch.inCurrentMaxA = f;
  }
  if (bodyGetFloat(body, "motor_max", &f)) {
    patch.setMotorCurrentMax = true;
    patch.motorCurrentMaxA = f;
  }
  if (bodyGetFloat(body, "motor_min", &f)) {
    patch.setMotorCurrentMin = true;
    patch.motorCurrentMinA = (f > 0.0f) ? -f : f;
  } else if (patch.setMotorCurrentMax) {
    patch.setMotorCurrentMin = true;
    const float mx = patch.motorCurrentMaxA;
    patch.motorCurrentMinA = (mx > 0.0f) ? -fminf(40.0f, mx * 0.5f) : -20.0f;
  }
  if (bodyGetFloat(body, "invert", &f)) {
    patch.setInvert = true;
    patch.invertDirection = (f > 0.5f) ? 1 : 0;
  }
  if (bodyGetFloat(body, "watt_max", &f)) {
    patch.setWattMax = true;
    patch.wattMaxW = f;
  }
  if (bodyGetFloat(body, "battery_ah", &f)) {
    patch.setBatteryAh = true;
    patch.batteryAh = f;
  }

  if (!(patch.setCells || patch.setCutStart || patch.setCutEnd || patch.setMinVin ||
        patch.setMaxVin || patch.setInCurrentMax || patch.setWattMax || patch.setMotorCurrentMax ||
        patch.setBatteryAh || patch.setBatteryType || patch.setInvert || patch.setMotorCurrentMin)) {
    sendMcconfJson(false, nullptr, "no fields");
    return;
  }

  char err[80] = "";
  VescBatteryLimits lim = {};
  const bool ok = vescUartApplyBatteryLimits(&patch, &lim, err, sizeof(err));
  sendMcconfJson(ok, &lim, err);
}

static void handleTrackDiag() {
  char buf[224];
  gpsLogDiagJson(buf, sizeof(buf));
  otaServer.send(200, "application/json", buf);
}

/** The name lands in a filesystem path, so accept only plain basenames. Without this an
 *  `f=../../something` would read straight out of the tracks directory. */
static bool safeTrackName(const String &n) {
  if (n.length() == 0 || n.length() > 24 || n.indexOf("..") >= 0) {
    return false;
  }
  for (unsigned i = 0; i < n.length(); i++) {
    const char c = n[i];
    const bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                    c == '_' || c == '.' || c == '-';
    if (!ok) {
      return false;
    }
  }
  return true;
}

/** Basename of whatever the FS handed back — name() is a full path on some core versions. */
static const char *baseName(const char *p) {
  const char *slash = strrchr(p, '/');
  return (slash != nullptr) ? slash + 1 : p;
}

static bool openTrack(fs::File *out, String *name) {
  *name = otaServer.arg("f");
  if (!safeTrackName(*name)) {
    otaServer.send(400, "text/plain", "bad name");
    return false;
  }
  char path[48];
  snprintf(path, sizeof(path), "/tracks/%s", name->c_str());
  *out = SD_MMC.open(path, FILE_READ);
  if (!*out) {
    otaServer.send(404, "text/plain", "not found");
    return false;
  }
  return true;
}

static void handleTrackList() {
  String out;
  out.reserve(1024);
  out += F("{\"ok\":true,\"tracks\":[");
  fs::File dir = SD_MMC.open("/tracks");
  bool first = true;
  if (dir && dir.isDirectory()) {
    fs::File e = dir.openNextFile();
    while (e) {
      if (!e.isDirectory()) {
        if (!first) {
          out += ',';
        }
        first = false;
        out += F("{\"name\":\"");
        out += baseName(e.name());
        out += F("\",\"bytes\":");
        out += (uint32_t)e.size();
        out += '}';
      }
      e = dir.openNextFile();
    }
  }
  out += F("]}");
  otaServer.send(200, "application/json", out);
}

static void handleTrackPage() {
  String out;
  out.reserve(2048);
  out += F("<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
           "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
           "<title>Tracks</title><style>body{font-family:sans-serif;background:#111;color:#eee;"
           "margin:16px}a{color:#4ea}li{margin:6px 0}</style></head><body><h2>GPS tracks</h2><ul>");
  fs::File dir = SD_MMC.open("/tracks");
  bool any = false;
  if (dir && dir.isDirectory()) {
    fs::File e = dir.openNextFile();
    while (e) {
      if (!e.isDirectory()) {
        any = true;
        const String n = baseName(e.name());
        out += F("<li>");
        out += n;
        out += F(" <small>");
        out += (uint32_t)(e.size() / 1024);
        out += F(" KB</small> &nbsp;<a href=\"/track.gpx?f=");
        out += n;
        out += F("\">GPX</a> &nbsp;<a href=\"/track.csv?f=");
        out += n;
        out += F("\">CSV</a></li>");
      }
      e = dir.openNextFile();
    }
  }
  if (!any) {
    out += F("<li>none yet</li>");
  }
  out += F("</ul><p><a href=\"/\">back</a></p></body></html>");
  otaServer.send(200, "text/html", out);
}

static void handleTrackCsv() {
  fs::File f;
  String name;
  if (!openTrack(&f, &name)) {
    return;
  }
  otaServer.sendHeader("Content-Disposition", "attachment; filename=\"" + name + "\"");
  otaServer.streamFile(f, "text/csv");
  f.close();
}

/** CSV to GPX on the fly. A track can be megabytes, so it is streamed in chunks rather than
 *  assembled in RAM, and the length is unknown up front. */
static void handleTrackGpx() {
  fs::File f;
  String name;
  if (!openTrack(&f, &name)) {
    return;
  }

  otaServer.sendHeader("Content-Disposition", "attachment; filename=\"" + name + ".gpx\"");
  otaServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  otaServer.send(200, "application/gpx+xml", "");

  String out;
  out.reserve(1600);
  out += F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<gpx version=\"1.1\" "
           "creator=\"ESP32S3 Dash\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n<trk><name>");
  out += name;
  out += F("</name><trkseg>\n");

  char line[160];
  while (f.available()) {
    const size_t n = f.readBytesUntil('\n', line, sizeof(line) - 1);
    if (n == 0) {
      continue;
    }
    line[n] = '\0';
    if (line[0] < '0' || line[0] > '9') {
      continue;  // the header row, or a partial line from a power cut
    }
    const char *utc = strtok(line, ",");
    const char *lat = strtok(nullptr, ",");
    const char *lon = strtok(nullptr, ",");
    const char *alt = strtok(nullptr, ",");
    if (utc == nullptr || lat == nullptr || lon == nullptr) {
      continue;
    }
    out += F("<trkpt lat=\"");
    out += lat;
    out += F("\" lon=\"");
    out += lon;
    out += F("\">");
    if (alt != nullptr) {
      out += F("<ele>");
      out += alt;
      out += F("</ele>");
    }
    out += F("<time>");
    out += utc;
    out += F("</time></trkpt>\n");

    if (out.length() > 1200) {
      otaServer.sendContent(out);
      out.remove(0);  // keeps the reserved capacity
    }
  }

  out += F("</trkseg></trk></gpx>\n");
  otaServer.sendContent(out);
  otaServer.sendContent("");  // terminates the chunked response
  f.close();
}

static void handleRoot() {
  otaServer.send(200, "text/html", kHtmlPage);
}

static void handleUpdateDone() {
  if (Update.hasError()) {
    otaServer.send(500, "text/plain", "Update failed");
    setStatus("update failed");
    return;
  }
  otaServer.send(200, "text/plain", "OK, rebooting...");
  setStatus("update ok, reboot");
  delay(500);
  ESP.restart();
}

static void handleUpdateData() {
  HTTPUpload &upload = otaServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    setStatus("update start");
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) {
      Update.printError(Serial);
    } else {
      Serial.printf("# ota: wrote %u bytes\n", (unsigned)upload.totalSize);
    }
  }
}

static void startServices() {
  if (otaReady) {
    return;
  }
  otaServer.on("/", HTTP_GET, handleRoot);
  otaServer.on("/info", HTTP_GET, handleInfo);
  otaServer.on("/api/live", HTTP_GET, handleLive);
  otaServer.on("/api/gps", HTTP_GET, handleGpsDiag);
  otaServer.on("/api/sd", HTTP_GET, handleSdDiag);
  otaServer.on("/api/mag", HTTP_GET, handleMagDiag);
  otaServer.on("/api/track", HTTP_GET, handleTrackDiag);
  otaServer.on("/api/tracks", HTTP_GET, handleTrackList);
  otaServer.on("/tracks", HTTP_GET, handleTrackPage);
  otaServer.on("/track.csv", HTTP_GET, handleTrackCsv);
  otaServer.on("/track.gpx", HTTP_GET, handleTrackGpx);
  otaServer.on("/api/mcconf", HTTP_GET, handleMcconfGet);
  otaServer.on("/api/mcconf", HTTP_POST, handleMcconfPost);
  otaServer.on("/update", HTTP_POST, handleUpdateDone, handleUpdateData);
  otaServer.begin();

  ArduinoOTA.setHostname("esp32s3-n16r8");
  ArduinoOTA.onStart([]() { setStatus("arduino-ota start"); });
  ArduinoOTA.onEnd([]() { setStatus("arduino-ota end"); });
  ArduinoOTA.onError([](ota_error_t err) {
    Serial.printf("# ota arduino err %u\n", (unsigned)err);
    setStatus("arduino-ota error");
  });
  ArduinoOTA.begin();

  otaReady = true;
  Serial.println("# ota: http://192.168.4.1/");
}

static void stopWifi() {
  if (!apActive && wifiClosed) {
    return;
  }
  otaServer.stop();
  otaReady = false;
  apActive = false;
  wifiClosed = true;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  setStatus("WiFi off (idle timeout or cmd)");
}

static void startAp() {
  wifiClosed = false;
  clientSeen = false;
  apStartMs = millis();

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  IPAddress ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(ip, gateway, subnet);

  if (!WiFi.softAP(kApSsid, kApPass)) {
    setStatus("AP start failed");
    wifiClosed = true;
    return;
  }

  snprintf(ipCache, sizeof(ipCache), "%s", WiFi.softAPIP().toString().c_str());
  apActive = true;
  startServices();
  snprintf(statusText, sizeof(statusText), "AP %s %s", kApSsid, ipCache);
  Serial.printf("# ota: SoftAP ssid=%s pass=%s ip=%s (auto-off 60s if no client)\n", kApSsid,
                kApPass, ipCache);
}

void otaWifiInit() {
  startAp();
}

void otaWifiLoop() {
  if (!apActive) {
    return;
  }

  if (WiFi.softAPgetStationNum() > 0) {
    if (!clientSeen) {
      clientSeen = true;
      setStatus("client connected, WiFi stays on");
    }
  }

  if (!clientSeen && (millis() - apStartMs >= kApIdleTimeoutMs)) {
    Serial.println("# ota: no client in 60s, closing WiFi");
    stopWifi();
    setStatus("WiFi off (no connect in 60s)");
    return;
  }

  if (otaReady) {
    otaServer.handleClient();
    ArduinoOTA.handle();
  }
}

bool otaWifiConnected() {
  return apActive;
}

const char *otaWifiIp() {
  return ipCache;
}

const char *otaWifiStatusText() {
  return statusText;
}

bool otaWifiHandleCommand(const char *line) {
  if (line == nullptr) {
    return false;
  }
  if (strstr(line, "\"cmd\":\"wifi_off\"") != nullptr ||
      strstr(line, "\"cmd\": \"wifi_off\"") != nullptr) {
    stopWifi();
    setStatus("WiFi off (cmd)");
    return true;
  }
  if (strstr(line, "\"cmd\":\"wifi_on\"") != nullptr ||
      strstr(line, "\"cmd\": \"wifi_on\"") != nullptr) {
    if (!apActive) {
      startAp();
    }
    return true;
  }
  return false;
}
