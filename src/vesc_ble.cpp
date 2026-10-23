#include "vesc_ble.h"

#include "pins.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <stdarg.h>
#include <string.h>

static const NimBLEUUID kNusSvc("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static const NimBLEUUID kNusWr("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
static const NimBLEUUID kNusNt("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

static constexpr size_t kRing = 1024;

static uint8_t ring[kRing];
static volatile uint16_t rHead = 0;
static volatile uint16_t rTail = 0;
static portMUX_TYPE ringMux = portMUX_INITIALIZER_UNLOCKED;

static NimBLEClient *client = nullptr;
static NimBLERemoteCharacteristic *wrChar = nullptr;
static NimBLERemoteCharacteristic *ntChar = nullptr;

static TaskHandle_t bleTaskH = nullptr;
static volatile bool bleWant = false;
static volatile bool bleOk = false;
static volatile uint8_t blePhase = 0;  // 0 off 1 scan 2 conn 3 ok 4 fail
static char bleHint[48] = "";
static NimBLEAddress foundAddr;
static volatile bool foundHave = false;
static volatile int scanHits = 0;
static char seenNames[48] = "";

static void setHint(const char *s) {
  snprintf(bleHint, sizeof(bleHint), "%s", s ? s : "");
}

static void blog(const char *fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
  Serial0.println(buf);
}

static void normName(const char *in, char *out, size_t outLen) {
  size_t j = 0;
  bool space = false;
  for (size_t i = 0; in && in[i] && j + 1 < outLen; i++) {
    char c = in[i];
    if (c == '\0') {
      break;
    }
    if (c >= 'A' && c <= 'Z') {
      c = (char)(c - 'A' + 'a');
    }
    if (c == ' ' || c == '_' || c == '-' || c == '\t') {
      if (j == 0 || space) {
        continue;
      }
      space = true;
      out[j++] = ' ';
      continue;
    }
    space = false;
    out[j++] = c;
  }
  while (j > 0 && out[j - 1] == ' ') {
    j--;
  }
  out[j] = '\0';
}

static bool nameMatch(const std::string &n) {
  if (n.empty()) {
    return false;
  }
  char got[48];
  char want[40];
  normName(n.c_str(), got, sizeof(got));
  normName(VESC_BLE_NAME, want, sizeof(want));
  if (got[0] == '\0') {
    return false;
  }
  if (strstr(got, want) != nullptr) {
    return true;
  }
  // Official dongle: "VESC BLE UART A1:B2" or "VESC BUILTIN BLE"
  return strstr(got, "vesc") != nullptr &&
         (strstr(got, "uart") != nullptr || strstr(got, "ble") != nullptr);
}

static void ringPush(const uint8_t *data, size_t len) {
  portENTER_CRITICAL(&ringMux);
  for (size_t i = 0; i < len; i++) {
    const uint16_t next = (uint16_t)((rHead + 1) % kRing);
    if (next == rTail) {
      break;
    }
    ring[rHead] = data[i];
    rHead = next;
  }
  portEXIT_CRITICAL(&ringMux);
}

static void ringClear() {
  portENTER_CRITICAL(&ringMux);
  rHead = 0;
  rTail = 0;
  portEXIT_CRITICAL(&ringMux);
}

static void notifyCb(NimBLERemoteCharacteristic *c, uint8_t *data, size_t len, bool isNotify) {
  (void)c;
  (void)isNotify;
  if (data != nullptr && len > 0) {
    ringPush(data, len);
  }
}

class ClientCb : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient *c) override {
    (void)c;
    bleOk = false;
    wrChar = nullptr;
    ntChar = nullptr;
    if (bleWant) {
      blePhase = 4;
    }
  }
};

static ClientCb clientCb;

static bool pickChars(NimBLEClient *c) {
  wrChar = nullptr;
  ntChar = nullptr;
  NimBLERemoteService *nus = c->getService(kNusSvc);
  if (nus) {
    wrChar = nus->getCharacteristic(kNusWr);
    ntChar = nus->getCharacteristic(kNusNt);
  }
  if (!wrChar || !ntChar) {
    const std::vector<NimBLERemoteService *> *svcs = c->getServices(true);
    if (svcs) {
      for (NimBLERemoteService *s : *svcs) {
        const std::vector<NimBLERemoteCharacteristic *> *chs = s->getCharacteristics(true);
        if (!chs) {
          continue;
        }
        for (NimBLERemoteCharacteristic *ch : *chs) {
          if (!ntChar && ch->canNotify()) {
            ntChar = ch;
          }
          if (!wrChar && (ch->canWrite() || ch->canWriteNoResponse())) {
            wrChar = ch;
          }
        }
      }
    }
  }
  if (!wrChar || !ntChar) {
    setHint("gatt: no uart char");
    return false;
  }
  if (!ntChar->subscribe(true, notifyCb, false) && !ntChar->subscribe(true, notifyCb, true)) {
    setHint("gatt: notify fail");
    return false;
  }
  return true;
}

static bool tryConnect(const NimBLEAddress &addr) {
  blePhase = 2;
  setHint("connecting...");
  if (client == nullptr) {
    client = NimBLEDevice::createClient();
    client->setClientCallbacks(&clientCb, false);
    client->setConnectTimeout(10);
  }
  if (client->isConnected()) {
    client->disconnect();
    delay(200);
  }
  blog("# BLE connecting %s type=%u", addr.toString().c_str(), (unsigned)addr.getType());
  if (!client->connect(addr, true)) {
    setHint("connect fail");
    blog("# BLE connect fail");
    return false;
  }
  if (!pickChars(client)) {
    client->disconnect();
    blog("# BLE gatt fail");
    return false;
  }
  ringClear();
  bleOk = true;
  blePhase = 3;
  setHint("ble ok");
  blog("# BLE connected %s", addr.toString().c_str());
  return true;
}

class ScanCb : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice *d) override {
    if (d == nullptr) {
      return;
    }
    scanHits++;
    const std::string nm = d->getName();
    const bool nus = d->isAdvertisingService(kNusSvc);
    blog("# BLE adv '%s' %s%s", nm.c_str(), d->getAddress().toString().c_str(), nus ? " NUS" : "");
    if (seenNames[0] == '\0' && !nm.empty()) {
      snprintf(seenNames, sizeof(seenNames), "%s", nm.c_str());
    }
    if (nameMatch(nm)) {
      foundAddr = d->getAddress();
      foundHave = true;
      snprintf(seenNames, sizeof(seenNames), "%s", nm.c_str());
      NimBLEDevice::getScan()->stop();
    } else if (nus && !foundHave) {
      foundAddr = d->getAddress();
      foundHave = true;
      if (seenNames[0] == '\0') {
        snprintf(seenNames, sizeof(seenNames), "NUS");
      }
    }
  }
};

static ScanCb scanCb;

static bool scanAndConnect() {
  blePhase = 1;
  foundHave = false;
  scanHits = 0;
  seenNames[0] = '\0';
  setHint("scanning...");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->stop();
  scan->clearResults();
  scan->setAdvertisedDeviceCallbacks(&scanCb, false);
  scan->setActiveScan(true);
  scan->setInterval(80);
  scan->setWindow(80);
  scan->setDuplicateFilter(false);
  scan->start(10, false);
  blog("# BLE scan done hits=%d have=%d name='%s'", scanHits, (int)foundHave, seenNames);
  if (!foundHave) {
    blePhase = 4;
    if (scanHits <= 0) {
      setHint("scan 0: power-cycle dongle");
    } else if (seenNames[0]) {
      char line[48];
      snprintf(line, sizeof(line), "saw %d: %s", scanHits, seenNames);
      setHint(line);
    } else {
      char line[48];
      snprintf(line, sizeof(line), "saw %d unnamed", scanHits);
      setHint(line);
    }
    return false;
  }
  char line[48];
  snprintf(line, sizeof(line), "found %s", seenNames[0] ? seenNames : foundAddr.toString().c_str());
  setHint(line);
  return tryConnect(foundAddr);
}

static void bleTask(void *arg) {
  (void)arg;
  Serial0.begin(115200);
  delay(200);
  NimBLEDevice::init("kk-vesc");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(247);
  blog("# VESC BLE scan for %s", VESC_BLE_NAME);

  while (bleWant) {
    if (!bleOk) {
      if (!scanAndConnect()) {
        for (int i = 0; i < 30 && bleWant; i++) {
          delay(100);
        }
      }
    } else {
      delay(200);
    }
  }

  bleOk = false;
  blePhase = 0;
  wrChar = nullptr;
  ntChar = nullptr;
  if (client) {
    if (client->isConnected()) {
      client->disconnect();
    }
    NimBLEDevice::deleteClient(client);
    client = nullptr;
  }
  NimBLEDevice::deinit(true);
  bleTaskH = nullptr;
  vTaskDelete(nullptr);
}

void vescBleBegin() {
  if (bleWant) {
    return;
  }
  bleWant = true;
  bleOk = false;
  blePhase = 1;
  ringClear();
  if (bleTaskH == nullptr) {
    xTaskCreatePinnedToCore(bleTask, "vescble", 8192, nullptr, 1, &bleTaskH, 0);
  }
}

void vescBleEnd() {
  bleWant = false;
  bleOk = false;
  const uint32_t start = millis();
  while (bleTaskH != nullptr && (millis() - start) < 2000) {
    delay(20);
  }
}

void vescBleLoop() {}

bool vescBleWrite(const uint8_t *data, size_t len) {
  if (!bleOk || wrChar == nullptr || data == nullptr || len == 0) {
    return false;
  }
  size_t mtu = 20;
  if (client) {
    const uint16_t v = client->getMTU();
    if (v > 6) {
      mtu = (size_t)(v - 3);
    }
  }
  size_t off = 0;
  while (off < len) {
    const size_t n = (len - off) < mtu ? (len - off) : mtu;
    if (!wrChar->writeValue(data + off, n, false)) {
      return false;
    }
    off += n;
  }
  return true;
}

int vescBleAvailable() {
  portENTER_CRITICAL(&ringMux);
  const int n = (int)((rHead + kRing - rTail) % kRing);
  portEXIT_CRITICAL(&ringMux);
  return n;
}

int vescBleRead() {
  int v = -1;
  portENTER_CRITICAL(&ringMux);
  if (rHead != rTail) {
    v = ring[rTail];
    rTail = (uint16_t)((rTail + 1) % kRing);
  }
  portEXIT_CRITICAL(&ringMux);
  return v;
}

bool vescBleConnected() {
  return bleOk;
}

const char *vescBleState() {
  switch (blePhase) {
  case 1:
    return "scan";
  case 2:
    return "conn";
  case 3:
    return "ok";
  case 4:
    return "fail";
  default:
    return "off";
  }
}

const char *vescBleHint() {
  return bleHint;
}
