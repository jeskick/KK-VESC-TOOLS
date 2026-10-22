#include "vesc_ble.h"

#include "pins.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
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

static bool nameMatch(const std::string &n) {
  if (n.empty()) {
    return false;
  }
  if (n == VESC_BLE_NAME) {
    return true;
  }
  return n.rfind("KK_VESC", 0) == 0;
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
    return false;
  }
  if (!ntChar->subscribe(true, notifyCb, true)) {
    return false;
  }
  return true;
}

static bool tryConnect(const NimBLEAddress &addr) {
  blePhase = 2;
  if (client == nullptr) {
    client = NimBLEDevice::createClient();
    client->setClientCallbacks(&clientCb, false);
    client->setConnectTimeout(8);
  }
  if (client->isConnected()) {
    client->disconnect();
    delay(200);
  }
  if (!client->connect(addr, true)) {
    return false;
  }
  if (!pickChars(client)) {
    client->disconnect();
    return false;
  }
  ringClear();
  bleOk = true;
  blePhase = 3;
  Serial.printf("# VESC BLE connected %s\n", addr.toString().c_str());
  return true;
}

static bool scanAndConnect() {
  blePhase = 1;
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(30);
  NimBLEScanResults results = scan->start(5, false);
  NimBLEAddress found;
  bool have = false;
  for (int i = 0; i < results.getCount(); i++) {
    NimBLEAdvertisedDevice d = results.getDevice(i);
    if (nameMatch(d.getName())) {
      found = d.getAddress();
      have = true;
      Serial.printf("# VESC BLE saw %s %s\n", d.getName().c_str(), found.toString().c_str());
      break;
    }
  }
  scan->clearResults();
  if (!have) {
    blePhase = 4;
    return false;
  }
  return tryConnect(found);
}

static void bleTask(void *arg) {
  (void)arg;
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(247);
  Serial.printf("# VESC BLE scan for %s\n", VESC_BLE_NAME);

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
    xTaskCreatePinnedToCore(bleTask, "vescble", 6144, nullptr, 1, &bleTaskH, 0);
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
