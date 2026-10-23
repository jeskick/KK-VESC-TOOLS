#include "ble_uart.h"

#include "pins.h"
#include "vesc_link.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

extern "C" {
#include "nimble/nimble/host/include/host/ble_hs.h"
}

static const NimBLEUUID kNusSvc("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static const NimBLEUUID kNusWr("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
static const NimBLEUUID kNusNt("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

static NimBLEServer *server = nullptr;
static NimBLECharacteristic *wrChar = nullptr;  // VESC Tool → VESC
static NimBLECharacteristic *ntChar = nullptr;  // VESC → VESC Tool
static volatile bool running = false;
static volatile bool phoneOn = false;
static volatile bool subscribed = false;
static volatile bool drainPending = false;
static volatile bool zeroDutyPending = false;
static volatile uint16_t connHandle = 0xFFFF;
static volatile uint16_t chunk = 20;
static uint32_t bytesToVesc = 0;
static uint32_t bytesFromVesc = 0;
static uint32_t lastStatMs = 0;

static constexpr size_t kRing = 2048;
static uint8_t ring[kRing];
static volatile uint16_t rHead = 0;
static volatile uint16_t rTail = 0;
static portMUX_TYPE ringMux = portMUX_INITIALIZER_UNLOCKED;

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

static int ringPop(uint8_t *buf, int maxLen) {
  int n = 0;
  portENTER_CRITICAL(&ringMux);
  while (n < maxLen && rHead != rTail) {
    buf[n++] = ring[rTail];
    rTail = (uint16_t)((rTail + 1) % kRing);
  }
  portEXIT_CRITICAL(&ringMux);
  return n;
}

static void ringClear() {
  portENTER_CRITICAL(&ringMux);
  rHead = 0;
  rTail = 0;
  portEXIT_CRITICAL(&ringMux);
}

static void logLine(const char *s) {
  Serial.println(s);
  Serial0.println(s);
}

static void useStaticRandomAddr() {
  // VESC Tool Qt always connects as RandomAddress. A public IEEE MAC
  // often fails on Windows; a static random address matches the nRF dongle.
  ble_addr_t rnd = {};
  if (ble_hs_id_copy_addr(BLE_ADDR_RANDOM, rnd.val, nullptr) != 0) {
    if (ble_hs_id_gen_rnd(0, &rnd) == 0) {
      ble_hs_id_set_rnd(rnd.val);
    }
  }
  NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
}

class ServerCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, ble_gap_conn_desc *desc) override {
    phoneOn = true;
    subscribed = false;
    chunk = 20;
    connHandle = desc->conn_handle;
    bytesToVesc = 0;
    bytesFromVesc = 0;
    drainPending = true;
    s->updateConnParams(desc->conn_handle, 6, 12, 0, 400);
    Serial.printf("# BLE in h=%u\n", desc->conn_handle);
    Serial0.println("# BLE in");
  }

  void onDisconnect(NimBLEServer *s, ble_gap_conn_desc *desc) override {
    (void)s;
    (void)desc;
    phoneOn = false;
    subscribed = false;
    connHandle = 0xFFFF;
    chunk = 20;
    ringClear();
    zeroDutyPending = true;
    logLine("# BLE out");
  }

  void onMTUChange(uint16_t mtu, ble_gap_conn_desc *desc) override {
    (void)desc;
    uint16_t n = (mtu > 3) ? (uint16_t)(mtu - 3) : 20;
    if (n > 180) {
      n = 180;
    }
    chunk = n;
    Serial.printf("# BLE MTU %u chunk %u\n", mtu, n);
  }
};

class WrCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) override {
    std::string v = c->getValue();
    if (v.empty()) {
      return;
    }
    ringPush((const uint8_t *)v.data(), v.size());
    bytesToVesc += (uint32_t)v.size();
  }
};

class NtCb : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *c, ble_gap_conn_desc *desc, uint16_t subValue) override {
    (void)c;
    (void)desc;
    subscribed = (subValue & 0x0001) != 0;
    Serial.printf("# BLE notify %s\n", subscribed ? "on" : "off");
    Serial0.printf("# BLE notify %s\n", subscribed ? "on" : "off");
  }
};

static ServerCb serverCb;
static WrCb wrCb;
static NtCb ntCb;

void bleUartStart() {
  if (running) {
    return;
  }
  NimBLEDevice::init(BLE_NAME);
  useStaticRandomAddr();
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(247);

  server = NimBLEDevice::createServer();
  server->setCallbacks(&serverCb);
  server->advertiseOnDisconnect(true);

  NimBLEService *svc = server->createService(kNusSvc);
  wrChar = svc->createCharacteristic(kNusWr, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR,
                                     512);
  ntChar = svc->createCharacteristic(kNusNt, NIMBLE_PROPERTY::NOTIFY, 512);
  wrChar->setCallbacks(&wrCb);
  ntChar->setCallbacks(&ntCb);
  svc->start();

  // Same layout as official nRF VESC dongle: NUS UUID in ADV (VESC Tool
  // Android/Linux filters on it), complete name in scan response.
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(kNusSvc);
  adv->setName(BLE_NAME);
  adv->setScanResponse(true);
  adv->setMinInterval(32);
  adv->setMaxInterval(96);
  const bool ok = adv->start();
  running = true;
  phoneOn = false;
  subscribed = false;
  drainPending = false;
  zeroDutyPending = false;
  ringClear();
  Serial.printf("# BLE advertise %s start=%d mac=%s\n", BLE_NAME, (int)ok,
                NimBLEDevice::getAddress().toString().c_str());
  Serial0.printf("# BLE advertise %s start=%d mac=%s\n", BLE_NAME, (int)ok,
                 NimBLEDevice::getAddress().toString().c_str());
}

void bleUartStop() {
  if (!running) {
    return;
  }
  running = false;
  phoneOn = false;
  subscribed = false;
  connHandle = 0xFFFF;
  ringClear();
  vescLinkSendZeroDuty();
  NimBLEDevice::getAdvertising()->stop();
  delay(50);
  NimBLEDevice::deinit(true);
  server = nullptr;
  wrChar = nullptr;
  ntChar = nullptr;
  logLine("# BLE off");
}

void bleUartLoop() {
  if (!running) {
    return;
  }
  if (drainPending) {
    drainPending = false;
    vescLinkDrain();
  }
  if (zeroDutyPending) {
    zeroDutyPending = false;
    vescLinkSendZeroDuty();
  }
  if (phoneOn) {
    uint8_t toVesc[180];
    for (int i = 0; i < 8; i++) {
      const int n = ringPop(toVesc, (int)sizeof(toVesc));
      if (n <= 0) {
        break;
      }
      vescLinkWrite(toVesc, n);
    }
  }
  if (!subscribed || ntChar == nullptr) {
    return;
  }

  uint16_t maxn = chunk;
  if (maxn < 1) {
    maxn = 20;
  }
  if (maxn > 180) {
    maxn = 180;
  }
  if (server != nullptr && connHandle != 0xFFFF) {
    const uint16_t mtu = server->getPeerMTU(connHandle);
    if (mtu >= 23) {
      uint16_t n = (uint16_t)(mtu - 3);
      if (n < maxn) {
        maxn = n;
      }
    }
  }

  uint8_t buf[180];
  for (int i = 0; i < 12; i++) {
    const int n = vescLinkRead(buf, maxn);
    if (n <= 0) {
      break;
    }
    ntChar->notify(buf, (size_t)n);
    bytesFromVesc += (uint32_t)n;
  }

  const uint32_t now = millis();
  if (now - lastStatMs >= 2000) {
    lastStatMs = now;
    Serial.printf("# BLE uart  tool→vesc %lu  vesc→tool %lu\n",
                  (unsigned long)bytesToVesc, (unsigned long)bytesFromVesc);
  }
}

bool bleUartRunning() {
  return running;
}
