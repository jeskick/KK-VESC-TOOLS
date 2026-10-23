#include "vesc_now.h"

#include "ota_wifi.h"
#include "pins.h"
#include "vesc_protocol.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

static const uint8_t kBcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t kMagic0 = 'K';
static const uint8_t kMagic1 = 'N';
static const uint8_t kTypeHello = 0;
static const uint8_t kTypeData = 1;
static const char *kSelfId = "KK-VESC-TOOL";
static const char *kBridgeId = "KK-VESC-BLE";

#pragma pack(push, 1)
struct NowPkt {
  uint8_t mag0;
  uint8_t mag1;
  uint8_t type;
  uint8_t seq;
  uint8_t len;
  uint8_t data[200];
};
#pragma pack(pop)

static constexpr size_t kRing = 2048;
static uint8_t ring[kRing];
static volatile uint16_t rHead = 0;
static volatile uint16_t rTail = 0;
static portMUX_TYPE ringMux = portMUX_INITIALIZER_UNLOCKED;

static bool running = false;
static uint8_t peerMac[6] = {};
static volatile bool havePeer = false;
static volatile bool peerDirty = false;
static volatile bool hintDirty = false;
static uint8_t txSeq = 0;
static uint32_t lastHelloMs = 0;
static volatile uint32_t lastPeerMs = 0;
static volatile int8_t lastRssi = 0;
static volatile bool rssiHave = false;
static char hint[40] = "wait KK-VESC-BLE";

static bool macEq(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

static int payloadLen(const NowPkt *p, int pktLen) {
  if (p == nullptr || pktLen < 5) {
    return -1;
  }
  int n = p->len;
  if (n < 0 || n > (int)sizeof(p->data) || 5 + n > pktLen) {
    return -1;
  }
  return n;
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

static void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) {
    return;
  }
  esp_now_peer_info_t info = {};
  memcpy(info.peer_addr, mac, 6);
  info.channel = NOW_CHANNEL;
  info.encrypt = false;
  info.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&info);
}

static void rememberPeer(const uint8_t *mac) {
  if (mac == nullptr || macEq(mac, kBcast)) {
    return;
  }
  if (havePeer) {
    if (macEq(peerMac, mac)) {
      lastPeerMs = millis();
    }
    return;
  }
  memcpy(peerMac, mac, 6);
  lastPeerMs = millis();
  havePeer = true;
  peerDirty = true;
  hintDirty = true;
}

static void sendPkt(const NowPkt *pkt, size_t n) {
  const uint8_t *to = havePeer ? peerMac : kBcast;
  addPeer(to);
  esp_now_send(to, (const uint8_t *)pkt, n);
}

static void sendZeroDutyPkt() {
  if (!running || !havePeer) {
    return;
  }
  uint8_t duty[4] = {0, 0, 0, 0};
  uint8_t frame[32];
  const size_t n = vescBuildPacket(COMM_SET_DUTY, duty, 4, frame, sizeof(frame));
  if (n == 0 || n > sizeof(NowPkt::data)) {
    return;
  }
  NowPkt p = {};
  p.mag0 = kMagic0;
  p.mag1 = kMagic1;
  p.type = kTypeData;
  p.seq = txSeq++;
  p.len = (uint8_t)n;
  memcpy(p.data, frame, n);
  sendPkt(&p, 5 + n);
}

static void sendHello() {
  NowPkt p = {};
  p.mag0 = kMagic0;
  p.mag1 = kMagic1;
  p.type = kTypeHello;
  p.seq = txSeq++;
  p.len = (uint8_t)strlen(kSelfId);
  if (p.len > sizeof(p.data)) {
    p.len = sizeof(p.data);
  }
  memcpy(p.data, kSelfId, p.len);
  sendPkt(&p, (size_t)(5 + p.len));
}

static bool helloFromBridge(const NowPkt *p, int n) {
  const size_t idLen = strlen(kBridgeId);
  return p->type == kTypeHello && n == (int)idLen && memcmp(p->data, kBridgeId, idLen) == 0;
}

static void onSniff(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (!havePeer || buf == nullptr) {
    return;
  }
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) {
    return;
  }
  const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
  if (pkt->rx_ctrl.sig_len < 16) {
    return;
  }
  // Addr2 = transmitter. ESP-NOW frames from the bridge land here.
  if (memcmp(pkt->payload + 10, peerMac, 6) != 0) {
    return;
  }
  lastRssi = (int8_t)pkt->rx_ctrl.rssi;
  rssiHave = true;
}

static void sniffOn() {
  wifi_promiscuous_filter_t filter = {};
  filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(onSniff);
  esp_wifi_set_promiscuous(true);
}

static void sniffOff() {
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_promiscuous_rx_cb(nullptr);
}

static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (mac == nullptr || data == nullptr || len < 5) {
    return;
  }
  const NowPkt *p = (const NowPkt *)data;
  if (p->mag0 != kMagic0 || p->mag1 != kMagic1) {
    return;
  }
  const int n = payloadLen(p, len);
  if (n < 0) {
    return;
  }
  if (p->type == kTypeHello) {
    if (helloFromBridge(p, n)) {
      rememberPeer(mac);
    }
    return;
  }
  if (p->type != kTypeData || n == 0) {
    return;
  }
  if (!havePeer || !macEq(peerMac, mac)) {
    return;
  }
  lastPeerMs = millis();
  ringPush(p->data, (size_t)n);
}

static void radioForNow() {
  WiFi.mode(otaWifiConnected() ? WIFI_AP_STA : WIFI_STA);
  WiFi.setSleep(false);
  if (!otaWifiConnected()) {
    WiFi.disconnect();
  }
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
}

static bool startNowStack() {
  radioForNow();
  if (esp_now_init() != ESP_OK) {
    Serial.println("# NOW init fail");
    snprintf(hint, sizeof(hint), "%s", "NOW init fail");
    return false;
  }
  esp_now_register_recv_cb(onRecv);
  addPeer(kBcast);
  if (havePeer) {
    addPeer(peerMac);
  }
  sniffOn();
  return true;
}

void vescNowBegin() {
  if (running) {
    return;
  }
  havePeer = false;
  peerDirty = false;
  rssiHave = false;
  ringClear();
  snprintf(hint, sizeof(hint), "%s", "wait KK-VESC-BLE");
  if (!startNowStack()) {
    return;
  }
  running = true;
  lastHelloMs = 0;
  lastPeerMs = 0;
  Serial.printf("# NOW ch=%d MAC %s -> %s\n", NOW_CHANNEL, WiFi.macAddress().c_str(), kBridgeId);
  sendHello();
}

void vescNowOnWifiChanged() {
  if (!running) {
    return;
  }
  sendZeroDutyPkt();
  sniffOff();
  esp_now_deinit();
  if (!startNowStack()) {
    running = false;
    havePeer = false;
    return;
  }
  Serial.printf("# NOW rebind ch=%d AP=%d\n", NOW_CHANNEL, (int)otaWifiConnected());
  sendHello();
}

void vescNowKeepChannel() {
  if (!running) {
    return;
  }
  radioForNow();
  addPeer(kBcast);
  if (havePeer) {
    addPeer(peerMac);
  }
  sniffOn();
}

void vescNowEnd() {
  if (!running) {
    return;
  }
  sendZeroDutyPkt();
  running = false;
  havePeer = false;
  rssiHave = false;
  ringClear();
  sniffOff();
  esp_now_deinit();
  if (otaWifiConnected()) {
    WiFi.mode(WIFI_AP);
  } else {
    WiFi.mode(WIFI_OFF);
  }
  snprintf(hint, sizeof(hint), "%s", "off");
  Serial.println("# NOW off");
}

void vescNowLoop() {
  if (!running) {
    return;
  }
  if (peerDirty) {
    peerDirty = false;
    addPeer(peerMac);
    if (hintDirty) {
      hintDirty = false;
      snprintf(hint, sizeof(hint), "%02X:%02X:%02X:%02X:%02X:%02X", peerMac[0], peerMac[1],
               peerMac[2], peerMac[3], peerMac[4], peerMac[5]);
      Serial.printf("# NOW peer %s\n", hint);
    }
  }
  const uint32_t now = millis();
  if (now - lastHelloMs >= 500) {
    lastHelloMs = now;
    sendHello();
  }
  if (havePeer && lastPeerMs != 0 && now - lastPeerMs > 1500) {
    sendZeroDutyPkt();
    havePeer = false;
    rssiHave = false;
    snprintf(hint, sizeof(hint), "%s", "wait KK-VESC-BLE");
    Serial.println("# NOW peer lost");
  }
}

bool vescNowWrite(const uint8_t *data, size_t len) {
  if (!running || !havePeer || data == nullptr || len == 0) {
    return false;
  }
  size_t off = 0;
  while (off < len) {
    size_t n = len - off;
    if (n > sizeof(NowPkt::data)) {
      n = sizeof(NowPkt::data);
    }
    NowPkt p = {};
    p.mag0 = kMagic0;
    p.mag1 = kMagic1;
    p.type = kTypeData;
    p.seq = txSeq++;
    p.len = (uint8_t)n;
    memcpy(p.data, data + off, n);
    sendPkt(&p, 5 + n);
    off += n;
  }
  return true;
}

int vescNowAvailable() {
  portENTER_CRITICAL(&ringMux);
  const int n = (rHead >= rTail) ? (int)(rHead - rTail) : (int)(kRing - rTail + rHead);
  portEXIT_CRITICAL(&ringMux);
  return n;
}

int vescNowRead() {
  int b = -1;
  portENTER_CRITICAL(&ringMux);
  if (rHead != rTail) {
    b = ring[rTail];
    rTail = (uint16_t)((rTail + 1) % kRing);
  }
  portEXIT_CRITICAL(&ringMux);
  return b;
}

bool vescNowLinked() {
  return running && havePeer;
}

bool vescNowRssiOk() {
  return running && havePeer && rssiHave && lastPeerMs != 0 && (millis() - lastPeerMs) < 1500;
}

int8_t vescNowRssi() {
  return lastRssi;
}

const char *vescNowState() {
  if (!running) {
    return "off";
  }
  return havePeer ? "ok" : "wait";
}

const char *vescNowHint() {
  return hint;
}
