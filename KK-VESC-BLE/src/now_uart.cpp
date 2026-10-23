#include "now_uart.h"

#include "pins.h"
#include "vesc_link.h"

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
static const char *kSelfId = BLE_NAME;
static const char *kRemoteId = "KK-VESC-TOOL";

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
static uint8_t txSeq = 0;
static uint32_t lastHelloMs = 0;
static volatile uint32_t lastPeerMs = 0;

static bool macEq(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

static int payloadLen(const NowPkt *p, int pktLen) {
  if (p == nullptr || pktLen < 5) {
    return -1;
  }
  int n = p->len;
  if (n > (int)sizeof(p->data) || 5 + n > pktLen) {
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

static void dropPeer(const char *why) {
  havePeer = false;
  peerDirty = false;
  ringClear();
  vescLinkSendZeroDuty();
  ringClear();
  Serial.printf("# NOW peer lost (%s)\n", why);
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
}

static void sendPkt(const NowPkt *pkt, size_t n) {
  const uint8_t *to = havePeer ? peerMac : kBcast;
  addPeer(to);
  esp_now_send(to, (const uint8_t *)pkt, n);
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

static bool helloFromRemote(const NowPkt *p, int n) {
  const size_t idLen = strlen(kRemoteId);
  return p->type == kTypeHello && n == (int)idLen && memcmp(p->data, kRemoteId, idLen) == 0;
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
    if (helloFromRemote(p, n)) {
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

void nowUartStart() {
  if (running) {
    return;
  }
  havePeer = false;
  peerDirty = false;
  ringClear();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("# NOW init fail");
    return;
  }
  esp_now_register_recv_cb(onRecv);
  addPeer(kBcast);
  running = true;
  lastHelloMs = 0;
  lastPeerMs = 0;
  Serial.printf("# NOW ch=%d MAC %s\n", NOW_CHANNEL, WiFi.macAddress().c_str());
  sendHello();
}

void nowUartStop() {
  if (!running) {
    return;
  }
  running = false;
  havePeer = false;
  peerDirty = false;
  ringClear();
  vescLinkSendZeroDuty();
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
  Serial.println("# NOW off");
}

void nowUartLoop() {
  if (!running) {
    return;
  }
  if (peerDirty) {
    peerDirty = false;
    addPeer(peerMac);
    Serial.printf("# NOW peer %02X:%02X:%02X:%02X:%02X:%02X\n", peerMac[0], peerMac[1], peerMac[2],
                  peerMac[3], peerMac[4], peerMac[5]);
  }

  const uint32_t now = millis();
  if (havePeer && lastPeerMs != 0 && now - lastPeerMs > 1500) {
    dropPeer("timeout");
  }
  if (now - lastHelloMs >= 500) {
    lastHelloMs = now;
    sendHello();
  }

  uint8_t toVesc[200];
  const int wn = ringPop(toVesc, sizeof(toVesc));
  if (wn > 0) {
    vescLinkWrite(toVesc, wn);
  }

  for (int i = 0; i < 4; i++) {
    uint8_t buf[200];
    const int n = vescLinkRead(buf, sizeof(buf));
    if (n <= 0) {
      break;
    }
    NowPkt p = {};
    p.mag0 = kMagic0;
    p.mag1 = kMagic1;
    p.type = kTypeData;
    p.seq = txSeq++;
    p.len = (uint8_t)n;
    memcpy(p.data, buf, (size_t)n);
    sendPkt(&p, (size_t)(5 + n));
  }
}

bool nowUartRunning() {
  return running;
}
