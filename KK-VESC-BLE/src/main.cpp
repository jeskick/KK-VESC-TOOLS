#include <Arduino.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>

#include "pins.h"
#include "vesc_link.h"
#include "ble_uart.h"
#include "now_uart.h"

enum Mode : uint8_t { MODE_NOW = 0, MODE_BLE = 1 };

static Preferences prefs;
static Mode mode = MODE_BLE;
static Adafruit_NeoPixel rgb(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
static uint32_t btnDownAt = 0;
static bool btnWasDown = false;
static bool armed = false;
static uint32_t lastBlinkMs = 0;
static bool blinkOn = true;

static void logBoth(const char *s) {
  Serial.println(s);
  Serial0.println(s);
}

static void saveMode() {
  prefs.begin("kkble", false);
  prefs.putUChar("mode", (uint8_t)mode);
  prefs.end();
}

static void loadMode() {
  prefs.begin("kkble", true);
  const uint8_t m = prefs.getUChar("mode", (uint8_t)MODE_BLE);
  prefs.end();
  mode = (m == (uint8_t)MODE_NOW) ? MODE_NOW : MODE_BLE;
}

static void showLed(bool on, uint8_t r, uint8_t g, uint8_t b) {
  digitalWrite(PIN_LED, on ? HIGH : LOW);
  rgb.setPixelColor(0, rgb.Color(on ? r : 0, on ? g : 0, on ? b : 0));
  rgb.show();
}

static void applyMode(Mode next) {
  mode = next;
  saveMode();
  bleUartStop();
  nowUartStop();
  delay(150);
  if (mode == MODE_BLE) {
    bleUartStart();
    showLed(true, 0, 40, 0);
    logBoth("# mode BLE  KK-VESC-BLE  LED solid");
  } else {
    nowUartStart();
    blinkOn = true;
    lastBlinkMs = millis();
    showLed(true, 0, 0, 40);
    logBoth("# mode ESP-NOW  LED slow blink");
  }
}

static void pollBtn() {
  if (!armed) {
    if (millis() > 2500 && digitalRead(PIN_BTN) == HIGH) {
      armed = true;
    }
    return;
  }
  const bool down = digitalRead(PIN_BTN) == LOW;
  const uint32_t now = millis();
  if (down && !btnWasDown) {
    btnDownAt = now;
  }
  if (!down && btnWasDown) {
    const uint32_t held = now - btnDownAt;
    if (held >= 1500 && held < 15000) {
      applyMode((mode == MODE_BLE) ? MODE_NOW : MODE_BLE);
    }
  }
  btnWasDown = down;
}

static void pollLed() {
  if (mode == MODE_BLE) {
    return;
  }
  if (millis() - lastBlinkMs >= 400) {
    lastBlinkMs = millis();
    blinkOn = !blinkOn;
    showLed(blinkOn, 0, 0, 40);
  }
}

void setup() {
  Serial.begin(115200);
  Serial0.begin(115200);
  delay(200);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  rgb.begin();
  rgb.setBrightness(40);
  showLed(false, 0, 0, 0);

  logBoth("KK-VESC-BLE bridge boot");
  loadMode();
  vescLinkBegin();
  applyMode(mode);
}

void loop() {
  pollBtn();
  pollLed();
  vescLinkLoop();
  if (mode == MODE_BLE) {
    bleUartLoop();
  } else {
    nowUartLoop();
  }
}
