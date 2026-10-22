#include "keypad.h"

#include "pins.h"

#include <Arduino.h>
#include "driver/gpio.h"

// Active-LOW, internal pull-up. Click on confirmed press. No "seenUp" latch:
// that flag was why OK worked exactly once per boot if the pin did not look
// HIGH for a few samples after release (SPI mux / bounce / slow rise).
static const uint8_t kDebounce = 2;
static const uint32_t kSampleMs = 5;

struct KeyPin {
  int pin;
  KeyId id;
};

static const KeyPin kKeys[] = {
    {KEY_PIN_OK, KEY_OK},
    {KEY_PIN_UP, KEY_UP},
    {KEY_PIN_DOWN, KEY_DOWN},
    {KEY_PIN_LEFT, KEY_LEFT},
    {KEY_PIN_RIGHT, KEY_RIGHT},
};
static const size_t kKeyCount = sizeof(kKeys) / sizeof(kKeys[0]);

struct KeyState {
  bool rawDown;
  bool latched;
  uint8_t sameCount;
};

static KeyState st[5];
static uint32_t lastSampleMs = 0;
static KeyId held = KEY_NONE;

static KeyId evtId[16];
static KeyAction evtAct[16];
static uint8_t evtHead = 0;
static uint8_t evtTail = 0;

static void pushEvt(KeyId id, KeyAction act) {
  const uint8_t next = (uint8_t)((evtHead + 1) & 15);
  if (next == evtTail) {
    return;
  }
  evtId[evtHead] = id;
  evtAct[evtHead] = act;
  evtHead = next;
}

static void claimPin(int pin) {
  const gpio_num_t p = (gpio_num_t)pin;
  gpio_reset_pin(p);
  gpio_set_direction(p, GPIO_MODE_INPUT);
  gpio_set_pull_mode(p, GPIO_PULLUP_ONLY);
  gpio_pulldown_dis(p);
  gpio_pullup_en(p);
}

static bool pinDown(int pin) {
  return gpio_get_level((gpio_num_t)pin) == 0;
}

static void refreshHeld() {
  held = KEY_NONE;
  for (size_t j = 0; j < kKeyCount; j++) {
    if (st[j].latched) {
      held = kKeys[j].id;
      break;
    }
  }
}

void keypadInit() {
  evtHead = evtTail = 0;
  held = KEY_NONE;
  lastSampleMs = 0;
  for (size_t i = 0; i < kKeyCount; i++) {
    claimPin(kKeys[i].pin);
  }
  delay(5);
  for (size_t i = 0; i < kKeyCount; i++) {
    const bool down = pinDown(kKeys[i].pin);
    st[i].rawDown = down;
    st[i].latched = down;  // match live level; do not fire a boot click
    st[i].sameCount = kDebounce;
  }
  refreshHeld();
}

void keypadLoop() {
  const uint32_t t = millis();
  if (lastSampleMs != 0 && (t - lastSampleMs) < kSampleMs) {
    return;
  }
  lastSampleMs = t;

  for (size_t i = 0; i < kKeyCount; i++) {
    const bool raw = pinDown(kKeys[i].pin);
    KeyState &s = st[i];
    if (raw == s.rawDown) {
      if (s.sameCount < 255) {
        s.sameCount++;
      }
    } else {
      s.rawDown = raw;
      s.sameCount = 1;
    }
    if (s.sameCount < kDebounce) {
      continue;
    }

    if (raw && !s.latched) {
      s.latched = true;
      refreshHeld();
      pushEvt(kKeys[i].id, KEY_ACT_CLICK);
    } else if (!raw && s.latched) {
      s.latched = false;
      refreshHeld();
    }
  }

  // Cheap reclaim if a peripheral stole a pad after init (do not gpio_reset:
  // that would glitch the level).
  static uint32_t lastClaimMs = 0;
  if (lastClaimMs == 0 || (t - lastClaimMs) >= 250) {
    lastClaimMs = t;
    for (size_t i = 0; i < kKeyCount; i++) {
      const gpio_num_t p = (gpio_num_t)kKeys[i].pin;
      gpio_set_direction(p, GPIO_MODE_INPUT);
      gpio_set_pull_mode(p, GPIO_PULLUP_ONLY);
    }
  }
}

KeyId keypadHeld() {
  return held;
}

const char *keypadHeldLabel() {
  switch (held) {
  case KEY_UP:
    return "UP";
  case KEY_DOWN:
    return "DN";
  case KEY_LEFT:
    return "LT";
  case KEY_RIGHT:
    return "RT";
  case KEY_OK:
    return "OK";
  default:
    return "-";
  }
}

bool keypadPollAction(KeyId *id, KeyAction *act) {
  if (evtTail == evtHead || id == nullptr || act == nullptr) {
    return false;
  }
  *id = evtId[evtTail];
  *act = evtAct[evtTail];
  evtTail = (uint8_t)((evtTail + 1) & 15);
  return true;
}

uint8_t keypadRawPressedMask() {
  uint8_t m = 0;
  if (pinDown(KEY_PIN_OK)) {
    m |= 1u << 0;
  }
  if (pinDown(KEY_PIN_UP)) {
    m |= 1u << 1;
  }
  if (pinDown(KEY_PIN_DOWN)) {
    m |= 1u << 2;
  }
  if (pinDown(KEY_PIN_LEFT)) {
    m |= 1u << 3;
  }
  if (pinDown(KEY_PIN_RIGHT)) {
    m |= 1u << 4;
  }
  return m;
}

const char *keypadRawLabel() {
  static char buf[40];
  const uint8_t m = keypadRawPressedMask();
  snprintf(buf, sizeof(buf), "U%d D%d L%d R%d O%d", (m & 2) ? 0 : 1, (m & 4) ? 0 : 1,
           (m & 8) ? 0 : 1, (m & 16) ? 0 : 1, (m & 1) ? 0 : 1);
  return buf;
}
