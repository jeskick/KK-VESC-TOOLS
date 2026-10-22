#pragma once

#include <stdint.h>

enum KeyId : uint8_t {
  KEY_NONE = 0,
  KEY_UP,
  KEY_DOWN,
  KEY_LEFT,
  KEY_RIGHT,
  KEY_OK,
};

/** Click = short press release; Long = held ~0.8s (fires once while held). */
enum KeyAction : uint8_t {
  KEY_ACT_NONE = 0,
  KEY_ACT_CLICK,
  KEY_ACT_LONG,
};

void keypadInit();
void keypadLoop();

KeyId keypadHeld();
const char *keypadHeldLabel();

/** Pop one edge action if available. Returns false if queue empty. */
bool keypadPollAction(KeyId *id, KeyAction *act);

/** Live raw levels: bit0=OK … bit4=RIGHT, 1=pressed(LOW). */
uint8_t keypadRawPressedMask();

/** Debug line e.g. "U0 D1 L1 R1 O1" (0=pressed). */
const char *keypadRawLabel();
