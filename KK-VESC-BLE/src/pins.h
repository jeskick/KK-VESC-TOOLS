#pragma once

// KK-VESC-BLE bridge — same ESP32-S3 N16R8 header as the field tool.
// Plug VESC COMM into the same UART pins the field tool uses.

#define VESC_UART_TX 41  // → VESC RX
#define VESC_UART_RX 42  // ← VESC TX
#define VESC_UART_BAUD 115200

#define PIN_BTN 0    // onboard BOOT, active-LOW, long-press toggles mode
#define PIN_LED 2    // header "LED ON"
#define PIN_RGB 48   // onboard WS2812

#define BLE_NAME "KK-VESC-BLE"
#define NOW_CHANNEL 1
