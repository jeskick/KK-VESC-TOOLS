#pragma once

// ESP32-S3 N16R8 pin map — laid out by PHYSICAL HEADER POSITION, not GPIO number.
//
// On this board the GPIO numbers are not in physical order: the left header runs
// 4,5,6,7,15,16,17,18,8,3,46,9,10,11,12,13,14 from top to bottom. Every module below is
// therefore assigned a *contiguous run of pins* and placed next to the supply rail it
// needs, so each cable goes straight into the header without crossing.
//
// LEFT HEADER (top → bottom)            RIGHT HEADER (top → bottom)
// ────────────────────────────────      ──────────────────────────────────────
//  1  3V3     → TFT VCC                 1  GPIO43  CH343 U0TXD — DO NOT USE
//  2  RST     (board reset)             2  GPIO44  CH343 U0RXD — DO NOT USE
//  3  GPIO4   TFT SCL / CLK             3  GPIO1   throttle ADC  (ADC1_CH0)
//  4  GPIO5   TFT SDA / MOSI            4  GPIO2   PPM / PWM out
//  5  GPIO6   TFT RST                   5  GPIO42  VESC RX  ← VESC TX
//  6  GPIO7   TFT DC                    6  GPIO41  VESC TX  → VESC RX
//  7  GPIO15  TFT CS                    7  GPIO40  SD DATA  (onboard)
//  8  GPIO16  TFT BL (backlight)        8  GPIO39  SD CLK   (onboard)
//  9  GPIO17  MAG I2C SDA/SCL          9  GPIO38  SD CMD   (onboard)
// 10  GPIO18  MAG I2C SDA/SCL         10  GPIO37  PSRAM — DO NOT USE
// 11  GPIO8   spare                   11  GPIO36  PSRAM — DO NOT USE
// 12  GPIO3   DO NOT USE (JTAG_SEL)   12  GPIO35  PSRAM — DO NOT USE
// 13  GPIO46  DO NOT USE (see below)   13  GPIO0   BOOT button — DO NOT USE
// 14  GPIO9   KEY UP                   14  GPIO45  strapping VDD_SPI — avoid
// 15  GPIO10  KEY DOWN                 15  GPIO48  onboard WS2812 RGB
// 16  GPIO11  KEY LEFT                 16  GPIO47  GPS TX   → GPS RX
// 17  GPIO12  KEY RIGHT                17  GPIO21  GPS RX   ← GPS TX
// 18  GPIO13  KEY OK                   18  GPIO20  USB D-  — DO NOT USE
// 19  GPIO14  spare                    19  GPIO19  USB D+  — DO NOT USE
// 20  5V      → GPS VCC                20  GND     → common ground
//
// Wiring notes
//   • TFT sits directly under 3V3 (pin 1) and its six signals follow the module's own
//     pin order (SCL, SDA, RST, DC, CS, BL), so the 8-pin cable drops straight in. The
//     module's GND comes from the right header's GND (pin 20) — there is no GND on the
//     left header.
//   • GPS needs 5V: its two signals are at right 16/17, and 5V (left 20) plus GND
//     (right 20) are both at the bottom edge, next to each other.
//   • Keypad is a contiguous block of five plain GPIOs with no strapping or ADC2 baggage.
//   • Common GND for every module.
//
// Why these pins and not others
//   • GPIO35-37 are the octal PSRAM on an N16R8 module and 38-40 are the onboard microSD,
//     so neither group is available. 4-bit SD is impossible here because GPIO43 is CH343.
//   • GPIO19/20 are the native USB used for flashing, 43/44 are the CH343 bridge, and
//     GPIO0 is the BOOT button.
//   • GPIO46 is a strapping pin with a fixed internal pull-down. An active-LOW key on it
//     can read as permanently pressed and holding it at power-on changes boot behaviour,
//     so it is left unconnected.
//   • Throttle must stay on ADC1 (GPIO1-10). ADC2 is unusable whenever WiFi is on, and
//     this firmware runs an AP for OTA, so any ADC2 pin would read garbage.
//   • The keypad no longer sits next to the SPI clock. The old layout had TFT SCK on
//     GPIO12 with keys on GPIO13-17, and that crosstalk produced phantom keypresses.
//
// Verified orientations — these two are known-good from live hardware, keep them
//   • GPS: ESP GPIO21 = RX ← GPS TX. Baud is remembered in NVS after the first lock.
//   • VESC: ESP TX → VESC RX, ESP RX ← VESC TX, 115200.

// ── TFT ST7789 2.4" SPI, 8-pin module (left header 3-8, VCC from left 1) ──
#define TFT_PIN_SCK 4
#define TFT_PIN_MOSI 5
#define TFT_PIN_RST 6
#define TFT_PIN_DC 7
#define TFT_PIN_CS 15
#define TFT_PIN_BL 16

// ── 5-way keypad, common GND, active-LOW, internal pull-up (left header 14-18) ──
#define KEY_PIN_UP 9
#define KEY_PIN_DOWN 10
#define KEY_PIN_LEFT 11
#define KEY_PIN_RIGHT 12
#define KEY_PIN_OK 13

// ── Throttle / output (right header 3-4) ──
#define PIN_THROTTLE 1  // 5k potentiometer wiper, ADC1_CH0
#define PIN_PPM 2       // MODE_PWM = single servo, MODE_CPPM = 4ch PPM (JR/neg)

// ── VESC UART (right header 5-6) ──
#define VESC_UART_RX 42  // ← VESC TX
#define VESC_UART_TX 41  // → VESC RX
#define VESC_UART_BAUD 115200
// Wireless: BLE UART dongle on the VESC serial header, advertised name:
#define VESC_BLE_NAME "KK_VESC_BLE"

// ── GPS M10G-5883 (UBLOX-M10050 + QMC5883L), 5V (right header 16-17) ──
// PPS LED: solid = power/no fix, blink = 3D fix.
#define GPS_UART_TX 47  // → GPS RX
#define GPS_UART_RX 21  // ← GPS TX  (verified against live module)
#define GPS_UART_BAUD 230400

// ── QMC5883 magnetometer I2C (left header 9-10) ──
// GPIO17/18 are adjacent, not strapping, and sit just above the keypad.
// Firmware probes both SDA/SCL orientations. GPIO3 is JTAG_SEL — leave it unused.
#define MAG_PIN_A 17
#define MAG_PIN_B 18

// ── Onboard microSD, 1-bit SDMMC (right header 7-9, fixed by the board) ──
#define SD_MMC_CLK 39
#define SD_MMC_CMD 38
#define SD_MMC_D0 40

// Fallback speed estimate if COMM_GET_VALUES_SETUP unavailable (override via VESC MCCONF)
#define MOTOR_POLE_PAIRS 7
#define WHEEL_DIAMETER_M 0.083f
