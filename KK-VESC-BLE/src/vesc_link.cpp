#include "vesc_link.h"

#include "pins.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <driver/gpio.h>

static HardwareSerial VescSerial(1);

void vescLinkBegin() {
  VescSerial.end();
  delay(5);
  gpio_reset_pin((gpio_num_t)VESC_UART_RX);
  gpio_reset_pin((gpio_num_t)VESC_UART_TX);
  VescSerial.setRxBufferSize(4096);
  VescSerial.setTxBufferSize(1024);
  VescSerial.begin(VESC_UART_BAUD, SERIAL_8N1, VESC_UART_RX, VESC_UART_TX);
  Serial.printf("# VESC UART TX=%d RX=%d @ %d\n", VESC_UART_TX, VESC_UART_RX, VESC_UART_BAUD);
}

void vescLinkLoop() {}

void vescLinkDrain() {
  while (VescSerial.available() > 0) {
    (void)VescSerial.read();
  }
}

int vescLinkRead(uint8_t *buf, int maxLen) {
  if (buf == nullptr || maxLen <= 0) {
    return 0;
  }
  int n = 0;
  while (n < maxLen && VescSerial.available() > 0) {
    buf[n++] = (uint8_t)VescSerial.read();
  }
  return n;
}

void vescLinkWrite(const uint8_t *data, int len) {
  if (data == nullptr || len <= 0) {
    return;
  }
  VescSerial.write(data, (size_t)len);
}

void vescLinkSendZeroDuty() {
  // COMM_SET_DUTY = 5, value 0. Built here so the bridge does not pull in vesc_protocol.
  uint8_t payload[5] = {5, 0, 0, 0, 0};
  uint16_t crc = 0;
  for (size_t i = 0; i < sizeof(payload); i++) {
    crc ^= (uint16_t)payload[i] << 8;
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  const uint8_t frame[10] = {0x02,
                             5,
                             5,
                             0,
                             0,
                             0,
                             0,
                             (uint8_t)(crc >> 8),
                             (uint8_t)(crc & 0xFF),
                             0x03};
  VescSerial.write(frame, sizeof(frame));
}
