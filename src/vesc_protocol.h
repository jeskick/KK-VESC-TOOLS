#pragma once

#include <stddef.h>
#include <stdint.h>

enum VescComm : uint8_t {
  COMM_FW_VERSION = 0,
  COMM_GET_VALUES = 4,
  COMM_SET_MCCONF = 13,
  COMM_GET_MCCONF = 14,
  COMM_GET_VALUES_SETUP = 47,
  COMM_GET_DECODED_PPM = 55,
};

struct VescValues {
  bool valid;
  float tempFetC;
  float tempMotorC;
  float currentMotorA;
  float currentInA;
  float duty;
  float rpm;
  float voltageInV;
  float ampHours;
  float ampHoursCharged;
  float wattHours;
  float wattHoursCharged;
  uint8_t fault;
};

struct VescSetupValues {
  bool valid;
  float speedMps;
  float batteryLevel;
  float batteryWh;
  float ampHours;
  float wattHours;
  float voltageInV;
  float rpm;
};

uint16_t vescCrc16(const uint8_t *data, size_t len);
size_t vescBuildPacket(uint8_t cmd, const uint8_t *data, size_t dataLen, uint8_t *out, size_t outMax);
bool vescPacketFeed(uint8_t byte, uint8_t *payload, size_t payloadMax, size_t *payloadLen);
void vescPacketParserReset();
bool vescParseGetValues(const uint8_t *payload, size_t len, VescValues *out);
bool vescParseGetValuesSetup(const uint8_t *payload, size_t len, VescSetupValues *out);
bool vescParseDecodedPpm(const uint8_t *payload, size_t len, float *ppmUs);
