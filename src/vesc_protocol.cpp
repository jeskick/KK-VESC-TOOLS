#include "vesc_protocol.h"

#include <string.h>

uint16_t vescCrc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

size_t vescBuildPacket(uint8_t cmd, const uint8_t *data, size_t dataLen, uint8_t *out,
                       size_t outMax) {
  const size_t plen = 1 + dataLen;
  if (plen < 256) {
    if (outMax < plen + 5) {
      return 0;
    }
    uint8_t payload[256];
    payload[0] = cmd;
    if (data != nullptr && dataLen > 0) {
      memcpy(&payload[1], data, dataLen);
    }
    const uint16_t crc = vescCrc16(payload, plen);
    out[0] = 0x02;
    out[1] = (uint8_t)plen;
    memcpy(&out[2], payload, plen);
    out[2 + plen] = (uint8_t)(crc >> 8);
    out[3 + plen] = (uint8_t)(crc & 0xFF);
    out[4 + plen] = 0x03;
    return 5 + plen;
  }
  if (outMax < plen + 6) {
    return 0;
  }
  uint8_t payload[512];
  payload[0] = cmd;
  if (data != nullptr && dataLen > 0) {
    memcpy(&payload[1], data, dataLen);
  }
  const uint16_t crc = vescCrc16(payload, plen);
  out[0] = 0x03;
  out[1] = (uint8_t)(plen >> 8);
  out[2] = (uint8_t)(plen & 0xFF);
  memcpy(&out[3], payload, plen);
  out[3 + plen] = (uint8_t)(crc >> 8);
  out[4 + plen] = (uint8_t)(crc & 0xFF);
  out[5 + plen] = 0x03;
  return 6 + plen;
}

static int32_t readBeI32(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]);
}

static int16_t readBeI16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] << 8 | p[1]);
}

enum VescParseState {
  VESC_WAIT_START,
  VESC_WAIT_LEN_SHORT,
  VESC_WAIT_LEN_HI,
  VESC_WAIT_LEN_LO,
  VESC_WAIT_PAYLOAD,
  VESC_WAIT_CRC_HI,
  VESC_WAIT_CRC_LO,
  VESC_WAIT_STOP,
};

static VescParseState parseState = VESC_WAIT_START;
static size_t parseLen = 0;
static size_t parseIdx = 0;
static uint16_t parseCrcRx = 0;

void vescPacketParserReset() {
  parseState = VESC_WAIT_START;
  parseLen = 0;
  parseIdx = 0;
  parseCrcRx = 0;
}

bool vescPacketFeed(uint8_t byte, uint8_t *payload, size_t payloadMax, size_t *payloadLen) {
  switch (parseState) {
  case VESC_WAIT_START:
    if (byte == 0x02) {
      parseState = VESC_WAIT_LEN_SHORT;
    } else if (byte == 0x03) {
      parseState = VESC_WAIT_LEN_HI;
    }
    break;
  case VESC_WAIT_LEN_SHORT:
    parseLen = byte;
    parseIdx = 0;
    if (parseLen == 0 || parseLen > payloadMax) {
      parseState = VESC_WAIT_START;
      break;
    }
    parseState = VESC_WAIT_PAYLOAD;
    break;
  case VESC_WAIT_LEN_HI:
    parseLen = (size_t)byte << 8;
    parseState = VESC_WAIT_LEN_LO;
    break;
  case VESC_WAIT_LEN_LO:
    parseLen |= byte;
    parseIdx = 0;
    if (parseLen == 0 || parseLen > payloadMax) {
      parseState = VESC_WAIT_START;
      break;
    }
    parseState = VESC_WAIT_PAYLOAD;
    break;
  case VESC_WAIT_PAYLOAD:
    payload[parseIdx++] = byte;
    if (parseIdx >= parseLen) {
      parseState = VESC_WAIT_CRC_HI;
    }
    break;
  case VESC_WAIT_CRC_HI:
    parseCrcRx = (uint16_t)byte << 8;
    parseState = VESC_WAIT_CRC_LO;
    break;
  case VESC_WAIT_CRC_LO:
    parseCrcRx |= byte;
    parseState = VESC_WAIT_STOP;
    break;
  case VESC_WAIT_STOP:
    parseState = VESC_WAIT_START;
    if (byte != 0x03) {
      return false;
    }
    if (vescCrc16(payload, parseLen) != parseCrcRx) {
      return false;
    }
    *payloadLen = parseLen;
    return true;
  }
  return false;
}

bool vescParseGetValues(const uint8_t *payload, size_t len, VescValues *out) {
  if (out == nullptr || len < 56 || payload[0] != COMM_GET_VALUES) {
    return false;
  }
  out->valid = true;
  out->tempFetC = readBeI16(&payload[1]) / 10.0f;
  out->tempMotorC = readBeI16(&payload[3]) / 10.0f;
  out->currentMotorA = readBeI32(&payload[5]) / 100.0f;
  out->currentInA = readBeI32(&payload[9]) / 100.0f;
  out->duty = readBeI16(&payload[21]) / 1000.0f;
  out->rpm = (float)readBeI32(&payload[23]);
  out->voltageInV = readBeI16(&payload[27]) / 10.0f;
  if (len >= 45) {
    out->ampHours = readBeI32(&payload[29]) / 10000.0f;
    out->ampHoursCharged = readBeI32(&payload[33]) / 10000.0f;
    out->wattHours = readBeI32(&payload[37]) / 10000.0f;
    out->wattHoursCharged = readBeI32(&payload[41]) / 10000.0f;
  }
  if (len >= 54) {
    out->fault = payload[53];
  } else if (len >= 44) {
    out->fault = 0;
  }
  return true;
}

bool vescParseGetValuesSetup(const uint8_t *payload, size_t len, VescSetupValues *out) {
  if (out == nullptr || len < 58 || payload[0] != COMM_GET_VALUES_SETUP) {
    return false;
  }
  size_t ind = 1;
  ind += 2;  // temp_mos
  ind += 2;  // temp_motor
  ind += 4;  // current_motor
  ind += 4;  // current_in
  ind += 2;  // duty
  out->rpm = (float)readBeI32(&payload[ind]);
  ind += 4;
  out->speedMps = readBeI32(&payload[ind]) / 1000.0f;
  ind += 4;
  out->voltageInV = readBeI16(&payload[ind]) / 10.0f;
  ind += 2;
  out->batteryLevel = readBeI16(&payload[ind]) / 1000.0f;
  ind += 2;
  out->ampHours = readBeI32(&payload[ind]) / 10000.0f;
  ind += 4;
  ind += 4;  // amp_hours_charged
  out->wattHours = readBeI32(&payload[ind]) / 10000.0f;
  ind += 4;
  ind += 4;  // watt_hours_charged
  ind += 4;  // tachometer
  ind += 4;  // tachometer_abs
  ind += 4;  // position
  ind += 1;  // fault
  ind += 1;  // vesc_id
  ind += 1;  // num_vescs
  out->batteryWh = readBeI32(&payload[ind]) / 1000.0f;
  out->valid = true;
  return true;
}

static float readBeF32(const uint8_t *p) {
  union {
    uint32_t u;
    float f;
  } v;
  v.u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
  return v.f;
}

bool vescParseDecodedPpm(const uint8_t *payload, size_t len, float *ppmUs) {
  if (ppmUs == nullptr || len < 5 || payload[0] != COMM_GET_DECODED_PPM) {
    return false;
  }
  *ppmUs = readBeF32(&payload[1]);
  return true;
}
