#include "vesc_mcconf.h"

#include <math.h>
#include <string.h>

// release_6_02 confgenerator_serialize_mcconf fixed offsets (signature at 0)
enum {
  OFF602_CURRENT_MAX = 8,
  OFF602_CURRENT_MIN = 12,
  OFF602_IN_CURRENT_MAX = 16,
  OFF602_MIN_VIN = 46,
  OFF602_MAX_VIN = 50,
  OFF602_CUT_START = 54,
  OFF602_CUT_END = 58,
  OFF602_WATT_MAX = 77,
  OFF602_INVERT = 420,
  OFF602_TYPE = 460,
  OFF602_CELLS = 461,
  OFF602_AH = 462,
  LEN602_MIN = 466,
};

static uint32_t getU32(const uint8_t *b, int32_t *i) {
  uint32_t r = ((uint32_t)b[*i] << 24) | ((uint32_t)b[*i + 1] << 16) | ((uint32_t)b[*i + 2] << 8) |
               (uint32_t)b[*i + 3];
  *i += 4;
  return r;
}

static void putU32(uint8_t *b, int32_t *i, uint32_t v) {
  b[(*i)++] = (uint8_t)(v >> 24);
  b[(*i)++] = (uint8_t)(v >> 16);
  b[(*i)++] = (uint8_t)(v >> 8);
  b[(*i)++] = (uint8_t)v;
}

static float getF32Auto(const uint8_t *b, int32_t *i) {
  uint32_t res = getU32(b, i);
  int e = (int)((res >> 23) & 0xFF);
  uint32_t sig_i = res & 0x7FFFFF;
  bool neg = (res & (1U << 31)) != 0;
  float sig = 0.0f;
  if (e != 0 || sig_i != 0) {
    sig = (float)sig_i / (8388608.0f * 2.0f) + 0.5f;
    e -= 126;
  }
  if (neg) {
    sig = -sig;
  }
  return ldexpf(sig, e);
}

static void putF32Auto(uint8_t *b, int32_t *i, float number) {
  if (fabsf(number) < 1.5e-38f) {
    number = 0.0f;
  }
  int e = 0;
  float sig = frexpf(number, &e);
  float sig_abs = fabsf(sig);
  uint32_t sig_i = 0;
  if (sig_abs >= 0.5f) {
    sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
    e += 126;
  }
  uint32_t res = ((uint32_t)(e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
  if (sig < 0) {
    res |= 1U << 31;
  }
  putU32(b, i, res);
}

static float getF16(const uint8_t *b, int32_t *i, float scale) {
  int16_t v = (int16_t)(((uint16_t)b[*i] << 8) | b[*i + 1]);
  *i += 2;
  return (float)v / scale;
}

static void putF16(uint8_t *b, int32_t *i, float number, float scale) {
  int16_t v = (int16_t)(number * scale);
  b[(*i)++] = (uint8_t)(v >> 8);
  b[(*i)++] = (uint8_t)v;
}

static float readF32At(const uint8_t *b, int off) {
  int32_t i = off;
  return getF32Auto(b, &i);
}

static void writeF32At(uint8_t *b, int off, float v) {
  int32_t i = off;
  putF32Auto(b, &i, v);
}

static void writeF16At(uint8_t *b, int off, float v, float scale) {
  int32_t i = off;
  putF16(b, &i, v, scale);
}

static bool parse602(const uint8_t *b, size_t len, VescBatteryLimits *out) {
  if (len < LEN602_MIN) {
    return false;
  }
  int32_t i = 0;
  uint32_t sig = getU32(b, &i);
  if (sig != VESC_MCCONF_SIG_602) {
    return false;
  }
  out->signature = sig;
  out->motorCurrentMaxA = readF32At(b, OFF602_CURRENT_MAX);
  out->motorCurrentMinA = readF32At(b, OFF602_CURRENT_MIN);
  out->inCurrentMaxA = readF32At(b, OFF602_IN_CURRENT_MAX);
  out->minVinV = readF32At(b, OFF602_MIN_VIN);
  out->maxVinV = readF32At(b, OFF602_MAX_VIN);
  out->cutStartV = readF32At(b, OFF602_CUT_START);
  out->cutEndV = readF32At(b, OFF602_CUT_END);
  out->wattMaxW = readF32At(b, OFF602_WATT_MAX);
  out->batteryType = b[OFF602_TYPE];
  out->batteryCells = b[OFF602_CELLS];
  out->batteryAh = readF32At(b, OFF602_AH);
  out->invertDirection = b[OFF602_INVERT] ? 1 : 0;
  out->valid = true;
  return true;
}

// 6.05 early fields only (vin/cuts are float16); cells not located here
static bool parse605Early(const uint8_t *b, size_t len, VescBatteryLimits *out) {
  if (len < 80) {
    return false;
  }
  int32_t i = 0;
  uint32_t sig = getU32(b, &i);
  if (sig != VESC_MCCONF_SIG_605) {
    return false;
  }
  out->signature = sig;
  i += 4;  // modes
  out->motorCurrentMaxA = getF32Auto(b, &i);
  getF32Auto(b, &i);
  out->inCurrentMaxA = getF32Auto(b, &i);
  getF32Auto(b, &i);
  getF16(b, &i, 10000);
  getF16(b, &i, 10000);
  getF32Auto(b, &i);
  getF32Auto(b, &i);
  getF32Auto(b, &i);
  getF16(b, &i, 10000);
  getF32Auto(b, &i);
  getF32Auto(b, &i);
  out->minVinV = getF16(b, &i, 10);
  out->maxVinV = getF16(b, &i, 10);
  out->cutStartV = getF16(b, &i, 10);
  out->cutEndV = getF16(b, &i, 10);
  getF16(b, &i, 10);
  getF16(b, &i, 10);
  i += 1;
  i += 4;
  getF16(b, &i, 10000);
  getF16(b, &i, 10000);
  getF16(b, &i, 10000);
  out->wattMaxW = getF32Auto(b, &i);
  out->batteryCells = 0;
  out->valid = true;
  return true;
}

bool vescMcconfParseLimits(const uint8_t *confData, size_t len, VescBatteryLimits *out) {
  if (confData == nullptr || out == nullptr || len < 8) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  int32_t i = 0;
  uint32_t sig = getU32(confData, &i);
  out->signature = sig;
  if (sig == VESC_MCCONF_SIG_602) {
    return parse602(confData, len, out);
  }
  if (sig == VESC_MCCONF_SIG_605) {
    return parse605Early(confData, len, out);
  }
  out->valid = false;
  return false;
}

bool vescMcconfPatchLimits(uint8_t *confData, size_t len, const VescBatteryPatch *patch) {
  if (confData == nullptr || patch == nullptr || len < 8) {
    return false;
  }
  int32_t i = 0;
  uint32_t sig = getU32(confData, &i);

  if (sig == VESC_MCCONF_SIG_602) {
    if (len < LEN602_MIN) {
      return false;
    }
    if (patch->setMotorCurrentMax) {
      writeF32At(confData, OFF602_CURRENT_MAX, patch->motorCurrentMaxA);
    }
    if (patch->setMotorCurrentMin) {
      writeF32At(confData, OFF602_CURRENT_MIN, patch->motorCurrentMinA);
    }
    if (patch->setInCurrentMax) {
      writeF32At(confData, OFF602_IN_CURRENT_MAX, patch->inCurrentMaxA);
    }
    if (patch->setMinVin) {
      writeF32At(confData, OFF602_MIN_VIN, patch->minVinV);
    }
    if (patch->setMaxVin) {
      writeF32At(confData, OFF602_MAX_VIN, patch->maxVinV);
    }
    if (patch->setCutStart) {
      writeF32At(confData, OFF602_CUT_START, patch->cutStartV);
    }
    if (patch->setCutEnd) {
      writeF32At(confData, OFF602_CUT_END, patch->cutEndV);
    }
    if (patch->setWattMax) {
      writeF32At(confData, OFF602_WATT_MAX, patch->wattMaxW);
    }
    if (patch->setBatteryType) {
      confData[OFF602_TYPE] = patch->batteryType;
    }
    if (patch->setCells) {
      confData[OFF602_CELLS] = patch->batteryCells;
    }
    if (patch->setBatteryAh) {
      writeF32At(confData, OFF602_AH, patch->batteryAh);
    }
    if (patch->setInvert) {
      confData[OFF602_INVERT] = patch->invertDirection ? 1 : 0;
    }
    return true;
  }

  if (sig == VESC_MCCONF_SIG_605) {
    VescBatteryLimits cur = {};
    if (!parse605Early(confData, len, &cur) || !cur.valid) {
      return false;
    }
    i = 4;
    i += 4;
    int offMot = i;
    getF32Auto(confData, &i);
    getF32Auto(confData, &i);
    int offIn = i;
    getF32Auto(confData, &i);
    getF32Auto(confData, &i);
    getF16(confData, &i, 10000);
    getF16(confData, &i, 10000);
    getF32Auto(confData, &i);
    getF32Auto(confData, &i);
    getF32Auto(confData, &i);
    getF16(confData, &i, 10000);
    getF32Auto(confData, &i);
    getF32Auto(confData, &i);
    int offMin = i;
    getF16(confData, &i, 10);
    int offMax = i;
    getF16(confData, &i, 10);
    int offCs = i;
    getF16(confData, &i, 10);
    int offCe = i;
    getF16(confData, &i, 10);
    getF16(confData, &i, 10);
    getF16(confData, &i, 10);
    i += 1;
    i += 4;
    getF16(confData, &i, 10000);
    getF16(confData, &i, 10000);
    getF16(confData, &i, 10000);
    int offW = i;

    if (patch->setMotorCurrentMax) {
      writeF32At(confData, offMot, patch->motorCurrentMaxA);
    }
    if (patch->setInCurrentMax) {
      writeF32At(confData, offIn, patch->inCurrentMaxA);
    }
    if (patch->setMinVin) {
      writeF16At(confData, offMin, patch->minVinV, 10.0f);
    }
    if (patch->setMaxVin) {
      writeF16At(confData, offMax, patch->maxVinV, 10.0f);
    }
    if (patch->setCutStart) {
      writeF16At(confData, offCs, patch->cutStartV, 10.0f);
    }
    if (patch->setCutEnd) {
      writeF16At(confData, offCe, patch->cutEndV, 10.0f);
    }
    if (patch->setWattMax) {
      writeF32At(confData, offW, patch->wattMaxW);
    }
    return true;
  }

  return false;
}
