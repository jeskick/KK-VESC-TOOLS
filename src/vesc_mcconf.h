#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// Serialized MCCONF signatures (confgenerator.h)
#define VESC_MCCONF_SIG_602 776184161u
#define VESC_MCCONF_SIG_605 1065524471u

struct VescBatteryLimits {
  bool valid;
  uint32_t signature;
  float motorCurrentMaxA;
  float motorCurrentMinA;
  float inCurrentMaxA;
  float minVinV;
  float maxVinV;
  float cutStartV;
  float cutEndV;
  float wattMaxW;
  float batteryAh;
  uint8_t batteryCells;
  uint8_t batteryType;
  uint8_t invertDirection;
};

// confData = serialized mcconf starting at signature (no COMM byte)
bool vescMcconfParseLimits(const uint8_t *confData, size_t len, VescBatteryLimits *out);

// Patch fields in-place. Only non-NaN / cells!=0xFF fields applied if useMask bits set.
struct VescBatteryPatch {
  bool setMotorCurrentMax;
  bool setMotorCurrentMin;
  bool setInCurrentMax;
  bool setMinVin;
  bool setMaxVin;
  bool setCutStart;
  bool setCutEnd;
  bool setWattMax;
  bool setCells;
  bool setBatteryAh;
  bool setBatteryType;
  bool setInvert;
  float motorCurrentMaxA;
  float motorCurrentMinA;
  float inCurrentMaxA;
  float minVinV;
  float maxVinV;
  float cutStartV;
  float cutEndV;
  float wattMaxW;
  float batteryAh;
  uint8_t batteryCells;
  uint8_t batteryType;
  uint8_t invertDirection;
};

bool vescMcconfPatchLimits(uint8_t *confData, size_t len, const VescBatteryPatch *patch);
