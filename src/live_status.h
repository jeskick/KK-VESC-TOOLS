#pragma once

#include <stddef.h>
#include <stdint.h>

void liveStatusSetThrottle(int ppm, int pct, float mv, const char *dir, const char *mode);
void liveStatusSetVesc(bool connected, const char *fw, float vIn, float iIn, float iMotor, float rpm,
                       float duty, float tFet, float tMotor, uint8_t fault);
size_t liveStatusToJson(char *buf, size_t bufLen);
