#pragma once

#include <stddef.h>
#include <stdint.h>

void vescBleBegin();
void vescBleEnd();
void vescBleLoop();
bool vescBleWrite(const uint8_t *data, size_t len);
int vescBleAvailable();
int vescBleRead();
bool vescBleConnected();
// "off" / "scan" / "conn" / "ok" / "fail"
const char *vescBleState();
