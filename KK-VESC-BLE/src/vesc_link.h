#pragma once

#include <stddef.h>
#include <stdint.h>

void vescLinkBegin();
void vescLinkLoop();
void vescLinkDrain();
int vescLinkRead(uint8_t *buf, int maxLen);
void vescLinkWrite(const uint8_t *data, int len);
// COMM_SET_DUTY 0 — call on peer/phone loss so the VESC timeout is not the only brake.
void vescLinkSendZeroDuty();
