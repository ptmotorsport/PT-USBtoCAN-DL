#pragma once

#include <stdint.h>

extern bool savvyCAN;

bool configureBridgeCAN(uint32_t bitrate, bool listenOnly);
void closeBridgeCAN();
bool sendBridgeCAN(uint32_t id, bool extended, uint8_t length, const uint8_t *data);

void slcanBegin();
bool slcanInitialCommand(const char *text, unsigned int length);
void slcanProcess();
void slcanReceive(uint32_t id, bool extended, uint8_t length, const uint8_t *data);
bool slcanCanRead();
uint32_t slcanDroppedFrames();