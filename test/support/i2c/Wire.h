#pragma once

#include <Arduino.h>
#include <array>
#include <deque>
#include <map>
#include <vector>

class MockWire {
public:
  std::map<uint8_t, std::array<uint8_t, 256>> devices;
  std::vector<uint8_t> tx;
  std::deque<uint8_t> rx;
  uint8_t address = 0;
  uint8_t reg = 0;
  int failAddress = -1;
  uint8_t failCode = 5;
  bool shortRead = false;
  bool ignoreWrites = false;
  unsigned int begins = 0;
  uint32_t clock = 0;

  void begin() { ++begins; }
  void setClock(uint32_t value) { clock = value; }
  void beginTransmission(uint8_t value) { address = value; tx.clear(); }
  size_t write(uint8_t value) { tx.push_back(value); return 1; }
  uint8_t endTransmission(bool = true) {
    if (address == failAddress) return failCode;
    if (devices.count(address) == 0) return 2;
    if (!tx.empty()) reg = tx[0];
    if (tx.size() == 2 && !ignoreWrites) {
      uint8_t value = tx[1];
      if (reg == 0xF4 && value == 0x25) value = 0x24;
      devices[address][reg] = value;
    }
    return 0;
  }
  size_t requestFrom(uint8_t value, size_t length, bool) {
    rx.clear();
    if (shortRead) --length;
    for (size_t i = 0; i < length; ++i) rx.push_back(devices[value][reg + i]);
    return length;
  }
  int read() {
    if (rx.empty()) return -1;
    const int value = rx.front();
    rx.pop_front();
    return value;
  }
};

extern MockWire Wire;
