#pragma once
#include <stdint.h>
#include <array>

class MockEEPROM {
public:
  std::array<uint8_t,1024> bytes;
  MockEEPROM() { bytes.fill(0xFF); }
  uint8_t read(unsigned i) const { return bytes.at(i); }
  void update(unsigned i,uint8_t value) { bytes.at(i)=value; }
};
extern MockEEPROM EEPROM;
