#pragma once

#include <stddef.h>
#include <stdint.h>
#include <iomanip>
#include <sstream>
#include <string>

#define F(text) text
#define HEX 16

class Print {
public:
  std::string output;

  template <typename T> void print(const T &value) {
    std::ostringstream stream;
    stream << value;
    output += stream.str();
  }
  void print(uint8_t value) { print(static_cast<unsigned int>(value)); }
  void print(uint8_t value, int base) {
    std::ostringstream stream;
    if (base == HEX) stream << std::hex << std::uppercase;
    stream << static_cast<unsigned int>(value);
    output += stream.str();
  }
  void print(double value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    output += stream.str();
  }
  template <typename T> void println(const T &value) { print(value); println(); }
  void println(double value, int precision) { print(value, precision); println(); }
  void println() { output += '\n'; }
};

unsigned long millis();
void yield();
