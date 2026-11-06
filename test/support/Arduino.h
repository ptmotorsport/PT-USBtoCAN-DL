#pragma once

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <deque>
#include <string>

class MockSerial {
public:
  bool connected = true;
  int writable = 64;
  std::deque<uint8_t> input;
  std::string output;
  size_t readCalls = 0;
  size_t writeCalls = 0;

  explicit operator bool() const { return connected; }
  int available() const { return static_cast<int>(input.size()); }
  int availableForWrite() const { return writable; }

  int read() {
    ++readCalls;
    if (input.empty()) return -1;
    const int character = input.front();
    input.pop_front();
    return character;
  }

  size_t write(const uint8_t *data, size_t length) {
    ++writeCalls;
    assert(connected && writable > 0);
    assert(length > 0 && length <= static_cast<size_t>(writable));
    output.append(reinterpret_cast<const char *>(data), length);
    return length;
  }

  void feed(const std::string &text) {
    input.insert(input.end(), text.begin(), text.end());
  }
};

extern MockSerial Serial;
unsigned long millis();