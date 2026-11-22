#include "../include/log_format.h"
#include <assert.h>
#include <stdio.h>

int main() {
  for (unsigned value = 0; value < 256; ++value) {
    const LogFormat expected = value == BinaryFormatMarker ? LogFormat::Binary : LogFormat::Csv;
    assert(logFormatFromStorage(static_cast<uint8_t>(value)) == expected);
  }
  assert(logFormatFromStorage(logFormatToStorage(LogFormat::Csv)) == LogFormat::Csv);
  assert(logFormatFromStorage(logFormatToStorage(LogFormat::Binary)) == LogFormat::Binary);
  puts("PASS: persisted format selection and erased EEPROM fallback");
}
