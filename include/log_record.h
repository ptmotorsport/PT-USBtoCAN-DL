#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

inline bool formatLogRecord(char *output, size_t capacity, uint32_t timestampMillis,
    uint32_t identifier, bool extended, unsigned int length, const uint8_t *data) {
  if (length > 8 || capacity == 0 || (length > 0 && data == nullptr)) {
    return false;
  }
  identifier &= extended ? 0x1FFFFFFFUL : 0x7FFUL;
  int used = snprintf(output, capacity, "%lu000,%03lX,%s,Rx,0,%u",
      static_cast<unsigned long>(timestampMillis), static_cast<unsigned long>(identifier),
      extended ? "true" : "false", length);
  if (used < 0 || static_cast<size_t>(used) >= capacity) {
    return false;
  }
  for (unsigned int dataIndex = 0; dataIndex < length; ++dataIndex) {
    const int appended = snprintf(output + used, capacity - used, ",%02X",
        static_cast<unsigned int>(data[dataIndex]));
    if (appended < 0 || static_cast<size_t>(appended) >= capacity - used) {
      return false;
    }
    used += appended;
  }
  return true;
}