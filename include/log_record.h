#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

inline bool formatLogRecord(char *output, size_t capacity, uint32_t timestampMillis,
    uint32_t identifier, bool extended, unsigned int length, const uint8_t *data) {
  if (length > 8 || capacity == 0 || (length > 0 && data == nullptr)) {
    return false;
  }
  char timestamp[10];
  size_t digits = 0;
  do {
    timestamp[digits++] = static_cast<char>('0' + timestampMillis % 10);
    timestampMillis /= 10;
  } while (timestampMillis);

  identifier &= extended ? 0x1FFFFFFFUL : 0x7FFUL;
  size_t idDigits = 3;
  for (uint32_t value = identifier >> 12; value; value >>= 4) ++idDigits;
  const char *suffix = extended ? ",true,Rx,0," : ",false,Rx,0,";
  const size_t suffixLength = strlen(suffix);
  const size_t required = digits + 3 + 1 + idDigits + suffixLength + 1 + 3 * length;
  if (required >= capacity) return false;

  size_t used = 0;
  while (digits) output[used++] = timestamp[--digits];
  memcpy(output + used, "000,", 4);
  used += 4;
  static const char hex[] = "0123456789ABCDEF";
  for (size_t digit = idDigits; digit; --digit) {
    output[used++] = hex[(identifier >> ((digit - 1) * 4)) & 0xF];
  }
  memcpy(output + used, suffix, suffixLength);
  used += suffixLength;
  output[used++] = static_cast<char>('0' + length);
  for (unsigned int i = 0; i < length; ++i) {
    output[used++] = ',';
    output[used++] = hex[data[i] >> 4];
    output[used++] = hex[data[i] & 0xF];
  }
  output[used] = '\0';
  return true;
}