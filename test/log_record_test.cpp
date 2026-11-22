#include "../include/log_record.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main() {
  char output[100];
  const uint8_t payload[] = {0x28, 0x00, 0x25, 0x00, 0x26, 0x00, 0x25, 0x00};
  assert(formatLogRecord(output, sizeof(output), 12345, 0x700, false, 8, payload));
  assert(strcmp(output, "12345000,700,false,Rx,0,8,28,00,25,00,26,00,25,00") == 0);
  assert(formatLogRecord(output, sizeof(output), UINT32_MAX, 0x1FFFFFFF, true, 8, payload));
  assert(strcmp(output, "4294967295000,1FFFFFFF,true,Rx,0,8,28,00,25,00,26,00,25,00") == 0);
  assert(formatLogRecord(output, sizeof(output), 12345, 0x98DAF110, true, 0, nullptr));
  assert(strcmp(output, "12345000,18DAF110,true,Rx,0,0") == 0);
  assert(formatLogRecord(output, sizeof(output), 0, 0, false, 0, nullptr));
  assert(strcmp(output, "0000,000,false,Rx,0,0") == 0);
  assert(!formatLogRecord(output, sizeof(output), 1, 0x700, false, 9, payload));
  assert(!formatLogRecord(output, sizeof(output), 1, 0x700, false, 1, nullptr));
  assert(!formatLogRecord(output, 0, 1, 0x700, false, 8, payload));
  char smallOutput[4];
  assert(!formatLogRecord(smallOutput, sizeof(smallOutput), 1, 0x700, false, 8, payload));
  char shortPayload[24];
  assert(!formatLogRecord(shortPayload, sizeof(shortPayload), 1, 0x700, false, 8, payload));
  uint32_t random = 1;
  for (unsigned int i = 0; i < 20000; ++i) {
    random = random * 1664525UL + 1013904223UL;
    const uint32_t timestamp = random;
    random = random * 1664525UL + 1013904223UL;
    const uint32_t id = random;
    const bool extended = (i & 1) != 0;
    const unsigned int length = i % 9;
    char expected[100];
    int used = snprintf(expected, sizeof(expected), "%lu000,%03lX,%s,Rx,0,%u",
        static_cast<unsigned long>(timestamp),
        static_cast<unsigned long>(id & (extended ? 0x1FFFFFFFUL : 0x7FFUL)),
        extended ? "true" : "false", length);
    for (unsigned int j = 0; j < length; ++j) {
      used += snprintf(expected + used, sizeof(expected) - used, ",%02X", payload[j]);
    }
    assert(formatLogRecord(output, sizeof(output), timestamp, id, extended, length, payload));
    assert(strcmp(output, expected) == 0);
    assert(!formatLogRecord(output, static_cast<size_t>(used), timestamp, id, extended, length, payload));
    assert(formatLogRecord(output, static_cast<size_t>(used) + 1, timestamp, id, extended, length, payload));
  }
  puts("PASS: CSV timestamps, IDs, payloads, invalid lengths, and truncation");
}