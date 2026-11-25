#include "../include/adc_sample_queue.h"
#include "../include/adc_binary_log.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main() {
  AdcSampleQueue<4> queue;
  uint16_t value = 0;
  assert(!queue.pop(value));
  assert(queue.push(0));
  assert(queue.push(16383));
  assert(queue.push(12));
  assert(!queue.push(99));
  assert(queue.size() == 3);
  assert(queue.pop(value) && value == 0);
  assert(queue.push(42));
  assert(queue.size() == 3);
  assert(queue.pop(value) && value == 16383);
  assert(queue.pop(value) && value == 12);
  assert(queue.pop(value) && value == 42);
  assert(!queue.pop(value));
  assert(queue.size() == 0);

  const uint8_t expectedHeader[] = {
      'P', 'T', 'A', 'D', 'C', 2, 14, 0, 0x10, 0x27, 0, 0,
      0xc0, 0xc6, 0x2d, 0, 1, 0, 0, 0
  };
  assert(AdcBinaryLog::HeaderSize == sizeof(expectedHeader));
  assert(memcmp(AdcBinaryLog::Header, expectedHeader, sizeof(expectedHeader)) == 0);
  uint8_t binary[12] = {};
  AdcBinaryLog::encodeSample(binary, {0, 0});
  AdcBinaryLog::encodeSample(binary + 6, {16383, 30000});
  const uint8_t expectedSamples[] = {0, 0, 0, 0, 0, 0, 0xff, 0x3f, 0x30, 0x75, 0, 0};
  assert(memcmp(binary, expectedSamples, sizeof(expectedSamples)) == 0);
  AdcSampleQueue<4, AdcBinaryLog::Sample> combined;
  assert(combined.push({42, 30000}));
  AdcBinaryLog::Sample result;
  assert(combined.pop(result));
  assert(result.adc == 42 && result.tachPeriodTicks == 30000);
  puts("PASS: ADC queue, binary header and little-endian 14-bit samples");
}
