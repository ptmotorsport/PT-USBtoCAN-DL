#pragma once

#include <stddef.h>
#include <stdint.h>
#include "tach_period.h"

namespace AdcBinaryLog {
constexpr uint32_t SampleRateHz = 10000;
constexpr uint8_t Header[] = {
    'P', 'T', 'A', 'D', 'C', 2, 14, 0,
    static_cast<uint8_t>(SampleRateHz),
    static_cast<uint8_t>(SampleRateHz >> 8),
    static_cast<uint8_t>(SampleRateHz >> 16),
    static_cast<uint8_t>(SampleRateHz >> 24),
    static_cast<uint8_t>(TachConfig::TickHz),
    static_cast<uint8_t>(TachConfig::TickHz >> 8),
    static_cast<uint8_t>(TachConfig::TickHz >> 16),
    static_cast<uint8_t>(TachConfig::TickHz >> 24),
    static_cast<uint8_t>(TachConfig::PulsesPerRevolution), 0, 0, 0
};
constexpr size_t HeaderSize = sizeof(Header);
constexpr size_t RecordSize = 6;

struct Sample {
  uint16_t adc;
  uint32_t tachPeriodTicks;
};

inline void encodeSample(uint8_t *destination, const Sample &sample) {
  destination[0] = static_cast<uint8_t>(sample.adc);
  destination[1] = static_cast<uint8_t>(sample.adc >> 8);
  for (size_t i = 0; i < 4; ++i) {
    destination[2 + i] = static_cast<uint8_t>(sample.tachPeriodTicks >> (8 * i));
  }
}
}
