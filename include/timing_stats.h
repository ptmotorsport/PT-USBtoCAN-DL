#pragma once
#include <stdint.h>

struct TimingStats {
  uint64_t totalMicros = 0;
  uint32_t calls = 0;
  uint32_t maxMicros = 0;

  void record(uint32_t duration) {
    totalMicros += duration;
    if (calls != UINT32_MAX) ++calls;
    if (duration > maxMicros) maxMicros = duration;
  }
};
