#pragma once

#include <stdint.h>

namespace TachConfig {
constexpr uint32_t TickHz = 3000000;
constexpr uint32_t CounterModulus = UINT32_MAX;
constexpr uint32_t TimeoutTicks = TickHz * 3 / 10;
constexpr uint32_t PulsesPerRevolution = 1;
}

// Accessed only by the same-priority GPT capture and ADC timer callbacks.
class TachPeriod {
 public:
  void edge(uint32_t captured) {
    const uint32_t interval = elapsed(captured, lastEdge_);
    period_ = haveEdge_ && interval < TachConfig::TimeoutTicks ? interval : 0;
    lastEdge_ = captured;
    haveEdge_ = true;
  }

  uint32_t period(uint32_t now) {
    if (haveEdge_ && elapsed(now, lastEdge_) >= TachConfig::TimeoutTicks) {
      haveEdge_ = false;
      period_ = 0;
    }
    return period_;
  }

 private:
  static uint32_t elapsed(uint32_t now, uint32_t before) {
    return now >= before ? now - before :
        TachConfig::CounterModulus - before + now;
  }

  uint32_t lastEdge_ = 0;
  uint32_t period_ = 0;
  bool haveEdge_ = false;
};
