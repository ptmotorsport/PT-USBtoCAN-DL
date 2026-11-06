#pragma once
#include <stddef.h>
#include <stdint.h>

struct LoggedFrame {
  uint32_t timestampMillis;
  uint32_t identifier;
  uint8_t length;
  bool extended;
  uint8_t data[8];
};

template <size_t Capacity> class FrameQueue {
 public:
  bool push(const LoggedFrame &frame) {
    if (count_ == Capacity) return false;
    frames_[(head_ + count_) % Capacity] = frame;
    ++count_;
    return true;
  }
  const LoggedFrame &front() const { return frames_[head_]; }
  void pop() {
    if (count_) { head_ = (head_ + 1) % Capacity; --count_; }
  }
  size_t size() const { return count_; }
 private:
  LoggedFrame frames_[Capacity];
  size_t head_ = 0;
  size_t count_ = 0;
};