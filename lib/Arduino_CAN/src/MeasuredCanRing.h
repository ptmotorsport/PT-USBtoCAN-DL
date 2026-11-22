#pragma once

#include <stddef.h>
#include <stdint.h>

template <typename Message, size_t Capacity>
class MeasuredCanRing {
public:
  void enqueue(const Message &message) {
    if (count_ == Capacity) {
      if (drops_ != UINT32_MAX) ++drops_;
      return;
    }
    buffer_[head_] = message;
    head_ = (head_ + 1) % Capacity;
    ++count_;
    if (count_ > highWater_) highWater_ = count_;
  }

  Message dequeue() {
    if (!count_) return Message();
    Message message = buffer_[tail_];
    tail_ = (tail_ + 1) % Capacity;
    --count_;
    return message;
  }

  size_t available() const { return count_; }
  size_t highWater() const { return highWater_; }
  uint32_t drops() const { return drops_; }

private:
  Message buffer_[Capacity];
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t count_ = 0;
  size_t highWater_ = 0;
  uint32_t drops_ = 0;
};
