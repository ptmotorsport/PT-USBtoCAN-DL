#pragma once

#include <stddef.h>
#include <stdint.h>
#include <atomic>

// Single interrupt producer and single main-loop consumer.
template <size_t Capacity, typename Sample = uint16_t>
class AdcSampleQueue {
  static_assert(Capacity > 1, "ADC queue must have at least two slots");
  static_assert(__atomic_always_lock_free(sizeof(size_t), nullptr),
      "ADC queue indices must be lock-free in the interrupt");

 public:
  bool push(Sample value) {
    const size_t head = head_.load(std::memory_order_relaxed);
    const size_t next = (head + 1) % Capacity;
    if (next == tail_.load(std::memory_order_acquire)) return false;
    samples_[head] = value;
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool pop(Sample &value) {
    const size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    value = samples_[tail];
    tail_.store((tail + 1) % Capacity, std::memory_order_release);
    return true;
  }

  size_t size() const {
    const size_t head = head_.load(std::memory_order_acquire);
    const size_t tail = tail_.load(std::memory_order_relaxed);
    return head >= tail ? head - tail : Capacity - tail + head;
  }

 private:
  Sample samples_[Capacity] = {};
  std::atomic<size_t> head_{0};
  std::atomic<size_t> tail_{0};
};
