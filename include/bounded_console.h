#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

class BoundedConsoleBuffer {
public:
  static constexpr size_t Capacity = 4096;

  size_t write(uint8_t value) { return write(&value, 1); }

  size_t write(const uint8_t *data, size_t length) {
    const size_t accepted = length < Capacity - used_ ? length : Capacity - used_;
    const size_t tail = (head_ + used_) % Capacity;
    const size_t first = accepted < Capacity - tail ? accepted : Capacity - tail;
    if (first != 0) memcpy(bytes_ + tail, data, first);
    if (accepted > first) memcpy(bytes_, data + first, accepted - first);
    used_ += accepted;
    const size_t lost = length - accepted;
    dropped_ = lost > UINT32_MAX - dropped_ ? UINT32_MAX : dropped_ + lost;
    return accepted;
  }

  template <typename Transport>
  size_t drain(Transport &transport, size_t budget) {
    const bool connected = static_cast<bool>(transport);
    if (!connected || used_ == 0 || budget == 0) return 0;
    const int space = transport.availableForWrite();
    if (space <= 0) return 0;
    size_t length = used_ < Capacity - head_ ? used_ : Capacity - head_;
    if (length > budget) length = budget;
    if (length > static_cast<size_t>(space)) length = static_cast<size_t>(space);
    const size_t written = transport.write(bytes_ + head_, length);
    head_ = (head_ + written) % Capacity;
    used_ -= written;
    return written;
  }

  bool empty() const { return used_ == 0; }
  size_t size() const { return used_; }
  uint32_t droppedBytes() const { return dropped_; }
  void clear() { head_ = 0; used_ = 0; }

private:
  uint8_t bytes_[Capacity] = {};
  size_t head_ = 0;
  size_t used_ = 0;
  uint32_t dropped_ = 0;
};