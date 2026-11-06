#pragma once
#include <stddef.h>
#include <stdint.h>

namespace LogPolicy {
constexpr uint32_t SegmentBytes = 4UL * 1024 * 1024;
constexpr uint32_t SegmentMillis = 300000;
constexpr uint32_t SyncMillis = 1000;

inline bool needsRotation(uint32_t position, size_t rowBytes, uint32_t elapsed) {
  return position > SegmentBytes || rowBytes > SegmentBytes - position || elapsed >= SegmentMillis;
}

template <typename File>
bool sync(File &file, uint32_t &confirmedBytes) {
  if (!file.sync()) return false;
  confirmedBytes = file.curPosition();
  return true;
}
}