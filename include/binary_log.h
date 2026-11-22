#pragma once

#include "csv_batch.h"

namespace BinaryLog {
constexpr size_t HeaderSize = 8;
constexpr size_t RecordSize = 17;
const uint8_t Header[HeaderSize] = {'P', 'T', 'C', 'A', 'N', 1, RecordSize, 0};

inline void write32(char *p, uint32_t value) {
  for (unsigned i = 0; i < 4; ++i) p[i] = static_cast<char>(value >> (8 * i));
}

template <size_t Capacity>
CsvBatch prepare(const FrameQueue<Capacity> &queue, char *buffer, size_t capacity,
    uint32_t position, uint32_t elapsed, size_t maxRows) {
  CsvBatch batch = {0, 0, BatchStop::Ready};
  while (batch.rows < queue.size() && batch.rows < maxRows) {
    const LoggedFrame &frame = queue.at(batch.rows);
    if (frame.length > 8 || frame.identifier > (frame.extended ? 0x1FFFFFFFUL : 0x7FFUL)) {
      batch.stop = BatchStop::Invalid;
      break;
    }
    if (LogPolicy::needsRotation(position, batch.bytes + RecordSize, elapsed)) {
      batch.stop = BatchStop::Rotate;
      break;
    }
    if (RecordSize > capacity - batch.bytes) {
      batch.stop = BatchStop::NoSpace;
      break;
    }
    char *record = buffer + batch.bytes;
    write32(record, frame.timestampMillis);
    write32(record + 4, frame.identifier | (frame.extended ? 0x80000000UL : 0));
    record[8] = static_cast<char>(frame.length);
    for (size_t i = 0; i < 8; ++i)
      record[9 + i] = i < frame.length ? static_cast<char>(frame.data[i]) : 0;
    batch.bytes += RecordSize;
    ++batch.rows;
  }
  return batch;
}
}
