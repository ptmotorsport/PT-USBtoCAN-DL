#pragma once

#include "frame_queue.h"
#include "log_policy.h"
#include "log_record.h"

enum class BatchStop { Ready, Rotate, Invalid, NoSpace };

struct CsvBatch {
  size_t bytes;
  size_t rows;
  BatchStop stop;
};

template <size_t Capacity>
CsvBatch prepareCsvBatch(const FrameQueue<Capacity> &queue, char *buffer, size_t capacity,
    uint32_t position, uint32_t elapsed, size_t maxRows) {
  CsvBatch batch = {0, 0, BatchStop::Ready};
  while (batch.rows < queue.size() && batch.rows < maxRows) {
    const LoggedFrame &frame = queue.at(batch.rows);
    char line[100];
    if (!formatLogRecord(line, sizeof(line), frame.timestampMillis, frame.identifier,
        frame.extended, frame.length, frame.data)) {
      batch.stop = BatchStop::Invalid;
      break;
    }
    const size_t length = strlen(line);
    const size_t rowBytes = length + 2;
    if (LogPolicy::needsRotation(position, batch.bytes + rowBytes, elapsed)) {
      batch.stop = BatchStop::Rotate;
      break;
    }
    if (rowBytes > capacity - batch.bytes) {
      batch.stop = BatchStop::NoSpace;
      break;
    }
    memcpy(buffer + batch.bytes, line, length);
    batch.bytes += length;
    buffer[batch.bytes++] = '\r';
    buffer[batch.bytes++] = '\n';
    ++batch.rows;
  }
  return batch;
}

template <size_t Capacity>
void removeAttemptedBatch(FrameQueue<Capacity> &queue, size_t rows) {
  while (rows--) queue.pop();
}
