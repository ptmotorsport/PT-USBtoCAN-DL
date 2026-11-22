#include "../include/csv_batch.h"
#include <assert.h>
#include <stdio.h>
#include <string>

int main() {
  FrameQueue<12> queue;
  LoggedFrame frame = {};
  frame.timestampMillis = UINT32_MAX;
  frame.identifier = 0x1FFFFFFF;
  frame.extended = true;
  frame.length = 8;
  for (size_t i = 0; i < 8; ++i) frame.data[i] = static_cast<uint8_t>(i);
  for (unsigned i = 0; i < 10; ++i) assert(queue.push(frame));
  for (unsigned i = 0; i < 3; ++i) queue.pop();
  for (unsigned i = 0; i < 3; ++i) assert(queue.push(frame));

  char expected[100];
  assert(formatLogRecord(expected, sizeof(expected), frame.timestampMillis, frame.identifier,
      frame.extended, frame.length, frame.data));
  const std::string row = std::string(expected) + "\r\n";
  char buffer[512];
  CsvBatch batch = prepareCsvBatch(queue, buffer, sizeof(buffer), 0, 0, 8);
  assert(batch.rows == 8 && batch.bytes == 8 * row.size());
  for (size_t i = 0; i < batch.rows; ++i)
    assert(std::string(buffer + i * row.size(), row.size()) == row);
  assert(queue.size() == 10);

  batch = prepareCsvBatch(queue, buffer, sizeof(buffer),
      LogPolicy::SegmentBytes - 2 * row.size(), 0, 8);
  assert(batch.rows == 2 && batch.stop == BatchStop::Rotate);
  removeAttemptedBatch(queue, batch.rows);
  assert(queue.size() == 8);
  batch = prepareCsvBatch(queue, buffer, sizeof(buffer), LogPolicy::SegmentBytes, 0, 8);
  assert(batch.rows == 0 && batch.stop == BatchStop::Rotate);
  batch = prepareCsvBatch(queue, buffer, sizeof(buffer), 0, LogPolicy::SegmentMillis, 8);
  assert(batch.rows == 0 && batch.stop == BatchStop::Rotate);
  batch = prepareCsvBatch(queue, buffer, row.size() - 1, 0, 0, 8);
  assert(batch.rows == 0 && batch.stop == BatchStop::NoSpace);
  batch = prepareCsvBatch(queue, buffer, 2 * row.size(), 0, 0, 8);
  assert(batch.rows == 2 && batch.stop == BatchStop::NoSpace);
  // Even a short write consumes the attempted rows; later rows remain queued.
  removeAttemptedBatch(queue, batch.rows);
  assert(queue.size() == 6);

  FrameQueue<4> invalid;
  assert(invalid.push(frame));
  frame.length = 9;
  assert(invalid.push(frame));
  batch = prepareCsvBatch(invalid, buffer, sizeof(buffer), 0, 0, 8);
  assert(batch.rows == 1 && batch.stop == BatchStop::Invalid);
  removeAttemptedBatch(invalid, batch.rows);
  batch = prepareCsvBatch(invalid, buffer, sizeof(buffer), 0, 0, 8);
  assert(batch.rows == 0 && batch.stop == BatchStop::Invalid);
  puts("PASS: CSV batch contents, FIFO wrap, rotation, capacity and attempted-row handling");
}
