#include "../include/binary_log.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main() {
  FrameQueue<32> queue;
  LoggedFrame frame = {};
  frame.timestampMillis = 0x12345678;
  frame.identifier = 0x1ABCDE;
  frame.extended = true;
  frame.length = 2;
  frame.data[0] = 0xAB;
  frame.data[1] = 0xCD;
  frame.data[2] = 0xFF;
  for (int i = 0; i < 30; ++i) assert(queue.push(frame));
  char buf[512];
  CsvBatch batch = BinaryLog::prepare(queue, buf, sizeof(buf), 8, 0, 24);
  assert(batch.rows == 24 && batch.bytes == 24 * BinaryLog::RecordSize);
  const unsigned char expected[] = {0x78, 0x56, 0x34, 0x12, 0xDE, 0xBC, 0x1A, 0x80,
                                    2, 0xAB, 0xCD, 0, 0, 0, 0, 0, 0};
  assert(memcmp(buf, expected, sizeof(expected)) == 0);
  batch = BinaryLog::prepare(queue, buf, sizeof(buf),
      LogPolicy::SegmentBytes - 2 * BinaryLog::RecordSize, 0, 24);
  assert(batch.rows == 2 && batch.stop == BatchStop::Rotate);
  removeAttemptedBatch(queue, batch.rows);
  assert(queue.size() == 28);
  batch = BinaryLog::prepare(queue, buf, 16, 0, 0, 24);
  assert(batch.rows == 0 && batch.stop == BatchStop::NoSpace);
  batch = BinaryLog::prepare(queue, buf, sizeof(buf), 0, LogPolicy::SegmentMillis, 24);
  assert(batch.rows == 0 && batch.stop == BatchStop::Rotate);
  frame.length = 9;
  FrameQueue<1> invalid;
  assert(invalid.push(frame));
  batch = BinaryLog::prepare(invalid, buf, sizeof(buf), 0, 0, 24);
  assert(batch.rows == 0 && batch.stop == BatchStop::Invalid);
  puts("PASS: binary record encoding, bounds and rotation");
}
