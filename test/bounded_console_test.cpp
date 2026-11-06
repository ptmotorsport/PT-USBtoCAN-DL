#include "../include/bounded_console.h"
#include <assert.h>
#include <stdio.h>
#include <vector>

struct FakeSerial {
  bool connected = true;
  int space = 4096;
  size_t maxWrite = 4096;
  size_t checks = 0;
  size_t writes = 0;
  std::vector<uint8_t> received;

  explicit operator bool() { ++checks; return connected; }
  int availableForWrite() { return space; }
  size_t write(const uint8_t *data, size_t length) {
    assert(connected && space > 0);
    assert(length <= static_cast<size_t>(space));
    ++writes;
    const size_t accepted = length < maxWrite ? length : maxWrite;
    received.insert(received.end(), data, data + accepted);
    space -= static_cast<int>(accepted);
    return accepted;
  }
};

int main() {
  BoundedConsoleBuffer queue;
  FakeSerial serial;
  uint8_t data[BoundedConsoleBuffer::Capacity];
  for (size_t index = 0; index < sizeof(data); ++index) {
    data[index] = static_cast<uint8_t>(index);
  }
  assert(queue.empty());
  assert(queue.write(data, sizeof(data)) == sizeof(data));
  assert(queue.write(static_cast<uint8_t>('!')) == 0);
  assert(queue.droppedBytes() == 1);
  assert(queue.size() == sizeof(data));
  serial.connected = false;
  assert(queue.drain(serial, 64) == 0);
  assert(serial.checks == 1 && serial.writes == 0);
  assert(queue.size() == sizeof(data));
  serial.connected = true;
  serial.space = 0;
  assert(queue.drain(serial, 64) == 0);
  serial.space = -1;
  assert(queue.drain(serial, 64) == 0);
  assert(serial.writes == 0);
  serial.space = 4096;
  assert(queue.drain(serial, 64) == 64);
  assert(queue.drain(serial, 64) == 64);
  assert(serial.received.size() == 128);
  assert(queue.write(data, 128) == 128);
  serial.space = 7;
  assert(queue.drain(serial, 64) == 7);
  serial.space = 4096;
  serial.maxWrite = 3;
  assert(queue.drain(serial, 64) == 3);
  serial.maxWrite = 0;
  const size_t pending = queue.size();
  assert(queue.drain(serial, 64) == 0);
  assert(queue.size() == pending);
  serial.maxWrite = 4096;
  while (!queue.empty()) {
    serial.space = 4096;
    assert(queue.drain(serial, 64) <= 64);
  }
  assert(serial.received.size() == sizeof(data) + 128);
  for (size_t index = 0; index < serial.received.size(); ++index) {
    assert(serial.received[index] == data[index % sizeof(data)]);
  }
  assert(queue.write(data, sizeof(data)) == sizeof(data));
  assert(queue.write(data, 10) == 0);
  assert(queue.droppedBytes() == 11);
  queue.clear();
  assert(queue.empty() && queue.droppedBytes() == 11);
  const size_t writes = serial.writes;
  assert(queue.drain(serial, 64) == 0);
  assert(serial.writes == writes);
  assert(queue.write(static_cast<uint8_t>('X')) == 1);
  assert(queue.drain(serial, 0) == 0);
  serial.space = 1;
  assert(queue.drain(serial, 64) == 1);
  assert(serial.received.back() == 'X');
  assert(queue.write(data, sizeof(data) - 2) == sizeof(data) - 2);
  assert(queue.write(data, 5) == 2);
  assert(queue.droppedBytes() == 14);
  assert(queue.write(data, UINT32_MAX) == 0);
  assert(queue.droppedBytes() == UINT32_MAX);
  assert(queue.write(static_cast<uint8_t>('!')) == 0);
  assert(queue.droppedBytes() == UINT32_MAX);
  puts("PASS: bounded FIFO, wraparound, overflow, connection servicing, TX space, short writes, and discard");
}