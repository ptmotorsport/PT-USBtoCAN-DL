#include "slcan_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string>

namespace {

bool parse(const char *text, Slcan::Command &command) {
  return Slcan::parseCommand(text, strlen(text), command);
}

void testCommands() {
  Slcan::Command command = {};
  const uint32_t rates[] = {10000, 20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000};
  for (size_t index = 0; index < 9; ++index) {
    char text[] = "S0";
    text[1] += static_cast<char>(index);
    assert(parse(text, command));
    assert(command.type == Slcan::CommandType::Bitrate && command.bitrate == rates[index]);
  }
  const char *valid[] = {"O", "L", "C", "V", "N", "F", "Z0", "Z1", "EXIT"};
  const Slcan::CommandType types[] = {
    Slcan::CommandType::Open, Slcan::CommandType::Listen, Slcan::CommandType::Close,
    Slcan::CommandType::Version, Slcan::CommandType::SerialNumber, Slcan::CommandType::Status,
    Slcan::CommandType::Timestamp, Slcan::CommandType::Timestamp, Slcan::CommandType::Exit
  };
  for (size_t index = 0; index < sizeof(valid) / sizeof(valid[0]); ++index) {
    assert(parse(valid[index], command));
    assert(command.type == types[index]);
    if (index == 6 || index == 7) assert(command.timestamp == (index == 7));
  }
  const char *invalid[] = {"", "S", "S9", "S00", "s0", "S-1", "O ", "O\r", "\n",
    "Z", "Z2", "Z01", "exit", "EXIT ", "M00000000", "mFFFFFFFF", "r1230",
    "R000001230", "?", "V0", "t1230\r", " t1230"};
  for (const char *text : invalid) assert(!parse(text, command));
  assert(!Slcan::parseCommand(nullptr, 1, command));
  const char embeddedNull[] = {'V', '\0', 'N'};
  assert(!Slcan::parseCommand(embeddedNull, sizeof(embeddedNull), command));
}

void testFrames() {
  Slcan::Frame frame = {};
  assert(Slcan::parseFrame("t0000", 5, frame));
  assert(frame.id == 0 && !frame.extended && frame.length == 0);
  assert(Slcan::parseFrame("T1fffffff8aB010203040506ff", 26, frame));
  assert(frame.id == 0x1FFFFFFF && frame.extended && frame.length == 8);
  assert(frame.data[0] == 0xAB && frame.data[7] == 0xFF);
  const char *invalid[] = {"t", "t120", "t00000", "t8000", "tFFF0", "T200000000",
    "TFFFFFFFF0", "T00000000", "T0000000000", "t1239", "t123A", "t1231",
    "t12310000", "t1231G0", "t12g0", "t1231 0", "r1230", "R000001230"};
  for (const char *text : invalid) assert(!Slcan::parseFrame(text, strlen(text), frame));
  assert(!Slcan::parseFrame(nullptr, 5, frame));
  const uint32_t ids[] = {0, 1, 0x7FF, 0x800, 0x1234567, 0x1FFFFFFF};
  for (uint32_t id : ids) {
    for (bool extended : {false, true}) {
      if (!extended && id > 0x7FF) continue;
      for (uint8_t length = 0; length <= 8; ++length) {
        uint8_t data[8] = {0x00, 0x01, 0x10, 0x7F, 0x80, 0xAB, 0xFE, 0xFF};
        char formatted[Slcan::MaxFrameLength + 1];
        const size_t used = Slcan::formatFrame(formatted, sizeof(formatted), id, extended, length, data);
        assert(used != 0 && formatted[used - 1] == '\r' && formatted[used] == '\0');
        assert(Slcan::parseFrame(formatted, used - 1, frame));
        assert(frame.id == id && frame.extended == extended && frame.length == length);
        assert(memcmp(frame.data, data, length) == 0);
        Slcan::Command command = {};
        assert(Slcan::parseCommand(formatted, used - 1, command));
        assert(command.type == Slcan::CommandType::Transmit);
        for (size_t capacity = 0; capacity <= used; ++capacity) {
          char small[Slcan::MaxFrameLength + 1];
          memset(small, '#', sizeof(small));
          assert(Slcan::formatFrame(small, capacity, id, extended, length, data) == 0);
          for (char character : small) assert(character == '#');
        }
        assert(Slcan::formatFrame(formatted, used + 1, id, extended, length, data) == used);
      }
    }
  }
}

void testFormatting() {
  char output[Slcan::MaxFrameLength + 1];
  assert(Slcan::formatFrame(output, sizeof(output), 0x123, false, 0, nullptr, true, 59999));
  assert(strcmp(output, "t1230EA5F\r") == 0);
  assert(Slcan::formatFrame(output, sizeof(output), 0x123, false, 0, nullptr, true, 60000));
  assert(strcmp(output, "t12300000\r") == 0);
  assert(Slcan::formatFrame(output, sizeof(output), 0, true, 0, nullptr, true, 120001));
  assert(strcmp(output, "T0000000000001\r") == 0);
  assert(!Slcan::formatFrame(output, sizeof(output), 0x800, false, 0, nullptr));
  assert(!Slcan::formatFrame(output, sizeof(output), 0x20000000, true, 0, nullptr));
  assert(!Slcan::formatFrame(output, sizeof(output), 0, false, 9, nullptr));
  assert(!Slcan::formatFrame(output, sizeof(output), 0, false, 1, nullptr));
  assert(!Slcan::formatFrame(nullptr, sizeof(output), 0, false, 0, nullptr));
  assert(Slcan::formatStatus(output, sizeof(output), 0) == 4 && strcmp(output, "F00\r") == 0);
  assert(Slcan::formatStatus(output, sizeof(output), Slcan::OverflowStatus) == 4);
  assert(strcmp(output, "F08\r") == 0);
  assert(Slcan::formatStatus(output, sizeof(output), 0xFF) == 4 && strcmp(output, "FFF\r") == 0);
  assert(!Slcan::formatStatus(output, 4, 0));
  assert(!Slcan::formatStatus(nullptr, 5, 0));
}

void testLines() {
  Slcan::LineReader reader;
  for (size_t index = 0; index < Slcan::MaxCommandLength; ++index) {
    assert(reader.push('T') == Slcan::LineResult::Pending);
  }
  assert(reader.push('\r') == Slcan::LineResult::Complete);
  assert(reader.length() == Slcan::MaxCommandLength);
  for (size_t index = 0; index < 10000; ++index) {
    assert(reader.push('X') == Slcan::LineResult::Pending);
  }
  assert(reader.push('\r') == Slcan::LineResult::Overlong);
  assert(reader.push('V') == Slcan::LineResult::Pending);
  assert(reader.push('\r') == Slcan::LineResult::Complete);
  assert(reader.length() == 1 && reader.data()[0] == 'V');
  reader.push('X');
  reader.reset();
  assert(reader.push('\r') == Slcan::LineResult::Complete && reader.length() == 0);
}

std::string drain(Slcan::OutputQueue &queue, size_t chunk) {
  std::string result;
  while (!queue.empty()) {
    size_t length = 0;
    const uint8_t *data = queue.front(length);
    assert(length != 0);
    if (length > chunk) length = chunk;
    result.append(reinterpret_cast<const char *>(data), length);
    queue.consume(length);
  }
  return result;
}

void testQueue() {
  Slcan::OutputQueue queue;
  const char frame[] = "T1FFFFFFF80001020304050607EA5F\r";
  std::string expected;
  for (size_t round = 0; round < 100; ++round) {
    size_t frames = 0;
    while (queue.enqueue(frame, strlen(frame), true)) {
      expected += frame;
      ++frames;
    }
    assert(frames != 0 && queue.pendingFrames() == frames);
    assert(queue.freeSpace() >= Slcan::ResponseReserve);
    assert(queue.enqueue("V0100\r", 6, false));
    expected += "V0100\r";
    assert(drain(queue, round % 7 + 1) == expected);
    assert(queue.pendingFrames() == 0 && queue.freeSpace() == Slcan::QueueCapacity);
    expected.clear();
  }
  assert(!queue.enqueue(nullptr, 1, false));
  assert(!queue.enqueue("", 0, false));
  assert(!queue.enqueue(frame, 256, false));
  size_t replies = 0;
  while (queue.enqueue("\r", 1, false)) ++replies;
  assert(replies == Slcan::QueueCapacity / 3);
  assert(drain(queue, 64) == std::string(replies, '\r'));
  assert(queue.enqueue(frame, strlen(frame), true));
  size_t length = 0;
  queue.front(length);
  queue.consume(1);
  assert(queue.pendingFrames() == 1);
  queue.reset();
  assert(queue.empty() && queue.pendingFrames() == 0);
}

}

int main() {
  testCommands();
  testFrames();
  testFormatting();
  testLines();
  testQueue();
  puts("SLCAN protocol tests passed");
}