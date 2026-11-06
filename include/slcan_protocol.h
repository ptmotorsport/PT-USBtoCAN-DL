#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace Slcan {

constexpr size_t MaxCommandLength = 26;
constexpr size_t MaxFrameLength = 31;
constexpr size_t QueueCapacity = 1024;
constexpr size_t ResponseReserve = 128;
constexpr uint8_t OverflowStatus = 0x08;

struct Frame {
  uint32_t id;
  bool extended;
  uint8_t length;
  uint8_t data[8];
};

enum class CommandType {
  Bitrate, Open, Listen, Close, Transmit, Version, SerialNumber,
  Status, Timestamp, Exit
};

struct Command {
  CommandType type;
  uint32_t bitrate;
  bool timestamp;
  Frame frame;
};

inline int hexValue(char character) {
  if (character >= '0' && character <= '9') return character - '0';
  if (character >= 'A' && character <= 'F') return character - 'A' + 10;
  if (character >= 'a' && character <= 'f') return character - 'a' + 10;
  return -1;
}

inline char hexDigit(uint32_t value) {
  return "0123456789ABCDEF"[value & 15U];
}

inline bool parseHex(const char *input, size_t length, uint32_t &output) {
  if (input == nullptr || length == 0 || length > 8) return false;
  uint32_t value = 0;
  for (size_t index = 0; index < length; ++index) {
    const int digit = hexValue(input[index]);
    if (digit < 0) return false;
    value = (value << 4) | static_cast<uint32_t>(digit);
  }
  output = value;
  return true;
}

inline bool parseFrame(const char *input, size_t length, Frame &output) {
  if (input == nullptr || length == 0 || (input[0] != 't' && input[0] != 'T')) {
    return false;
  }
  Frame parsed = {};
  parsed.extended = input[0] == 'T';
  const size_t width = parsed.extended ? 8 : 3;
  if (length < width + 2 || !parseHex(input + 1, width, parsed.id) ||
      parsed.id > (parsed.extended ? 0x1FFFFFFFUL : 0x7FFUL)) return false;
  const int dlc = hexValue(input[width + 1]);
  if (dlc < 0 || dlc > 8 || length != width + 2 + 2 * static_cast<size_t>(dlc)) {
    return false;
  }
  parsed.length = static_cast<uint8_t>(dlc);
  for (size_t index = 0; index < parsed.length; ++index) {
    uint32_t value = 0;
    if (!parseHex(input + width + 2 + 2 * index, 2, value)) return false;
    parsed.data[index] = static_cast<uint8_t>(value);
  }
  output = parsed;
  return true;
}

inline bool parseCommand(const char *input, size_t length, Command &output) {
  if (input == nullptr || length == 0 || length > MaxCommandLength) return false;
  Command parsed = {};
  if (length == 4 && memcmp(input, "EXIT", 4) == 0) {
    parsed.type = CommandType::Exit;
  } else if (input[0] == 't' || input[0] == 'T') {
    if (!parseFrame(input, length, parsed.frame)) return false;
    parsed.type = CommandType::Transmit;
  } else if (length == 2 && input[0] == 'S' && input[1] >= '0' && input[1] <= '8') {
    static const uint32_t rates[] = {
      10000, 20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000
    };
    parsed.type = CommandType::Bitrate;
    parsed.bitrate = rates[input[1] - '0'];
  } else if (length == 2 && input[0] == 'Z' && (input[1] == '0' || input[1] == '1')) {
    parsed.type = CommandType::Timestamp;
    parsed.timestamp = input[1] == '1';
  } else if (length == 1) {
    switch (input[0]) {
      case 'O': parsed.type = CommandType::Open; break;
      case 'L': parsed.type = CommandType::Listen; break;
      case 'C': parsed.type = CommandType::Close; break;
      case 'V': parsed.type = CommandType::Version; break;
      case 'N': parsed.type = CommandType::SerialNumber; break;
      case 'F': parsed.type = CommandType::Status; break;
      default: return false;
    }
  } else {
    return false;
  }
  output = parsed;
  return true;
}

inline bool isConnectionCommand(const char *input, size_t length) {
  Command command = {};
  if (!parseCommand(input, length, command)) return false;
  return command.type != CommandType::Transmit && command.type != CommandType::Exit;
}

inline size_t formatFrame(char *output, size_t capacity, uint32_t id, bool extended,
    uint8_t length, const uint8_t *data, bool timestamp = false, uint32_t millis = 0) {
  const size_t width = extended ? 8 : 3;
  const size_t required = width + 3 + 2 * static_cast<size_t>(length) + (timestamp ? 4 : 0);
  if (output == nullptr || length > 8 || id > (extended ? 0x1FFFFFFFUL : 0x7FFUL) ||
      (length != 0 && data == nullptr) || capacity <= required) return 0;
  size_t used = 0;
  output[used++] = extended ? 'T' : 't';
  for (size_t digit = width; digit > 0; --digit) {
    output[used++] = hexDigit(id >> (4 * (digit - 1)));
  }
  output[used++] = hexDigit(length);
  for (size_t index = 0; index < length; ++index) {
    output[used++] = hexDigit(data[index] >> 4);
    output[used++] = hexDigit(data[index]);
  }
  if (timestamp) {
    const uint32_t value = millis % 60000U;
    for (size_t digit = 4; digit > 0; --digit) {
      output[used++] = hexDigit(value >> (4 * (digit - 1)));
    }
  }
  output[used++] = '\r';
  output[used] = '\0';
  return used;
}

inline size_t formatStatus(char *output, size_t capacity, uint8_t status) {
  if (output == nullptr || capacity < 5) return 0;
  output[0] = 'F';
  output[1] = hexDigit(status >> 4);
  output[2] = hexDigit(status);
  output[3] = '\r';
  output[4] = '\0';
  return 4;
}

enum class LineResult { Pending, Complete, Overlong };

class LineReader {
public:
  void reset() { used_ = 0; discarding_ = false; }

  LineResult push(char character) {
    if (character == '\r') {
      const bool discarded = discarding_;
      discarding_ = false;
      line_[used_] = '\0';
      length_ = used_;
      used_ = 0;
      return discarded ? LineResult::Overlong : LineResult::Complete;
    }
    if (!discarding_) {
      if (used_ == MaxCommandLength) discarding_ = true;
      else line_[used_++] = character;
    }
    return LineResult::Pending;
  }

  const char *data() const { return line_; }
  size_t length() const { return length_; }

private:
  char line_[MaxCommandLength + 1] = {};
  size_t used_ = 0;
  size_t length_ = 0;
  bool discarding_ = false;
};

class OutputQueue {
public:
  size_t freeSpace() const { return QueueCapacity - used_; }
  bool empty() const { return used_ == 0 && remaining_ == 0; }
  uint32_t pendingFrames() const { return frames_; }

  void reset() {
    head_ = 0;
    tail_ = 0;
    used_ = 0;
    remaining_ = 0;
    frames_ = 0;
    frame_ = false;
  }

  bool enqueue(const char *data, size_t length, bool frame) {
    const size_t required = length + 2;
    if (data == nullptr || length == 0 || length > MaxFrameLength ||
        freeSpace() < required || (frame && freeSpace() - required < ResponseReserve)) {
      return false;
    }
    append(static_cast<uint8_t>(length));
    append(frame ? 1 : 0);
    for (size_t index = 0; index < length; ++index) append(static_cast<uint8_t>(data[index]));
    if (frame) ++frames_;
    return true;
  }

  const uint8_t *front(size_t &length) {
    if (remaining_ == 0 && used_ != 0) {
      remaining_ = take();
      frame_ = take() != 0;
    }
    length = remaining_;
    if (length > QueueCapacity - head_) length = QueueCapacity - head_;
    return bytes_ + head_;
  }

  void consume(size_t length) {
    if (length > remaining_) return;
    head_ = (head_ + length) % QueueCapacity;
    used_ -= length;
    remaining_ -= length;
    if (length != 0 && remaining_ == 0 && frame_) {
      --frames_;
      frame_ = false;
    }
  }

private:
  void append(uint8_t value) {
    bytes_[tail_] = value;
    tail_ = (tail_ + 1) % QueueCapacity;
    ++used_;
  }

  uint8_t take() {
    const uint8_t value = bytes_[head_];
    head_ = (head_ + 1) % QueueCapacity;
    --used_;
    return value;
  }

  uint8_t bytes_[QueueCapacity] = {};
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t used_ = 0;
  size_t remaining_ = 0;
  uint32_t frames_ = 0;
  bool frame_ = false;
};

}