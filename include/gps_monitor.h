#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

class GPSMonitor {
public:
  static constexpr size_t SentenceCapacity = 96;

  bool partialSentence() const { return collecting_; }

  void feed(uint8_t byte, uint32_t now) {
    ++bytesReceived;
    lastByteMillis = now;
    if (byte == '$') {
      if (collecting_) ++malformedSentences;
      collecting_ = true;
      damaged_ = false;
      length_ = 0;
      lastSentence[length_++] = '$';
      lastSentence[length_] = '\0';
      return;
    }
    if (!collecting_) return;
    if (byte == '\n') {
      collecting_ = false;
      if (damaged_) {
        ++malformedSentences;
        return;
      }
      const char *checksum = strchr(lastSentence, '*');
      if (length_ < 10 || lastSentence[6] != ',' || !checksum ||
          checksum != lastSentence + length_ - 3 ||
          hexDigit(checksum[1]) < 0 || hexDigit(checksum[2]) < 0) {
        ++malformedSentences;
        return;
      }
      uint8_t computed = 0;
      for (const char *p = lastSentence + 1; p < checksum; ++p) {
        computed ^= static_cast<uint8_t>(*p);
      }
      const uint8_t expected = static_cast<uint8_t>(
          hexDigit(checksum[1]) * 16 + hexDigit(checksum[2]));
      if (computed != expected) {
        ++checksumErrors;
        return;
      }
      ++validSentences;
      lastValidMillis = now;
      return;
    }
    if (byte == '\r') return;
    if (byte < 32 || byte > 126) {
      damaged_ = true;
      return;
    }
    if (length_ == SentenceCapacity - 1) {
      if (!damaged_) ++overlongSentences;
      damaged_ = true;
      return;
    }
    lastSentence[length_++] = static_cast<char>(byte);
    lastSentence[length_] = '\0';
  }

  uint32_t bytesReceived = 0;
  uint32_t validSentences = 0;
  uint32_t checksumErrors = 0;
  uint32_t malformedSentences = 0;
  uint32_t overlongSentences = 0;
  uint32_t lastByteMillis = 0;
  uint32_t lastValidMillis = 0;
  char lastSentence[SentenceCapacity] = {};

private:
  static int hexDigit(char value) {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    return -1;
  }

  size_t length_ = 0;
  bool collecting_ = false;
  bool damaged_ = false;
};
