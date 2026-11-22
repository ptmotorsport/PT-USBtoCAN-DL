#pragma once
#include <stdint.h>

enum class LogFormat : uint8_t { Csv, Binary };
enum class FormatChangeResult : uint8_t { Unchanged, Changed, CannotClose, ResumeFailed };

constexpr uint8_t BinaryFormatMarker = 0xB1;
constexpr uint8_t CsvFormatMarker = 0xC0;
inline LogFormat logFormatFromStorage(uint8_t value) {
  return value == BinaryFormatMarker ? LogFormat::Binary : LogFormat::Csv;
}
inline uint8_t logFormatToStorage(LogFormat format) {
  return format == LogFormat::Binary ? BinaryFormatMarker : CsvFormatMarker;
}
