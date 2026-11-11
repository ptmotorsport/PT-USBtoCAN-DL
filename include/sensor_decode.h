#pragma once

#include <math.h>
#include <stdint.h>

namespace SensorDecode {

inline int16_t signed16(uint16_t value) {
  return static_cast<int16_t>(value < 0x8000 ? static_cast<int32_t>(value) :
      static_cast<int32_t>(value) - 65536);
}

inline uint16_t little16(const uint8_t *bytes) {
  return static_cast<uint16_t>(bytes[0] | (static_cast<uint16_t>(bytes[1]) << 8));
}

inline int16_t bigSigned16(const uint8_t *bytes) {
  return signed16(static_cast<uint16_t>(
      (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]));
}

struct IMUSample {
  float accelerationG[3];
  float gyroDegreesPerSecond[3];
  float temperatureC;
};

inline IMUSample decodeMPU6050(const uint8_t (&bytes)[14]) {
  IMUSample sample = {};
  for (int axis = 0; axis < 3; ++axis) {
    sample.accelerationG[axis] = bigSigned16(bytes + axis * 2) / 16384.0f;
    sample.gyroDegreesPerSecond[axis] = bigSigned16(bytes + 8 + axis * 2) / 131.0f;
  }
  sample.temperatureC = bigSigned16(bytes + 6) / 340.0f + 36.53f;
  return sample;
}

struct BMPCalibration {
  uint16_t t1;
  int16_t t2, t3;
  uint16_t p1;
  int16_t p2, p3, p4, p5, p6, p7, p8, p9;
};

inline bool decodeBMPCalibration(const uint8_t (&bytes)[24], BMPCalibration &c) {
  c.t1 = little16(bytes);
  c.t2 = signed16(little16(bytes + 2));
  c.t3 = signed16(little16(bytes + 4));
  c.p1 = little16(bytes + 6);
  c.p2 = signed16(little16(bytes + 8));
  c.p3 = signed16(little16(bytes + 10));
  c.p4 = signed16(little16(bytes + 12));
  c.p5 = signed16(little16(bytes + 14));
  c.p6 = signed16(little16(bytes + 16));
  c.p7 = signed16(little16(bytes + 18));
  c.p8 = signed16(little16(bytes + 20));
  c.p9 = signed16(little16(bytes + 22));
  return c.t1 != 0 && c.t1 != 0xFFFF && c.p1 != 0 && c.p1 != 0xFFFF;
}

struct BMPSample {
  double temperatureC;
  double pressurePa;
};

inline uint32_t unsigned20(const uint8_t *bytes) {
  return (static_cast<uint32_t>(bytes[0]) << 12) |
      (static_cast<uint32_t>(bytes[1]) << 4) | (bytes[2] >> 4);
}

inline bool decodeBMP280(const uint8_t (&bytes)[6], const BMPCalibration &c,
                         BMPSample &sample) {
  const uint32_t rawPressure = unsigned20(bytes);
  const uint32_t rawTemperature = unsigned20(bytes + 3);
  if (c.t1 == 0 || c.p1 == 0 || rawPressure == 0x80000 || rawTemperature == 0x80000) {
    return false;
  }
  // BMP280 datasheet floating-point compensation, using factory trim values.
  double v1 = (rawTemperature / 16384.0 - c.t1 / 1024.0) * c.t2;
  double v2 = rawTemperature / 131072.0 - c.t1 / 8192.0;
  v2 = v2 * v2 * c.t3;
  const double fine = v1 + v2;
  const double temperature = fine / 5120.0;
  v1 = fine / 2.0 - 64000.0;
  v2 = v1 * v1 * c.p6 / 32768.0;
  v2 += v1 * c.p5 * 2.0;
  v2 = v2 / 4.0 + c.p4 * 65536.0;
  v1 = (c.p3 * v1 * v1 / 524288.0 + c.p2 * v1) / 524288.0;
  v1 = (1.0 + v1 / 32768.0) * c.p1;
  if (v1 == 0.0) return false;
  double pressure = (1048576.0 - rawPressure - v2 / 4096.0) * 6250.0 / v1;
  v1 = c.p9 * pressure * pressure / 2147483648.0;
  v2 = pressure * c.p8 / 32768.0;
  pressure += (v1 + v2 + c.p7) / 16.0;
  if (!isfinite(temperature) || !isfinite(pressure) || pressure <= 0) return false;
  sample.temperatureC = temperature;
  sample.pressurePa = pressure;
  return true;
}

}
