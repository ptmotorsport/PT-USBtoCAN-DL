#include "sensor_decode.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

namespace {

void testMPU6050() {
  const uint8_t bytes[14] = {
    0x40, 0x00, 0xC0, 0x00, 0x80, 0x00, 0xFE, 0xAC,
    0x00, 0x83, 0xFF, 0x7D, 0x7F, 0xFF
  };
  const SensorDecode::IMUSample sample = SensorDecode::decodeMPU6050(bytes);
  assert(sample.accelerationG[0] == 1.0f);
  assert(sample.accelerationG[1] == -1.0f);
  assert(sample.accelerationG[2] == -2.0f);
  assert(sample.gyroDegreesPerSecond[0] == 1.0f);
  assert(sample.gyroDegreesPerSecond[1] == -1.0f);
  assert(fabs(sample.gyroDegreesPerSecond[2] - 32767.0 / 131.0) < 0.001);
  assert(fabs(sample.temperatureC - 35.53) < 0.001);
  assert(SensorDecode::signed16(0xFFFF) == -1);
  assert(SensorDecode::signed16(0x7FFF) == 32767);
}

void testBMP280() {
  // Bosch datasheet example trim and uncompensated readings.
  const uint16_t coefficients[] = {
    27504, 26435, static_cast<uint16_t>(-1000), 36477,
    static_cast<uint16_t>(-10685), 3024, 2855, 140,
    static_cast<uint16_t>(-7), 15500, static_cast<uint16_t>(-14600), 6000
  };
  uint8_t trim[24] = {};
  for (size_t i = 0; i < 12; ++i) {
    trim[i * 2] = static_cast<uint8_t>(coefficients[i]);
    trim[i * 2 + 1] = static_cast<uint8_t>(coefficients[i] >> 8);
  }
  SensorDecode::BMPCalibration c = {};
  assert(SensorDecode::decodeBMPCalibration(trim, c));
  assert(c.t1 == 27504 && c.t2 == 26435 && c.t3 == -1000);
  assert(c.p1 == 36477 && c.p2 == -10685 && c.p6 == -7 && c.p8 == -14600);
  const uint8_t bytes[6] = {0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00};
  assert(SensorDecode::unsigned20(bytes) == 415148);
  assert(SensorDecode::unsigned20(bytes + 3) == 519888);
  SensorDecode::BMPSample sample = {};
  assert(SensorDecode::decodeBMP280(bytes, c, sample));
  assert(fabs(sample.temperatureC - 25.08248) < 0.0001);
  assert(fabs(sample.pressurePa - 100653.266) < 0.01);
  assert(fabs(sample.pressurePa / 100.0 - 1006.53266) < 0.0001);
  uint8_t disabled[6];
  memcpy(disabled, bytes, sizeof(bytes));
  disabled[0] = 0x80;
  disabled[1] = disabled[2] = 0;
  assert(!SensorDecode::decodeBMP280(disabled, c, sample));
  memcpy(disabled, bytes, sizeof(bytes));
  disabled[3] = 0x80;
  disabled[4] = disabled[5] = 0;
  assert(!SensorDecode::decodeBMP280(disabled, c, sample));
  c.p1 = 0;
  assert(!SensorDecode::decodeBMP280(bytes, c, sample));
  memset(trim, 0, sizeof(trim));
  assert(!SensorDecode::decodeBMPCalibration(trim, c));
  memset(trim, 0xFF, sizeof(trim));
  assert(!SensorDecode::decodeBMPCalibration(trim, c));
}

}

int main() {
  testMPU6050();
  testBMP280();
  puts("sensor_decode_test: PASS");
}
