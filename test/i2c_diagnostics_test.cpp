#include "i2c_diagnostics.h"
#include <Wire.h>
#include <assert.h>
#include <stdio.h>

MockWire Wire;
unsigned long now = 0;
size_t canServices = 0;
size_t gpsServices = 0;
unsigned long millis() { return now++; }
void yield() {}
void logCAN() { ++canServices; }
void serviceGPS() { ++gpsServices; }

namespace {

bool contains(const Print &output, const char *text) {
  return output.output.find(text) != std::string::npos;
}

void addIMU(uint8_t address = 0x69) {
  auto &registers = Wire.devices[address];
  registers[0x75] = 0x68;
  registers[0x3F] = 0x40; // Z acceleration = 1 g.
  registers[0x43] = 0;
  registers[0x44] = 131; // X gyro = 1 deg/s.
}

void addBMP(uint8_t address = 0x77) {
  auto &registers = Wire.devices[address];
  registers[0xD0] = 0x58;
  const uint16_t trim[] = {
    27504, 26435, static_cast<uint16_t>(-1000), 36477,
    static_cast<uint16_t>(-10685), 3024, 2855, 140,
    static_cast<uint16_t>(-7), 15500, static_cast<uint16_t>(-14600), 6000
  };
  for (size_t i = 0; i < 12; ++i) {
    registers[0x88 + i * 2] = static_cast<uint8_t>(trim[i]);
    registers[0x89 + i * 2] = static_cast<uint8_t>(trim[i] >> 8);
  }
  const uint8_t sample[] = {0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00};
  for (size_t i = 0; i < 6; ++i) registers[0xF7 + i] = sample[i];
}

void testScanAndSamples() {
  Print output;
  scanI2C(output);
  assert(contains(output, "ERR: No I2C devices"));
  assert(Wire.begins == 1 && Wire.clock == 100000);
  addIMU();
  addBMP();
  output.output.clear();
  scanI2C(output);
  assert(contains(output, "ACK: 0x69") && contains(output, "ACK: 0x77"));
  assert(contains(output, "Devices found: 2"));
  output.output.clear();
  showIMUDiagnostics(output);
  assert(!contains(output, "ERR:"));
  assert(contains(output, "Z=1.000"));
  assert(contains(output, "Gyroscope (deg/s): X=1.000"));
  assert(Wire.devices[0x69][0x6B] == 1);
  assert(Wire.devices[0x69][0x1B] == 0 && Wire.devices[0x69][0x1C] == 0);
  output.output.clear();
  showBaroDiagnostics(output);
  assert(!contains(output, "ERR:"));
  assert(contains(output, "Temperature (C): 25.08"));
  assert(contains(output, "Absolute pressure (hPa): 1006.53"));
  assert(Wire.devices[0x77][0xF4] == 0x24);
  assert(canServices > 0 && canServices == gpsServices);
}

void testFailures() {
  Print output;
  Wire = MockWire{};
  showIMUDiagnostics(output);
  assert(contains(output, "ERR: Sensor not found"));
  addIMU(0x68);
  Wire.devices[0x68][0x75] = 0x70;
  output.output.clear();
  showIMUDiagnostics(output);
  assert(contains(output, "Unsupported chip ID"));
  assert(!contains(output, "Acceleration (g):"));
  Wire.devices[0x68][0x75] = 0x68;
  Wire.shortRead = true;
  output.output.clear();
  showIMUDiagnostics(output);
  assert(contains(output, "short/failed read"));
  Wire.shortRead = false;
  Wire.ignoreWrites = true;
  output.output.clear();
  showIMUDiagnostics(output);
  assert(contains(output, "configuration readback failed"));
  Wire.ignoreWrites = false;
  Wire.failAddress = 0x08;
  output.output.clear();
  scanI2C(output);
  assert(contains(output, "Scan stopped at 0x08; Wire error 5"));
  Wire = MockWire{};
  addBMP(0x76);
  Wire.devices[0x76][0xD0] = 0x60;
  output.output.clear();
  showBaroDiagnostics(output);
  assert(contains(output, "Unsupported chip ID"));
  assert(!contains(output, "Absolute pressure"));
  Wire.devices[0x76][0xD0] = 0x58;
  Wire.devices[0x76][0xF3] = 0x08;
  output.output.clear();
  showBaroDiagnostics(output);
  assert(contains(output, "BMP280 busy timeout"));
  Wire.devices[0x76][0xF3] = 0;
  Wire.devices[0x76][0x8E] = Wire.devices[0x76][0x8F] = 0;
  output.output.clear();
  showBaroDiagnostics(output);
  assert(contains(output, "Invalid BMP280 factory calibration"));
  addBMP(0x76);
  Wire.devices[0x76][0xF7] = 0x80;
  Wire.devices[0x76][0xF8] = Wire.devices[0x76][0xF9] = 0;
  output.output.clear();
  showBaroDiagnostics(output);
  assert(contains(output, "Invalid BMP280 sample"));
}

}

int main() {
  testScanAndSamples();
  testFailures();
  puts("i2c_diagnostics_test: PASS");
}
