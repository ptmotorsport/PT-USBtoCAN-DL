#include "i2c_diagnostics.h"
#include "sensor_decode.h"
#include <Wire.h>

extern void logCAN();
extern void serviceGPS();

namespace {

void beginBus(Print &output) {
  static bool started = false;
  if (!started) {
    Wire.begin();
    Wire.setClock(100000);
    started = true;
  }
  output.println(F("Wire: SDA/A4, SCL/A5; 100 kHz; common GND required"));
}

void serviceCommunications() {
  logCAN();
  serviceGPS();
}

void waitForSensor(uint32_t duration) {
  const uint32_t started = millis();
  while (millis() - started < duration) {
    serviceCommunications();
    yield();
  }
}

void printAddress(Print &output, uint8_t address) {
  output.print(F("0x"));
  if (address < 16) output.print('0');
  output.print(address, HEX);
}

void transactionError(Print &output, uint8_t address, uint8_t reg, uint8_t code) {
  output.print(F("ERR: I2C address "));
  printAddress(output, address);
  output.print(F(", register "));
  printAddress(output, reg);
  output.print(F(", Wire error "));
  output.println(code);
}

bool readRegisters(Print &output, uint8_t address, uint8_t reg,
                   uint8_t *bytes, size_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  const uint8_t error = Wire.endTransmission(false);
  if (error != 0) {
    transactionError(output, address, reg, error);
    return false;
  }
  const size_t received = Wire.requestFrom(address, length, true);
  if (received != length) {
    output.print(F("ERR: I2C short/failed read at "));
    printAddress(output, address);
    output.print(F(", register "));
    printAddress(output, reg);
    output.print(F("; expected bytes "));
    output.print(length);
    output.print(F(", received "));
    output.println(received);
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    const int byte = Wire.read();
    if (byte < 0) {
      output.println(F("ERR: I2C receive buffer exhausted"));
      return false;
    }
    bytes[i] = static_cast<uint8_t>(byte);
  }
  serviceCommunications();
  return true;
}

bool writeRegister(Print &output, uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  const uint8_t error = Wire.endTransmission();
  if (error != 0) {
    transactionError(output, address, reg, error);
    return false;
  }
  serviceCommunications();
  return true;
}

bool findSensor(Print &output, uint8_t first, uint8_t second, uint8_t reg,
                uint8_t expected, uint8_t &address) {
  bool found = false;
  const uint8_t candidates[] = {first, second};
  for (uint8_t candidate : candidates) {
    Wire.beginTransmission(candidate);
    const uint8_t error = Wire.endTransmission();
    serviceCommunications();
    if (error == 2) continue; // Address NACK means no device at this address.
    if (error != 0) {
      transactionError(output, candidate, reg, error);
      return false;
    }
    uint8_t identity = 0;
    if (!readRegisters(output, candidate, reg, &identity, 1)) return false;
    output.print(F("Device at "));
    printAddress(output, candidate);
    output.print(F(": chip ID "));
    printAddress(output, identity);
    output.println();
    if (identity != expected) {
      output.print(F("ERR: Unsupported chip ID; expected "));
      printAddress(output, expected);
      output.println(F(" (BMP280=0x58, BME280=0x60, MPU-6050=0x68)"));
      continue;
    }
    if (!found) address = candidate;
    found = true;
  }
  if (!found) output.println(F("ERR: Sensor not found; check I2C scan, wiring, power and address straps"));
  return found;
}

bool waitBMPReady(Print &output, uint8_t address) {
  const uint32_t started = millis();
  do {
    uint8_t status = 0;
    if (!readRegisters(output, address, 0xF3, &status, 1)) return false;
    if ((status & 0x09) == 0) return true;
    waitForSensor(2);
  } while (millis() - started < 100);
  output.println(F("ERR: BMP280 busy timeout (measurement or calibration copy)"));
  return false;
}

void printVector(Print &output, const float (&values)[3]) {
  output.print(F("X="));
  output.print(values[0], 3);
  output.print(F(" Y="));
  output.print(values[1], 3);
  output.print(F(" Z="));
  output.println(values[2], 3);
}

}

void scanI2C(Print &output) {
  output.println(F("\n========== I2C SCAN =========="));
  beginBus(output);
  uint8_t count = 0;
  for (uint8_t address = 0x08; address <= 0x77; ++address) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission();
    serviceCommunications();
    if (error == 2) continue;
    if (error != 0) {
      output.print(F("ERR: Scan stopped at "));
      printAddress(output, address);
      output.print(F("; Wire error "));
      output.println(error);
      return;
    }
    ++count;
    output.print(F("ACK: "));
    printAddress(output, address);
    if (address == 0x68 || address == 0x69) output.print(F(" (possible MPU-6050)"));
    if (address == 0x76 || address == 0x77) output.print(F(" (possible BMP280/BME280)"));
    output.println();
  }
  output.print(F("Devices found: "));
  output.println(count);
  if (count == 0) output.println(F("ERR: No I2C devices; check power, SDA/SCL and pull-ups"));
  output.println(F("ACK confirms an address response, not sensor identity.\n"));
}

void showIMUDiagnostics(Print &output) {
  output.println(F("\n========== GY-521 / MPU-6050 DIAGNOSTICS =========="));
  beginBus(output);
  uint8_t address = 0;
  if (!findSensor(output, 0x68, 0x69, 0x75, 0x68, address)) return;
  // Wake the MPU-6050, enable all axes, and select +/-2 g / +/-250 deg/s.
  if (!writeRegister(output, address, 0x6B, 0x01) ||
      !writeRegister(output, address, 0x6C, 0x00) ||
      !writeRegister(output, address, 0x1A, 0x03) ||
      !writeRegister(output, address, 0x19, 0x09) ||
      !writeRegister(output, address, 0x1B, 0x00) ||
      !writeRegister(output, address, 0x1C, 0x00)) return;
  waitForSensor(50);
  uint8_t ranges[2] = {};
  uint8_t power[2] = {};
  if (!readRegisters(output, address, 0x1B, ranges, sizeof(ranges)) ||
      !readRegisters(output, address, 0x6B, power, sizeof(power))) return;
  if ((ranges[0] & 0xF8) != 0 || (ranges[1] & 0xF8) != 0 ||
      power[0] != 0x01 || power[1] != 0x00) {
    output.println(F("ERR: MPU-6050 configuration readback failed"));
    return;
  }
  uint8_t bytes[14] = {};
  if (!readRegisters(output, address, 0x3B, bytes, sizeof(bytes))) return;
  const SensorDecode::IMUSample sample = SensorDecode::decodeMPU6050(bytes);
  output.print(F("Sample from "));
  printAddress(output, address);
  output.println(F(" (+/-2 g, +/-250 deg/s; uncalibrated)"));
  output.print(F("Acceleration (g): "));
  printVector(output, sample.accelerationG);
  output.print(F("Gyroscope (deg/s): "));
  printVector(output, sample.gyroDegreesPerSecond);
  output.print(F("Chip temperature (C): "));
  output.println(sample.temperatureC, 2);
  output.println(F("At rest: acceleration magnitude about 1 g; gyro near zero."));
  output.println(F("Tilt/rotate and repeat IMU to check changing readings. No magnetometer.\n"));
}

void showBaroDiagnostics(Print &output) {
  output.println(F("\n========== BMP280 BAROMETER DIAGNOSTICS =========="));
  beginBus(output);
  uint8_t address = 0;
  if (!findSensor(output, 0x76, 0x77, 0xD0, 0x58, address)) return;
  if (!writeRegister(output, address, 0xF4, 0x00) ||
      !waitBMPReady(output, address)) return;
  if (!writeRegister(output, address, 0xF5, 0x00)) return;
  uint8_t trim[24] = {};
  SensorDecode::BMPCalibration calibration = {};
  if (!readRegisters(output, address, 0x88, trim, sizeof(trim))) return;
  if (!SensorDecode::decodeBMPCalibration(trim, calibration)) {
    output.println(F("ERR: Invalid BMP280 factory calibration"));
    return;
  }
  // One forced conversion: temperature x1, pressure x1, then return to sleep.
  if (!writeRegister(output, address, 0xF4, 0x25)) return;
  waitForSensor(10);
  if (!waitBMPReady(output, address)) return;
  uint8_t control = 0;
  uint8_t config = 0;
  if (!readRegisters(output, address, 0xF4, &control, 1)) return;
  if (!readRegisters(output, address, 0xF5, &config, 1)) return;
  if (control != 0x24 || config != 0x00) {
    output.println(F("ERR: BMP280 forced measurement configuration readback failed"));
    return;
  }
  uint8_t bytes[6] = {};
  SensorDecode::BMPSample sample = {};
  if (!readRegisters(output, address, 0xF7, bytes, sizeof(bytes))) return;
  if (!SensorDecode::decodeBMP280(bytes, calibration, sample)) {
    output.println(F("ERR: Invalid BMP280 sample/pressure compensation"));
    return;
  }
  output.print(F("Sample from "));
  printAddress(output, address);
  output.println(F(" (factory compensated, forced measurement)"));
  output.print(F("Temperature (C): "));
  output.println(sample.temperatureC, 2);
  output.print(F("Absolute pressure (hPa): "));
  output.println(sample.pressurePa / 100.0, 2);
  if (sample.temperatureC < -40 || sample.temperatureC > 85 ||
      sample.pressurePa < 30000 || sample.pressurePa > 110000) {
    output.println(F("WARN: Reading outside BMP280 rated temperature/pressure range"));
  }
  output.println(F("Pressure is local absolute pressure, not sea-level corrected."));
  output.println(F("Repeat BARO for a fresh sample; no humidity or altitude estimate.\n"));
}
