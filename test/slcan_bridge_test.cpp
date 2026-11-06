#include <Arduino.h>
#include "slcan_bridge.h"
#include "slcan_protocol.h"
#include <stdio.h>
#include <vector>

#ifdef NDEBUG
#error SLCAN bridge tests require assertions
#endif

MockSerial Serial;
bool savvyCAN = false;

namespace {

struct Configuration {
  uint32_t bitrate;
  bool listenOnly;
};

std::vector<Configuration> configurations;
std::vector<Slcan::Frame> transmissions;
size_t closeCalls = 0;
bool configureResult = true;
bool sendResult = true;
unsigned long mockMillis = 0;

void processOnce() {
  const size_t previousReads = Serial.readCalls;
  const size_t previousBytes = Serial.output.size();
  slcanProcess();
  assert(Serial.readCalls - previousReads <= 64);
  assert(Serial.output.size() - previousBytes <= 64);
}

void enterBridge() {
  Serial = MockSerial{};
  configurations.clear();
  transmissions.clear();
  closeCalls = 0;
  configureResult = true;
  sendResult = true;
  mockMillis = 0;
  savvyCAN = true;
  slcanBegin();
  assert(closeCalls == 1);
  assert(!slcanCanRead());
  assert(slcanDroppedFrames() == 0);
}

void command(const std::string &text, const std::string &expected) {
  assert(Serial.input.empty());
  Serial.output.clear();
  Serial.feed(text);
  processOnce();
  assert(Serial.input.empty());
  assert(Serial.output == expected);
}

void openBridge() {
  enterBridge();
  command("S6\rO\r", "\r\r");
  assert(configurations.size() == 1);
  assert(configurations[0].bitrate == 500000);
  assert(!configurations[0].listenOnly);
  assert(slcanCanRead());
  Serial.output.clear();
}

void expectDrained(const std::string &expected) {
  for (size_t iteration = 0; iteration < 256; ++iteration) processOnce();
  assert(Serial.input.empty());
  assert(Serial.output == expected);
}

void testAutomaticHandoff() {
  const char *cliCommands[] = {"HELP", "CAN", "CONFIG", "SD", "SAVVYCAN", "LOG STOP", "EXIT", "t1230"};
  for (const char *text : cliCommands) {
    assert(!Slcan::isConnectionCommand(text, strlen(text)));
  }
  for (const char *text : {"C", "S6", "O", "V", "N", "F", "Z1"}) {
    assert(Slcan::isConnectionCommand(text, strlen(text)));
  }
  enterBridge();
  assert(slcanInitialCommand("C", 1));
  Serial.feed("\nS6\rO\r");
  processOnce();
  assert(Serial.output == "\r\r\r");
  assert(configurations.size() == 1 && configurations[0].bitrate == 500000);
  assert(slcanCanRead());
  enterBridge();
  assert(slcanInitialCommand("S8", 2));
  Serial.feed("O");
  processOnce();
  assert(configurations.empty());
  Serial.feed("\r");
  processOnce();
  assert(configurations.size() == 1 && configurations[0].bitrate == 1000000);
  assert(slcanCanRead());
}

void testEnterClosed() {
  enterBridge();
  slcanReceive(0x123, false, 0, nullptr);
  slcanReceive(0x1234567, true, 0, nullptr);
  processOnce();
  assert(Serial.output.empty());
  assert(Serial.writeCalls == 0);
  assert(configurations.empty());
  assert(slcanDroppedFrames() == 0);
  command("t1230\rT012345670\r", "\a\a");
  assert(transmissions.empty());
  assert(savvyCAN);
}

void testReceiveFrames() {
  openBridge();
  const uint8_t payload[] = {0x00, 0x01, 0x10, 0x7F, 0x80, 0xAB, 0xFE, 0xFF};
  slcanReceive(0x123, false, 3, payload);
  slcanReceive(0x1FFFFFFF, true, 8, payload);
  processOnce();
  assert(Serial.output == "t1233000110\rT1FFFFFFF80001107F80ABFEFF\r");
  assert(slcanDroppedFrames() == 0);
  command("Z1\r", "\r");
  mockMillis = 60001;
  Serial.output.clear();
  slcanReceive(0x123, false, 0, nullptr);
  processOnce();
  assert(Serial.output == "t12300001\r");
}

void testTransmitHooks() {
  openBridge();
  command("t1233Aa00ff\rT1FFFFFFF80001107F80ABFEFF\r", "z\rZ\r");
  assert(transmissions.size() == 2);
  const Slcan::Frame &standard = transmissions[0];
  assert(standard.id == 0x123 && !standard.extended && standard.length == 3);
  const uint8_t standardPayload[] = {0xAA, 0x00, 0xFF};
  assert(memcmp(standard.data, standardPayload, sizeof(standardPayload)) == 0);
  const Slcan::Frame &extended = transmissions[1];
  assert(extended.id == 0x1FFFFFFF && extended.extended && extended.length == 8);
  const uint8_t extendedPayload[] = {0x00, 0x01, 0x10, 0x7F, 0x80, 0xAB, 0xFE, 0xFF};
  assert(memcmp(extended.data, extendedPayload, sizeof(extendedPayload)) == 0);
  sendResult = false;
  command("t4560\r", "\a");
  assert(transmissions.size() == 3 && transmissions.back().id == 0x456);
  command("S5\rEXIT\r", "\a\a");
  assert(savvyCAN && slcanCanRead());
  assert(configurations.size() == 1);
}

void testListenOnly() {
  enterBridge();
  command("S6\rL\r", "\r\r");
  assert(configurations.size() == 1);
  assert(configurations[0].bitrate == 500000 && configurations[0].listenOnly);
  assert(slcanCanRead());
  command("t1230\rT012345670\r", "\a\a");
  assert(transmissions.empty());
  Serial.output.clear();
  slcanReceive(0x456, false, 0, nullptr);
  processOnce();
  assert(Serial.output == "t4560\r");
}

void testConfigureFailure() {
  enterBridge();
  configureResult = false;
  command("O\r", "\a");
  assert(configurations.size() == 1 && configurations[0].bitrate == 1000000);
  assert(closeCalls == 2 && !slcanCanRead());
  command("t1230\r", "\a");
  assert(transmissions.empty());
}

void testCloseAndExit() {
  openBridge();
  command("C\r", "\r");
  assert(closeCalls == 2 && !slcanCanRead() && savvyCAN);
  Serial.output.clear();
  slcanReceive(0x123, false, 0, nullptr);
  processOnce();
  assert(Serial.output.empty());
  Serial.writable = 0;
  Serial.feed("EXIT\rV\r");
  processOnce();
  assert(savvyCAN && !slcanCanRead());
  assert(Serial.available() == 2 && Serial.output.empty());
  Serial.writable = 64;
  processOnce();
  assert(!savvyCAN && Serial.output == "\r");
  const size_t previousReads = Serial.readCalls;
  processOnce();
  assert(Serial.readCalls == previousReads && Serial.available() == 2);
}

void testCrLfHandoff() {
  enterBridge();
  command("\nS6\r\nO\r\nV\r\n", "\r\rV0100\r");
  assert(configurations.size() == 1 && configurations[0].bitrate == 500000);
  assert(slcanCanRead());
  command("C\r\nEXIT\r", "\r\r");
  assert(!savvyCAN && !slcanCanRead());
}

void testDisconnectedDrops() {
  openBridge();
  Serial.writable = 0;
  slcanReceive(0x123, false, 0, nullptr);
  slcanReceive(0x456, false, 0, nullptr);
  processOnce();
  assert(Serial.output.empty());
  const size_t previousWrites = Serial.writeCalls;
  Serial.connected = false;
  Serial.feed("t1230\r");
  slcanReceive(0x789, false, 0, nullptr);
  assert(slcanDroppedFrames() == 3);
  processOnce();
  assert(slcanDroppedFrames() == 3);
  slcanReceive(0x1234567, true, 0, nullptr);
  assert(slcanDroppedFrames() == 4);
  assert(Serial.writeCalls == previousWrites && Serial.output.empty());
  assert(transmissions.empty() && Serial.available() == 6);
  Serial.input.clear();
  Serial.connected = true;
  Serial.writable = 64;
  command("F\r", "F08\r");
  openBridge();
  Serial.writable = 0;
  slcanReceive(0x123, false, 0, nullptr);
  slcanReceive(0x456, false, 0, nullptr);
  Serial.connected = false;
  const size_t writesBeforeDisconnect = Serial.writeCalls;
  processOnce();
  assert(slcanDroppedFrames() == 2);
  slcanReceive(0x789, false, 0, nullptr);
  assert(slcanDroppedFrames() == 3);
  assert(Serial.writeCalls == writesBeforeDisconnect && Serial.output.empty());
  enterBridge();
  command("F\r", "F00\r");
}

void testBoundedProcessing() {
  openBridge();
  Serial.writable = 0;
  Serial.feed(std::string(200, '\n'));
  processOnce();
  assert(Serial.readCalls == 5 + 64);
  assert(Serial.available() == 136);
  assert(Serial.writeCalls == 2 && Serial.output.empty());
  while (Serial.available() != 0) processOnce();
  const uint8_t payload[] = {0x00, 0x01, 0x10, 0x7F, 0x80, 0xAB, 0xFE, 0xFF};
  const std::string frame = "t12380001107F80ABFEFF\r";
  std::string expected;
  for (size_t index = 0; index < 10; ++index) {
    slcanReceive(0x123, false, 8, payload);
    expected += frame;
  }
  const size_t previousWrites = Serial.writeCalls;
  for (size_t iteration = 0; iteration < 10; ++iteration) processOnce();
  assert(Serial.writeCalls == previousWrites && Serial.output.empty());
  Serial.writable = 7;
  processOnce();
  assert(Serial.output.size() == 64);
  expectDrained(expected);
}

void testReplyBackpressureRecovery() {
  enterBridge();
  Serial.writable = 0;
  std::string commands;
  std::string expected;
  for (size_t index = 0; index < 600; ++index) {
    commands += "V\r";
    expected += "V0100\r";
  }
  Serial.feed(commands);
  for (size_t iteration = 0; iteration < 32; ++iteration) processOnce();
  assert(Serial.available() > 0 && Serial.available() < static_cast<int>(commands.size()));
  const size_t previousReads = Serial.readCalls;
  processOnce();
  assert(Serial.readCalls == previousReads);
  assert(Serial.writeCalls == 0 && Serial.output.empty());
  Serial.writable = 64;
  expectDrained(expected);
}

void testResponseReserve() {
  openBridge();
  Serial.writable = 0;
  const uint8_t payload[] = {0x00, 0x01, 0x10, 0x7F, 0x80, 0xAB, 0xFE, 0xFF};
  const std::string frame = "T1FFFFFFF80001107F80ABFEFF\r";
  const size_t accepted = (Slcan::QueueCapacity - Slcan::ResponseReserve) / (frame.size() + 2);
  std::string expected;
  for (size_t index = 0; index < accepted + 5; ++index) {
    slcanReceive(0x1FFFFFFF, true, 8, payload);
    if (index < accepted) expected += frame;
  }
  assert(slcanDroppedFrames() == 5);
  Serial.feed("V\rF\r");
  const size_t previousWrites = Serial.writeCalls;
  processOnce();
  assert(Serial.input.empty() && Serial.writeCalls == previousWrites);
  assert(Serial.output.empty());
  Serial.writable = 64;
  expectDrained(expected + "V0100\rF08\r");
  slcanReceive(0x123, false, 0, nullptr);
  Serial.output.clear();
  processOnce();
  assert(Serial.output == "t1230\r" && slcanDroppedFrames() == 5);
}

void testOverlongRecovery() {
  enterBridge();
  Serial.feed(std::string(200, 'X') + "\rV\r");
  processOnce();
  assert(Serial.readCalls == 64 && Serial.output.empty());
  expectDrained("\aV0100\r");
  assert(configurations.empty() && transmissions.empty());
  command("S6\rO\r", "\r\r");
  assert(slcanCanRead() && configurations[0].bitrate == 500000);
}

}

unsigned long millis() { return mockMillis; }

bool configureBridgeCAN(uint32_t bitrate, bool listenOnly) {
  configurations.push_back(Configuration{bitrate, listenOnly});
  return configureResult;
}

void closeBridgeCAN() { ++closeCalls; }

bool sendBridgeCAN(uint32_t id, bool extended, uint8_t length, const uint8_t *data) {
  assert(length <= 8 && (length == 0 || data != nullptr));
  Slcan::Frame frame = {};
  frame.id = id;
  frame.extended = extended;
  frame.length = length;
  if (length != 0) memcpy(frame.data, data, length);
  transmissions.push_back(frame);
  return sendResult;
}

int main() {
  testAutomaticHandoff();
  testEnterClosed();
  testReceiveFrames();
  testTransmitHooks();
  testListenOnly();
  testConfigureFailure();
  testCloseAndExit();
  testCrLfHandoff();
  testDisconnectedDrops();
  testBoundedProcessing();
  testReplyBackpressureRecovery();
  testResponseReserve();
  testOverlongRecovery();
  puts("SLCAN bridge integration tests passed (13 tests)");
}