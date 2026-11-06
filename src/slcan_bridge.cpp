#include "slcan_bridge.h"
#include "slcan_protocol.h"
#include <Arduino.h>
#include "usb_connection.h"

namespace {

enum class AdapterState { Closed, Open, Listen };
AdapterState state = AdapterState::Closed;
uint32_t bitrate = 1000000;
uint32_t dropped = 0;
bool timestamps = false;
bool overflow = false;
bool exiting = false;
Slcan::LineReader input;
Slcan::OutputQueue output;

void countDropped(uint32_t count) {
  if (count == 0) return;
  overflow = true;
  dropped = UINT32_MAX - dropped < count ? UINT32_MAX : dropped + count;
}

void disconnected() {
  countDropped(output.pendingFrames());
  output.reset();
  input.reset();
  exiting = false;
}

void reply(const char *text, size_t length) {
  output.enqueue(text, length, false);
}

void reject() { reply("\a", 1); }

void execute(const Slcan::Command &command) {
  switch (command.type) {
    case Slcan::CommandType::Bitrate:
      if (state != AdapterState::Closed) { reject(); return; }
      bitrate = command.bitrate;
      break;
    case Slcan::CommandType::Open:
    case Slcan::CommandType::Listen: {
      if (state != AdapterState::Closed) { reject(); return; }
      const bool listen = command.type == Slcan::CommandType::Listen;
      if (!configureBridgeCAN(bitrate, listen)) {
        closeBridgeCAN();
        reject();
        return;
      }
      state = listen ? AdapterState::Listen : AdapterState::Open;
      break;
    }
    case Slcan::CommandType::Close:
      closeBridgeCAN();
      state = AdapterState::Closed;
      break;
    case Slcan::CommandType::Transmit:
      if (state != AdapterState::Open ||
          !sendBridgeCAN(command.frame.id, command.frame.extended,
              command.frame.length, command.frame.data)) {
        reject();
      } else {
        reply(command.frame.extended ? "Z\r" : "z\r", 2);
      }
      return;
    case Slcan::CommandType::Version: reply("V0100\r", 6); return;
    case Slcan::CommandType::SerialNumber: reply("N0001\r", 6); return;
    case Slcan::CommandType::Status: {
      char response[5];
      reply(response, Slcan::formatStatus(response, sizeof(response),
          overflow ? Slcan::OverflowStatus : 0));
      return;
    }
    case Slcan::CommandType::Timestamp:
      timestamps = command.timestamp;
      break;
    case Slcan::CommandType::Exit:
      if (state != AdapterState::Closed) { reject(); return; }
      exiting = true;
      break;
  }
  reply("\r", 1);
}

void serviceOutput() {
  size_t budget = 64;
  while (budget != 0 && !output.empty() && usbSerialConnected()) {
    const int available = Serial.availableForWrite();
    if (available <= 0) break;
    size_t length = 0;
    const uint8_t *data = output.front(length);
    if (length > budget) length = budget;
    if (length > static_cast<size_t>(available)) length = static_cast<size_t>(available);
    const size_t written = Serial.write(data, length);
    output.consume(written);
    budget -= written;
    if (written != length) break;
  }
}

}

void slcanBegin() {
  closeBridgeCAN();
  state = AdapterState::Closed;
  bitrate = 1000000;
  dropped = 0;
  timestamps = false;
  overflow = false;
  exiting = false;
  input.reset();
  output.reset();
}

void slcanProcess() {
  if (!savvyCAN) return;
  if (!usbSerialConnected()) { disconnected(); return; }
  for (size_t processed = 0; processed < 64 && !exiting &&
      output.freeSpace() >= 8 && Serial.available() > 0; ++processed) {
    const int character = Serial.read();
    if (character < 0) break;
    if (character == '\n') continue;
    const Slcan::LineResult result = input.push(static_cast<char>(character));
    if (result == Slcan::LineResult::Pending) continue;
    Slcan::Command command = {};
    if (result == Slcan::LineResult::Overlong ||
        !Slcan::parseCommand(input.data(), input.length(), command)) reject();
    else execute(command);
  }
  serviceOutput();
  if (!usbSerialConnected()) { disconnected(); return; }
  if (exiting && output.empty()) {
    exiting = false;
    savvyCAN = false;
  }
}

bool slcanInitialCommand(const char *text, unsigned int length) {
  Slcan::Command command = {};
  if (!savvyCAN || !Slcan::parseCommand(text, length, command)) return false;
  execute(command);
  return true;
}

void slcanReceive(uint32_t id, bool extended, uint8_t length, const uint8_t *data) {
  if (!slcanCanRead()) return;
  if (!usbSerialConnected()) {
    disconnected();
    countDropped(1);
    return;
  }
  char frame[Slcan::MaxFrameLength + 1];
  const size_t used = Slcan::formatFrame(frame, sizeof(frame), id, extended,
      length, data, timestamps, static_cast<uint32_t>(millis()));
  if (used == 0 || !output.enqueue(frame, used, true)) countDropped(1);
}

bool slcanCanRead() {
  return savvyCAN && state != AdapterState::Closed && !exiting;
}

uint32_t slcanDroppedFrames() { return dropped; }