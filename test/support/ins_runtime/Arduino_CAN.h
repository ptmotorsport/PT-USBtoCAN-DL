#pragma once
#include <Arduino.h>
#include <vector>
#include <string.h>

constexpr int FSP_ERR_CAN_TRANSMIT_NOT_READY=50;
inline uint32_t CanStandardId(uint32_t value) { return value; }
inline uint32_t CanExtendedId(uint32_t value) { return value|0x80000000UL; }
struct CanMsg {
  uint32_t id;
  uint8_t data_length,data[8];
  CanMsg(uint32_t identifier,uint8_t n,const uint8_t *bytes):id(identifier),data_length(n) {
    memcpy(data,bytes,n);
  }
  bool isExtendedId() const { return (id&0x80000000UL)!=0; }
  uint32_t getStandardId() const { return id; }
  uint32_t getExtendedId() const { return id&0x1FFFFFFF; }
};
class MockCAN {
public:
  std::vector<CanMsg> sent;
  std::vector<unsigned long> sentAt;
  bool opened=false,error=false;
  int errorCode=1;
  int writeResult=1;
  uint32_t bitrate=0;
  void end() { opened=false; }
  void clearError() { error=false; }
  bool begin(uint32_t speed) { bitrate=speed; opened=true; return true; }
  bool isError(int &value) { value=error?errorCode:0; return error; }
  unsigned available() { return 0; }
  CanMsg read() { const uint8_t bytes[8]={}; return CanMsg(0,0,bytes); }
  int write(const CanMsg &message) {
    if (writeResult>0) { sent.push_back(message); sentAt.push_back(millis()); }
    return writeResult;
  }
};
extern MockCAN CAN;
