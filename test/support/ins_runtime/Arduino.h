#pragma once
#include <stddef.h>
#include <stdint.h>
#include <deque>
#include <iomanip>
#include <sstream>
#include <string>

class __FlashStringHelper {};
#define F(value) reinterpret_cast<const __FlashStringHelper *>(value)
constexpr unsigned SDA=18,SCL=19;
constexpr uint32_t IOPORT_CFG_DRIVE_MID=1,IOPORT_CFG_PERIPHERAL_PIN=2,IOPORT_PERIPHERAL_IIC=4;
constexpr int INPUT=0,OUTPUT=1,LOW=0,HEX=16,SERIAL_8N1=0;
struct Pin { unsigned pin; };
static const Pin g_pin_cfg[20]={};
static int g_ioport_ctrl=0;

class Print {
public:
  virtual ~Print() {}
  virtual size_t write(uint8_t)=0;
  virtual size_t write(const uint8_t *bytes,size_t n) {
    for (size_t i=0;i<n;++i) write(bytes[i]);
    return n;
  }
  void print(const __FlashStringHelper *s) { print(reinterpret_cast<const char *>(s)); }
  void print(const char *s) {
    while (*s) write(static_cast<uint8_t>(*s++));
  }
  void print(char c) { write(static_cast<uint8_t>(c)); }
  template <typename T> void print(T value) {
    std::ostringstream s; s << +value; print(s.str().c_str());
  }
  template <typename T> void print(T value,int precision) {
    std::ostringstream s; s << std::fixed << std::setprecision(precision) << value;
    print(s.str().c_str());
  }
  void print(uint16_t value,int base) {
    std::ostringstream s;
    if (base==HEX) s << std::hex;
    s << value; print(s.str().c_str());
  }
  void println() { print('\n'); }
  template <typename T> void println(T value) { print(value); println(); }
  template <typename T> void println(T value,int precision) { print(value,precision); println(); }
};
class MockSerial : public Print {
public:
  bool connected=true;
  uint32_t baud=0;
  std::deque<uint8_t> rx;
  std::string output;
  explicit operator bool() const { return connected; }
  void dtr() {}
  void begin(uint32_t value,int=0) { baud=value; }
  void end() {}
  int available() const { return static_cast<int>(rx.size()); }
  int availableForWrite() const { return 64; }
  int read() {
    if (rx.empty()) return -1;
    const uint8_t v=rx.front(); rx.pop_front(); return v;
  }
  using Print::write;
  size_t write(uint8_t c) override { output+=static_cast<char>(c); return 1; }
  void feed(const std::string &text) { for (unsigned char c:text) rx.push_back(c); }
};
extern MockSerial Serial,Serial1;
unsigned long millis();
unsigned long micros();
void pinMode(unsigned,int);
void digitalWrite(unsigned,int);
