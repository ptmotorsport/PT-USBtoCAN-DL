#pragma once

#include <stdint.h>
#include <string.h>

// Minimal u-blox 6 (NEO-6M) UBX configuration frames for the INS GPS rate option.
namespace Ubx {

constexpr uint32_t FastBaud=38400;
constexpr unsigned MaxFrame=32;

inline unsigned frame(uint8_t cls,uint8_t id,const uint8_t *payload,uint16_t length,
                      uint8_t (&out)[MaxFrame]) {
  if (8u+length>MaxFrame) return 0;
  out[0]=0xB5; out[1]=0x62; out[2]=cls; out[3]=id;
  out[4]=static_cast<uint8_t>(length); out[5]=static_cast<uint8_t>(length>>8);
  if (length) memcpy(out+6,payload,length);
  uint8_t a=0,b=0;
  for (unsigned i=2;i<6u+length;++i) { a=static_cast<uint8_t>(a+out[i]); b=static_cast<uint8_t>(b+a); }
  out[6+length]=a; out[7+length]=b;
  return 8u+length;
}

inline unsigned portBaud(uint32_t baud,uint8_t (&out)[MaxFrame]) {
  // UART1, 8N1, UBX+NMEA input, NMEA-only output so ACKs do not reach the NMEA parser.
  const uint8_t payload[20]={1,0,0,0,0xD0,0x08,0,0,
    static_cast<uint8_t>(baud),static_cast<uint8_t>(baud>>8),
    static_cast<uint8_t>(baud>>16),static_cast<uint8_t>(baud>>24),
    3,0,2,0,0,0,0,0};
  return frame(0x06,0x00,payload,sizeof(payload),out);
}

inline unsigned messageRate(uint8_t id,uint8_t rate,uint8_t (&out)[MaxFrame]) {
  const uint8_t payload[3]={0xF0,id,rate};
  return frame(0x06,0x01,payload,sizeof(payload),out);
}

// Fast sequence: index 0 is sent at the receiver's current baud; the caller
// then switches the device UART to FastBaud and sends indices 1..9.
// Revert sequence (fast=false) restores receiver defaults (9600 baud, 1Hz):
// index 0 is sent at FastBaud, index 1 at the configured baud.
inline unsigned sequence(bool fast,unsigned index,uint8_t (&out)[MaxFrame]) {
  if (!fast) {
    if (index>1) return 0;
    const uint8_t revert[13]={0xFF,0xFF,0,0,0,0,0,0,0xFF,0xFF,0,0,0x17};
    return frame(0x06,0x09,revert,sizeof(revert),out);
  }
  switch (index) {
    case 0: case 1: return portBaud(FastBaud,out);
    case 2: return messageRate(0x01,0,out); // GLL off.
    case 3: return messageRate(0x05,0,out); // VTG off.
    case 4: return messageRate(0x03,5,out); // GSV once per second at 5Hz.
    case 5: return messageRate(0x00,1,out); // GGA every epoch.
    case 6: return messageRate(0x04,1,out); // RMC every epoch.
    case 7: return messageRate(0x02,1,out); // GSA every epoch.
    case 8: {
      const uint8_t rate[6]={200,0,1,0,1,0}; // 200ms, 1 cycle, GPS time.
      return frame(0x06,0x08,rate,sizeof(rate),out);
    }
    case 9: {
      const uint8_t save[13]={0,0,0,0,0xFF,0xFF,0,0,0,0,0,0,0x17};
      return frame(0x06,0x09,save,sizeof(save),out);
    }
    default: return 0;
  }
}

// The device UART changes baud after this index has been fully transmitted.
inline bool switchesBaudAfter(unsigned index) { return index==0; }

}
