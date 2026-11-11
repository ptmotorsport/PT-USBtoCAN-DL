#pragma once

#include "ins_filter.h"

namespace Ins {

static_assert(sizeof(float)==4,"INS EEPROM format requires 32-bit float");

struct Config {
  Mount mount;
  Vec gyroOffset={};
  uint32_t canBitrate=1000000,gpsBaud=9600;
  bool calibrated=false,courseAid=false,gpsFast=false; // gpsFast: UBX 5Hz/38400 at boot.
};

constexpr unsigned ConfigBytes=40;
inline uint32_t crc32(const uint8_t *bytes,unsigned size) {
  uint32_t crc=0xFFFFFFFF;
  for (unsigned i=0;i<size;++i) {
    crc^=bytes[i];
    for (unsigned bit=0;bit<8;++bit) crc=(crc>>1)^(crc&1?0xEDB88320UL:0);
  }
  return ~crc;
}
inline void put32(uint8_t *bytes,uint32_t value) {
  for (unsigned i=0;i<4;++i) bytes[i]=static_cast<uint8_t>(value>>(i*8));
}
inline uint32_t get32(const uint8_t *bytes) {
  uint32_t value=0;
  for (unsigned i=0;i<4;++i) value|=static_cast<uint32_t>(bytes[i])<<(i*8);
  return value;
}
inline bool validConfig(const Config &c) {
  return c.mount.valid() && finite(c.gyroOffset) && norm(c.gyroOffset)<0.35f &&
      (c.canBitrate==125000 || c.canBitrate==250000 || c.canBitrate==500000 ||
       c.canBitrate==1000000) &&
      (c.gpsBaud==4800 || c.gpsBaud==9600 || c.gpsBaud==19200 ||
       c.gpsBaud==38400 || c.gpsBaud==57600 || c.gpsBaud==115200);
}
inline bool encodeConfig(const Config &c,uint8_t (&bytes)[ConfigBytes]) {
  if (!validConfig(c)) return false;
  memset(bytes,0,sizeof(bytes));
  bytes[0]='I'; bytes[1]='N'; bytes[2]='S'; bytes[3]=1;
  for (unsigned i=0;i<3;++i) bytes[4+i]=static_cast<uint8_t>(c.mount.axis[i]);
  bytes[7]=(c.calibrated?1:0)|(c.courseAid?2:0)|(c.gpsFast?4:0);
  put32(bytes+8,c.canBitrate); put32(bytes+12,c.gpsBaud);
  for (unsigned i=0;i<3;++i) {
    const float value=c.gyroOffset[i];
    memcpy(bytes+16+i*4,&value,4);
  }
  put32(bytes+36,crc32(bytes,36));
  return true;
}
inline bool decodeConfig(const uint8_t (&bytes)[ConfigBytes],Config &out) {
  if (bytes[0]!='I' || bytes[1]!='N' || bytes[2]!='S' || bytes[3]!=1 ||
      bytes[7]>7 || get32(bytes+36)!=crc32(bytes,36)) return false;
  Config c;
  for (unsigned i=0;i<3;++i) c.mount.axis[i]=static_cast<int8_t>(
      bytes[4+i]<128?bytes[4+i]:static_cast<int>(bytes[4+i])-256);
  c.calibrated=(bytes[7]&1)!=0; c.courseAid=(bytes[7]&2)!=0; c.gpsFast=(bytes[7]&4)!=0;
  c.canBitrate=get32(bytes+8); c.gpsBaud=get32(bytes+12);
  for (unsigned i=0;i<3;++i) memcpy(&c.gyroOffset[i],bytes+16+i*4,4);
  if (!validConfig(c)) return false;
  out=c; return true;
}

}
