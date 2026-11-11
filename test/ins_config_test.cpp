#include "ins_config.h"
#include <assert.h>
#include <stdio.h>

int main() {
  Ins::Config config,decoded;
  uint8_t bytes[Ins::ConfigBytes];
  assert(Ins::validConfig(config));
  assert(!config.calibrated && config.gyroOffset.x==0);
  config.mount.axis[0]=-2; config.mount.axis[1]=1; config.mount.axis[2]=3;
  config.gyroOffset={0.01f,-0.02f,0.03f}; config.calibrated=true;
  config.courseAid=true; config.gpsBaud=38400; config.canBitrate=500000;
  assert(Ins::encodeConfig(config,bytes));
  assert(Ins::decodeConfig(bytes,decoded));
  assert(decoded.mount.axis[0]==-2 && decoded.mount.axis[1]==1);
  assert(decoded.calibrated && decoded.courseAid && decoded.canBitrate==500000);
  assert(decoded.gpsBaud==38400 && decoded.gyroOffset.y==-0.02f && !decoded.gpsFast);
  config.gpsFast=true;
  assert(Ins::encodeConfig(config,bytes) && bytes[7]==7);
  assert(Ins::decodeConfig(bytes,decoded) && decoded.gpsFast && decoded.courseAid);
  bytes[7]=8; Ins::put32(bytes+36,Ins::crc32(bytes,36));
  assert(!Ins::decodeConfig(bytes,decoded));
  assert(Ins::encodeConfig(config,bytes));
  for (unsigned i=0;i<sizeof(bytes);++i) {
    bytes[i]^=1;
    assert(!Ins::decodeConfig(bytes,decoded));
    bytes[i]^=1;
  }
  config.mount.axis[2]=-3;
  assert(!Ins::encodeConfig(config,bytes));
  config=Ins::Config{}; config.canBitrate=42;
  assert(!Ins::validConfig(config));
  config=Ins::Config{}; config.gyroOffset.x=NAN;
  assert(!Ins::validConfig(config));
  puts("ins_config_test: PASS");
}
