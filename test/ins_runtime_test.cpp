#include "../src/ins_main.cpp"
#include <assert.h>
#include <map>
#include <array>

MockSerial Serial,Serial1;
MockCAN CAN;
MockEEPROM EEPROM;

namespace {
uint32_t fakeUs=0;
const i2c_master_cfg_t *driverConfig=nullptr;
std::map<uint8_t,std::array<uint8_t,256>> devices;
uint8_t device=0,reg=0;
bool sensorTimeout=false,exitAdapter=false;
void callback(i2c_master_event_t event) {
  i2c_master_callback_args_t args={driverConfig->p_context,event};
  driverConfig->p_callback(&args);
}
void run(unsigned ms) {
  for (unsigned i=0;i<ms;++i) { fakeUs+=1000; loop(); }
}
void execute(const char *text) {
  char line[128];
  assert(strlen(text)<sizeof(line));
  strcpy(line,text); command(line);
}
void feedGPS(const std::string &payload) {
  uint8_t checksum=0;
  for (unsigned char c:payload) checksum^=c;
  char suffix[8];
  snprintf(suffix,sizeof(suffix),"*%02X\r\n",checksum);
  Serial1.feed("$"+payload+suffix);
}
}

unsigned long micros() { return ++fakeUs; }
unsigned long millis() { return fakeUs/1000; }
void pinMode(unsigned,int) {}
void digitalWrite(unsigned,int) {}
fsp_err_t R_IOPORT_PinCfg(int *,unsigned,uint32_t) { return FSP_SUCCESS; }
fsp_err_t R_IIC_MASTER_Open(iic_master_instance_ctrl_t *,const i2c_master_cfg_t *c) {
  assert(c->channel==1);
  driverConfig=c; return FSP_SUCCESS;
}
fsp_err_t R_IIC_MASTER_Close(iic_master_instance_ctrl_t *) { return FSP_SUCCESS; }
fsp_err_t R_IIC_MASTER_Abort(iic_master_instance_ctrl_t *) { return FSP_SUCCESS; }
fsp_err_t R_IIC_MASTER_SlaveAddressSet(iic_master_instance_ctrl_t *,uint32_t address,int) {
  device=static_cast<uint8_t>(address); return FSP_SUCCESS;
}
fsp_err_t R_IIC_MASTER_Write(iic_master_instance_ctrl_t *,uint8_t *bytes,uint32_t n,bool) {
  if (sensorTimeout) return FSP_SUCCESS;
  if (!devices.count(device)) { callback(I2C_MASTER_EVENT_ABORTED); return FSP_SUCCESS; }
  assert(n>=1);
  reg=bytes[0];
  if (n==2) devices[device][reg]=reg==0xF4 && bytes[1]==0x25?0x24:bytes[1];
  callback(I2C_MASTER_EVENT_TX_COMPLETE); return FSP_SUCCESS;
}
fsp_err_t R_IIC_MASTER_Read(iic_master_instance_ctrl_t *,uint8_t *bytes,uint32_t n,bool) {
  if (sensorTimeout) return FSP_SUCCESS;
  for (uint32_t i=0;i<n;++i) bytes[i]=devices[device][reg+i];
  callback(I2C_MASTER_EVENT_RX_COMPLETE); return FSP_SUCCESS;
}
void slcanBegin() { closeBridgeCAN(); }
bool slcanInitialCommand(const char *,unsigned int) { return true; }
void slcanProcess() { if (exitAdapter) { savvyCAN=false; exitAdapter=false; } }
void slcanReceive(uint32_t,bool,uint8_t,const uint8_t *) {}
bool slcanCanRead() { return false; }
uint32_t slcanDroppedFrames() { return 0; }

int main() {
  auto &mpu=devices[0x68];
  mpu[0x75]=0x68; mpu[0x3A]=1;
  mpu[0x3F]=0x10; // Upside down, +1g on Z at +/-8g.
  auto &bmp=devices[0x76];
  bmp[0xD0]=0x58;
  const uint16_t trim[]={27504,26435,static_cast<uint16_t>(-1000),36477,
    static_cast<uint16_t>(-10685),3024,2855,140,static_cast<uint16_t>(-7),
    15500,static_cast<uint16_t>(-14600),6000};
  for (unsigned i=0;i<12;++i) {
    bmp[0x88+i*2]=static_cast<uint8_t>(trim[i]);
    bmp[0x89+i*2]=static_cast<uint8_t>(trim[i]>>8);
  }
  const uint8_t raw[]={0x65,0x5A,0xC0,0x7E,0xED,0};
  for (unsigned i=0;i<6;++i) bmp[0xF7+i]=raw[i];
  setup();
  assert(imuReady && baroReady && txEnabled && !config.calibrated);
  assert(Serial1.baud==9600 && CAN.bitrate==1000000);
  feedGPS("GPGSV,1,1,12");
  run(100);
  bool earlyInfo=false,earlyImu=false,earlyQuality=false;
  for (const CanMsg &frame:CAN.sent) {
    if (frame.id==0x401) {
      earlyInfo=true;
      assert(frame.data[5]==12 && frame.data[7]==1);
    }
    if (frame.id==0x403) {
      if (frame.data[6]==0 && frame.data[7]==100) earlyImu=true;
    }
    if (frame.id==0x40D) {
      earlyQuality=true;
      const uint16_t bits=static_cast<uint16_t>((frame.data[6]<<8)|frame.data[7]);
      assert((bits&InsCan::ImuUnavailable)!=0);
    }
  }
  assert(earlyInfo && earlyImu && earlyQuality && txEnabled && !filter.initialized);
  execute("CANSPEED 500");
  assert(txEnabled && CAN.bitrate==500000);
  execute("CANSPEED 1000");
  assert(txEnabled && CAN.bitrate==1000000);
  execute("TX ON"); assert(txEnabled);
  execute("MOUNT 1 1 3"); assert(config.mount.axis[1]==2);
  execute("CAL GYRO"); run(3300);
  assert(config.calibrated && filter.healthy && !filter.yawValid && !filter.originSet);
  assert(Ins::norm(config.gyroOffset)==0);
  assert(fabsf(fabsf(filter.attitude.euler().x)-Ins::Pi)<0.001f);
  assert(txEnabled);
  run(100);
  bool position=false,info=false,attitude=false,quality=false;
  for (const CanMsg &frame:CAN.sent) {
    if (frame.id==0x400) {
      position=true;
      for (uint8_t byte:frame.data) assert(byte==0);
    }
    if (frame.id==0x401 || frame.id==0x409) { info=true; assert(frame.data[7]==1); }
    if (frame.id==0x40C) {
      if (frame.data[6]&1) {
        attitude=true;
        assert((frame.data[6]&6)==0);
      }
    }
    if (frame.id==0x40D) quality=true;
  }
  assert(position && info && attitude && quality && txDropped==0);
  bool spaced=false;
  for (size_t k=0;k+9<CAN.sent.size();++k) {
    if (CAN.sent[k].id!=0x400 || CAN.sent[k+9].id!=0x40D) continue;
    spaced=true;
    for (unsigned j=1;j<10;++j) assert(CAN.sentAt[k+j]-CAN.sentAt[k]>=j*FrameSpacingMs);
    assert(CAN.sentAt[k+9]-CAN.sentAt[k]<=PendingExpiryMs);
  }
  assert(spaced);
  // Clipped (+8g) samples are transient: fusion and CAN continue, current RangeFault only.
  const uint32_t saturatedBefore=saturations;
  mpu[0x3B]=0x7F; mpu[0x3C]=0xFF;
  run(30);
  assert(filter.healthy && txEnabled && saturations>saturatedBefore);
  assert((faultBits(millis())&RangeFault) && (stickyFaults&RangeFault)==0);
  mpu[0x3B]=0; mpu[0x3C]=0;
  run(1100);
  assert(filter.healthy && (faultBits(millis())&RangeFault)==0 && gravityAccepted>0);
  assert(fabsf(fabsf(filter.attitude.euler().x)-Ins::Pi)<0.01f);
  const size_t accepted=CAN.sent.size();
  CAN.writeResult=-FSP_ERR_CAN_TRANSMIT_NOT_READY; run(100);
  assert(txEnabled && txBusy>0 && txDropped>0 && (stickyFaults&QueueFault));
  assert(CAN.sent.size()==accepted);
  execute("FAULT CLEAR"); assert(stickyFaults&QueueFault);
  execute("TX OFF"); execute("FAULT CLEAR"); assert(stickyFaults==0);
  execute("CAN INIT"); assert(!txEnabled);
  CAN.writeResult=1;
  execute("TX ON"); run(50); assert(txEnabled && CAN.sent.size()>accepted);
  execute("SAVVYCAN");
  assert(savvyCAN && !txEnabled && !canReady);
  exitAdapter=true; run(2);
  assert(!savvyCAN && txEnabled && canReady);
  execute("TX OFF");
  execute("SAVE"); assert(configLoaded && !filter.healthy);
  assert(filterState(millis())==InsCan::Uncalibrated);
  assert(faultBits(millis())&CalibrationRequired);
  run(3300); assert(filter.healthy && config.calibrated);
  execute("HEADING 0");
  assert(filter.yawValid && !filter.originSet && !txEnabled);
  feedGPS("GPGSA,A,3,,,,,,,,,,,,,1.5,0.9,1.2");
  feedGPS("GPRMC,120000.00,A,3300.0000,S,15100.0000,E,0.0,0.0,071126,,,A");
  feedGPS("GPGGA,120000.00,3300.0000,S,15100.0000,E,1,08,0.9,25.0,M,0,M,,");
  run(10);
  assert(gps.fresh(millis()) && filter.originSet && haveAided && globalValid(millis()));
  assert(faultBits(millis())&InsCan::CourseUnavailable);
  execute("TX ON"); run(100); assert(txEnabled);
  CAN.errorCode=8; CAN.error=true; run(1);
  assert(!txEnabled && (stickyFaults&CanFault) && lastCanEvent==8);
  CAN.errorCode=1;
  execute("CAN INIT"); assert(txEnabled);
  run(2200);
  assert(!globalValid(millis()) && filterState(millis())==InsCan::DeadReckoning);
  bool latestFused=false;
  for (auto it=CAN.sent.rbegin();it!=CAN.sent.rend();++it) {
    if (it->id==0x409) { latestFused=true; assert(it->data[7]==1); break; }
  }
  assert(latestFused);
  run(8000);
  assert(txEnabled && !filter.healthy && (stickyFaults&FilterFault));
  assert(initializing && reinits>0 && filterState(millis())==InsCan::Fault);
  // Slow 2 deg/s yaw (131 LSB at +/-500 deg/s) is turning, not stillness.
  mpu[0x47]=0; mpu[0x48]=0x83;
  const uint32_t turningBefore=stillRejectTurning;
  run(3300);
  assert(initializing && !filter.healthy && stillRejectTurning>turningBefore);
  mpu[0x48]=0;
  run(3300);
  assert(filter.healthy && !filter.yawValid && !filter.originSet && !haveAided && txEnabled);
  assert(filterState(millis())==InsCan::TiltOnly);
  execute("CAL GYRO"); run(3300); assert(filter.healthy);
  assert(txEnabled);
  CAN.writeResult=-2; run(50);
  assert(!txEnabled && (stickyFaults&CanFault));
  CAN.writeResult=1;   execute("CAN INIT"); assert(txEnabled);
  mpu[0x3A]=0;
  fakeUs=nextImu;
  const uint32_t previousSamples=samples;
  serviceIMU();
  assert(samples==previousSamples && nextImu>fakeUs && nextImu-fakeUs<=1000);
  run(60);
  assert(txEnabled && !filter.healthy && (stickyFaults&ImuFault));
  mpu[0x3A]=1;
  execute("SENSOR INIT"); run(3300); assert(filter.healthy && imuReady);
  assert(txEnabled);
  Serial1.output.clear();
  execute("GPS RATE 5");
  assert(ubxActive && config.gpsFast && Serial1.baud==9600);
  run(1500);
  assert(!ubxActive && Serial1.baud==Ubx::FastBaud && gpsBaudInUse()==38400);
  const std::string rate5("\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A",14);
  const std::string prt=Serial1.output.substr(0,6);
  assert(prt==std::string("\xB5\x62\x06\x00\x14\x00",6));
  assert(Serial1.output.find(rate5)!=std::string::npos && Serial1.output.size()==157);
  assert(filter.healthy && txEnabled);
  Serial1.output.clear();
  execute("GPS RATE 1"); run(1000);
  assert(!ubxActive && !config.gpsFast && Serial1.baud==9600 && Serial1.output.size()==42);
  // IMU bus loss: automatic quiet retry every 5s restores sensors without a command.
  sensorTimeout=true; run(20);
  assert(txEnabled && !imuReady && !bus.ready());
  assert(bus.error==InsI2C::Error::NotOpen || bus.error==InsI2C::Error::Timeout);
  sensorTimeout=false; run(5100);
  assert(imuReady && sensorRetries>0 && txEnabled);
  run(3300); assert(filter.healthy);
  puts("ins_runtime_test: PASS");
}
