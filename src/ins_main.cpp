#include <Arduino.h>
#include <Arduino_CAN.h>
#include <EEPROM.h>
#include "bounded_console.h"
#include "usb_connection.h"
#include "slcan_bridge.h"
#include "slcan_protocol.h"
#include "ins_i2c.h"
#include "ins_filter.h"
#include "ins_gps.h"
#include "ins_config.h"
#include "ins_can.h"
#include "ins_ubx.h"
#include "sensor_decode.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

bool savvyCAN=false;

namespace {

class Console : public Print {
public:
  using Print::write;
  size_t write(uint8_t v) override { return buffer.write(v); }
  size_t write(const uint8_t *v,size_t n) override { return buffer.write(v,n); }
  void drain() { if (usbSerialConnected()) buffer.drain(Serial,64); }
  BoundedConsoleBuffer buffer;
} console;

InsI2C bus;
Ins::Config config;
Ins::Filter filter;
Ins::GPS gps;
Ins::StationaryCalibration calibration;
SensorDecode::BMPCalibration baroTrim;
Ins::Vec acceleration={},gyro={};
InsCan::Frame pending[10]={};
uint8_t pendingCount=0,pendingIndex=0,rawCounter=0,rawEmpty=0,fusedCounter=0,fusedEmpty=0;
uint8_t imuAddress=0,baroAddress=0;
bool imuReady=false,baroReady=false,canReady=false,txEnabled=false;
bool calibrating=false,initializing=false,baroMeasuring=false,resumeAfterBridge=false;
bool txManualOff=false;
bool configLoaded=false,haveImu=false,haveBaro=false,haveAided=false;
uint32_t lastImu=0,lastImuUs=0,nextImu=0,lastBaro=0,baroStarted=0;
uint32_t lastGPSSequence=0,lastAided=0,lastPublish=0,pendingStarted=0;
uint32_t samples=0,overruns=0,maxLoopUs=0,maxFilterUs=0,txAccepted=0,txBusy=0,txDropped=0;
uint32_t calibrationStart=0,lastHealth=0;
uint16_t lastCanEvent=0;
int lastCanWriteResult=0;
uint16_t stickyFaults=0;
float pressure=0,baroTemperature=0,pressureOrigin=0,baroDown=0,baroOriginDown=0;
bool baroAligned=false;
// Robustness: transient saturation, gravity aiding, automatic recovery.
bool saturationSeen=false,quietInit=false,gpsAccelValid=false,previousSpeedValid=false;
uint32_t saturations=0,lastSaturation=0,gravityAccepted=0,gravityRejected=0,gravitySkipped=0;
uint32_t lastGravity=0,lastReinit=0,reinits=0,lastSensorRetry=0,sensorRetries=0;
uint32_t gpsEpochMs=0,previousFixReceived=0,gpsAccelTime=0;
uint32_t stillRejectSample=0,stillRejectNoise=0,stillRejectTurning=0;
float gpsAccel=0,previousSpeed=0;
// Non-blocking UBX configuration (one byte per loop; UART::write busy-waits).
uint8_t ubxBuffer[Ubx::MaxFrame]={};
uint8_t ubxIndex=0,ubxLength=0,ubxOffset=0;
bool ubxActive=false,ubxFast=false,ubxSwitchPending=false;
uint32_t ubxWait=0;
constexpr uint32_t FrameSpacingMs=4,PendingExpiryMs=38;
constexpr float StillTurnRate=1.0f*Ins::Deg; // Batch-mean gyro vs saved bias.
constexpr int ConfigAddress=512;
constexpr uint16_t BusFault=1,ImuFault=2,BaroFault=4,TimingFault=8,GPSUnavailable=16;
constexpr uint16_t CalibrationRequired=32,FilterFault=64,CanFault=128,ConfigFault=256;
constexpr uint16_t RangeFault=512,InnovationFault=1024,QueueFault=2048;

void clearPending() {
  pendingCount=pendingIndex=0;
}

void stopTX() {
  txEnabled=false;
  clearPending();
}

void startTX() {
  if (canReady && !savvyCAN && !txManualOff) {
    txEnabled=true;
    lastPublish=millis();
  }
}

void notice(const __FlashStringHelper *text) {
  if (!savvyCAN) console.println(text);
}

bool imuFresh(uint32_t now) { return imuReady && haveImu && now-lastImu<50; }
bool baroFresh(uint32_t now) { return baroReady && haveBaro && now-lastBaro<250; }

void resetNavigation() {
  clearPending();
  filter.healthy=filter.initialized=filter.yawValid=filter.originSet=false;
  haveAided=false; baroAligned=false;
  calibration.reset();
  calibrating=false; initializing=config.calibrated;
  calibrationStart=millis(); lastImuUs=0;
}

void loadConfig() {
  uint8_t bytes[Ins::ConfigBytes];
  for (unsigned i=0;i<sizeof(bytes);++i) bytes[i]=EEPROM.read(ConfigAddress+i);
  configLoaded=Ins::decodeConfig(bytes,config);
  if (!configLoaded) {
    config=Ins::Config{};
    notice(F("Config absent/invalid: defaults; calibration required; automatic CAN follows init."));
  }
}

void saveConfig() {
  if (txEnabled || calibrating) { notice(F("ERR: TX OFF and finish calibration before SAVE")); return; }
  uint8_t bytes[Ins::ConfigBytes],check[Ins::ConfigBytes];
  if (!Ins::encodeConfig(config,bytes)) { notice(F("ERR: Invalid config")); return; }
  EEPROM.update(ConfigAddress,0);
  for (unsigned i=1;i<sizeof(bytes);++i) EEPROM.update(ConfigAddress+i,bytes[i]);
  EEPROM.update(ConfigAddress,bytes[0]);
  for (unsigned i=0;i<sizeof(check);++i) check[i]=EEPROM.read(ConfigAddress+i);
  if (memcmp(bytes,check,sizeof(bytes))!=0) {
    stickyFaults|=ConfigFault;
    notice(F("ERR: EEPROM verification failed"));
  } else {
    configLoaded=true;
    notice(F("OK: Mount/config/gyro bias saved with CRC; TX is never saved"));
  }
  resetNavigation();
}

void sensorError(const __FlashStringHelper *what) {
  if (!bus.ready()) stickyFaults|=BusFault;
  if (quietInit) return;
  console.print(what);
  console.print(F("; I2C error enum="));
  console.println(static_cast<unsigned>(bus.error));
}

bool read(uint8_t address,uint8_t reg,uint8_t *data,size_t size) {
  return bus.read(address,reg,data,size);
}

bool find(uint8_t first,uint8_t reg,uint8_t identity,uint8_t &address) {
  for (uint8_t i=0;i<2;++i) {
    uint8_t id=0;
    if (read(first+i,reg,&id,1)) {
      if (id==identity) { address=first+i; return true; }
      console.print(F("ERR: Unexpected chip ID 0x")); console.println(id,HEX);
    } else if (!bus.ready()) return false;
  }
  return false;
}

void initSensors() {
  resetNavigation();
  imuReady=baroReady=haveImu=haveBaro=false;
  baroMeasuring=false; pressureOrigin=0;
  if (!bus.begin()) { stickyFaults|=BusFault; sensorError(F("ERR: I2C init")); return; }
  if (find(0x68,0x75,0x68,imuAddress)) {
    imuReady=bus.write(imuAddress,0x6B,1) && bus.write(imuAddress,0x6C,0) &&
      bus.write(imuAddress,0x1A,3) && bus.write(imuAddress,0x19,9) &&
      bus.write(imuAddress,0x1B,0x08) && bus.write(imuAddress,0x1C,0x10) &&
      bus.write(imuAddress,0x38,1);
    uint8_t ranges[2]={},power[2]={};
    imuReady=imuReady && read(imuAddress,0x1B,ranges,2) && read(imuAddress,0x6B,power,2) &&
        ranges[0]==0x08 && ranges[1]==0x10 && power[0]==1 && power[1]==0;
  }
  if (!imuReady) { stickyFaults|=ImuFault; sensorError(F("ERR: MPU-6050 init/readback")); }
  else notice(F("MPU-6050: +/-8g, +/-500deg/s, 100Hz; acquisition starts after settling"));
  if (bus.ready() && find(0x76,0xD0,0x58,baroAddress)) {
    uint8_t bytes[24]={};
    uint8_t status=0;
    baroReady=bus.write(baroAddress,0xF4,0) && bus.write(baroAddress,0xF5,0) &&
      read(baroAddress,0xF3,&status,1) && (status&9)==0 &&
      read(baroAddress,0x88,bytes,24) && SensorDecode::decodeBMPCalibration(bytes,baroTrim);
  }
  if (!baroReady) { stickyFaults|=BaroFault; sensorError(F("ERR: BMP280 init/calibration")); }
  else notice(F("BMP280: asynchronous forced conversions, 20Hz"));
  nextImu=micros()+50000;
  lastImuUs=0;
  lastSensorRetry=millis();
}

void beginInitialization(uint32_t now) {
  // Keeps queued CAN frames; only the fused state is re-acquired.
  calibration.reset();
  initializing=true; calibrating=false;
  calibrationStart=now; lastImuUs=0;
  haveAided=false; baroAligned=false;
}

void calibrationSample(uint32_t now,bool saturated) {
  if (!calibrating && !initializing) return;
  // Tilt must come from gravity alone: reject GPS motion and turning/bias mismatch.
  // A saved bias tolerates engine vibration; turning is judged by the batch mean.
  const Ins::StillLimits &limits=calibrating?Ins::StrictStill:Ins::VibrationStill;
  const bool moving=gps.fresh(now) && gps.fix.velocityValid && gps.fix.speed>1;
  if (saturated || moving || !calibration.addSample(acceleration,gyro,limits)) {
    if (calibration.count) ++stillRejectSample;
    calibration.reset();
    if (calibrating && now-calibrationStart>15000) {
      calibrating=false;
      notice(F("ERR: Stationary calibration timed out; hold still and CAL GYRO"));
    }
    return;
  }
  if (calibration.count>=300) {
    const bool turning=initializing &&
        Ins::norm(Ins::sub(calibration.meanGyro,config.gyroOffset))>StillTurnRate;
    if (turning || !calibration.ready(limits)) {
      ++(turning?stillRejectTurning:stillRejectNoise);
      calibration.reset();
      if (calibrating && now-calibrationStart>15000) {
        calibrating=false;
        notice(F("ERR: Excess motion/noise during calibration; CAL GYRO to retry"));
      }
      return;
    }
    if (calibrating) {
      config.gyroOffset=calibration.meanGyro;
      config.calibrated=true;
    }
    if (!filter.initialize(calibration.meanAccel,config.gyroOffset)) {
      stickyFaults|=FilterFault; notice(F("ERR: Filter initialization failed"));
    } else {
      notice(F("OK: Tilt initialized; yaw unknown. Gyro bias only; no accel/level offsets learned."));
      if (calibrating) notice(F("Use SAVE to persist this new calibration, then hold still for tilt initialization."));
    }
    calibrating=initializing=false;
    lastImuUs=0;
  }
}

void gravityAid(uint32_t now) {
  if (!filter.healthy || now-lastGravity<100) return;
  lastGravity=now;
  const Ins::Vec w=Ins::sub(gyro,filter.gyroBias);
  float speed=0;
  bool speedKnown=false;
  if (gps.fresh(now) && gps.fix.velocityValid) { speed=gps.fix.speed; speedKnown=true; }
  else if (filter.originSet && haveAided && now-lastAided<2000) {
    speed=sqrtf(filter.velocity.x*filter.velocity.x+filter.velocity.y*filter.velocity.y);
    speedKnown=true;
  }
  Ins::Vec kinematic={};
  float noise=1.0f;
  if (speedKnown) {
    // Forward, low-slip body motion: dv/dt along X, centripetal v*yawRate on Y.
    const bool along=gpsAccelValid && now-gpsAccelTime<1500;
    kinematic={along?gpsAccel:0,speed*w.z,-speed*w.y};
    noise=0.25f+0.05f*Ins::dot(kinematic,kinematic)+(speed>1?(along?1.0f:4.0f):0);
  } else if (Ins::norm(w)>3*Ins::Deg) {
    ++gravitySkipped; return; // Unknown dynamics while rotating.
  }
  const Ins::Vec expected=Ins::add(kinematic,filter.accelBias);
  if (fabsf(Ins::norm(Ins::sub(acceleration,expected))-Ins::Gravity)>2.0f ||
      !filter.correctGravity(acceleration,kinematic,noise)) {
    if (filter.healthy) ++gravityRejected;
    else {
      stickyFaults|=FilterFault;
      notice(F("ERR: Gravity update numerical fault; fused navigation invalid"));
    }
    return;
  }
  ++gravityAccepted;
}

void serviceIMU() {
  const uint32_t due=micros();
  if (!imuReady || static_cast<int32_t>(due-nextImu)<0) return;
  nextImu=due+10000;
  uint8_t registers[15]={};
  if (!read(imuAddress,0x3A,registers,sizeof(registers))) {
    imuReady=false; filter.healthy=false; haveImu=false;
    stickyFaults|=ImuFault; lastSensorRetry=millis();
    sensorError(F("ERR: MPU read failed; fused navigation invalid; automatic retry every 5s"));
    return;
  }
  if ((registers[0]&1)==0) {
    // Sensor and MCU clocks drift; do not skip a period on an early poll.
    nextImu=micros()+1000;
    return;
  }
  const uint8_t *bytes=registers+1;
  Ins::Vec a={},w={};
  bool saturated=false;
  for (unsigned i=0;i<3;++i) {
    const int16_t rawA=SensorDecode::bigSigned16(bytes+i*2);
    const int16_t rawW=SensorDecode::bigSigned16(bytes+8+i*2);
    if (rawA>=32760 || rawA<=-32760 || rawW>=32760 || rawW<=-32760) saturated=true;
    a[i]=rawA*(Ins::Gravity/4096.0f);
    w[i]=rawW*(Ins::Deg/65.5f);
  }
  acceleration=config.mount.apply(a); gyro=config.mount.apply(w);
  const uint32_t now=millis(),sampleUs=micros();
  haveImu=true; lastImu=now; ++samples;
  if (saturated) {
    // Clipped samples are used as-is; uncertainty is inflated instead of failing.
    ++saturations; lastSaturation=now; saturationSeen=true;
  }
  const bool wasInitialized=filter.initialized;
  calibrationSample(now,saturated);
  if (filter.healthy && wasInitialized && lastImuUs!=0) {
    const uint32_t elapsed=sampleUs-lastImuUs;
    if (elapsed>20000) ++overruns;
    const uint32_t started=micros();
    if (!filter.predict(acceleration,gyro,elapsed*1e-6f)) {
      ++overruns; stickyFaults|=TimingFault|FilterFault;
      notice(F("ERR: Prediction/timing fault; fused navigation invalid; raw CAN continues"));
    } else if (saturated) {
      if (!filter.addUncertainty(0.0003f,0.04f)) stickyFaults|=FilterFault;
    } else gravityAid(now);
    const uint32_t duration=micros()-started;
    if (duration>maxFilterUs) maxFilterUs=duration;
  }
  lastImuUs=sampleUs;
}

void serviceBaro() {
  const uint32_t now=millis();
  if (!baroReady) return;
  if (!baroMeasuring && now-baroStarted>=50) {
    if (!bus.write(baroAddress,0xF4,0x25)) {
      baroReady=false; stickyFaults|=BaroFault;
      sensorError(F("ERR: BMP conversion start")); return;
    }
    baroStarted=now; baroMeasuring=true;
    return;
  }
  if (!baroMeasuring || now-baroStarted<10) return;
  uint8_t status=0;
  if (!read(baroAddress,0xF3,&status,1)) {
    baroReady=false; stickyFaults|=BaroFault;
    sensorError(F("ERR: BMP status read")); return;
  }
  if ((status&9)!=0) {
    if (now-baroStarted>50) {
      baroReady=false; stickyFaults|=BaroFault; notice(F("ERR: BMP conversion timeout"));
    }
    return;
  }
  uint8_t bytes[6]={};
  SensorDecode::BMPSample sample={};
  if (!read(baroAddress,0xF7,bytes,6) || !SensorDecode::decodeBMP280(bytes,baroTrim,sample) ||
      sample.pressurePa<30000 || sample.pressurePa>110000 ||
      sample.temperatureC<-40 || sample.temperatureC>85) {
    baroReady=false; stickyFaults|=BaroFault;
    sensorError(F("ERR: BMP invalid sample")); return;
  }
  baroMeasuring=false; haveBaro=true; lastBaro=now;
  pressure=sample.pressurePa; baroTemperature=sample.temperatureC;
  if (pressureOrigin==0) pressureOrigin=pressure;
  baroDown=44330.0f*(powf(pressure/pressureOrigin,0.19029495f)-1);
  if (filter.healthy && filter.originSet) {
    if (!baroAligned) { baroOriginDown=baroDown-filter.position.z; baroAligned=true; }
    if (!filter.correctHeight(baroDown-baroOriginDown,9)) stickyFaults|=InnovationFault;
  }
}

void updateGPS() {
  if (lastGPSSequence==gps.fix.sequence) return;
  lastGPSSequence=gps.fix.sequence;
  const Ins::Fix &fix=gps.fix;
  const uint32_t interval=fix.received-previousFixReceived;
  if (previousFixReceived!=0 && interval>0 && interval<3000)
    gpsEpochMs=gpsEpochMs?(gpsEpochMs*3+interval)/4:interval;
  if (fix.valid && fix.velocityValid) {
    if (previousSpeedValid && interval>=50 && interval<=1500) {
      gpsAccel=(fix.speed-previousSpeed)*1000.0f/interval;
      gpsAccelTime=fix.received; gpsAccelValid=true;
    }
    previousSpeed=fix.speed; previousSpeedValid=true;
  } else previousSpeedValid=false;
  previousFixReceived=fix.received;
  if (!gps.fresh(millis()) || !fix.heightValid || !filter.healthy) return;
  if (fabs(fix.latitude)>85) {
    if ((stickyFaults&FilterFault)==0) notice(F("ERR: Local INS frame unsupported above 85deg latitude"));
    stickyFaults|=FilterFault;
    return;
  }
  if (!filter.yawValid && config.courseAid && fix.velocityValid && fix.speed>=3 &&
      fabsf(gyro.z)<0.1f && fabsf(Ins::norm(acceleration)-Ins::Gravity)<0.5f) {
    if (filter.setHeading(fix.course*Ins::Deg)) {
      baroAligned=false;
      notice(F("Heading initialized from moving course (forward/low-slip assumption)"));
    }
  }
  if (!filter.yawValid) return;
  if (!filter.originSet) {
    filter.setOrigin(fix.latitude,fix.longitude,fix.height);
    if (fix.velocityValid) {
      filter.velocity={fix.speed*cosf(fix.course*Ins::Deg),fix.speed*sinf(fix.course*Ins::Deg),0};
    }
    baroAligned=false;
  }
  // 5Hz GPS errors are time-correlated; do not treat epochs as independent.
  const float epochScale=gpsEpochMs!=0 && gpsEpochMs<500?2.0f:1.0f;
  const float sigma=fmaxf(3,fix.hdop*3);
  const Ins::Vec local=filter.local(fix.latitude,fix.longitude,fix.height);
  if (!filter.correctPosition(local,sigma*sigma*epochScale,sigma*sigma*4*epochScale)) {
    stickyFaults|=InnovationFault; return;
  }
  if (fix.velocityValid) {
    const Ins::Vec velocity={fix.speed*cosf(fix.course*Ins::Deg),
                            fix.speed*sinf(fix.course*Ins::Deg),0};
    if (!filter.correctVelocity(velocity,epochScale)) stickyFaults|=InnovationFault;
  }
  haveAided=true; lastAided=millis();
}

uint32_t gpsBaudInUse() { return config.gpsFast?Ubx::FastBaud:config.gpsBaud; }

void restartGPSUart(uint32_t baud,bool transmit) {
  Serial1.end(); Serial1.begin(baud,SERIAL_8N1);
  if (!transmit) pinMode(1,INPUT); // D1 stays high-impedance unless configuring.
}

void startGPSConfig(bool fast) {
  ubxActive=true; ubxFast=fast; ubxIndex=0; ubxLength=ubxOffset=0;
  ubxSwitchPending=false; ubxWait=millis();
  restartGPSUart(fast?config.gpsBaud:Ubx::FastBaud,true);
}

void serviceGPSConfig(uint32_t now) {
  if (!ubxActive || static_cast<int32_t>(now-ubxWait)<0) return;
  if (ubxOffset<ubxLength) {
    Serial1.write(ubxBuffer[ubxOffset++]);
    if (ubxOffset==ubxLength) ubxWait=now+(ubxSwitchPending?150:30);
    return;
  }
  if (ubxSwitchPending) {
    ubxSwitchPending=false;
    restartGPSUart(ubxFast?Ubx::FastBaud:config.gpsBaud,true);
    ubxWait=now+100;
    return;
  }
  ubxLength=static_cast<uint8_t>(Ubx::sequence(ubxFast,ubxIndex,ubxBuffer));
  if (ubxLength==0) {
    ubxActive=false;
    restartGPSUart(gpsBaudInUse(),false);
    notice(ubxFast?F("GPS: UBX 5Hz/38400 sent; verify STATUS GPS epoch interval ~200ms"):
                   F("GPS: UBX revert-to-defaults sent; verify epoch interval ~1000ms"));
    return;
  }
  ubxSwitchPending=Ubx::switchesBaudAfter(ubxIndex);
  ubxOffset=0; ++ubxIndex;
}

void serviceGPS() {
  for (unsigned i=0;i<192 && Serial1.available()>0;++i) {
    const int value=Serial1.read();
    if (value<0) break;
    gps.feed(static_cast<uint8_t>(value),millis());
  }
  updateGPS();
}

bool yawUsable() {
  return filter.healthy && filter.yawValid && filter.variance(8)<0.274156f;
}

bool globalValid(uint32_t now) {
  return yawUsable() && filter.originSet && haveAided &&
      gps.fresh(now) && now-lastAided<2000 && imuFresh(now) &&
      filter.variance(0)+filter.variance(1)<625 && filter.variance(2)<900;
}

float headingDegrees(float radians) {
  float deg=radians/Ins::Deg;
  if (deg<0) deg+=360;
  return deg>=360?0:deg;
}

InsCan::State filterState(uint32_t now) {
  if (!imuFresh(now)) return InsCan::Fault;
  if (!config.calibrated || !filter.initialized) return InsCan::Uncalibrated;
  if (!filter.healthy) return InsCan::Fault;
  if (globalValid(now)) return InsCan::GpsAided;
  return filter.originSet?InsCan::DeadReckoning:InsCan::TiltOnly;
}

uint16_t faultBits(uint32_t now) {
  uint16_t bits=stickyFaults;
  if (!bus.ready()) bits|=BusFault;
  if (!imuFresh(now)) bits|=ImuFault;
  if (!imuFresh(now) || !filter.healthy) bits|=InsCan::ImuUnavailable;
  if (!baroFresh(now)) bits|=BaroFault;
  if (!gps.fresh(now)) bits|=GPSUnavailable;
  if (!config.calibrated || !filter.initialized) bits|=CalibrationRequired;
  if (!gps.fresh(now) || !gps.fix.velocityValid || gps.fix.speed<0.5f)
    bits|=InsCan::CourseUnavailable;
  if (!yawUsable()) bits|=InsCan::HeadingUnavailable;
  if (filter.initialized && !filter.healthy) bits|=FilterFault;
  if (saturationSeen && now-lastSaturation<1000) bits|=RangeFault; // Current, not sticky.
  return bits;
}

bool buildBank(bool fused,InsCan::Frame (&frames)[4],uint32_t now) {
  InsCan::Snapshot snapshot={};
  const bool valid=fused?globalValid(now):gps.fresh(now);
  snapshot.status=valid?(gps.fix.heightValid?4:3):1;
  snapshot.positionValid=valid;
  snapshot.imuValid=imuFresh(now) && (!fused || filter.healthy);
  snapshot.satellites=gps.satellites(now);
  if (valid) {
    snapshot.lat=fused?filter.latitude():gps.fix.latitude;
    snapshot.lon=fused?filter.longitude():gps.fix.longitude;
    snapshot.heightM=fused?filter.height():gps.fix.heightValid?gps.fix.height:0;
    snapshot.courseValid=fused?
      sqrtf(filter.velocity.x*filter.velocity.x+filter.velocity.y*filter.velocity.y)>1:
      gps.fix.velocityValid && gps.fix.speed>=0.5f;
    snapshot.speedMps=fused?sqrtf(filter.velocity.x*filter.velocity.x+
                                filter.velocity.y*filter.velocity.y):gps.fix.speed;
    snapshot.courseDeg=fused?headingDegrees(atan2f(filter.velocity.y,filter.velocity.x)):gps.fix.course;
  }
  snapshot.headingValid=fused && yawUsable();
  snapshot.headingDeg=snapshot.headingValid?headingDegrees(filter.attitude.euler().z):0;
  if (snapshot.imuValid) {
    const Ins::Vec w=fused?Ins::sub(gyro,filter.gyroBias):gyro;
    const Ins::Vec a=fused?Ins::sub(acceleration,filter.accelBias):acceleration;
    for (unsigned i=0;i<3;++i) {
      const float rate=w[i]/Ins::Deg,accel=a[i]/Ins::Gravity;
      snapshot.rateDps[i]=fmaxf(-InsCan::MaxRateDps,fminf(InsCan::MaxRateDps,rate));
      snapshot.accelG[i]=fmaxf(-InsCan::MaxAccelG,fminf(InsCan::MaxAccelG,accel));
      if (snapshot.rateDps[i]!=rate || snapshot.accelG[i]!=accel) {
        lastSaturation=now; saturationSeen=true;
      }
    }
  }
  uint8_t &counter=fused?fusedCounter:rawCounter;
  uint8_t &empty=fused?fusedEmpty:rawEmpty;
  if (valid) counter=(counter+1)&15; else empty=(empty+1)&15;
  return InsCan::encodeBank(snapshot,fused,counter,empty,frames);
}

void publish(uint32_t now) {
  if (!txEnabled || savvyCAN || !canReady || now-lastPublish<40) return;
  lastPublish=now;
  if (pendingIndex<pendingCount) {
    txDropped+=pendingCount-pendingIndex; stickyFaults|=QueueFault;
  }
  pendingIndex=pendingCount=0;
  InsCan::Frame frames[4];
  if (!buildBank(false,frames,now)) {
    stickyFaults|=RangeFault; stopTX(); notice(F("ERR: Raw CAN value out of protocol range; TX OFF")); return;
  }
  for (unsigned i=0;i<4;++i) pending[pendingCount++]=frames[i];
  if (!buildBank(true,frames,now)) {
    stickyFaults|=RangeFault; stopTX(); notice(F("ERR: Fused CAN value out of protocol range; TX OFF")); return;
  }
  for (unsigned i=0;i<4;++i) pending[pendingCount++]=frames[i];
  const bool tilt=filter.healthy && imuFresh(now);
  const Ins::Vec angles=tilt?filter.attitude.euler():Ins::Vec{};
  InsCan::Attitude attitude={};
  attitude.rollDeg=angles.x/Ins::Deg; attitude.pitchDeg=angles.y/Ins::Deg;
  attitude.yawDeg=yawUsable()?headingDegrees(angles.z):0;
  attitude.tiltValid=tilt; attitude.yawValid=tilt && yawUsable();
  attitude.globalValid=globalValid(now); attitude.calibrated=config.calibrated;
  attitude.imuFresh=imuFresh(now); attitude.gpsFresh=gps.fresh(now);
  attitude.baroFresh=baroFresh(now); attitude.txFault=(stickyFaults&(CanFault|RangeFault|QueueFault))!=0;
  attitude.state=filterState(now);
  if (!InsCan::encodeAttitude(attitude,pending[pendingCount])) {
    stickyFaults|=RangeFault; stopTX(); notice(F("ERR: Attitude CAN encoding failed; TX OFF")); return;
  }
  ++pendingCount;
  InsCan::Quality quality={};
  quality.gpsAgeMs=gps.fix.sequence?now-gps.fix.received:UINT32_MAX;
  quality.horizontalSigmaM=filter.originSet && filter.healthy?
      sqrtf(filter.variance(0)+filter.variance(1)):65535;
  quality.verticalSigmaM=filter.originSet && filter.healthy?sqrtf(filter.variance(2)):65535;
  quality.faults=faultBits(now);
  // Unknown/unrepresentable quality uses the codec's explicit sentinel/fault.
  static_cast<void>(InsCan::encodeQuality(quality,pending[pendingCount++]));
  pendingStarted=now;
}

void serviceCAN() {
  int error=0;
  if (canReady && CAN.isError(error)) {
    lastCanEvent=static_cast<uint16_t>(error);
    if (txEnabled) {
      console.print(F("ERR: CAN controller event 0x"));
      console.print(lastCanEvent,HEX);
      console.println(F("; TX OFF; CAN INIT to recover"));
    }
    stickyFaults|=CanFault; stopTX();
  }
  for (unsigned i=0;i<16 && canReady && CAN.available();++i) {
    const CanMsg frame=CAN.read();
    if (savvyCAN) slcanReceive(frame.isExtendedId()?frame.getExtendedId():frame.getStandardId(),
                              frame.isExtendedId(),frame.data_length,frame.data);
  }
  if (!txEnabled || savvyCAN || pendingIndex>=pendingCount) return;
  const uint32_t elapsed=millis()-pendingStarted;
  if (elapsed>PendingExpiryMs) {
    txDropped+=pendingCount-pendingIndex; pendingIndex=pendingCount;
    stickyFaults|=QueueFault;
    return;
  }
  // Spread the 10-frame burst across the 40ms cycle for receivers/loggers.
  if (elapsed<pendingIndex*FrameSpacingMs) return;
  const InsCan::Frame &frame=pending[pendingIndex];
  const CanMsg message(CanStandardId(frame.id),8,frame.data);
  const int result=CAN.write(message);
  if (result>0) { ++pendingIndex; ++txAccepted; }
  else if (result==-FSP_ERR_CAN_TRANSMIT_NOT_READY) ++txBusy;
  else {
    lastCanWriteResult=result;
    stickyFaults|=CanFault; stopTX();
    console.print(F("ERR: CAN write result ")); console.println(result);
  }
}

void status() {
  const uint32_t now=millis();
  console.println(F("\nINS VEHICLE / no SD / automatic CAN broadcast"));
  console.print(F("State (0 uncal,1 tilt,2 GPS,3 DR,4 fault): ")); console.println(filterState(now));
  console.print(F("TX: ")); console.println(txEnabled?F("ON, 25Hz"):F("OFF"));
  console.print(F("CAN bitrate: ")); console.println(config.canBitrate);
  console.print(F("Mount vehicle XYZ <- signed sensor axes: "));
  for (unsigned i=0;i<3;++i) { console.print(config.mount.axis[i]); console.print(' '); }
  console.println();
  console.print(F("Calibrated / saved config / course aid: "));
  console.print(config.calibrated); console.print('/'); console.print(configLoaded);
  console.print('/'); console.println(config.courseAid);
  console.print(F("Gyro offset deg/s XYZ: "));
  for (unsigned i=0;i<3;++i) { console.print(config.gyroOffset[i]/Ins::Deg,4); console.print(' '); }
  console.println();
  console.print(F("I2C ready/error/failures/max us: "));
  console.print(bus.ready()); console.print('/');
  console.print(static_cast<unsigned>(bus.error)); console.print('/');
  console.print(bus.failures); console.print('/'); console.println(bus.maxTransactionUs);
  console.print(F("Samples / >20ms intervals / max filter us / max loop us: "));
  console.print(samples); console.print('/'); console.print(overruns); console.print('/');
  console.print(maxFilterUs); console.print('/'); console.println(maxLoopUs);
  console.print(F("Fault bits hex (sticky + current): ")); console.println(faultBits(now),HEX);
  console.print(F("Calibrating / initializing / auto re-inits / sensor retries: "));
  console.print(calibrating); console.print('/'); console.print(initializing); console.print('/');
  console.print(reinits); console.print('/'); console.println(sensorRetries);
  console.print(F("Still batch rejects sample-motion / noise / turning: "));
  console.print(stillRejectSample); console.print('/'); console.print(stillRejectNoise);
  console.print('/'); console.println(stillRejectTurning);
  console.print(F("IMU saturated samples / last saturation age ms: "));
  console.print(saturations); console.print('/');
  if (saturationSeen) console.println(now-lastSaturation); else console.println(F("never"));
  console.print(F("Gravity updates accepted / rejected / skipped: "));
  console.print(gravityAccepted); console.print('/'); console.print(gravityRejected);
  console.print('/'); console.println(gravitySkipped);
  console.print(F("CAN accepted / busy retries / dropped: "));
  console.print(txAccepted); console.print('/'); console.print(txBusy); console.print('/'); console.println(txDropped);
  console.print(F("CAN controller event hex / last write result: "));
  console.print(lastCanEvent,HEX); console.print('/'); console.println(lastCanWriteResult);
  console.print(F("Console dropped bytes: ")); console.println(console.buffer.droppedBytes());
  console.print(F("GPS bytes / valid NMEA / parse errors / epoch: "));
  console.print(gps.bytes); console.print('/'); console.print(gps.validSentences);
  console.print('/'); console.print(gps.errors); console.print('/'); console.println(gps.fix.sequence);
  console.print(F("GPS device baud / 5Hz config / UBX sending / epoch interval ms: "));
  console.print(gpsBaudInUse()); console.print('/'); console.print(config.gpsFast);
  console.print('/'); console.print(ubxActive); console.print('/'); console.println(gpsEpochMs);
  console.print(F("GPS fresh / dimension / height valid: "));
  console.print(gps.fresh(now)); console.print('/'); console.print(gps.fix.dimension);
  console.print('/'); console.println(gps.fix.heightValid);
  if (gps.fresh(now)) {
    console.print(F("GPS lat,lon,MSL m,speed m/s,course deg: "));
    console.print(gps.fix.latitude,7); console.print(',');
    console.print(gps.fix.longitude,7); console.print(','); console.print(gps.fix.height,2);
    console.print(','); console.print(gps.fix.speed,2); console.print(','); console.println(gps.fix.course,2);
  }
  if (haveImu) {
    console.print(F("Body acceleration g / gyro deg/s: "));
    for (unsigned i=0;i<3;++i) { console.print(acceleration[i]/Ins::Gravity,3); console.print(' '); }
    console.print(F(" / "));
    for (unsigned i=0;i<3;++i) { console.print(gyro[i]/Ins::Deg,3); console.print(' '); }
    console.println();
  }
  if (filter.healthy) {
    const Ins::Vec angles=filter.attitude.euler();
    console.print(F("Roll/pitch/yaw deg (yaw only valid if flagged): "));
    console.print(angles.x/Ins::Deg,2); console.print('/');
    console.print(angles.y/Ins::Deg,2); console.print('/'); console.println(angles.z/Ins::Deg,2);
    console.print(F("Yaw valid / global valid / rejected corrections: "));
    console.print(yawUsable()); console.print('/'); console.print(globalValid(now));
    console.print('/'); console.println(filter.rejected);
  }
  console.print(F("BMP fresh / hPa / chip C / relative down m: "));
  console.print(baroFresh(now)); console.print('/'); console.print(pressure/100,2);
  console.print('/'); console.print(baroTemperature,2); console.print('/'); console.println(baroDown,2);
}

bool integer(const char *text,long &value) {
  if (!text || !*text) return false;
  char *end=nullptr;
  value=strtol(text,&end,10);
  return end!=text && *end==0;
}

void command(char *line) {
  if (Slcan::isConnectionCommand(line,strlen(line))) {
    resumeAfterBridge=txEnabled;
    stopTX(); console.buffer.clear();
    savvyCAN=true; slcanBegin(); slcanInitialCommand(line,strlen(line));
    return;
  }
  for (char *p=line;*p;++p) *p=static_cast<char>(toupper(static_cast<unsigned char>(*p)));
  char *token[6]={};
  unsigned count=0;
  for (char *p=strtok(line," ");p;p=strtok(nullptr," ")) {
    if (count==6) { notice(F("ERR: Too many arguments")); return; }
    token[count++]=p;
  }
  if (count==0) return;
  const char *name=token[0];
  if (count==1 && strcmp(name,"HELP")==0) {
    console.println(F("INS vehicle test commands (115200 USB):"));
    console.println(F("STATUS | INS | GPS | IMU | BARO : cached sensor/filter report"));
    console.println(F("MOUNT <signed sensor axis for vehicle X> <Y> <Z>, e.g. MOUNT 1 2 3"));
    console.println(F("  Right-handed axes only; X forward/Y right/Z down. Changing mount clears calibration."));
    console.println(F("CAL GYRO : stationary 3s, any orientation; learns gyro bias, not accel/level offsets"));
    console.println(F("HEADING <0..359.99 deg> : explicit known heading; resets navigation origin"));
    console.println(F("COURSE AID ON|OFF : optional forward straight-line GPS heading initialization"));
    console.println(F("SAVE : persist mount/bias/config; pause TX first; stationary tilt init follows"));
    console.println(F("DEFAULTS : RAM defaults, clears calibration, does not erase stored config"));
    console.println(F("SENSOR INIT : explicit sensor/bus reinit; broadcast status continues"));
    console.println(F("TX OFF : pause automatic broadcast; TX ON : resume if CAN initialized"));
    console.println(F("CAN INIT | CANSPEED <125|250|500|1000> (kbps)"));
    console.println(F("GPS BAUD <4800|9600|19200|38400|57600|115200> (receiver unchanged)"));
    console.println(F("GPS RATE 5 : UBX 38400/5Hz via D1->GPS RX (3.3V level shift required); SAVE persists"));
    console.println(F("GPS RATE 1 : UBX revert receiver to defaults (9600/1Hz); D1 TX during command only"));
    console.println(F("FAULT CLEAR : TX OFF; clear historical flags, current faults remain"));
    console.println(F("SAVVYCAN or SLCAN commands: exclusive adapter mode; C then EXIT to return"));
    console.println(F("No SD/logging/RTC sync in this firmware. Isolated test bus only."));
    return;
  }
  if (count==1 && (strcmp(name,"STATUS")==0 || strcmp(name,"INS")==0 ||
      strcmp(name,"GPS")==0 || strcmp(name,"IMU")==0 || strcmp(name,"BARO")==0)) { status(); return; }
  if (count==1 && strcmp(name,"SAVE")==0) { saveConfig(); return; }
  if (count==1 && strcmp(name,"DEFAULTS")==0) {
    config=Ins::Config{}; resetNavigation(); ubxActive=false;
    restartGPSUart(gpsBaudInUse(),false);
    canReady=configureBridgeCAN(config.canBitrate,false);
    notice(F("OK: RAM defaults restored; calibration required; SAVE to persist"));
    return;
  }
  if (count==4 && strcmp(name,"MOUNT")==0) {
    Ins::Mount m;
    for (unsigned i=0;i<3;++i) {
      long value;
      if (!integer(token[i+1],value) || value<-3 || value>3 || value==0) {
        notice(F("ERR: Mount axes must be signed 1,2,3")); return;
      }
      m.axis[i]=static_cast<int8_t>(value);
    }
    if (!m.valid()) { notice(F("ERR: Mount must be unique, right-handed axes")); return; }
    config.mount=m; config.calibrated=false; config.gyroOffset=Ins::Vec{};
    resetNavigation();
    notice(F("OK: Mount set; calibration cleared; CAN broadcast continues; run CAL GYRO"));
    return;
  }
  if (count==2 && strcmp(name,"CAL")==0 && strcmp(token[1],"GYRO")==0) {
    if (!imuFresh(millis())) { notice(F("ERR: IMU unavailable; SENSOR INIT")); return; }
    resetNavigation(); config.calibrated=false;
    calibrating=true; initializing=false; calibrationStart=millis();
    notice(F("CAL: Hold stationary for 3s (any orientation). No previous calibration reused."));
    return;
  }
  if (count==2 && strcmp(name,"HEADING")==0) {
    char *end=nullptr;
    const float degrees=strtof(token[1],&end);
    if (*end || !isfinite(degrees) || degrees<0 || degrees>=360 || !filter.healthy) {
      notice(F("ERR: HEADING requires initialized tilt and degrees 0..<360")); return;
    }
    clearPending();
    if (!filter.setHeading(degrees*Ins::Deg)) { notice(F("ERR: Heading initialization failed")); return; }
    haveAided=false; baroAligned=false;
    notice(F("OK: Known heading set; origin cleared; CAN broadcast continues"));
    return;
  }
  if (count==3 && strcmp(name,"COURSE")==0 && strcmp(token[1],"AID")==0 &&
      (strcmp(token[2],"ON")==0 || strcmp(token[2],"OFF")==0)) {
    config.courseAid=strcmp(token[2],"ON")==0;
    notice(F("OK: Course aid changed. ON assumes forward, low-slip motion above 3m/s; not reverse."));
    return;
  }
  if (count==2 && strcmp(name,"TX")==0) {
    if (strcmp(token[1],"OFF")==0) {
      txManualOff=true; stopTX(); notice(F("OK: TX OFF (manual pause)")); return;
    }
    if (strcmp(token[1],"ON")==0) {
      if (!canReady) { notice(F("ERR: TX requires CAN INIT")); return; }
      txManualOff=false; startTX();
      notice(F("OK: CAN broadcast resumed; raw sensors publish before GPS/fusion validity."));
      return;
    }
  }
  if (count==2 && strcmp(name,"CAN")==0 && strcmp(token[1],"INIT")==0) {
    clearPending(); canReady=configureBridgeCAN(config.canBitrate,false);
    if (!canReady) stopTX();
    else startTX();
    notice(canReady?(txEnabled?F("OK: CAN initialized; broadcast resumed"):
                               F("OK: CAN initialized; TX remains OFF")):
                    F("ERR: CAN initialization failed")); return;
  }
  if (count==2 && strcmp(name,"CANSPEED")==0) {
    long speed;
    if (!integer(token[1],speed) || (speed!=125 && speed!=250 && speed!=500 && speed!=1000)) {
      notice(F("ERR: CANSPEED 125|250|500|1000")); return;
    }
    clearPending(); config.canBitrate=static_cast<uint32_t>(speed)*1000;
    canReady=configureBridgeCAN(config.canBitrate,false);
    if (!canReady) stopTX();
    else startTX();
    notice(canReady?(txEnabled?F("OK: CAN bitrate set; broadcast continues"):
                               F("OK: CAN bitrate set; TX remains OFF")):
                    F("ERR: CAN initialization failed")); return;
  }
  if (count==3 && strcmp(name,"GPS")==0 && strcmp(token[1],"RATE")==0 &&
      (strcmp(token[2],"5")==0 || strcmp(token[2],"1")==0)) {
    if (ubxActive) { notice(F("ERR: GPS configuration already in progress")); return; }
    config.gpsFast=strcmp(token[2],"5")==0;
    gps=Ins::GPS{}; lastGPSSequence=0; gpsEpochMs=0; previousFixReceived=0;
    previousSpeedValid=gpsAccelValid=false;
    startGPSConfig(config.gpsFast);
    notice(F("OK: Sending UBX on D1 (non-blocking). SAVE to persist; boot re-sends when 5Hz."));
    return;
  }
  if (count==3 && strcmp(name,"GPS")==0 && strcmp(token[1],"BAUD")==0) {
    long baud;
    if (ubxActive) { notice(F("ERR: GPS configuration in progress")); return; }
    if (!integer(token[2],baud) || baud<0 || baud>115200) { notice(F("ERR: Invalid baud")); return; }
    Ins::Config candidate=config; candidate.gpsBaud=static_cast<uint32_t>(baud);
    if (!Ins::validConfig(candidate)) { notice(F("ERR: Unsupported GPS baud")); return; }
    config=candidate; resetNavigation(); gps=Ins::GPS{}; lastGPSSequence=0;
    restartGPSUart(gpsBaudInUse(),false);
    notice(config.gpsFast?F("OK: Initial GPS baud changed; 5Hz mode still uses 38400"):
                          F("OK: Device GPS baud changed; receiver unchanged; CAN continues")); return;
  }
  if (count==2 && strcmp(name,"SENSOR")==0 && strcmp(token[1],"INIT")==0) { initSensors(); return; }
  if (count==2 && strcmp(name,"FAULT")==0 && strcmp(token[1],"CLEAR")==0) {
    if (txEnabled) { notice(F("ERR: TX OFF before clearing flags")); return; }
    stickyFaults=0; notice(F("OK: Historical faults cleared; current faults still reported")); return;
  }
  if (count==1 && strcmp(name,"SAVVYCAN")==0) {
    resumeAfterBridge=txEnabled;
    stopTX(); console.buffer.clear(); savvyCAN=true; slcanBegin(); return;
  }
  notice(F("ERR: Unknown command/arguments; HELP"));
}

void serviceCLI() {
  static char line[128]={};
  static unsigned length=0;
  static bool overflow=false;
  if (!usbSerialConnected()) { length=0; overflow=false; console.buffer.clear(); return; }
  console.drain();
  if (!console.buffer.empty()) return;
  for (unsigned i=0;i<64 && Serial.available();++i) {
    const int c=Serial.read();
    if (c=='\r' || c=='\n') {
      if (overflow) notice(F("ERR: Command too long"));
      else if (length) { line[length]=0; command(line); }
      length=0; overflow=false; return;
    }
    if ((c==8 || c==127) && length) --length;
    else if (c>=32 && c<127) {
      if (length<sizeof(line)-1) line[length++]=static_cast<char>(c);
      else overflow=true;
    }
  }
}

}

bool configureBridgeCAN(uint32_t bitrate,bool listenOnly) {
  if (listenOnly) return false;
  CAN.end(); CAN.clearError();
  canReady=CAN.begin(bitrate);
  if (!canReady) stickyFaults|=CanFault;
  return canReady;
}
void closeBridgeCAN() { CAN.end(); canReady=false; stopTX(); }
bool sendBridgeCAN(uint32_t id,bool extended,uint8_t length,const uint8_t *data) {
  if (!canReady || length>8) return false;
  const CanMsg message(extended?CanExtendedId(id):CanStandardId(id),length,data);
  return CAN.write(message)>0;
}

void setup() {
  Serial.begin(115200);
  pinMode(3,OUTPUT); digitalWrite(3,LOW); // Termination defaults OFF.
  loadConfig();
  Serial1.begin(config.gpsBaud,SERIAL_8N1);
  pinMode(1,INPUT);
  canReady=configureBridgeCAN(config.canBitrate,false);
  initSensors();
  txEnabled=canReady;
  notice(txEnabled?F("INS vehicle-test firmware. CAN broadcast started at configured bitrate. TX OFF to pause."):
                   F("INS vehicle-test firmware. CAN init failed; HELP."));
  if (config.gpsFast) startGPSConfig(true);
}

void loop() {
  const uint32_t started=micros();
  serviceGPS(); serviceIMU(); serviceGPS();
  serviceBaro(); serviceGPS();
  serviceCAN();
  if (savvyCAN) {
    slcanProcess();
    if (!savvyCAN) {
      stopTX(); console.buffer.clear();
      canReady=configureBridgeCAN(config.canBitrate,false);
      txEnabled=resumeAfterBridge && canReady && !txManualOff;
      resumeAfterBridge=false;
      if (txEnabled) lastPublish=millis();
      notice(txEnabled?F("Returned from adapter; CAN broadcast resumed"):
                       F("Returned from adapter; TX OFF"));
    }
  } else serviceCLI();
  const uint32_t now=millis();
  if (filter.healthy && !imuFresh(now)) {
    stickyFaults|=ImuFault;
    filter.healthy=false;
    notice(F("ERR: IMU stale; fused navigation invalid; CAN reports sensor status"));
  }
  if (filter.healthy && now-lastHealth>=1000) {
    lastHealth=now;
    if (!filter.positiveDefinite()) {
      stickyFaults|=FilterFault;
      notice(F("ERR: Covariance lost positive definiteness; fused navigation invalid; raw CAN continues"));
    }
  }
  if (filter.healthy && haveAided && now-lastAided>10000) {
    filter.healthy=false; stickyFaults|=FilterFault;
    notice(F("ERR: GPS aiding lost >10s; fused navigation invalid; raw CAN continues"));
  }
  if (config.calibrated && !filter.healthy && imuFresh(now) && !calibrating && !initializing &&
      !savvyCAN && now-lastReinit>=1000) {
    lastReinit=now; ++reinits; beginInitialization(now);
    notice(F("Fused navigation invalid: automatic tilt re-init when stationary (heading/origin re-acquired)"));
  }
  if (!imuReady && !savvyCAN && !calibrating && now-lastSensorRetry>=5000) {
    ++sensorRetries; quietInit=true; initSensors(); quietInit=false;
    if (imuReady) notice(F("OK: IMU recovered automatically; stationary tilt initialization armed"));
  }
  serviceGPSConfig(now);
  publish(now); serviceCAN();
  const uint32_t duration=micros()-started;
  if (duration>maxLoopUs) maxLoopUs=duration;
}
