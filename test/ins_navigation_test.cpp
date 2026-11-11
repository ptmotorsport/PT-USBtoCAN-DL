#define INS_TESTING
#include "ins_filter.h"
#include "ins_gps.h"
#include <assert.h>
#include <stdio.h>
#include <string>

namespace {
std::string sentence(const std::string &text) {
  unsigned checksum=0;
  for (unsigned char c:text) checksum^=c;
  char suffix[8];
  snprintf(suffix,sizeof(suffix),"*%02X\r\n",checksum);
  return "$"+text+suffix;
}
void feed(Ins::GPS &gps,const std::string &text,uint32_t now) {
  for (unsigned char c:sentence(text)) gps.feed(c,now);
}
void testGPS() {
  Ins::GPS gps;
  feed(gps,"GPGSV,1,1,12",50);
  assert(gps.satellites(100)==12 && gps.satellites(5100)==0);
  feed(gps,"GPGSA,A,3,,,,,,,,,,,,,1.5,0.9,1.2",100);
  feed(gps,"GPRMC,120000.00,A,3300.00000,S,15100.00000,E,10.0,90.0,071126,,,A",100);
  assert(!gps.fix.valid);
  feed(gps,"GPGGA,120000.00,3300.00000,S,15100.00000,E,1,08,0.9,25.0,M,0,M,,",200);
  assert(gps.fix.valid && gps.fix.heightValid && gps.fix.dimension==3);
  assert(gps.fix.latitude==-33 && gps.fix.longitude==151);
  assert(fabsf(gps.fix.speed-5.1444444f)<1e-5f && gps.fix.course==90);
  assert(gps.fix.sequence==1 && gps.fresh(2100) && !gps.fresh(2200));
  feed(gps,"GPGGA,120000.00,3300.00000,S,15100.00000,E,1,08,0.9,25.0,M,0,M,,",210);
  assert(gps.fix.sequence==1);
  feed(gps,"GPRMC,120001.00,V,,,,,,,071126,,,N",1000);
  assert(!gps.fix.valid);
  feed(gps,"GPRMC,120002.00,A,3300.00001,S,15100.00001,E,10,90,071126,,,A",2000);
  assert(!gps.fix.valid);
  feed(gps,"GPGGA,120002.00,3300.00001,S,15100.00001,E,1,08,0.9,25.0,M,0,M,,",2001);
  assert(gps.fix.valid && gps.fix.latitude<-33 && gps.fix.longitude>151);
  feed(gps,"GPGGA,120003.00,,,,,0,00,99,,,,,,,",3000);
  assert(!gps.fix.valid);
  const uint32_t errors=gps.errors;
  feed(gps,"GPRMC,999999.0,A,3300.0,S,15100.0,E,10,90,071126,,,A",3100);
  assert(gps.errors>errors);
  for (unsigned char c:std::string("$GPGGA,*00\r\n")) gps.feed(c,3200);
  assert(gps.errors>errors+1);
  feed(gps,"GPGSA,A,1,,,,,,,,,,,,,1.5,0.9,1.2",3300);
  feed(gps,"GPRMC,120004.00,A,3300.00000,S,15100.00000,E,10,90,071126,,,A",3400);
  feed(gps,"GPGGA,120004.00,3300.00000,S,15100.00000,E,1,08,0.9,25.0,M,0,M,,",3401);
  assert(!gps.fix.valid);
}
void testMountCalibration() {
  Ins::Mount m;
  assert(m.valid());
  m.axis[0]=-1; assert(!m.valid());
  m.axis[1]=-2; assert(m.valid());
  const Ins::Vec v=m.apply({1,2,3});
  assert(v.x==-1 && v.y==-2 && v.z==3);
  m.axis[0]=0; assert(!m.valid());
  Ins::StationaryCalibration c;
  for (unsigned i=0;i<300;++i) assert(c.addSample({0,0,Ins::Gravity},{0.01f,-0.02f,0.03f}));
  assert(c.ready() && fabsf(c.meanGyro.x-0.01f)<1e-6f);
  c.reset();
  for (unsigned i=0;i<300;++i)
    assert(c.addSample({0,0,-Ins::Gravity},{i%2?0.03f:-0.03f,0,0}));
  assert(!c.ready());
  assert(!c.addSample({0,0,0},{0,0,0}));
  // Session 4 idle: ~2 deg/s gyro and ~0.06 g accel per-axis noise.
  Ins::StationaryCalibration strict,vibration;
  bool strictAccepted=true;
  for (unsigned i=0;i<300;++i) {
    const float s=i%2?1.0f:-1.0f;
    const Ins::Vec a={s*0.55f,-s*0.5f,-Ins::Gravity+s*0.5f},w={s*2*Ins::Deg,-s*1.5f*Ins::Deg,s*Ins::Deg};
    strictAccepted=strict.addSample(a,w) && strictAccepted;
    assert(vibration.addSample(a,w,Ins::VibrationStill));
  }
  assert(vibration.ready(Ins::VibrationStill) && !vibration.ready());
  assert(!strictAccepted || !strict.ready());
  assert(!vibration.addSample({0,0,-Ins::Gravity+3.5f},{0,0,0},Ins::VibrationStill));
}
void testFilter() {
  static Ins::Filter filter;
  assert(filter.initialize({0,0,-Ins::Gravity},{0,0,0}));
  filter.setOrigin(-33,151,25);
  for (unsigned i=0;i<1000;++i) {
    assert(filter.predict({0,0,-Ins::Gravity},{0,0,0},0.01f));
    if (i%100==0) assert(filter.positiveDefinite());
  }
  assert(Ins::norm(filter.position)<0.001f);
  assert(Ins::norm(filter.velocity)<0.001f);
  assert(!filter.yawValid && filter.originSet);
  assert(filter.initialize({0,0,Ins::Gravity},{0,0,0})); // Upside-down bench.
  filter.setOrigin(-33,151,25);
  for (unsigned i=0;i<100;++i) assert(filter.predict({0,0,Ins::Gravity},{0,0,0},0.01f));
  assert(Ins::norm(filter.position)<0.01f);
  assert(fabsf(fabsf(filter.attitude.euler().x)-Ins::Pi)<0.001f);
  const Ins::Vec tilted={3,4,-sqrtf(Ins::Gravity*Ins::Gravity-25)};
  assert(filter.initialize(tilted,{0,0,0}));
  filter.setOrigin(-33,151,25);
  for (unsigned i=0;i<100;++i) assert(filter.predict(tilted,{0,0,0},0.01f));
  assert(Ins::norm(filter.position)<0.001f);
  assert(filter.setHeading(90*Ins::Deg));
  assert(fabsf(Ins::wrap(filter.attitude.euler().z-90*Ins::Deg))<0.001f);
  assert(filter.initialize({0,0,-Ins::Gravity},{0,0,0}));
  assert(filter.setHeading(0));
  filter.setOrigin(-33,151,25);
  const Ins::Vec local=filter.local(-33+1e-7,151+1e-7,26);
  assert(local.x>0.01f && local.x<0.012f);
  assert(local.y>0.009f && local.y<0.01f && local.z==-1);
  for (unsigned i=0;i<1000;++i) {
    assert(filter.predict({0.05f,0,-Ins::Gravity},{0,0,0},0.01f));
    if (i%100==0) {
      assert(filter.correctPosition({0,0,0},4,9));
      assert(filter.correctVelocity({0,0,0},0.25f));
      assert(filter.correctHeight(0,4));
      assert(filter.positiveDefinite());
    }
  }
  assert(filter.healthy && filter.covarianceValid());
  assert(!filter.correctPosition({100000,0,0},4,9));
  assert(filter.rejected==1);
  assert(!filter.predict({0,0,-Ins::Gravity},{0,0,0},0.1f));
  assert(!filter.healthy);
  assert(filter.initialize({0,0,-Ins::Gravity},{0,0,0}));
  for (unsigned i=0;i<100;++i)
    assert(filter.predict({0,0,-Ins::Gravity},{0,0,90*Ins::Deg},0.01f));
  assert(fabsf(filter.attitude.euler().z-90*Ins::Deg)<0.001f);
}
void testWaitingForOrigin() {
  static Ins::Filter filter;
  const Ins::Vec acceleration={1,2,-9.8f};
  assert(filter.initialize(acceleration,{0,0,0}));
  for (unsigned step=0;step<12000;++step) {
    if (!filter.predict(acceleration,{0,0,0},0.01f)) {
      printf("Pre-origin failure at %.2fs; position variance %g/%g/%g\n",
        (step+1)*0.01f,filter.variance(0),filter.variance(1),filter.variance(2));
      fflush(stdout);
      assert(false);
    }
    if (step%100==99) assert(filter.positiveDefinite());
  }
  assert(!filter.originSet && !filter.yawValid && filter.healthy);
  assert(Ins::norm(filter.position)==0 && Ins::norm(filter.velocity)==0);
}
void testJacobianAndLongRun() {
  static Ins::Filter reference,perturbed;
  const Ins::Vec a={0.3f,-0.5f,-Ins::Gravity};
  assert(reference.initialize({1,2,-9.5f},{0,0,0}));
  reference.setOrigin(-33,151,25);
  const Ins::Quaternion q=reference.attitude;
  assert(reference.predict(a,{0,0,0},0.01f));
  for (unsigned col=0;col<6;++col) {
    assert(perturbed.initialize({1,2,-9.5f},{0,0,0}));
    perturbed.setOrigin(-33,151,25);
    perturbed.attitude=q;
    const float epsilon=0.001f;
    if (col<3) {
      Ins::Vec theta={}; theta[col]=epsilon;
      perturbed.attitude=perturbed.attitude.multiply(Ins::Quaternion::rotation(theta));
    } else perturbed.accelBias[col-3]=epsilon;
    assert(perturbed.predict(a,{0,0,0},0.01f));
    for (unsigned row=0;row<3;++row) {
      const float numerical=(perturbed.velocity[row]-reference.velocity[row])/epsilon;
      assert(fabsf(numerical-reference.transition(row+3,col+6))<0.0002f);
    }
  }
  assert(reference.initialize({0,0,-Ins::Gravity},{0,0,0}));
  assert(reference.setHeading(0));
  reference.setOrigin(-33,151,25);
  const Ins::Vec initialVelocity={5,0,0};
  reference.velocity=initialVelocity;
  for (unsigned step=0;step<12000;++step) {
    assert(reference.predict({0,0,-Ins::Gravity},{0,0,0},0.01f));
    if (step%100==99) {
      const float elapsed=(step+1)*0.01f;
      assert(reference.correctPosition({5*elapsed,0,0},9,36));
      assert(reference.correctVelocity(initialVelocity,1));
      assert(reference.correctHeight(0,9));
      assert(reference.positiveDefinite());
    }
  }
  assert(fabsf(reference.position.x-600)<0.1f && fabsf(reference.position.y)<0.01f);
  assert(fabsf(reference.velocity.x-5)<0.01f);
}
void testGravityAiding() {
  static Ins::Filter filter;
  const Ins::Vec level={0,0,-Ins::Gravity};
  assert(filter.initialize(level,{0,0,0}));
  assert(filter.setHeading(30*Ins::Deg));
  // Tilt error of 5 deg roll (e.g. gyro cross-axis error accumulated in turns).
  filter.attitude=filter.attitude.multiply(Ins::Quaternion::rotation({5*Ins::Deg,0,0}));
  for (unsigned i=0;i<3000;++i) {
    assert(filter.predict(level,{0,0,0},0.01f));
    if (i%10==9) assert(filter.correctGravity(level,{0,0,0},0.25f));
  }
  const Ins::Vec e=filter.attitude.euler();
  assert(fabsf(e.x)<0.3f*Ins::Deg && fabsf(e.y)<0.3f*Ins::Deg);
  assert(fabsf(Ins::wrap(e.z-30*Ins::Deg))<0.5f*Ins::Deg && filter.yawValid);
  assert(filter.positiveDefinite() && filter.healthy);
  // Steady right turn: lateral specific force is explained by the kinematic term.
  const Ins::Vec turning={0,3,-Ins::Gravity};
  for (unsigned i=0;i<100;++i) assert(filter.correctGravity(turning,{0,3,0},1));
  assert(fabsf(filter.attitude.euler().x)<0.3f*Ins::Deg);
  // Unexplained 1g lateral force is rejected instead of tilting the estimate.
  assert(!filter.correctGravity({0,Ins::Gravity,-Ins::Gravity},{0,0,0},0.25f));
  assert(filter.healthy);
  const float before=filter.variance(6);
  assert(filter.addUncertainty(0.001f,0.1f) && fabsf(filter.variance(6)-before-0.001f)<1e-6f);
  // Rotation grows attitude uncertainty faster than rest.
  static Ins::Filter still,spinning;
  assert(still.initialize(level,{0,0,0}) && spinning.initialize(level,{0,0,0}));
  for (unsigned i=0;i<100;++i) {
    assert(still.predict(level,{0,0,0},0.01f));
    assert(spinning.predict(level,{0,0,1},0.01f));
  }
  assert(spinning.variance(6)>still.variance(6)+0.005f);
}
}

int main() {
  testGPS(); testMountCalibration(); testFilter(); testWaitingForOrigin(); testJacobianAndLongRun();
  testGravityAiding();
  puts("ins_navigation_test: PASS");
}
