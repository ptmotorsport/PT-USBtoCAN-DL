#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

namespace Ins {

constexpr float Gravity = 9.80665f;
constexpr float Pi = 3.14159265358979323846f;
constexpr float Deg = Pi / 180.0f;
constexpr unsigned States = 15;

struct Vec {
  float x, y, z;
  float &operator[](unsigned i) { return i == 0 ? x : i == 1 ? y : z; }
  float operator[](unsigned i) const { return i == 0 ? x : i == 1 ? y : z; }
};
inline Vec add(Vec a, Vec b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec sub(Vec a, Vec b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Vec scale(Vec a, float s) { return {a.x*s,a.y*s,a.z*s}; }
inline float dot(Vec a, Vec b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec cross(Vec a, Vec b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
inline float norm(Vec a) { return sqrtf(dot(a,a)); }
inline bool finite(Vec a) { return isfinite(a.x) && isfinite(a.y) && isfinite(a.z); }
inline float wrap(float a) { return atan2f(sinf(a),cosf(a)); }

struct Quaternion {
  float w=1, x=0, y=0, z=0;
  bool normalize() {
    const float n=sqrtf(w*w+x*x+y*y+z*z);
    if (!isfinite(n) || n < 0.1f) return false;
    w/=n; x/=n; y/=n; z/=n;
    return true;
  }
  Quaternion multiply(const Quaternion &b) const {
    Quaternion r;
    r.w=w*b.w-x*b.x-y*b.y-z*b.z;
    r.x=w*b.x+x*b.w+y*b.z-z*b.y;
    r.y=w*b.y-x*b.z+y*b.w+z*b.x;
    r.z=w*b.z+x*b.y-y*b.x+z*b.w;
    return r;
  }
  static Quaternion rotation(Vec angle) {
    Quaternion r;
    const float length=norm(angle);
    const float s=length < 1e-5f ? 0.5f : sinf(length*0.5f)/length;
    r.w=cosf(length*0.5f); r.x=angle.x*s; r.y=angle.y*s; r.z=angle.z*s;
    return r;
  }
  Vec rotate(Vec v) const {
    const Vec u={x,y,z};
    return add(v,scale(cross(u,add(cross(u,v),scale(v,w))),2));
  }
  Vec inverseRotate(Vec v) const {
    Quaternion c=*this; c.x=-x; c.y=-y; c.z=-z;
    return c.rotate(v);
  }
  Vec euler() const {
    const float s=2*(w*y-z*x);
    return {atan2f(2*(w*x+y*z),1-2*(x*x+y*y)),
            asinf(fmaxf(-1,fminf(1,s))),atan2f(2*(w*z+x*y),1-2*(y*y+z*z))};
  }
};

struct Mount {
  // Each vehicle axis names one signed sensor axis, e.g. +1,-2,-3.
  int8_t axis[3]={1,2,3};
  bool valid() const {
    int8_t a[3];
    for (unsigned i=0;i<3;++i) {
      a[i]=axis[i]<0 ? -axis[i] : axis[i];
      if (a[i]<1 || a[i]>3) return false;
    }
    if (a[0]==a[1] || a[0]==a[2] || a[1]==a[2]) return false;
    const Vec x=apply({1,0,0}), y=apply({0,1,0}), z=apply({0,0,1});
    return dot(cross(x,y),z)>0.5f;
  }
  Vec apply(Vec v) const {
    Vec out={};
    for (unsigned i=0;i<3;++i) {
      const int a=axis[i];
      out[i]=(a<0 ? -1.0f : 1.0f)*v[static_cast<unsigned>((a<0?-a:a)-1)];
    }
    return out;
  }
};

// Strict limits estimate gyro bias (engine off). Vibration limits only
// initialize tilt with a known bias: idling engines measured ~2 deg/s and
// ~0.06 g per-axis noise, so motion is instead judged by the batch mean.
struct StillLimits { float sampleAccel,accelVar,gyroVar; };
constexpr StillLimits StrictStill={1.0f,0.04f,0.0001f};
constexpr StillLimits VibrationStill={3.0f,1.0f,0.0027f};

class StationaryCalibration {
public:
 void reset() { *this=StationaryCalibration{}; }
 bool addSample(Vec acceleration, Vec gyro, const StillLimits &limits=StrictStill) {
   if (!finite(acceleration) || !finite(gyro) ||
       fabsf(norm(acceleration)-Gravity)>limits.sampleAccel || norm(gyro)>0.35f) return false;
    ++count;
    for (unsigned i=0;i<3;++i) {
      const float da=acceleration[i]-meanAccel[i], dw=gyro[i]-meanGyro[i];
      meanAccel[i]+=da/count; meanGyro[i]+=dw/count;
      m2Accel[i]+=da*(acceleration[i]-meanAccel[i]);
      m2Gyro[i]+=dw*(gyro[i]-meanGyro[i]);
    }
    return true;
  }
  bool ready(const StillLimits &limits=StrictStill) const {
    if (count<300 || fabsf(norm(meanAccel)-Gravity)>0.5f) return false;
    for (unsigned i=0;i<3;++i) {
      if (m2Accel[i]/(count-1)>limits.accelVar || m2Gyro[i]/(count-1)>limits.gyroVar) return false;
    }
    return true;
  }
  uint32_t count=0;
  Vec meanAccel={},meanGyro={};
private:
  Vec m2Accel={},m2Gyro={};
};

class Filter {
public:
  Vec position={}, velocity={}, accelBias={},gyroBias={};
  Quaternion attitude;
  bool initialized=false, healthy=false, yawValid=false, originSet=false;
  uint32_t rejected=0;

  bool initialize(Vec acceleration, Vec gyroOffset) {
    initialized=healthy=yawValid=originSet=false;
    rejected=0; position=velocity=accelBias=gyroBias=Vec{};
    attitude=Quaternion{};
    memset(p_,0,sizeof(p_));
    const float g=norm(acceleration);
    if (!finite(acceleration) || !finite(gyroOffset) || g<8 || g>12) return false;
    const Vec from=scale(acceleration,1/g), to={0,0,-1};
    const Vec c=cross(from,to);
    attitude.w=1+dot(from,to); attitude.x=c.x; attitude.y=c.y; attitude.z=c.z;
    if (attitude.w<1e-5f) { attitude.w=0; attitude.x=1; attitude.y=0; attitude.z=0; }
    if (!attitude.normalize()) return false;
    gyroBias=gyroOffset;
    for (unsigned i=0;i<States;++i) {
      p_[i][i]=i<3?100.0f:i<6?25.0f:i<9?(i==8?Pi*Pi:0.03f):i<12?0.25f:0.00001f;
    }
    initialized=healthy=true;
    return true;
  }

  bool predict(Vec acceleration, Vec gyro, float dt) {
    if (!healthy || !finite(acceleration) || !finite(gyro) ||
        !isfinite(dt) || dt<0.002f || dt>0.04f) return fail();
    const Vec f=sub(acceleration,accelBias), w=sub(gyro,gyroBias);
    const Quaternion half=attitude.multiply(Quaternion::rotation(scale(w,dt*0.5f)));
    const Vec a=add(half.rotate(f),{0,0,Gravity});
    position=add(position,add(scale(velocity,dt),scale(a,0.5f*dt*dt)));
    velocity=add(velocity,scale(a,dt));
    attitude=attitude.multiply(Quaternion::rotation(scale(w,dt)));
    if (!attitude.normalize()) return fail();

    memset(phi_,0,sizeof(phi_));
    for (unsigned i=0;i<States;++i) phi_[i][i]=1;
    const Vec basis[3]={{1,0,0},{0,1,0},{0,0,1}};
    for (unsigned col=0;col<3;++col) {
      const Vec vf=half.rotate(cross(basis[col],f));
      const Vec bias=half.rotate(basis[col]);
      const Vec theta=cross(basis[col],w);
      phi_[col][col+3]=dt;
      phi_[col+6][col+12]=-dt;
      for (unsigned row=0;row<3;++row) {
        phi_[row+3][col+6]=vf[row]*dt;
        phi_[row][col+6]=vf[row]*0.5f*dt*dt;
        phi_[row+3][col+9]=-bias[row]*dt;
        phi_[row][col+9]=-bias[row]*0.5f*dt*dt;
        phi_[row+6][col+6]+=theta[row]*dt;
      }
    }
    // Rate-scaled attitude noise covers gyro scale/cross-axis errors in turns.
    attitudeNoise_=0.0004f+0.01f*dot(w,w);
    propagate(dt);
    if (!originSet) {
      // Navigation starts at the first accepted origin, not during GPS wait.
      position=velocity=Vec{};
      resetMotionCovariance();
    }
    return check();
  }

  bool setHeading(float radians) {
    if (!healthy || !isfinite(radians)) return false;
    Quaternion turn=Quaternion::rotation({0,0,wrap(radians-attitude.euler().z)});
    attitude=turn.multiply(attitude);
    if (!attitude.normalize()) return fail();
    yawValid=true;
    // Heading initialization changes the world frame; callers must reset origin.
    position=velocity=Vec{};
    originSet=false;
    resetMotionCovariance();
    for (unsigned i=0;i<States;++i) {
      p_[8][i]=p_[i][8]=0;
    }
    p_[8][8]=0.04f;
    return true;
  }

  void setOrigin(double latitude, double longitude, float height) {
    latitude_=latitude; longitude_=longitude; height_=height;
    constexpr double radians=3.14159265358979323846/180.0;
    const double phi=latitude*radians,s=sin(phi);
    const double d=1-0.00669437999014*s*s;
    northScale_=6378137.0*(1-0.00669437999014)/(d*sqrt(d))*radians;
    eastScale_=6378137.0/sqrt(d)*cos(phi)*radians;
    position=Vec{};
    resetMotionCovariance();
    originSet=true;
  }
  Vec local(double latitude, double longitude, float height) const {
    return {static_cast<float>((latitude-latitude_)*northScale_),
      static_cast<float>((longitude-longitude_)*eastScale_),
      height_-height};
  }
  double latitude() const { return latitude_+position.x/northScale_; }
  double longitude() const { return longitude_+position.y/eastScale_; }
  float height() const { return height_-position.z; }
  float variance(unsigned index) const { return p_[index][index]; }
#if defined(INS_TESTING)
  float transition(unsigned row,unsigned col) const { return phi_[row][col]; }
#endif

  bool correctPosition(Vec value, float horizontalVariance, float verticalVariance) {
    if (!originSet || !finite(value) || !isfinite(horizontalVariance) ||
        !isfinite(verticalVariance) || horizontalVariance<=0 || verticalVariance<=0) return false;
    if (!gate(0,value.x,horizontalVariance) || !gate(1,value.y,horizontalVariance) ||
        !gate(2,value.z,verticalVariance)) { ++rejected; return false; }
    return observe(0,value.x,horizontalVariance) && observe(1,value.y,horizontalVariance) &&
        observe(2,value.z,verticalVariance);
  }
  bool correctVelocity(Vec value, float horizontalVariance) {
    if (!originSet || !finite(value) || !isfinite(horizontalVariance) || horizontalVariance<=0) return false;
    if (!gate(3,value.x,horizontalVariance) || !gate(4,value.y,horizontalVariance)) {
      ++rejected; return false;
    }
    return observe(3,value.x,horizontalVariance) && observe(4,value.y,horizontalVariance);
  }
  bool correctHeight(float down, float variance) {
    if (!originSet || !isfinite(down) || !isfinite(variance) || variance<=0) return false;
    if (!gate(2,down,variance)) { ++rejected; return false; }
    return observe(2,down,variance);
  }
  // Accelerometer gravity aiding for roll/pitch. kinematic is the expected
  // non-gravity specific force in the body frame (e.g. v*yawRate laterally).
  bool correctGravity(Vec measured, Vec kinematic, float noise) {
    if (!healthy || !finite(measured) || !finite(kinematic) || !isfinite(noise) || noise<=0)
      return false;
    float h[3][States]={},innovation[3];
    if (!gravityRows(measured,kinematic,h,innovation)) return false;
    for (unsigned i=0;i<3;++i) {
      float s=noise;
      for (unsigned j=6;j<9;++j)
        for (unsigned k=6;k<9;++k) s+=h[i][j]*p_[j][k]*h[i][k];
      if (innovation[i]*innovation[i]>9*s) return false;
    }
    for (unsigned i=0;i<3;++i) {
      // Re-linearize each axis about the state updated by the previous axis.
      if (!gravityRows(measured,kinematic,h,innovation) || !observe(h[i],innovation[i],noise))
        return false;
    }
    return true;
  }
  // Inflate attitude/velocity uncertainty after an unobservable (clipped) sample.
  bool addUncertainty(float attitudeVariance,float velocityVariance) {
    if (!healthy || !isfinite(attitudeVariance) || !isfinite(velocityVariance) ||
        attitudeVariance<0 || velocityVariance<0) return false;
    for (unsigned i=0;i<3;++i) {
      p_[6+i][6+i]+=attitudeVariance;
      if (originSet) p_[3+i][3+i]+=velocityVariance;
    }
    return check();
  }
  bool covarianceValid() const {
    for (unsigned i=0;i<States;++i) {
      if (!isfinite(p_[i][i]) || p_[i][i]<=0 || p_[i][i]>1e8f) return false;
      for (unsigned j=0;j<States;++j)
        if (!isfinite(p_[i][j]) || fabsf(p_[i][j]-p_[j][i])>0.002f) return false;
    }
    return true;
  }
  bool positiveDefinite() {
    memset(scratch_,0,sizeof(scratch_));
    for (unsigned i=0;i<States;++i) {
      for (unsigned j=0;j<=i;++j) {
        float s=p_[i][j];
        for (unsigned k=0;k<j;++k) s-=scratch_[i][k]*scratch_[j][k];
        if (i==j) {
          if (!isfinite(s) || s<=0) return fail();
          scratch_[i][j]=sqrtf(s);
        } else scratch_[i][j]=s/scratch_[j][j];
      }
    }
    return true;
  }

private:
  float p_[States][States]={},phi_[States][States]={},scratch_[States][States]={};
  double latitude_=0,longitude_=0;
  double northScale_=1,eastScale_=1;
  float height_=0;
  float attitudeNoise_=0.0004f;
  void resetMotionCovariance() {
    for (unsigned i=0;i<6;++i) {
      for (unsigned j=0;j<States;++j) p_[i][j]=p_[j][i]=0;
      p_[i][i]=i<3?25.0f:4.0f;
    }
  }
  bool fail() { healthy=false; return false; }
  bool check() {
    if (!finite(position) || !finite(velocity) || !finite(accelBias) ||
        !finite(gyroBias) || norm(accelBias)>3 || norm(gyroBias)>0.5f ||
        !covarianceValid()) return fail();
    return true;
  }
  void propagate(float dt) {
    memset(scratch_,0,sizeof(scratch_));
    for (unsigned i=0;i<States;++i)
      for (unsigned k=0;k<States;++k) {
        const float v=phi_[i][k];
        if (v!=0) for (unsigned j=0;j<States;++j) scratch_[i][j]+=v*p_[k][j];
      }
    for (unsigned i=0;i<States;++i)
      for (unsigned j=i;j<States;++j) {
        float value=0;
        for (unsigned k=0;k<States;++k) value+=scratch_[i][k]*phi_[j][k];
        if (i==j) value+=(i<3?0.0001f:i<6?0.16f:i<9?attitudeNoise_:i<12?0.0001f:0.000001f)*dt;
        p_[i][j]=p_[j][i]=value;
      }
  }
  float component(unsigned i) const { return i<3?position[i]:velocity[i-3]; }
  bool gate(unsigned index,float value,float noise) const {
    if (!healthy) return false;
    const float error=value-component(index);
    return error*error<=25*(p_[index][index]+noise);
  }
  bool gravityRows(Vec measured,Vec kinematic,float (&h)[3][States],float (&innovation)[3]) const {
    const Vec u=attitude.inverseRotate({0,0,-Gravity});
    const Vec predicted=add(add(u,kinematic),accelBias);
    const Vec basis[3]={{1,0,0},{0,1,0},{0,0,1}};
    for (unsigned j=0;j<3;++j) {
      const Vec column=cross(u,basis[j]);
      for (unsigned i=0;i<3;++i) h[i][6+j]=column[i];
    }
    for (unsigned i=0;i<3;++i) innovation[i]=measured[i]-predicted[i];
    return finite(predicted);
  }
  bool observe(unsigned index,float value,float noise) {
    float h[States]={};
    h[index]=1;
    return observe(h,value-component(index),noise);
  }
  bool observe(const float (&h)[States],float innovation,float noise) {
    float row[States],gain[States],delta[States];
    float s=noise;
    for (unsigned i=0;i<States;++i) {
      row[i]=0;
      for (unsigned k=0;k<States;++k) if (h[k]!=0) row[i]+=p_[i][k]*h[k];
      s+=h[i]*row[i];
    }
    if (!isfinite(s) || s<=0 || !isfinite(innovation)) return fail();
    for (unsigned i=0;i<States;++i) { gain[i]=row[i]/s; delta[i]=gain[i]*innovation; }
    // Joseph update for one scalar row h: P-K(Ph)'-(Ph)K'+K*s*K'.
    for (unsigned i=0;i<States;++i)
      for (unsigned j=i;j<States;++j) {
        const float v=p_[i][j]-gain[i]*row[j]-row[i]*gain[j]+gain[i]*s*gain[j];
        p_[i][j]=p_[j][i]=v;
      }
    for (unsigned i=0;i<3;++i) {
      position[i]+=delta[i]; velocity[i]+=delta[i+3];
      accelBias[i]+=delta[i+9]; gyroBias[i]+=delta[i+12];
    }
    const Vec angle={delta[6],delta[7],delta[8]};
    attitude=attitude.multiply(Quaternion::rotation(angle));
    if (!attitude.normalize()) return fail();
    // Right-multiplicative error injection covariance reset.
    memset(phi_,0,sizeof(phi_));
    for (unsigned i=0;i<States;++i) phi_[i][i]=1;
    const Vec basis[3]={{1,0,0},{0,1,0},{0,0,1}};
    for (unsigned j=0;j<3;++j) {
      const Vec c=scale(cross(angle,basis[j]),-0.5f);
      for (unsigned i=0;i<3;++i) phi_[6+i][6+j]+=c[i];
    }
    propagate(0);
    return check();
  }
};

}
