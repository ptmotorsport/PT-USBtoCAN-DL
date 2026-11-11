#include "ins_can.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

namespace {

uint16_t u16(const uint8_t* p) {
  return static_cast<uint16_t>((uint16_t(p[0]) << 8) | p[1]);
}

int32_t s16(const uint8_t* p) {
  const uint16_t bits = u16(p);
  return bits & 0x8000 ? int32_t(bits) - 65536 : bits;
}

int64_t s32(const uint8_t* p) {
  const uint32_t bits = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
                        (uint32_t(p[2]) << 8) | p[3];
  return bits & 0x80000000U ? int64_t(bits) - 4294967296LL : bits;
}

InsCan::Snapshot snapshot() {
  InsCan::Snapshot s = {};
  s.lat = -33.865143;
  s.lon = 151.209900;
  s.speedMps = 12.34f;
  s.heightM = -12.5f;
  s.courseDeg = 123.4f;
  s.headingDeg = 234.6f;
  s.rateDps[0] = -250.0f;
  s.rateDps[1] = 250.0f;
  s.rateDps[2] = -1.25f;
  s.accelG[0] = -4.0f;
  s.accelG[1] = 4.0f;
  s.accelG[2] = -0.5f;
  s.satellites = 18;
  s.status = 4;
  s.positionValid = s.courseValid = s.headingValid = s.imuValid = true;
  return s;
}

void testBankBytes() {
  const InsCan::Snapshot s = snapshot();
  InsCan::Frame raw[4], fused[4];
  assert(InsCan::encodeBank(s, false, 0x21, 0x3E, raw));
  assert(InsCan::encodeBank(s, true, 0x21, 0x3E, fused));
  for (unsigned i = 0; i < 4; ++i) {
    assert(raw[i].id == 0x400 + i);
    assert(fused[i].id == 0x408 + i);
    assert(std::memcmp(raw[i].data, fused[i].data, 8) == 0);
  }
  assert(s32(raw[0].data) == -338651430);
  assert(s32(raw[0].data + 4) == 1512099000);
  assert(s16(raw[1].data) == 1234); // 44.424 km/h / 0.036.
  assert(s16(raw[1].data + 2) == -13);
  assert(raw[1].data[4] == 0 && raw[1].data[5] == 18);
  assert(raw[1].data[6] == 0xE1 && raw[1].data[7] == 4);
  assert(u16(raw[2].data) == 123 && u16(raw[2].data + 2) == 235);
  assert(s16(raw[2].data + 4) == -25000);
  assert(s16(raw[2].data + 6) == 25000);
  assert(s16(raw[3].data) == -125);
  assert(s16(raw[3].data + 2) == -400);
  assert(s16(raw[3].data + 4) == 400);
  assert(s16(raw[3].data + 6) == -50);
  const uint8_t expected[] = {0xFF, 0x83, 0xFE, 0x70, 0x01, 0x90, 0xFF, 0xCE};
  assert(std::memcmp(raw[3].data, expected, 8) == 0);
  for (unsigned gps = 0; gps < 32; ++gps) {
    for (unsigned empty = 0; empty < 32; ++empty) {
      assert(InsCan::encodeBank(s, false, gps, empty, raw));
      assert(raw[1].data[6] == ((gps & 15) | ((empty & 15) << 4)));
      assert(raw[1].data[4] == 0 && (raw[1].data[7] & 0xF8) == 0);
    }
  }
}

void testBankBoundaries() {
  InsCan::Snapshot s = snapshot();
  InsCan::Frame frames[4];
  s.lat = -90; s.lon = -180; s.speedMps = -327.68f; s.heightM = -32768;
  s.courseDeg = 0; s.headingDeg = 359.99f; s.satellites = 0; s.status = 3;
  assert(InsCan::encodeBank(s, false, 0, 0, frames));
  assert(s32(frames[0].data) == -900000000);
  assert(s32(frames[0].data + 4) == -1800000000);
  assert(s16(frames[1].data) == -32768 && s16(frames[1].data + 2) == -32768);
  assert(u16(frames[2].data) == 0 && u16(frames[2].data + 2) == 0);
  s.lat = 90; s.lon = 180; s.speedMps = 327.67f; s.heightM = 32767;
  s.satellites = 72;
  assert(InsCan::encodeBank(s, false, 15, 15, frames));
  assert(s32(frames[0].data) == 900000000);
  assert(s32(frames[0].data + 4) == 1800000000);
  assert(s16(frames[1].data) == 32767 && s16(frames[1].data + 2) == 32767);
  assert(frames[1].data[6] == 255);
  s.lat = -0.00000005; s.lon = 0.00000005;
  s.heightM = 0.5f; s.courseDeg = 0.5f;
  assert(InsCan::encodeBank(s, false, 0, 0, frames));
  assert(s32(frames[0].data) == -1 && s32(frames[0].data + 4) == 1);
  assert(s16(frames[1].data + 2) == 1 && u16(frames[2].data) == 1);
  for (int sign = -1; sign <= 1; sign += 2) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      s = snapshot();
      s.rateDps[axis] = sign * 327.67f;
      s.accelG[axis] = sign * 8.0f;
      assert(InsCan::encodeBank(s, false, 0, 0, frames));
      const uint8_t* rate = axis < 2 ? frames[2].data + 4 + axis * 2 :
                                             frames[3].data;
      assert(s16(rate) == sign * 32767);
      assert(s16(frames[3].data + 2 + axis * 2) == sign * 800);
    }
  }
  s = snapshot(); s.courseDeg = 359.49f; s.headingDeg = 359.5f;
  assert(InsCan::encodeBank(s, false, 0, 0, frames));
  assert(u16(frames[2].data) == 359 && u16(frames[2].data + 2) == 0);
}

void rejects(const InsCan::Snapshot& s) {
  InsCan::Frame frames[4], before[4];
  std::memset(frames, 0xA5, sizeof(frames));
  std::memcpy(before, frames, sizeof(frames));
  assert(!InsCan::encodeBank(s, false, 0, 0, frames));
  assert(std::memcmp(frames, before, sizeof(frames)) == 0);
}

void testBankRejection() {
  InsCan::Snapshot s = snapshot();
  s.lat = 90.0000001; rejects(s);
  s = snapshot(); s.lat = -90.0000001; rejects(s);
  s = snapshot(); s.lon = 180.0000001; rejects(s);
  s = snapshot(); s.lon = -180.0000001; rejects(s);
  s = snapshot(); s.speedMps = 327.68f; rejects(s);
  s = snapshot(); s.speedMps = -327.69f; rejects(s);
  s = snapshot(); s.heightM = 32768; rejects(s);
  s = snapshot(); s.heightM = -32769; rejects(s);
  s = snapshot(); s.courseDeg = -0.01f; rejects(s);
  s = snapshot(); s.courseDeg = 360; rejects(s);
  s = snapshot(); s.headingDeg = -0.01f; rejects(s);
  s = snapshot(); s.headingDeg = 360; rejects(s);
  s = snapshot(); s.satellites = 73; rejects(s);
  for (unsigned status = 0; status < 256; ++status) {
    if (status == 3 || status == 4) continue;
    s = snapshot(); s.status = status; rejects(s);
  }
  s = snapshot(); s.positionValid = false; rejects(s);
  for (unsigned axis = 0; axis < 3; ++axis) {
    s = snapshot(); s.rateDps[axis] = 327.68f; rejects(s);
    s = snapshot(); s.rateDps[axis] = -327.68f; rejects(s);
    s = snapshot(); s.accelG[axis] = 8.01f; rejects(s);
    s = snapshot(); s.accelG[axis] = -8.01f; rejects(s);
  }
  const float bad[] = {std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity(),
                       -std::numeric_limits<float>::infinity()};
  for (unsigned i = 0; i < 3; ++i) {
    s = snapshot(); s.lat = bad[i]; rejects(s);
    s = snapshot(); s.lon = bad[i]; rejects(s);
    float InsCan::Snapshot::* fields[] = {
      &InsCan::Snapshot::speedMps, &InsCan::Snapshot::heightM,
      &InsCan::Snapshot::courseDeg, &InsCan::Snapshot::headingDeg
    };
    for (unsigned j = 0; j < 4; ++j) {
      s = snapshot(); s.*fields[j] = bad[i]; rejects(s);
    }
    for (unsigned axis = 0; axis < 3; ++axis) {
      s = snapshot(); s.rateDps[axis] = bad[i]; rejects(s);
      s = snapshot(); s.accelG[axis] = bad[i]; rejects(s);
    }
  }
}

void testInvalidChannels() {
  InsCan::Snapshot s = snapshot();
  s.positionValid = s.courseValid = s.headingValid = s.imuValid = false;
  s.status = 1;
  InsCan::Frame frames[4];
  assert(InsCan::encodeBank(s, false, 3, 5, frames));
  const uint8_t zero[8] = {};
  assert(std::memcmp(frames[0].data, zero, 8) == 0);
  assert(std::memcmp(frames[1].data, zero, 5) == 0);
  assert(frames[1].data[6] == 0x53 && frames[1].data[7] == 1);
  assert(std::memcmp(frames[2].data, zero, 8) == 0);
  assert(std::memcmp(frames[3].data, zero, 8) == 0);
  s = snapshot(); s.headingValid = false;
  assert(InsCan::encodeBank(s, false, 0, 0, frames));
  assert(u16(frames[2].data + 2) == 0 && u16(frames[2].data) == 123);
  assert(s16(frames[2].data + 4) == -25000);
  s = snapshot(); s.courseValid = false;
  assert(InsCan::encodeBank(s, false, 0, 0, frames));
  assert(u16(frames[2].data) == 0 && u16(frames[2].data + 2) == 235);
  s = snapshot(); s.imuValid = false;
  assert(InsCan::encodeBank(s, false, 0, 0, frames));
  assert(u16(frames[2].data + 2) == 235);
  assert(std::memcmp(frames[2].data + 4, zero, 4) == 0);
  assert(std::memcmp(frames[3].data, zero, 8) == 0);
  s.lat = std::numeric_limits<double>::quiet_NaN();
  s.positionValid = false; s.status = 1;
  rejects(s); // Invalid flags never bypass physical input validation.
}

void testAttitude() {
  InsCan::Attitude a = {};
  a.rollDeg = -180; a.pitchDeg = 90; a.yawDeg = 123.45f;
  a.tiltValid = a.yawValid = a.globalValid = a.calibrated = true;
  a.imuFresh = a.gpsFresh = a.baroFresh = a.txFault = true;
  a.state = InsCan::Fault;
  InsCan::Frame f;
  assert(InsCan::encodeAttitude(a, f));
  assert(f.id == 0x40C && s16(f.data) == -18000 && s16(f.data + 2) == 9000);
  assert(u16(f.data + 4) == 12345 && f.data[6] == 0xFF && f.data[7] == 4);
  a.rollDeg = 180; a.pitchDeg = -90; a.yawDeg = 359.999f;
  assert(InsCan::encodeAttitude(a, f));
  assert(s16(f.data) == 18000 && s16(f.data + 2) == -9000);
  assert(u16(f.data + 4) == 0);
  a.rollDeg = -0.125f; a.pitchDeg = 0.125f; a.yawDeg = 359.99f;
  assert(InsCan::encodeAttitude(a, f));
  assert(s16(f.data) == -13 && s16(f.data + 2) == 13);
  assert(u16(f.data + 4) == 35999);
  for (unsigned state = 0; state < 5; ++state) {
    a.state = static_cast<InsCan::State>(state);
    assert(InsCan::encodeAttitude(a, f));
    assert(f.data[7] == state);
  }
  // Isolate every flag bit and the invalid-value zero placeholders.
  bool InsCan::Attitude::* flags[] = {
    &InsCan::Attitude::tiltValid, &InsCan::Attitude::yawValid,
    &InsCan::Attitude::globalValid, &InsCan::Attitude::calibrated,
    &InsCan::Attitude::imuFresh, &InsCan::Attitude::gpsFresh,
    &InsCan::Attitude::baroFresh, &InsCan::Attitude::txFault
  };
  for (unsigned i = 0; i < 8; ++i) {
    a = InsCan::Attitude();
    a.rollDeg = 12.5f; a.pitchDeg = -7.25f; a.yawDeg = 90;
    a.*flags[i] = true;
    assert(InsCan::encodeAttitude(a, f));
    assert(f.data[6] == (1 << i));
    assert(s16(f.data) == (i == 0 ? 1250 : 0));
    assert(s16(f.data + 2) == (i == 0 ? -725 : 0));
    assert(u16(f.data + 4) == (i == 1 ? 9000 : 0));
  }
  float InsCan::Attitude::* angles[] = {
    &InsCan::Attitude::rollDeg, &InsCan::Attitude::pitchDeg,
    &InsCan::Attitude::yawDeg
  };
  const float bad[][5] = {
    {-180.01f, 180.01f, INFINITY, -INFINITY, NAN},
    {-90.01f, 90.01f, INFINITY, -INFINITY, NAN},
    {-0.01f, 360.0f, INFINITY, -INFINITY, NAN}
  };
  const InsCan::Frame before = f;
  for (unsigned i = 0; i < 3; ++i) {
    for (unsigned j = 0; j < 5; ++j) {
      a = InsCan::Attitude(); a.*angles[i] = bad[i][j];
      assert(!InsCan::encodeAttitude(a, f));
      assert(std::memcmp(&before, &f, sizeof(f)) == 0);
    }
  }
  a = InsCan::Attitude(); a.state = static_cast<InsCan::State>(5);
  assert(!InsCan::encodeAttitude(a, f));
}

void testQuality() {
  InsCan::Quality q = {};
  q.gpsAgeMs = 1234; q.horizontalSigmaM = 12.5f; q.verticalSigmaM = 23.75f;
  q.faults = InsCan::HeadingUnavailable | 0x123;
  InsCan::Frame f;
  assert(InsCan::encodeQuality(q, f));
  const uint8_t expected[] = {0x04, 0xD2, 0x00, 0x7D, 0x00, 0xEE, 0x21, 0x23};
  assert(f.id == 0x40D && std::memcmp(f.data, expected, 8) == 0);
  q.gpsAgeMs = 65534; q.horizontalSigmaM = 6553.4f; q.verticalSigmaM = 0;
  assert(InsCan::encodeQuality(q, f));
  assert(u16(f.data) == 65534 && u16(f.data + 2) == 65534);
  const uint32_t old[] = {65535, 65536, UINT32_MAX};
  for (unsigned i = 0; i < 3; ++i) {
    q.gpsAgeMs = old[i];
    assert(!InsCan::encodeQuality(q, f));
    assert(u16(f.data) == 65535);
    assert(u16(f.data + 6) == (q.faults | InsCan::TelemetryUnrepresentable));
  }
  const float bad[] = {-0.1f, 6553.5f, 1e30f, INFINITY, -INFINITY, NAN};
  for (unsigned i = 0; i < 6; ++i) {
    q = InsCan::Quality(); q.horizontalSigmaM = bad[i]; q.verticalSigmaM = 1;
    assert(!InsCan::encodeQuality(q, f));
    assert(u16(f.data + 2) == 65535 && u16(f.data + 4) == 10);
    assert(u16(f.data + 6) == InsCan::TelemetryUnrepresentable);
    q.horizontalSigmaM = 1; q.verticalSigmaM = bad[i];
    assert(!InsCan::encodeQuality(q, f));
    assert(u16(f.data + 2) == 10 && u16(f.data + 4) == 65535);
  }
  q = InsCan::Quality();
  assert(InsCan::encodeQuality(q, f));
  const uint8_t zero[8] = {};
  assert(std::memcmp(f.data, zero, 8) == 0);
}

} // namespace

int main() {
  testBankBytes();
  testBankBoundaries();
  testBankRejection();
  testInvalidChannels();
  testAttitude();
  testQuality();
  std::puts("ins_can tests passed");
}
