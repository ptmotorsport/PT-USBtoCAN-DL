#ifndef INS_CAN_H
#define INS_CAN_H

#include <cmath>
#include <cstdint>

namespace InsCan {

struct Frame {
  uint16_t id;
  uint8_t data[8];
};

// Wire limits for signed 0.01-scaled IMU fields; callers clamp before encoding.
constexpr float MaxRateDps = 327.67f;
constexpr float MaxAccelG = 8.0f;

struct Snapshot {
  double lat, lon;
  float speedMps, heightM, courseDeg, headingDeg;
  float rateDps[3], accelG[3];
  uint8_t satellites, status; // Fresh GSV visible count, else GGA used count; status 1/3/4.
  bool positionValid, courseValid, headingValid, imuValid;
};

enum State : uint8_t {
  Uncalibrated = 0, TiltOnly = 1, GpsAided = 2,
  DeadReckoning = 3, Fault = 4
};

struct Attitude {
  float rollDeg, pitchDeg, yawDeg;
  bool tiltValid, yawValid, globalValid, calibrated;
  bool imuFresh, gpsFresh, baroFresh, txFault;
  State state;
};

// These bits identify unavailable bank fields; remaining fault bits are caller-defined.
enum FaultBits : uint16_t {
  CourseUnavailable = 0x1000,
  HeadingUnavailable = 0x2000,
  ImuUnavailable = 0x4000,
  TelemetryUnrepresentable = 0x8000
};

struct Quality {
  uint32_t gpsAgeMs; // UINT32_MAX means unknown; 65535 on wire means unknown/old.
  float horizontalSigmaM, verticalSigmaM; // Negative/nonfinite means unknown.
  uint16_t faults;
};

namespace Detail {

inline bool inRange(double value, double minimum, double maximum) {
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

inline void put16(uint8_t* out, int32_t value) {
  const uint16_t bits = static_cast<uint16_t>(value);
  out[0] = static_cast<uint8_t>(bits >> 8);
  out[1] = static_cast<uint8_t>(bits);
}

inline void put32(uint8_t* out, int32_t value) {
  const uint32_t bits = static_cast<uint32_t>(value);
  out[0] = static_cast<uint8_t>(bits >> 24);
  out[1] = static_cast<uint8_t>(bits >> 16);
  out[2] = static_cast<uint8_t>(bits >> 8);
  out[3] = static_cast<uint8_t>(bits);
}

inline int32_t rounded(double value) {
  return static_cast<int32_t>(std::round(value));
}

inline bool angle(float value) {
  return std::isfinite(value) && value >= 0.0f && value < 360.0f;
}

inline uint16_t sigma(float value, bool& exact) {
  if (!inRange(value, 0.0, 6553.4)) {
    exact = false;
    return 65535;
  }
  return static_cast<uint16_t>(rounded(static_cast<double>(value) * 10.0));
}

} // namespace Detail

// GPS bank layout source: docs/gpsV2Manual.pdf, pages 6-7.
// All multibyte values are big-endian; rounding is nearest, ties away from zero.
// Validate ALL physical inputs, even invalid ones: supply finite zero placeholders.
// On failure bank/attitude output is unchanged; neither encoder silently clamps.
// NoFix requires !positionValid; 2D/3D require positionValid. Invalid position
// zeros lat/lon/speed/height. Other invalid fields zero only their own channels.
// Caller MUST publish Course/Heading/ImuUnavailable quality bits when applicable:
// zero heading is unavailable, NOT a measured north heading. Gate emission of
// position frames on validity. Counters are supplied modulo 16 by the caller;
// increment GPS for each position frame and empty for each empty frame.
inline bool encodeBank(const Snapshot& s, bool fused, uint8_t gpsCounter,
                       uint8_t emptyCounter, Frame (&frames)[4]) {
  using namespace Detail;
  if (!inRange(s.lat, -90.0, 90.0) ||
      !inRange(s.lon, -180.0, 180.0) ||
      !inRange(s.speedMps, -327.68f, 327.67f) ||
      !inRange(s.heightM, -32768.0, 32767.0) ||
      !angle(s.courseDeg) || !angle(s.headingDeg) ||
      s.satellites > 72 ||
      (s.status != 1 && s.status != 3 && s.status != 4) ||
      (s.positionValid != (s.status != 1))) {
    return false;
  }
  for (unsigned i = 0; i < 3; ++i) {
    if (!inRange(s.rateDps[i], -MaxRateDps, MaxRateDps) ||
        !inRange(s.accelG[i], -MaxAccelG, MaxAccelG)) {
      return false;
    }
  }
  Frame out[4] = {};
  const uint16_t base = fused ? 0x408 : 0x400;
  for (unsigned i = 0; i < 4; ++i) out[i].id = base + i;
  if (s.positionValid) {
    put32(out[0].data, rounded(s.lat * 1e7));
    put32(out[0].data + 4, rounded(s.lon * 1e7));
    put16(out[1].data, rounded(static_cast<double>(s.speedMps) * 100.0));
    put16(out[1].data + 2, rounded(s.heightM));
  }
  out[1].data[5] = s.satellites;
  out[1].data[6] = static_cast<uint8_t>((gpsCounter & 15) |
                                       ((emptyCounter & 15) << 4));
  out[1].data[7] = s.status;
  // Circular rounding near 360 degrees represents north as zero.
  if (s.courseValid) put16(out[2].data, rounded(s.courseDeg) % 360);
  if (s.headingValid) put16(out[2].data + 2, rounded(s.headingDeg) % 360);
  if (s.imuValid) {
    put16(out[2].data + 4, rounded(static_cast<double>(s.rateDps[0]) * 100.0));
    put16(out[2].data + 6, rounded(static_cast<double>(s.rateDps[1]) * 100.0));
    put16(out[3].data, rounded(static_cast<double>(s.rateDps[2]) * 100.0));
    for (unsigned i = 0; i < 3; ++i)
      put16(out[3].data + 2 + i * 2,
            rounded(static_cast<double>(s.accelG[i]) * 100.0));
  }
  for (unsigned i = 0; i < 4; ++i) frames[i] = out[i];
  return true;
}

// Invalid tilt/yaw is zero with its validity flag clear. Yaw is [0,360);
// circular rounding to 0.01 degree wraps north to zero.
inline bool encodeAttitude(const Attitude& a, Frame& frame) {
  using namespace Detail;
  if (!inRange(a.rollDeg, -180.0, 180.0) ||
      !inRange(a.pitchDeg, -90.0, 90.0) || !angle(a.yawDeg) ||
      static_cast<unsigned>(a.state) > 4) {
    return false;
  }
  Frame out = {};
  out.id = 0x40C;
  if (a.tiltValid) {
    put16(out.data, rounded(static_cast<double>(a.rollDeg) * 100.0));
    put16(out.data + 2, rounded(static_cast<double>(a.pitchDeg) * 100.0));
  }
  if (a.yawValid)
    put16(out.data + 4, rounded(static_cast<double>(a.yawDeg) * 100.0) % 36000);
  out.data[6] = static_cast<uint8_t>(
      (a.tiltValid ? 1 : 0) | (a.yawValid ? 2 : 0) |
      (a.globalValid ? 4 : 0) | (a.calibrated ? 8 : 0) |
      (a.imuFresh ? 16 : 0) | (a.gpsFresh ? 32 : 0) |
      (a.baroFresh ? 64 : 0) | (a.txFault ? 128 : 0));
  out.data[7] = static_cast<uint8_t>(a.state);
  frame = out;
  return true;
}

// Quality ALWAYS emits conservative telemetry. 65535 is the sentinel for
// unknown/overflow age or sigma; finite sigma supports [0,6553.4] m.
// Return false and set TelemetryUnrepresentable for ANY sentinel/saturation;
// caller may also set attitude.txFault. Other fault bits are preserved.
inline bool encodeQuality(const Quality& q, Frame& frame) {
  bool exact = q.gpsAgeMs < 65535;
  Frame out = {};
  out.id = 0x40D;
  Detail::put16(out.data, exact ? static_cast<int32_t>(q.gpsAgeMs) : 65535);
  Detail::put16(out.data + 2, Detail::sigma(q.horizontalSigmaM, exact));
  Detail::put16(out.data + 4, Detail::sigma(q.verticalSigmaM, exact));
  Detail::put16(out.data + 6, q.faults |
                (exact ? 0 : TelemetryUnrepresentable));
  frame = out;
  return exact;
}

} // namespace InsCan

#endif
