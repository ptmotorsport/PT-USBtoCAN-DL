# Sensor fusion and CAN implementation pathway

**Implementation update:** the experimental baseline now exists in the separate
`ins_vehicle` environment. See [INS vehicle testing](INS_VEHICLE_TEST.md)
for implemented behavior, calibration, limitations and acceptance gates.
The sections below preserve the pre-implementation assessment; proposed items
are not evidence of completed hardware or field validation. The supplied vendor
manual has since confirmed NoFix=1, 2D=3 and 3D=4 and custom 0x40C/0x40D
have been approved and added to the DBC.

## Scope and decisions

This is an implementation assessment, not an enabled navigation filter or CAN
broadcaster. Existing diagnostics and logger behavior are unchanged by this
assessment. The intended filter is a **quaternion error-state EKF (ESKF)**,
not an iterated or invariant Kalman filter.

- Hardware: Uno R4 Minima / RA4M1, 48 MHz, 32 KiB RAM, 256 KiB flash;
  one MPU-6050 at 0x68, BMP280 at 0x76, and GY-GPS6MV2 on Serial1.
- Raw ECUMaster-compatible bank: **0x400-0x403**.
- Fused bank: **0x408-0x40B**, the same layout with **+0x008** offset.
- Full attitude needs an additional custom frame: the screenshot contains
  headings and angular rates, **not roll/pitch angles**.
- An INS-only build may sacrifice SD logging to fit the filter. Keep the
  existing datalogger build available rather than globally deleting logging.
- Byte order: multibyte values are **big-endian**, confirmed by the user.
  Screenshot byte/bit positions take precedence over conflicting reference code.

The test DBC is [ecumaster_raw_fused.dbc](../can/ecumaster_raw_fused.dbc).
It includes both banks, but deliberately does not invent custom attitude/status
frames, GPS status enumerations, or valid-looking no-fix values.
Validation passed with cantools strict parsing: eight standard eight-byte frames,
independently packed big-endian golden payloads, signed-coordinate/rate checks,
round trips, all signal range boundaries, all 256 counter-nibble combinations
in each bank, reserved-bit checks, and rejection of counter value 16.

## Reference assessed

[ptmotorsport/PT-IMU2CAN, Feature-EKF-INS](https://github.com/ptmotorsport/PT-IMU2CAN/tree/48cc028a4e5f97892accf6883175c4e067632dcf),
commit **48cc028a4e5f97892accf6883175c4e067632dcf**.

The branch already has separation between IMU, GPS, EKF and CAN code, a
15-state position/velocity/Euler-attitude/bias model, body-frame IMU alignment,
two-IMU averaging with disagreement rejection, position correction, and a
Joseph-form covariance update. These are useful starting ideas, not a
validated INS implementation. Its test directory has no implemented filter
regression tests. README claims of centimetre-level positioning are not
supported by this receiver or by the inspected implementation.

### Corrections required before reuse

| Area | Evidence in reference | Required change |
| --- | --- | --- |
| Position units | [gps.cpp](https://github.com/ptmotorsport/PT-IMU2CAN/blob/48cc028a4e5f97892accf6883175c4e067632dcf/src/gps.cpp) stores latitude/longitude in degrees; [main.cpp](https://github.com/ptmotorsport/PT-IMU2CAN/blob/48cc028a4e5f97892accf6883175c4e067632dcf/src/main.cpp) passes them directly to metre-based `updateGPS` | Convert to a local metric frame before correction; establish a consistent altitude origin and vertical sign. |
| Coordinate precision | [gps.h](https://github.com/ptmotorsport/PT-IMU2CAN/blob/48cc028a4e5f97892accf6883175c4e067632dcf/include/gps.h) uses default float matrices for absolute coordinates | Retain latitude/longitude as double or integer degrees x 1e7, with float local displacements. At longitude 151 degrees, adjacent float32 values are about 1.42 m apart at latitude -33 degrees; 1e-7 degree changes disappear. |
| Covariance model | [ekf.cpp](https://github.com/ptmotorsport/PT-IMU2CAN/blob/48cc028a4e5f97892accf6883175c4e067632dcf/src/ekf.cpp) reconstructs body acceleration from gravity-subtracted world acceleration in `computePhi` | Use bias-corrected **specific force including gravity response** for attitude sensitivity. At level rest the existing reconstruction becomes zero, incorrectly eliminating velocity-attitude coupling. |
| Bias observability | Same `computePhi` has no velocity/accelerometer-bias or attitude/gyro-bias blocks | Add bias Jacobians and process noise. Otherwise bias states start uncorrelated and GPS corrections cannot estimate them. |
| Attitude propagation | Same file uses Euler angles and uses bias values, not measured bias-corrected gyro rates, in attitude Jacobian terms | Use normalized nominal quaternions plus a 3D small-angle error state; consistent injection and covariance reset. Avoid Euler singularities and mounting-dependent axis hacks. |
| Integration/noise | Same file integrates position with already-updated velocity and adds constant `Q` each call | Use consistent position/velocity integration and dt-dependent noise discretization; validate transition Jacobians by finite differences. Reject timing faults rather than substituting fictitious elapsed time. |
| GPS quality | [gps.cpp](https://github.com/ptmotorsport/PT-IMU2CAN/blob/48cc028a4e5f97892accf6883175c4e067632dcf/src/gps.cpp) assembles fields with separate validity/update state | Emit coherent timestamped fixes with per-field validity/freshness; reject stale, invalid and duplicate measurements. Use speed/course as velocity observations only when valid. |
| Initialization | [main.cpp](https://github.com/ptmotorsport/PT-IMU2CAN/blob/48cc028a4e5f97892accf6883175c4e067632dcf/src/main.cpp) starts with zero attitude, then predicts even without a global origin | Initialize tilt from stationary gravity after mounting transform and calibrate gyro bias. Allow relative attitude indoors, but do not publish valid absolute position without a fix. |
| CAN units | Same file sends EKF roll/pitch angles to `canSendGyroXY` | Angular-rate fields must receive angular rates, not attitude angles. Our current decoder produces g and degrees/s; convert once to m/s2 and rad/s for the filter, once back for CAN. |
| CAN packing | [can_send.cpp](https://github.com/ptmotorsport/PT-IMU2CAN/blob/48cc028a4e5f97892accf6883175c4e067632dcf/src/canbus/can_send.cpp) puts GPS counter in upper nibble, zeros speed/satellites, hardcodes flags/time and headings | Screenshot requires GPS counter in **low nibble**, empty counter in high nibble, satellites at byte 5 and status in byte 7 low three bits. Implement real data, explicit validity, bounded encoding and checked TX results. |
| Barometer | Reference startup ignores it; `updateAltimeter` is only a private declaration | Implement a relative vertical observation, initial pressure/height datum and optional barometer-bias state. Do not equate pressure altitude directly with GPS height. |
| Transport/hardware | Reference uses MCP2515 and requires two IMUs | Keep this project's native Arduino_CAN. Support one IMU without fabricating a second sample; skip dual-IMU fusion here. |

## Hardware limits and what "attitude" means

A single-antenna GPS provides position and, with valid moving fixes, speed/course.
It does **not** measure full vehicle attitude, stationary yaw, roll or pitch.
Course and vehicle heading differ during reversing, sideslip and cornering.
The MPU-6050 has no magnetometer: stationary gravity constrains tilt but not
absolute yaw. Gyro yaw will drift until observable motion or an external heading
source permits correction. Do not label GPS course as independently measured yaw.

Use stationary updates conservatively: accelerometer tilt updates are invalid
during significant longitudinal/lateral acceleration. Zero-velocity updates
must pass a stationary detector, ideally corroborated by vehicle speed.
Vehicle nonholonomic constraints are optional and must be disabled for slip,
airborne motion, reversing ambiguity or other violated assumptions.

The measured stationary X gyro offset was approximately -3.7 degrees/s.
Calibrate startup bias, apply an explicit mounting transform (the test board's
Z acceleration was negative), and measure sensor noise before tuning the filter.
Do not change a single CAN Y-axis sign to compensate for arbitrary mounting.
Start with a documented right-handed frame, e.g. vehicle forward/right/down
and navigation north/east/down; test gravity and all axis signs in that frame.

The GY-GPS6MV2 is commonly a NEO-6 family breakout, but identify the actual
receiver/firmware and measure update rate before relying on specifications.
This setup does not inherit the reference README's 25 Hz GPS claim. At 9600
baud the UART carries about 960 bytes/s (8N1); sentence selection and navigation
rate must fit that budget. GPS PPS, receiver timestamps and UBX accuracy/velocity
reports are useful future improvements, not prerequisites to the first NMEA test.
Bidirectional receiver configuration requires appropriate GPS RX level shifting.

## Proposed runtime architecture

```text
timestamped MPU sample -> calibration + mounting -> ESKF predict
timestamped GPS fix -> local-frame position/velocity -> gated correction
timestamped BMP pressure -> relative height -> gated vertical correction
filter + raw sensor snapshots -> checked CAN encoding -> bounded CAN TX
USB CLI / adapter lifecycle -> diagnostics, configuration, explicit modes
```

Use one scheduler and one owner for each UART/I2C/CAN resource. Split today's
on-demand sensor test initialization from production sampling:

- [i2c_diagnostics.cpp](../src/i2c_diagnostics.cpp) currently reconfigures sensors
  and waits when a CLI command is run. Production diagnostics must report cached
  state, not reconfigure the IMU during fusion or trigger overlapping conversions.
- [gps_monitor.h](../include/gps_monitor.h) currently validates NMEA checksums
  without parsing a navigation solution. Add coherent GGA/RMC decoding or a
  bounded existing parser, while preserving byte/checksum diagnostics.
- [USBtoCAN_Datalogger.ino](../src/USBtoCAN_Datalogger.ino) owns native CAN,
  GPS polling and logger/adapter mode transitions. Preserve CAN ownership:
  SLCAN `C`, bitrate changes and adapter entry currently close/reconfigure CAN.
  An INS build needs an explicit mode contract, not simultaneous uncontrolled TX.
- Keep RTC/VBATT/LED diagnostics, but make SD, logger buttons and CLI commands
  clearly unavailable in the INS build. An absent SD card must not trigger
  remount attempts or consume navigation CPU time.

Initial targets, to be **measured rather than assumed**:

| Task | Starting target |
| --- | --- |
| MPU acquisition + prediction | 100 Hz, hardware data-ready/FIFO if practical |
| GPS UART reception | Continuously serviced, measurement correction once per new fix |
| BMP280 | 10-25 Hz asynchronous forced measurements, no settling delay in main loop |
| Raw/fused CAN snapshots | Start at 25 Hz; optional IMU rates at 100 Hz after profiling |
| USB diagnostics | On-demand bounded output; no blocking wait for a connected PC |

Handle a delayed sample with its real acquisition time. Bound acceptable dt,
count overruns/lost samples, and enter a degraded state on timing/data faults.
Do not conceal a one-second I2C timeout with a nominal 10 ms prediction.
The installed Renesas Wire driver has synchronous transaction waits up to
about one second and no public adjustable bus-timeout API in its header;
production acquisition needs verified bus-fault containment/recovery or a
bounded lower-level driver. Current successful bench readings do not prove
fault-path timing. Avoid busy sensor-settling waits used by today's diagnostics.

## Filter design and memory

Nominal state: local position (3), velocity (3), normalized quaternion (4),
accelerometer bias (3), gyro bias (3). Error covariance: 15 x 15 for errors
`[delta-p, delta-v, delta-theta, delta-ba, delta-bg]`.
For NED, rotate bias-corrected specific force and **add** downward gravity;
do not copy the reference's positive-up gravity subtraction without changing
its entire coordinate convention.

Implement:

1. Stationary calibration, tilt initialization and motion/validity state machine.
2. Quaternion integration, local-frame kinematics and consistent Jacobians.
3. Continuous noise model discretized for measured dt.
4. GPS position and, when supported by coherent fixes, velocity corrections;
   adaptive measurement uncertainty and innovation/Mahalanobis gating.
5. BMP relative height correction; pressure baseline first, atmospheric bias
   state later if necessary. Explicitly reconcile GPS MSL/ellipsoid heights.
6. Joseph update, covariance symmetry/positive-definiteness checks, quaternion
   normalization, error injection and covariance reset.
7. States such as uninitialized, tilt-only, GPS-aided, dead-reckoning and fault.
   Suppress valid-looking global outputs until initialized; limit dead reckoning
   by configured uncertainty/time bounds. Report stale/invalid outputs explicitly.

Current linked firmware reports **23,092 bytes** static RAM, excluding the
reserved 8 KiB heap and approximately 1 KiB stack. The link is already close
to its heap/stack boundary, despite the size summary saying 70.5% RAM.
Measured ELF symbols:

| Allocation | Bytes |
| --- | ---: |
| 448-frame CAN logging queue, including indices | 8,968 |
| SdFat global object | 1,188 |
| Bounded USB console object | 4,116 |
| Wire object | 1,096 |
| Serial1 object | 1,196 |
| SLCAN output object | 1,048 |

An INS-only build can reclaim the 8,968-byte log queue plus the SdFat object
and other linked SD/file state. Exact savings must be measured after linking;
some library/runtime storage is shared. No code should allocate a log queue
merely because logging is disabled at runtime.

One float 15 x 15 covariance is **900 bytes**; a 16-error-state variant with
barometer bias is 1,024 bytes. Two dense 15 x 15 arrays plus a nominal state
are already roughly 1.9 KiB, before correction workspaces. The reference keeps
P and Q, constructs Phi, Joseph update matrices and library expression
temporaries: do not transplant that stack footprint into a ~1 KiB stack.
Use fixed persistent scratch storage, sparse propagation, diagonal noise and
scalar/sequential measurement updates where mathematically appropriate.
Profile stack high-water, CPU time and worst-case correction time on target.
Keep global-coordinate conversions in double; keep filter matrices in float.

## CAN contract and DBC

| Raw ID | Fused ID | Contents |
| --- | --- | --- |
| 0x400 | 0x408 | Latitude and longitude, signed 32-bit, degrees x 1e7 |
| 0x401 | 0x409 | Speed, height, satellites, counters and GPS status |
| 0x402 | 0x40A | Motion/vehicle headings, X/Y angular rates |
| 0x403 | 0x40B | Z angular rate and XYZ acceleration |

The raw bank combines GNSS and raw calibrated IMU; there is no separate
third bank required just for IMU. Raw GPS vehicle heading is unavailable on
this hardware. Specify suppression/validity for unsupported raw fields before TX.
The fused bank uses estimated position/velocity/yaw, bias-corrected body angular
rates, and an explicitly defined acceleration convention. Gravity-removed
navigation acceleration and gravity-including body specific force must not
silently share an ECU channel.

DBC details:

- Height occupies bytes 2-3; **byte 4 stays reserved**; satellites is byte 5.
- GPS frame index is byte 6 bits 0-3; empty index bits 4-7.
- Screenshot index ranges 0-16 are impossible for four bits; DBC uses 0-15.
- GPS status is byte 7 bits 0-2. Screenshot lists 1-5 without meanings:
  leave enumerations/no-fix encoding unresolved, rather than copy literal 0x19.
- Speed factor is 36/1000 = 0.036 km/h per raw count.
- Angle rates and acceleration use 0.01 degrees/s and 0.01 g.
- Signed boundaries and protocol ranges differ (e.g. speed); round, validate
  and handle out-of-range values explicitly instead of unchecked narrowing.
- Screenshot has no UTC/barometer/custom attitude frames: do not copy the
  reference's 0x399/0x404 stubs into this test DBC.

**Proposed, not yet approved:** custom attitude at 0x40C with signed roll/pitch
at 0.01 degrees, yaw at 0.01 degrees, and explicit validity bits; optional
0x40D for filter state/age/uncertainty. Confirm bus occupancy, units, wrapping
and invalid encodings before adding either to the DBC. This matters because
the fused bank's headings only have one-degree resolution, and GPS status
cannot express all filter quality states.

At 25 Hz, eight eight-byte standard frames produce 200 frames/s. Using a
conservative ~135 bits/frame gives about 27 kbit/s, before custom frames:
approximately 2.7% of 1 Mbit/s, or 21.6% of 125 kbit/s.
Keep bitrate configurable, check arbitration ID conflicts and CAN write results,
and use bounded/latest-snapshot TX instead of accumulating stale navigation.
Use an independent CAN adapter or confirmed local TX mirroring for observation:
native transmitted frames need not appear in the device's receive/SLCAN stream.

## Staged implementation and acceptance

1. **Build separation and timing:** add an INS/no-SD PlatformIO environment
   using compile-time exclusion of queue, SdFat, SD services and logging UX.
   Build both environments, compare ELF RAM/flash and stack use; no absent-card
   retries in INS mode. Decide adapter/INS coexistence before enabling TX.
2. **Production acquisition:** initialize sensors once, schedule bounded samples,
   calibrated mounting and startup gyro calibration, coherent GPS parsing and
   async BMP conversion. Verify stationary/tilted readings, UART bandwidth,
   missing sensors, I2C fault paths, GPS loss and sample timestamps.
3. **Raw CAN first:** implement a pure, transport-independent ECUMaster encoder.
   Verify negative coordinates, endian layout, height/reserved byte, status
   masks, both counters 0-15, rounding, saturation/fault reporting, rates,
   and zero unsolicited traffic before explicit enable. Compare captures against DBC.
4. **Attitude ESKF:** unit tests for identity rotation, inverted mounting,
   gravity cancellation, yaw integration, quaternion normalization and stationary
   gyro offset. Verify finite-difference Jacobians and dt-scaled covariance.
   Keep position invalid indoors; validate tilt and bias on the bench.
5. **GPS/barometer navigation:** synthetic constant velocity/turns, GPS outliers,
   duplicate/stale fixes, dropout/reacquisition, reverse/slip, pressure drift,
   and height-datum consistency. Require covariance symmetry/positive definiteness,
   deterministic resets and no NaN/success-shaped invalid states.
6. **Fused/custom CAN and field validation:** approve quality/attitude frame
   definitions; capture raw/fused together on an isolated CAN bus. Outdoor
   repeatable stationary and vehicle trials, independent reference where possible,
   bias/noise tuning, end-to-end latency and worst-case CPU/stack measurements.
   Set accuracy/drift targets from measured results, not centimetre claims.

The recommended next coding step is **stage 1 + stage 2**, then raw CAN.
A correct acquisition/coordinate/validity layer is a prerequisite to meaningful
filter tuning; enabling the reference filter unchanged would hide errors behind
plausible-looking ECU data.
