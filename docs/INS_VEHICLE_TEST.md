# INS vehicle-test firmware

## Status and boundaries

`ins_vehicle` is a separate **experimental, non-safety-critical vehicle-test
build**. It contains scheduled MPU-6050/BMP280 acquisition, coherent GPS
parsing, a quaternion error-state EKF, and raw/fused CAN output. The original
`uno_r4_minima` datalogger/diagnostic build remains available.

Do not use these estimates for steering, braking, traction control or other
safety-critical ECU functions. A successful build or simulation is **not**
validation of navigation accuracy, real-time performance, sensor wiring or CAN
delivery on the vehicle. Complete the acceptance gates below before driving.

**Automatic CAN output:** this firmware starts transmitting its ten experimental
standard-ID frames at boot as soon as the CAN controller initializes, even
without GPS lock, calibration or a valid fused solution. First test on an
isolated bus with a receiver at the configured bitrate. Do not connect it to a
live vehicle network until checking for identifier conflicts and ensuring no
ECU strategy acts on these experimental frames. `TX OFF` pauses output after
the CLI becomes available; it does not prevent the initial boot transmissions.

There are no historical bench calibration constants. Defaults are zero gyro
offset, **uncalibrated**, vehicle X forward / Y right / Z down, with sensor
X/Y/Z mapped directly to those axes. No accelerometer level offset is learned.
The current upside-down/askew bench position is not assumed to be vehicle level.

SD, SdFat, logging queues, SD retries, RTC sync, logger buttons, LED menus and
VBATT monitoring are excluded from this build. Termination control D3 defaults
OFF. Fit appropriate bus termination externally; it is not enabled by a command
in this build. GPS D1 is returned to input after UART initialization; keep GPS
RX disconnected unless suitably level shifted. Validate all sensor supply and
I2C pull-up levels.

## Build and upload

```powershell
pio run -e ins_vehicle
pio run -e uno_r4_minima
pio run -e ins_vehicle -t upload --upload-port COM7
pio device monitor --port COM7 --baud 115200
```

Replace COM7 with the actual board port. Close other serial monitors before
uploading. Do not select the logger environment accidentally. Firmware never
waits for a PC connection or GPS fix at startup.

The INS build generates its own linker script under the build directory to
reserve **4 KiB of stack** (verified through linker symbols), leaving installed
framework files and the logger's memory layout unchanged. Compiler `.su` files
are emitted with per-function stack estimates; they are not whole-program
high-water measurements.

The INS build uses native CAN, not an external MCP2515. The configured CAN
bitrate defaults to 1 Mbit/s. A separate CAN receiver is needed to verify
transmitted traffic and provide acknowledgments. Successful driver acceptance
is not proof of receiver delivery.

## Mounting and calibration

Configure the mounting **after physically mounting the device in the vehicle**.
The command names the signed **sensor** axis corresponding to each vehicle
axis, in order vehicle forward, right, down:

```text
TX OFF
MOUNT 1 2 3
CAL GYRO
```

For example `MOUNT -2 1 3` means vehicle forward is sensor -Y, vehicle right
is sensor +X and vehicle down is sensor +Z. Only unique right-handed signed
axis permutations are accepted. A rigid mounting is required; arbitrary
non-axis-aligned installation needs a future full rotation-matrix configuration.
Do not use mounting configuration to subtract gravity or hide a physical tilt.

`CAL GYRO` requires approximately 300 consecutive stationary samples (~3 s).
The board may be stationary at **any orientation**; no old measured offsets
are used. It estimates gyro bias and initializes tilt from the measured gravity
vector, not from an assumed horizontal surface. Motion/noise, accelerometer or
gyro saturation, or fresh GPS speed above 1 m/s rejects a batch.
`CAL GYRO` stops after repeated failures over 15 s; boot/automatic stationary
reinitialization with a saved bias keeps retrying silently until the vehicle is
still. This is not a full
six-position accelerometer scale/bias calibration, and cannot reliably distinguish
every constant slow rotation from gyro bias. Run `CAL GYRO` with the **engine off**
and the vehicle completely still: it uses strict noise limits (per-axis gyro
std 0.57 deg/s, accel 0.2 m/s2) because it estimates the bias.

Saved-bias tilt initialization (boot, `SAVE`, `SENSOR INIT`, automatic
reinitialization) tolerates an **idling engine**. Session 4 measured up to
~2 deg/s gyro and ~0.06 g accelerometer per-axis noise at idle, which the strict
limits rejected for the whole drive. These batches accept per-axis gyro std up
to ~3 deg/s, accel std up to 1 m/s2 and per-sample |accel| within 3 m/s2 of 1 g,
but require the 3 s **mean** gyro to be within 1 deg/s of the saved bias, so slow
turns are still rejected. Idle vibration can bias initial tilt by a few degrees;
gravity aiding then corrects it. `STATUS` reports still-batch rejects by
sample motion / noise / turning.

On success:

```text
SAVE
```

`SAVE` persists mounting, gyro bias, GPS baud, GPS 5 Hz choice, CAN bitrate and
course-aid setting.
EEPROM record: bytes **512-551**, versioned, CRC32-protected, verified after
writing, with invalidation-first writes. It does not overlap logger settings.
An interrupted save yields invalid/default configuration rather than partially
accepted calibration. TX is not persisted.

SAVE resets navigation; keep stationary for another 3 s so tilt can initialize
using the **explicitly saved** gyro bias. On later boots, the saved bias can be
used after stationary tilt initialization. No tilt quaternion, global origin or
absolute heading is saved. `DEFAULTS` resets RAM configuration and clears
calibration; `SAVE` is needed to overwrite an existing saved record.

Changing `MOUNT` invalidates calibration; CAN continues with fusion validity
cleared. Recalibrate after
changing mounting or if gyro temperature behavior/offset warrants it. For this
upside-down bench setup, diagnostics can be read without calibration, and an
explicit temporary `CAL GYRO` can test tilt, but **do not SAVE a bench test as
vehicle calibration**.

## Heading and GPS initialization

This single-antenna GPS does not measure stationary yaw or full attitude.
Indoors, acceleration, gyro, barometer and relative tilt can be tested, but
global fused position and vehicle heading remain invalid.

Two explicit heading-initialization choices:

- `HEADING 123.45`: supply a known true heading of the vehicle front, in degrees.
  Requires tilt initialization. Clears navigation origin; CAN continues with
  global position invalid until the next accepted fix.
  Use a genuinely known heading, not the board's arbitrary bench orientation.
- `COURSE AID ON`: opt into one-time heading initialization from GPS course
  while moving above 3 m/s, with low measured yaw rate and near-1 g magnitude.
  **This assumes forward, low-slip motion**. Do not use it to initialize during
  reversing, sideslip or tight corners. Course aid defaults OFF. `SAVE` if this
  choice should survive reboot.

Turning course aid OFF stops future course-based heading initialization; it
does not undo a previously initialized heading. `CAL GYRO` resets navigation.
Once initialized, yaw is propagated by gyro and corrected indirectly through
GPS position/velocity coupling; there is no magnetic heading observation.

Recommended in-vehicle setup (Session 1/2 logs indicated this mounting; confirm
it on your installation before saving):

```text
MOUNT -1 -2 3
CAL GYRO
COURSE AID ON
TX OFF
SAVE
```

Broadcast resumes automatically after the next reboot. Without `COURSE AID ON`
or `HEADING`, fusion stays TiltOnly and global outputs never become valid.

### Gravity aiding

Roll/pitch are corrected at 10 Hz from the accelerometer gravity vector, so
gyro-axis misalignment or bias cannot integrate into unbounded tilt drift.
The expected specific force includes a vehicle-motion term: GPS along-track
acceleration (dv/dt) on X, and speed x yaw rate / speed x pitch rate on Y/Z,
using fresh GPS speed or recently aided filter velocity. Measurement noise is
inflated with motion and unknown along-track acceleration. Updates are skipped
when speed is unknown and the vehicle is rotating faster than 3 deg/s, rejected
when the measured magnitude differs from 1 g by more than 2 m/s2, and also
chi-square gated per axis. The model assumes forward, low-slip motion; reverse
and large sideslip are handled only by the gates/noise. `STATUS` reports
gravity accepted/rejected/skipped counters. Gyro process noise grows with
rotation rate to account for scale/misalignment error.

An offline replay of Session 2 logged raw IMU/GPS through this filter reduced
stationary tilt error from about 19 degrees (no gravity aiding) to under
1 degree mean (max about 4.5 degrees). This is a log replay, not a substitute for
an in-vehicle validation.

GPS requires coherent checksum-valid RMC and GGA with matching UTC/coordinates,
at least four satellites, HDOP <= 10 and a fresh GSA dimension report to use 3D
altitude. A fresh fix without 3D confirmation can be raw 2D; it does not initialize
3D navigation. GGA altitude is mean-sea-level, matching the ECUMaster manual.
By default no receiver configuration is sent. If needed, use `GPS BAUD 38400`
to change only the device UART. At 9600 baud, use a reasonable receiver sentence/update
rate: UART capacity is approximately 960 bytes/s. GGA/RMC/GSA must be enabled.
GSV supplies the visible-satellite count when fresh; otherwise a fresh GGA's
used-satellite count is supplied as a conservative fallback.

### Optional 5 Hz GPS (`GPS RATE 5`)

`GPS RATE 5` sends UBX configuration to the NEO-6M over D1 (Serial1 TX):
UART 38400 baud, GLL/VTG off, GSV every 5th epoch, GGA/RMC/GSA every epoch,
200 ms measurement rate, then saves to the receiver (BBR/flash/EEPROM where
fitted). The device UART follows to 38400. `GPS RATE 1` sends a
clear-and-load-defaults command at both 38400 and the configured baud, returning
to 1 Hz NMEA at the configured baud. Bytes are paced one per loop so sampling is
not blocked. `SAVE` (with TX OFF) persists the choice; the sequence is re-sent at
each boot because module configuration retention is not guaranteed.

**Wiring warning:** the Uno R4 Minima D1 is 5 V logic. The GPS RX input on many
GY-GPS6MV2 boards is 3.3 V; use a level shifter/divider before connecting D1.
UBX acknowledgments are not read. Verify success with `STATUS`: GPS baud 38400,
5 Hz flag 1, UBX inactive, and epoch interval near 200 ms. While the sequence
is active `GPS BAUD` and another `GPS RATE` are rejected. At 5 Hz, GPS position
and velocity variances are doubled to account for correlated epochs.

The first usable 3D fix after heading initialization establishes the local
north/east/down origin. Absolute coordinates are retained in double precision;
local filter state/covariance use float. GPS position uses a conservative
HDOP-based variance; horizontal speed/course supply velocity only when valid.
Before an origin exists, position and velocity remain zero and their covariance
is reinitialized each prediction; they are not navigation estimates yet.
Attitude, its uncertainty and bias uncertainty continue propagating. This
prevents unused motion covariance from tripping the safety bound while waiting
indoors. Once an origin exists, normal motion propagation and GPS-loss policies
apply.
No vertical GPS velocity is invented. Linearized local coordinates use WGS84
meridional/transverse curvature at the origin, and are intended for a local
vehicle test route, not polar or intercontinental navigation; aiding is rejected
above 85 degrees absolute latitude.
This prototype does not compensate GPS receiver transport latency with a
delayed-state history or PPS. Evaluate dynamic latency on the actual receiver.

## Automatic CAN output

CAN broadcasts automatically at boot. To select a bitrate and inspect status:

```text
CANSPEED 1000
STATUS
```

All **ten** frames use standard identifiers and 8-byte payloads, at 25 Hz:

| IDs | Meaning |
| --- | --- |
| 0x400-0x403 | Raw GNSS + mounting-transformed raw IMU |
| 0x408-0x40B | Fused navigation + bias-corrected IMU |
| 0x40C | Roll/pitch/yaw at 0.01 deg, explicit validity flags and filter state |
| 0x40D | GPS age, horizontal/vertical position sigma, fault bits |

Use [ecumaster_raw_fused.dbc](../can/ecumaster_raw_fused.dbc).
The vendor protocol is [gpsV2Manual.pdf](gpsV2Manual.pdf), pages 6-7:
big-endian multibyte values, status **1 NoFix / 3 2D / 4 3D**, GPS index in
byte 6 low nibble, empty index in the high nibble. Counters wrap at 15.
Position and empty counters increment independently for each bank snapshot.
No fake UTC or additional vendor barometer frame is sent.

Raw rates are degrees/s; fused rates subtract estimated gyro bias. Both banks'
accelerations are **vehicle-body specific force, including gravity response**,
in g, not gravity-removed navigation acceleration. Fused acceleration subtracts
estimated accelerometer bias. The filter uses m/s2 and rad/s internally.
At level rest with forward/right/down axes, Z specific force is approximately
**-1 g**; down gravity is added after rotation for navigation.

NoFix position/height/speed and unavailable headings are zero placeholders,
not legitimate coordinates or headings. Read status/flags and ignore them.
Raw vehicle heading is always unavailable on this GPS; use the fused/custom
heading only when yaw is flagged valid. Quality bits indicate unavailable
motion course/vehicle heading/IMU. Consumers must also honor custom
per-source validity; a zero value alone cannot indicate validity.
Raw motion course is unavailable below 0.5 m/s; the course quality bit describes
this raw GNSS availability. Fused motion course additionally requires valid
global navigation and horizontal speed above 1 m/s. Check speed/status before
using either bank's course: stationary course is not vehicle heading.
The screenshot's one-degree heading fields are coarser than custom yaw.

Fused global output is valid only with a fresh accepted GPS correction,
initialized heading/origin, healthy/fresh IMU and bounded position uncertainty.
Heading validity also requires estimated yaw sigma below 30 degrees.
After GPS fix/accepted correction age reaches 2 s, global coordinate channels
become NoFix, even while the local state propagates in dead-reckoning mode.
After 10 s without accepted GPS aiding, the fused navigation estimate
invalidates; raw/status CAN frames continue. Stationary reinitialization is
required and starts automatically (at most once per second) when calibrated and
the IMU is fresh; it completes when the vehicle is held still for ~3 s. State
reports 4 until then and 1 (TiltOnly) afterwards; heading/origin must be
re-acquired by course aid or `HEADING`. `STATUS` counts reinitializations. This deliberately does not pretend that inertial dead reckoning still
has a GNSS fix. Short dropouts (<10 s) can be corrected on reacquisition if
innovation gating accepts the new fix.

CAN initializes and starts broadcasting automatically at boot using the
configured bitrate (default **1000 kbps**) and a 25 Hz snapshot rate. No GPS
fix, calibration, heading or `TX ON` command is needed to transmit raw sensors
and explicit validity/status frames. TX intent is not saved; boot always starts
the default automatic broadcast if CAN initializes. `TX OFF` is a manual pause;
`TX ON` resumes when CAN is available. `CANSPEED 125|250|500|1000` changes the
bitrate immediately and broadcast continues unless manually paused. Pause with
`TX OFF` before `SAVE` if the new bitrate should persist across reboot.
`CAN INIT` reinitializes CAN and resumes automatic broadcast unless manually
paused.

In raw frame **0x401**, payload byte 5 (zero-based, the sixth byte on the bus)
contains the satellite count: fresh GSV visible-satellite count when available,
otherwise the used-satellite count from a fresh fix, or zero when neither is
available. GPS fix status is separately encoded in byte 7: 1 NoFix, 3 2D,
4 3D. The fused GPS-info frame 0x409 carries the same satellite count and fix
status. Raw IMU rates/accelerations are broadcast in **0x402/0x403** from the
first available samples, before GPS lock or filter initialization. Fused IMU
channels remain zero until the filter is healthy; the 0x40D IMU-unavailable
fault bit marks those fused channels, not the raw IMU frames.

`TX OFF` clears pending software output; an already accepted in-flight CAN
frame may still complete. IMU, GPS, timing or covariance faults invalidate
fused channels and are reported while raw/status broadcasts continue. A CAN
controller error or unrepresentable CAN payload suspends transmission; use
`CAN INIT` to recover CAN errors (which resumes unless manually paused), or
`TX ON` after fixing a payload-range issue. Missing optional barometer prevents
vertical aiding but does not stop IMU/GPS navigation. IMU rate/acceleration
channels are clamped to the wire range (+/-327.67 deg/s, +/-8 g) and the clamp
raises the current range-fault bit 0x200 for 1 s; other out-of-protocol values
are rejected, not silently wrapped/clipped.

The latest-snapshot queue is bounded to ten frames, spaced at least 4 ms apart
(frame i no earlier than 4 x i ms after the snapshot) to spread the burst across
the 40 ms cycle, expires after 38 ms and counts drops. The driver has one standard-ID mailbox: busy returns are retried
without blocking or treating them as successful delivery. TX failures/controller
errors are reported; `CAN INIT` reinitializes and resumes unless manually paused.
Validate frame rates on the actual bus. Ten frames at 25 Hz use approximately 33.75 kbit/s
worst-case planning bandwidth (~3.4% at 1 Mbit/s; ~27% at 125 kbit/s).

If `txAccepted` stays near 1 while `txBusy` rapidly increases, the controller
accepted a frame for transmission but has not completed it. Common causes are
no other normal-mode CAN node to acknowledge it, a listen-only receiver,
bitrate mismatch, missing transceiver, swapped/disconnected CANH/CANL or bus
termination/wiring issues. A transmitter alone cannot receive its own ACK.
On the Uno R4 Minima, native CAN logic is on D4 (CAN TX) and D5 (CAN RX); use
the board's correctly wired CAN transceiver to connect differential CANH/CANL.
Use a powered second node configured to ACK, matching bitrate and standard CAN.
With power off, a correctly terminated two-end bus is typically about 60 ohms
measured between CANH and CANL; do not add a third terminator.

INS `STATUS` reports the latest controller event in hex: `0x2` warning, `0x4`
error-passive, `0x8` bus-off, `0x10` bus recovery, `0x100` channel error, and
`0x200` transmit abort. These event values are from the Renesas Arduino_CAN
driver. `CAN INIT` clears the driver's current event and retries initialization;
if the physical cause remains, the transmission error will recur. A CAN event
does not identify which wire/component is at fault, so check the physical bus
and ACK-capable receiver before retrying.

## Diagnostics and failure recovery

`STATUS`, `INS`, `GPS`, `IMU`, and `BARO` report cached state without changing
sensor configuration. `HELP` lists all commands. Output is bounded; slow USB
consumers cannot stall sampling. Console dropped-byte count is visible.
No navigation measurement is synthesized in place of a failed sensor.

Fault bits 0-11 (hex masks):

| Mask | Meaning |
| --- | --- |
| 0x001 | I2C bus/controller unavailable |
| 0x002 | IMU missing/stale/read failure |
| 0x004 | Barometer missing/stale/failure |
| 0x008 | Prediction timing fault |
| 0x010 | No fresh usable GPS fix |
| 0x020 | Calibration or stationary initialization required |
| 0x040 | Filter/covariance invalid |
| 0x080 | CAN error |
| 0x100 | Config verification error |
| 0x200 | Current (1 s) sensor saturation/CAN clamp, or sticky CAN payload encoding failure |
| 0x400 | Rejected measurement innovation |
| 0x800 | CAN snapshot expiry/overwrite |

DBC quality bits 12-15 add unavailable course, unavailable heading, unavailable
IMU, and unknown/unrepresentable telemetry. Age 65535 means unknown/too old;
sigma raw 65535 is unknown/unrepresentable, **not** high-confidence zero.
Historical faults are sticky until manually paused with `TX OFF`, then `FAULT CLEAR`; current
unavailable conditions remain visible.

The INS build directly owns RA4M1 IIC1 (A4/P101 SDA, A5/P100 SCL)
using the installed FSP/IRQManager,
not Wire. It uses hardware 100 kHz transactions with a **5 ms wait deadline**,
abort on failure, and closes a timed-out/failed controller until `SENSOR INIT`.
It does not enable internal 5 V pull-ups. Use suitable external pull-ups/level
shifting. This avoids Wire's one-second fault waits without modifying installed
framework files. Sensor register reads have at most two such transfer waits.
The deadline depends on a working MCU timer/interrupt system; it is not a
hardware watchdog or a proof of every driver call's worst-case execution time.
Do not instantiate another Wire owner on this bus.

`SENSOR INIT` explicitly retries bus/sensors and stops TX; if saved calibration
exists, hold stationary for tilt reinitialization. Otherwise use `CAL GYRO`.
If the IMU is not ready (missing at boot or after a bus timeout), sensors are
quietly retried automatically every 5 s; `STATUS` counts sensor retries.
MPU config is +/-8 g, +/-500 degrees/s (verified by register readback), 100 Hz,
44/42 Hz low-pass. A saturated sample is a transient: it is counted, excluded
from calibration and gravity aiding, inflates attitude/velocity uncertainty, and
raises current fault 0x200 for 1 s; it no longer permanently faults fusion.
`STATUS` reports saturation count and age. BMP uses
20 Hz asynchronous forced x1/x1 conversions and factory compensation.
Pressure height is relative to startup pressure and anchored to GPS-filter
height on origin establishment. Weather drift and cabin pressure changes are
not separately estimated; no barometer-bias state is claimed.

SLCAN/SavvyCAN uses exclusive CAN ownership. Entry stops autonomous TX;
`C`, then `EXIT`, returns to INS with TX still OFF. Fusion acquisition continues.
Closing/reopening the adapter cannot silently resume broadcasts. Use another
CAN adapter for observing this device's own TX: the bridge streams received
frames, not guaranteed local transmit echoes.

## Acceptance gates

Software validation is necessary but target tests below remain mandatory.
Native tests cover CRC config/torn-record rejection, signed mounting and
stationary calibration, upside-down/askew gravity cancellation, quaternion
integration, covariance positive definiteness, synthetic GPS/velocity/height
updates, innovation rejection, GPS epoch/freshness checks, and exact CAN packing.
The production-runtime integration test uses mocked hardware to exercise
calibration/save/reinitialization, indoor NoFix output, GPS-aided/dead-reckoning
transitions, mailbox busy/error handling, queue expiry, adapter ownership,
stale IMU and I2C timeout shutdown. Noise/covariance settings are initial
engineering assumptions, not measured sensor calibration or vehicle tuning.
They do not establish receiver accuracy or vehicle performance.

Software verification completed:

- Both PlatformIO firmware environments build.
- All 15 native C++11 test executables pass with warnings treated as errors.
- DBC strict parsing and independent golden-payload checks pass.
- INS image: 14,388 bytes static RAM, 106,108 bytes flash; actual linker stack
  reservation is 4 KiB, not a measured runtime high-water mark.

### Indoor bench verification, 2026-11-07

The connected board initially ran the logger environment. The corrected
`ins_vehicle` image was built and uploaded successfully over COM78.
Hardware testing exposed and corrected three issues:

- A4/P101 and A5/P100 select IIC1, not IIC0; the driver and mock assertions now
  use channel 1. Both sensors initialize and sensor reinitialization succeeds.
- An early IMU poll with no data-ready now retries 1 ms after completion,
  rather than waiting another full 10 ms sensor period.
- Unused pre-origin position covariance previously exceeded its safety bound
  after about a minute. Motion state is now held inactive before origin.
  A synthetic two-minute stationary test reproduces the old failure and passes
  with the correction; gravity-cancellation/Jacobian tests still exercise actual
  motion propagation after setting an origin.

A temporary, newly measured **RAM-only** gyro calibration initialized tilt at
the current bench orientation. The final firmware then passed a 90-second
fusion/USB-status soak and SLCAN entry/close/exit:

| Observation | Result |
| --- | --- |
| IMU sample rate | Approximately 100 Hz |
| Sample gaps over 20 ms | 0 |
| I2C failures | 0 |
| Maximum observed I2C transfer wait | 2,264 us |
| Maximum observed EKF prediction | 1,172 us |
| Maximum observed loop since boot | 9,302 us |
| Console dropped bytes | 0 |
| GPS at final calibrated status | 18,222 bytes / 666 valid NMEA / 0 parse errors |
| Indoor navigation validity | Tilt only; yaw/global position invalid |
| BMP280 | Fresh; approximately 1011 hPa and 17.7 C |
| SLCAN close/exit | Returned to INS; prior broadcast policy resumed; filter remained healthy |

These are observed maxima, not worst-case bounds or full CAN/GPS-aiding load
measurements. CAN TX was intentionally held OFF during this earlier bench test;
no bus-delivery claims are made.
`DEFAULTS` discarded the temporary bench bias afterward. Final calibration,
saved-config and course-aid flags were all zero, gyro offsets were zero, and
TX was OFF. EEPROM was not written; the serial port was released.

The build is **not yet vehicle-qualified**. Actual CAN capture/acknowledgments,
outdoor GPS-aided navigation, physical sensor fault injection, rotation/sign
checks and stack high-water measurements remain outstanding. Complete the
applicable gates below before driving.

1. **Upload/identity:** select `ins_vehicle`; verify automatic 25 Hz broadcast
   at the configured bitrate, no SD initialization, both sensors identified
   and increasing GPS byte/NMEA counts. Keep output on an isolated test bus.
2. **Bench, any orientation:** leave uncalibrated initially. Explicit temporary
   calibration may test attitude, but do not save bench calibration. At rest
   confirm gravity magnitude near 1 g, near-zero corrected gyro, plausible BMP
   pressure. Tilt/rotate each axis; verify signs, raw/fused/custom flags and
   no valid global coordinates indoors. Confirm upside-down gravity cancellation.
3. **Timing/RAM:** inspect STATUS sample count (approximately 100/s), max
   prediction/loop/transaction times and >20 ms intervals. Require no prediction
   faults/dropped samples under CAN/USB load. Measure actual stack high-water
   with a debugger before field testing; the build size alone is insufficient.
4. **Fault injection:** unplug sensors/GPS on a safe bench; verify stale/error
   flags, bounded bus timeout and continued CAN status/raw broadcasts with
   invalid sensor fields. Reinitialize explicitly. Exercise automatic TX at
   boot, TX OFF/ON, bitrate changes, SLCAN close/exit, invalid config and missing
   CAN acknowledgments. Verify recovery and that TX OFF remains respected
   during CAN reinitialization.
5. **Mounted stationary vehicle, engine off:** configure axes, calibrate gyro,
   SAVE, stationary initialization; check all axes. Outdoors require valid GSA
   3D plus matching RMC/GGA and stable MSL altitude. Set a known true heading
   or deliberately opt into forward course-aid initialization.
6. **Isolated CAN capture:** verify all ten identifiers, raw 0x402/0x403 IMU
   before GPS/fusion, 0x401 byte-5 satellite count, GPS status, endian scaling,
   both counters, NoFix/validity transitions, custom uncertainty, bus rate and
   no stale backlog. Keep these experimental channels disconnected from ECU control
   strategies. Compare DBC decoding with STATUS and a reference instrument.
7. **Low-speed controlled test:** initialize heading appropriately, travel
   forward straight, stop, turn and reverse; assess latency, signs, bias/noise
   and covariance consistency. Test GPS dropout/reacquisition without safety
   dependence on output. Record raw/fused CAN externally (this build has no SD).
8. **Release for wider testing only after evidence:** record timing/stack
   measurements, sensor-noise tuning, stationary drift, repeated-route error
   and failure behavior. This prototype does not claim centimetre accuracy or
   validated automotive-grade INS performance.
