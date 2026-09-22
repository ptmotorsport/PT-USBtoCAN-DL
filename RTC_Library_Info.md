# Arduino RTC Library and Project Integration

## Scope and Source

This reference is verified against the Arduino Renesas RTC library installed by PlatformIO for this project:

- Header: `framework-arduinorenesas-uno/libraries/RTC/src/RTC.h`
- Implementation: `framework-arduinorenesas-uno/libraries/RTC/src/RTC.cpp`
- FSP RTC API: `r_rtc_api.h`

Include the library with:

```cpp
#include <RTC.h>
```

The public global instance is `RTC` of type `RTClock`.

## RTClock API

```cpp
bool RTC.begin();
bool RTC.getTime(RTCTime &time);
bool RTC.setTime(RTCTime &time);
bool RTC.setTimeIfNotRunning(RTCTime &time);
bool RTC.isRunning();

bool RTC.setPeriodicCallback(rtc_cbk_t callback, Period period);
bool RTC.setAlarmCallback(rtc_cbk_t callback, RTCTime &time, AlarmMatch &match);
bool RTC.setAlarm(RTCTime &time, AlarmMatch &match);
```

`RTC.begin()` opens the FSP RTC driver and marks the Arduino wrapper initialized when that succeeds. `getTime()` and `setTime()` return `false` until initialization succeeds. `setTimeIfNotRunning()` writes the time only when `RTC.isRunning()` is false; it returns `false` when the RTC is already running.

`setAlarm()` is equivalent to `setAlarmCallback(nullptr, time, match)`. It configures the FSP alarm interrupt but does not provide an Arduino-level notification mechanism when no callback is supplied. Use `setAlarmCallback()` when application code needs to run at the alarm.

`RTC.setClockSource()` is not part of this Arduino API.

## RTCTime API

```cpp
RTCTime();
RTCTime(time_t unixTime);
RTCTime(struct tm &time);
RTCTime(int day, Month month, int year, int hour, int minute, int second, DayOfWeek dayOfWeek, SaveLight saveLight);
```

The default value is `2000-01-01 00:00:00`, Saturday, with daylight saving inactive.

```cpp
bool setDayOfMonth(int day);       // accepts 1 through 31
bool setMonthOfYear(Month month);  // stores the supplied enum value
bool setYear(int year);            // years >= 1900 are stored as tm_year
bool setHour(int hour);            // accepts 0 through 23
bool setMinute(int minute);        // accepts 0 through 59
bool setSecond(int second);        // accepts 0 through 59
bool setDayOfWeek(DayOfWeek day);
bool setSaveLight(SaveLight value);
bool setUnixTime(time_t unixTime);
void setTM(struct tm &time);
```

The library validates only the day-of-month range, hour, minute, and second. In particular, it does not validate the day against the selected month or validate a year range.

```cpp
int getDayOfMonth() const;
Month getMonth() const;
int getYear() const;
int getHour() const;
int getMinutes() const;
int getSeconds() const;
DayOfWeek getDayOfWeek() const;
time_t getUnixTime();
struct tm getTmTime();
String toString() const;
```

`toString()` returns an ISO-8601-style local date and time, for example `2026-09-22T14:30:00`.

## Enums and Alarm Matching

`Month` is zero-based (`Month::JANUARY` through `Month::DECEMBER`). `DayOfWeek` uses Sunday as `0`, Monday through Saturday as `1` through `6`. `SaveLight` provides `SAVING_TIME_INACTIVE` and `SAVING_TIME_ACTIVE`.

`Period` supports a periodic callback every 2 seconds, 1 second, or 2 through 256 times per second.

`AlarmMatch` selects the calendar fields that must match for an alarm. The library provides matching getters and add/remove methods for second, minute, hour, day, month, year, and day of week.

```cpp
void alarmCallback() {
  // Keep interrupt callbacks short.
}

RTCTime alarmTime;
alarmTime.setHour(14);
alarmTime.setMinute(30);
alarmTime.setSecond(0);

AlarmMatch match;
match.addMatchHour();
match.addMatchMinute();
match.addMatchSecond();

RTC.setAlarmCallback(alarmCallback, alarmTime, match);
```

## Clock Source in This Project

The FSP RTC driver offers `RTC_CLOCK_SOURCE_SUBCLK` for the external sub-clock oscillator and `RTC_CLOCK_SOURCE_LOCO` for the internal low-power oscillator.

`RTC.cpp` is compiled separately from the sketch. Therefore, defining `RTC_CLOCK_SOURCE` before `#include <RTC.h>` in the `.ino` file does not configure the source used by the RTC library translation unit.

This project makes the selection reproducible with [scripts/configure_rtc_subclk.py](scripts/configure_rtc_subclk.py), registered in [platformio.ini](platformio.ini) as a pre-build script. It updates the installed framework's `RTC.cpp` default from LOCO to SUBCLK before PlatformIO compiles the RTC archive. The hook is idempotent: it accepts a framework already configured for SUBCLK and fails if the expected setting is absent.

In the Arduino RTC wrapper, the configured source is applied when the RTC is opened while stopped by calling the FSP `R_RTC_ClockSourceSet()` function.

## Current SOSC and RTC Diagnostics

Before `RTC.begin()`, [src/USBtoCAN_Datalogger.ino](src/USBtoCAN_Datalogger.ino) currently:

1. Starts serial output and reads `SOSCCR`, `SOMCR`, and `VBTCR1`.
2. Writes `SOMCR = 0x02`, `VBTCR1 = 0x00`, and `SOSCCR = 0x00`.
3. Reads back `SOSCCR.SOSTP` and waits 2.5 seconds.

This is the code as implemented, not a guarantee that every write takes effect. RA4M1 requires `SOMCR.SODRV` to be configured while the sub-clock oscillator is stopped. The observed hardware reports SOSC already running and a low drive setting, so the runtime `SOMCR` write may be ignored. Choose oscillator drive strength from the fitted crystal's electrical specification and configure it while SOSC is stopped.

The `RTC` CLI command reports:

- RTC running state and calendar time.
- SOSC state and `SOMCR.SODRV`.
- The RTC source from `RCR4.RCKSEL`: `0` indicates SUBCLK and `1` indicates LOCO.

Do not use `RCR1` to identify the RA4M1 RTC clock source.

## Project Time Synchronization

The CLI accepts `RTC SYNC YYYY-MM-DD HH:MM:SS`. The supplied [sync_rtc_time.py](sync_rtc_time.py) script sends that command using the PC's current local date and time. See [RTC_SYNC_README.md](RTC_SYNC_README.md) for the script's prerequisites and use.