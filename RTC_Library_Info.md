# Arduino RTC Library for UNO R4 Minima (RA4M1)

## Library Information
- **Library**: Arduino_RTC (built into Renesas RA framework)
- **Version**: 1.0
- **Location**: `.platformio/packages/framework-arduinorenesas-uno/libraries/RTC/`
- **Framework**: renesas-ra@1.7.0
- **Header**: `#include <RTC.h>`

---

## Available RTClock Class Methods

### Initialization
```cpp
bool begin()
```
Initializes the RTC hardware. Opens the RTC controller and configures the clock source.

---

### Time Management
```cpp
bool getTime(RTCTime &t)
```
Reads the current time from the RTC and stores it in the RTCTime object.

```cpp
bool setTime(RTCTime &t)
```
Sets the current time in the RTC (requires RTC to be initialized).

```cpp
bool setTimeIfNotRunning(RTCTime &t)
```
Sets the time only if the RTC is not currently running. Useful to avoid overwriting persistent time on resets.

```cpp
bool isRunning()
```
Returns true if the RTC counter is currently running.

---

### Alarm & Periodic Interrupts
```cpp
bool setAlarm(RTCTime &t, AlarmMatch &m)
```
Sets an alarm without a callback function. Requires manual polling to check if alarm triggered.

```cpp
bool setAlarmCallback(rtc_cbk_t fnc, RTCTime &t, AlarmMatch &m)
```
Sets an alarm with a callback function that fires when the alarm condition is met.
- `fnc`: Function pointer to call when alarm triggers
- `t`: RTCTime object with the alarm time
- `m`: AlarmMatch object specifying which time fields to match

```cpp
bool setPeriodicCallback(rtc_cbk_t fnc, Period p)
```
Sets up a periodic interrupt callback at regular intervals.
- `fnc`: Function pointer to call at each period
- `p`: Period enum value (ONCE_EVERY_2_SEC through N256_TIMES_EVERY_SEC)

---

## RTCTime Class - Time Structure

### Constructors
```cpp
RTCTime()                           // Defaults to 2000-01-01 00:00:00
RTCTime(time_t t)                   // From Unix timestamp
RTCTime(struct tm &t)               // From C standard tm struct
RTCTime(int day, Month m, int year, 
        int hours, int minutes, int seconds,
        DayOfWeek dow, SaveLight sl)  // Full constructor
```

### Setters (all return bool - true if valid)
```cpp
bool setDayOfMonth(int day)         // 1-31
bool setMonthOfYear(Month m)        // Month::JANUARY through DECEMBER
bool setYear(int year)              // e.g., 2024 or 124 (for 2024)
bool setHour(int hour)              // 0-23
bool setMinute(int minute)          // 0-59
bool setSecond(int second)          // 0-59
bool setDayOfWeek(DayOfWeek d)      // MONDAY-SUNDAY
bool setSaveLight(SaveLight sl)     // SAVING_TIME_INACTIVE or SAVING_TIME_ACTIVE
bool setUnixTime(time_t time)       // Set from Unix timestamp
void setTM(struct tm &t)            // Set from tm struct
```

### Getters
```cpp
int getDayOfMonth() const
Month getMonth() const
int getYear() const
int getHour() const
int getMinutes() const
int getSeconds() const
DayOfWeek getDayOfWeek() const
time_t getUnixTime()                // Get as Unix timestamp
struct tm getTmTime()               // Get as tm struct
String toString() const             // ISO 8601: "YYYY-MM-DDTHH:MM:SS"
```

### Enums
```cpp
enum class Month {
    JANUARY = 0, FEBRUARY, MARCH, APRIL, MAY, JUNE,
    JULY, AUGUST, SEPTEMBER, OCTOBER, NOVEMBER, DECEMBER
};

enum class DayOfWeek {
    MONDAY = 1, TUESDAY = 2, WEDNESDAY = 3, THURSDAY = 4,
    FRIDAY = 5, SATURDAY = 6, SUNDAY = 0
};

enum class SaveLight {
    SAVING_TIME_INACTIVE = 0,
    SAVING_TIME_ACTIVE = 1
};

enum class Period {
    ONCE_EVERY_2_SEC,
    ONCE_EVERY_1_SEC,
    N2_TIMES_EVERY_SEC,
    N4_TIMES_EVERY_SEC,
    N8_TIMES_EVERY_SEC,
    N16_TIMES_EVERY_SEC,
    N32_TIMES_EVERY_SEC,
    N64_TIMES_EVERY_SEC,
    N128_TIMES_EVERY_SEC,
    N256_TIMES_EVERY_SEC
};
```

---

## AlarmMatch Class

Used to specify which time fields trigger an alarm:

```cpp
void addMatchSecond()
void addMatchMinute()
void addMatchHour()
void addMatchDay()
void addMatchMonth()
void addMatchYear()
void addMatchDayOfWeek()

void removeMatchSecond()
void removeMatchMinute()
// ... etc for all fields

bool isMatchingSecond() const
// ... etc for all fields
```

**Example**: Match on seconds = 35
```cpp
AlarmMatch am;
am.addMatchSecond();
RTCTime alarmTime;
alarmTime.setSecond(35);
RTC.setAlarmCallback(alarm_callback, alarmTime, am);
```

---

## External Crystal/Subclock Configuration for RA4M1

### ⚠️ IMPORTANT: `setClockSource()` Method Does NOT Exist

**Your code contains an error:**
```cpp
RTC.setClockSource(CLOCK_SOURCE_SUBCLOCK);  // ❌ This method does not exist!
```

The RTClock class does NOT have a `setClockSource()` method. The constant `CLOCK_SOURCE_SUBCLOCK` also does not exist.

### Available Clock Sources (in FSP Driver)
From `r_rtc_api.h`, the Renesas FSP layer defines:
```c
typedef enum e_rtc_count_source {
    RTC_CLOCK_SOURCE_SUBCLK = 0,  // Sub-clock oscillator (32.768 kHz external crystal)
    RTC_CLOCK_SOURCE_LOCO   = 1   // Low power On Chip Oscillator
} rtc_clock_source_t;
```

### How to Enable External Subclock

**Method 1: Modify RTC Configuration (RECOMMENDED)**

Edit the RTC.cpp file in your Arduino framework installation and change:
```cpp
#ifndef RTC_CLOCK_SOURCE
#define RTC_CLOCK_SOURCE RTC_CLOCK_SOURCE_LOCO  // Change this to:
#endif
```

To:
```cpp
#ifndef RTC_CLOCK_SOURCE
#define RTC_CLOCK_SOURCE RTC_CLOCK_SOURCE_SUBCLK
#endif
```

Or add to your sketch before `#include <RTC.h>`:
```cpp
#define RTC_CLOCK_SOURCE RTC_CLOCK_SOURCE_SUBCLK
#include <RTC.h>
```

**Method 2: Use Renesas FSP Driver Directly**

If you need runtime clock source switching, use the FSP driver API:
```cpp
#include "r_rtc_api.h"

extern rtc_instance_ctrl_t rtc_ctrl;
extern rtc_cfg_t rtc_cfg;

// Change clock source before opening RTC
rtc_cfg.clock_source = RTC_CLOCK_SOURCE_SUBCLK;
R_RTC_Open(&rtc_ctrl, &rtc_cfg);
R_RTC_ClockSourceSet(&rtc_ctrl);
```

### Hardware Considerations for RA4M1 (UNO R4 Minima)

- **External Subclock**: 32.768 kHz crystal (±20 ppm recommended for accurate timekeeping)
- **VRTC Pin**: Maintains RTC even when main MCU is powered off (requires backup power)
- **Accuracy**:
  - Subclock (SUBCLK): ±2% typical (with external crystal)
  - LOCO: ±5% typical

### Example: Proper RTC Configuration
```cpp
#define RTC_CLOCK_SOURCE RTC_CLOCK_SOURCE_SUBCLK

#include <RTC.h>

void setup() {
  Serial.begin(115200);
  
  // Initialize RTC
  if (RTC.begin()) {
    Serial.println("RTC initialized with subclock");
  } else {
    Serial.println("RTC initialization failed");
  }
  
  // Set time if not running
  RTCTime initialTime(21, Month::SEPTEMBER, 2024, 10, 30, 0, 
                      DayOfWeek::FRIDAY, SaveLight::SAVING_TIME_INACTIVE);
  RTC.setTimeIfNotRunning(initialTime);
}

void loop() {
  RTCTime now;
  if (RTC.getTime(now)) {
    Serial.println(now.toString());
  }
  delay(1000);
}
```

---

## Example: Using Alarms

```cpp
#include <RTC.h>

void alarmCallback() {
  Serial.println("ALARM TRIGGERED!");
}

void setup() {
  Serial.begin(115200);
  RTC.begin();
  
  // Create alarm for 14:30:00 on any day
  RTCTime alarmTime;
  alarmTime.setHour(14);
  alarmTime.setMinute(30);
  alarmTime.setSecond(0);
  
  AlarmMatch match;
  match.addMatchHour();
  match.addMatchMinute();
  match.addMatchSecond();
  
  RTC.setAlarmCallback(alarmCallback, alarmTime, match);
}

void loop() {
  // Main code here
}
```

---

## Example: Using Periodic Callbacks

```cpp
#include <RTC.h>

void periodicCallback() {
  Serial.println("Called every 2 seconds");
}

void setup() {
  Serial.begin(115200);
  RTC.begin();
  RTC.setPeriodicCallback(periodicCallback, Period::ONCE_EVERY_2_SEC);
}

void loop() {
  // Callback fires independently
}
```

---

## Documentation Links

- Full UNO R4 RTC Tutorial: https://docs.arduino.cc/tutorials/uno-r4-wifi/rtc
- Arduino RTC Library Source: https://github.com/arduino/ArduinoCore-renesas/tree/master/libraries/RTC
- Renesas RA4M1 RTC Documentation: https://www.renesas.com/us/en/document/mcu-datasheet

---

## Summary of Issues Found in Your Code

1. **`RTC.setClockSource(CLOCK_SOURCE_SUBCLOCK)`** - This method doesn't exist
2. **`CLOCK_SOURCE_SUBCLOCK`** - This constant doesn't exist (correct name is `RTC_CLOCK_SOURCE_SUBCLK`)
3. **Solution**: Use `#define RTC_CLOCK_SOURCE RTC_CLOCK_SOURCE_SUBCLK` before including `RTC.h`

This configuration enables the external 32.768 kHz crystal for accurate timekeeping on the RA4M1 microcontroller.
