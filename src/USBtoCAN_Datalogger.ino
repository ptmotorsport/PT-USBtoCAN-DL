/*
 * PT Motorsport AU - USB to CAN DL
 *
 * Original firmware developed by Tessa Shea https://github.com/MythicalSheep
 * while working at PT Motorsport AU.
 *
 * Subsequent development and maintenance:
 * Peter @ PT Motorsport AU
 *
 * Copyright (c) 2025-2026 PT Motorsport AU
 */

// Enable external 32.768kHz crystal for RTC accuracy
#define RTC_CLOCK_SOURCE RTC_CLOCK_SOURCE_SUBCLK

#include <Adafruit_NeoPixel.h>
#include <Arduino_CAN.h>
#include <EEPROM.h>
#include <SdFat.h>
#include <RTC.h>
#include <SPI.h>
#include "cli.h"
#include "log_record.h"
#include "log_policy.h"
#include "frame_queue.h"
#include "can_filter.h"
#include "slcan_bridge.h"
#include "status_config.h"
#include <analog.h>

// NeoPixel variables
int pwrLED = 4;
int recordingLED = 1;
int terminationLED = 2;
int errorLED = 3;
int SDLED = 0;
int neoPixelsPin = 2;
int numNeoPixels = 5;
int neoPixelsDelay = 500;
int neoPixelsBrightness = 25;
boolean txDelay = false;
boolean rxDelay = false;
boolean falseInput = false;
Adafruit_NeoPixel pixels(numNeoPixels, neoPixelsPin, NEO_GRB + NEO_KHZ800);
bool canReady = false;
bool canMessageReceived = false;
unsigned long lastCANReceiveMillis = 0;
bool logWriteCompleted = false;
unsigned long lastLogWriteMillis = 0;
bool logBufferOverflow = false;
int vbattRaw = 0;
float vbattVolts = 0.0f;
ADC_Container vbattADC(0, nullptr);
bool vbattADCReady = false;
bool vbattValid = false;
bool booting = true;
bool terminationEnabled = false;

// Button menu variables
struct Button {
  uint8_t pin;
  bool rawPressed;
  bool pressed;
  bool longHandled;
  unsigned long rawChangedMillis;
  unsigned long pressedMillis;
};
enum ButtonEvent { BUTTON_NONE, BUTTON_SHORT, BUTTON_LONG };
enum MenuItem { MENU_CAN_SPEED, MENU_TERMINATION, MENU_LOGGING, MENU_ITEM_COUNT };
Button button1 = {StatusConfig::BUTTON1_PIN};
Button button2 = {StatusConfig::BUTTON2_PIN};
bool menuActive = false;
uint8_t menuSelection = MENU_CAN_SPEED;

// CAN filter variables
static uint32_t const CAN_FILTER_MASK_STANDARD = 0x1FFC0000;
static uint32_t const CAN_FILTER_MASK_EXTENDED = 0x1FFFFFFF;

// Serial input variables
boolean newData = false;
boolean savvyCAN = false;
const byte NUM_CHARS = 32;
char receivedChars[NUM_CHARS];

// White/blacklist variables
byte listState = 0;
byte filterState = 0;
const int NUM_ROWS = 5;
const int NUM_COLS = 4;
char whitelist[NUM_ROWS][NUM_COLS] = {};
char blacklist[NUM_ROWS][NUM_COLS] = {};
FilterRule whitelistRules[5];
FilterRule blacklistRules[5];

// Timer variables
RTCTime currentTime;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

// SD card variables
SdFat sd;
int fileCount;
FsFile dataFile;
String segmentBase;
uint32_t sdSyncedBytes = 0;
unsigned long segmentStartedMillis = 0;
int fileNum = 0;
extern const int CHIP_SELECT = 10;
uint32_t sdSpiClockMHz = 12;
bool sdStartupReady = false;
unsigned long sdStartupMillis = 0;
unsigned long sdStartupDurationMillis = 0;
uint8_t sdStartupErrorCode = 0;
uint8_t sdStartupErrorData = 0;
extern const uint8_t SD_STARTUP_MAX_ATTEMPTS = 5;
extern const unsigned long SD_STARTUP_RETRY_DELAY_MS = 500;
uint8_t sdStartupAttempts = 0;
uint8_t sdFirstErrorCode = 0;
uint8_t sdFirstErrorData = 0;
bool sdReady = false;
bool sdFaultLatched = false;
unsigned long sdLastServiceMillis = 0;
uint32_t sdRuntimeAttempts = 0;
uint32_t sdDroppedMessages = 0;
uint8_t sdLastErrorCode = 0;
uint8_t sdLastErrorData = 0;
const char *sdLastFailure = "None";
cid_t sdCardIdentity;
bool sdCardIdentityValid = false;
const unsigned long SD_HEALTH_INTERVAL_MS = 1000;
const unsigned long SD_RUNTIME_RETRY_MS = 2000;

// EEPROM variables
int filterIndex = 0;
int fileCountIndex = 1;
int CANSpeedIndex = 2;
int listStateIndex = 3;
int whitelistIndex = 4;
int blacklistIndex = whitelistIndex + NUM_ROWS*NUM_COLS;
// White/blacklists interleave across bytes 4-59
int terminationIndex = 64;

// CANSpeed variables
byte CANSpeed;
int CANSpeedArray[4] = {125000, 250000, 500000, 1000000};

// SD buffer variables
int bufferIndex = 0;
const int MSG_LENGTH = 100;
const int BUFFER_SIZE = 512;
FrameQueue<BUFFER_SIZE> logQueue;
uint32_t sdQueueOverflows = 0;
uint32_t sdQueueHighWater = 0;
uint32_t sdUnconfirmedMessages = 0;
uint32_t sdPendingMessages = 0;
unsigned long sdLastSyncMillis = 0;
uint32_t sdLastSyncMicros = 0;
uint32_t sdMaxSyncMicros = 0;
bool loggingEnabled = true;
bool loggingBeforeBridge = true;

void setup() {
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(pwrLED, pixels.Color(neoPixelsBrightness, 0, neoPixelsBrightness));
  pixels.show();

  // Read & update values from the EEPROM
  filterState = EEPROM.read(filterIndex);
  fileCount = EEPROM.read(fileCountIndex) + 1;
  CANSpeed = EEPROM.read(CANSpeedIndex);
  // Default to 1000 kbps (index 3) if EEPROM is uninitialized (0xFF)
  if (CANSpeed > 3) {
    CANSpeed = 3;  // 1000 kbps
    EEPROM.update(CANSpeedIndex, CANSpeed);
  }
  listState = EEPROM.read(listStateIndex);
  EEPROM.update(fileCountIndex, fileCount);
  pinMode(StatusConfig::TERMINATION_CONTROL_PIN, OUTPUT);
  setTermination(EEPROM.read(terminationIndex) == 1);
  pinMode(StatusConfig::BUTTON1_PIN, INPUT_PULLUP);
  pinMode(StatusConfig::BUTTON2_PIN, INPUT_PULLUP);

  // Initialize Serial early for diagnostics
  Serial.begin(115200);
  delay(500);  // Give serial time to initialize

  // CRITICAL: Initialize external 32.768kHz subclock oscillator BEFORE RTC.begin()
  // According to RA4M1 hardware manual, need to:
  // 1. Configure SOMCR while SOSC is stopped
  // 2. Handle VBTCR1.BPWSWSTP if VBATT not used
  // 3. Clear SOSCCR.SOSTP to enable SOSC
  // 4. Wait stabilization time

  Serial.println(F("\n=== SOSC (32.768kHz Crystal) Initialization ==="));

  // System register base (RA4M1)
  const uint32_t SYSTEM_BASE = 0x4001E000UL;
  const uint32_t SOSCCR_OFFSET = 0x480;  // Subclock Control Register
  const uint32_t SOMCR_OFFSET = 0x481;   // Subclock Mode Control Register
  const uint32_t VBTCR1_OFFSET = 0x41F;  // VBATT Control Register 1

  volatile uint8_t *sosccr = (volatile uint8_t *)(SYSTEM_BASE + SOSCCR_OFFSET);
  volatile uint8_t *somcr = (volatile uint8_t *)(SYSTEM_BASE + SOMCR_OFFSET);
  volatile uint8_t *vbtcr1 = (volatile uint8_t *)(SYSTEM_BASE + VBTCR1_OFFSET);

  // Print initial SOSC status
  Serial.print(F("Initial SOSCCR.SOSTP: "));
  Serial.print((*sosccr & 0x01) ? F("STOPPED (1)") : F("RUNNING (0)"));
  Serial.print(F(" [value=0x"));
  Serial.print(*sosccr, HEX);
  Serial.println(F("]"));

  Serial.print(F("Initial SOMCR.SODRV: 0x"));
  Serial.println((*somcr & 0x03), HEX);

  Serial.print(F("Initial VBTCR1.BPWSWSTP: "));
  Serial.println((*vbtcr1 & 0x01) ? F("STOPPING (1)") : F("OPERATING (0)"));

  // Step 1: Configure SOMCR with appropriate drive strength while SOSC stopped
  // SODRV[1:0]: 00=low, 01=medium-low, 10=medium-high, 11=high
  // For 32.768kHz crystal with typical load, use medium-high (0x2)
  *somcr = 0x02;  // SODRV = 10 (medium-high)
  Serial.println(F("Configured SOMCR.SODRV = 0x02 (medium-high drive)"));

  // Step 2: Handle VBTCR1 if VBATT not used
  // Set BPWSWSTP to indicate power supply switch is stopping VBATT operation
  *vbtcr1 = 0x00;  // BPWSWSTP = 0 (normal operation)
  Serial.println(F("Set VBTCR1.BPWSWSTP = 0 (VBATT operating normally)"));

  // Step 3: Clear SOSCCR.SOSTP to enable the subclock oscillator
  *sosccr = 0x00;  // SOSTP = 0 (enable oscillator)

  // Step 4: Read back to confirm the bit changed
  uint8_t sostp_verify = (*sosccr & 0x01);
  Serial.print(F("Verified SOSCCR.SOSTP after enable: "));
  Serial.println(sostp_verify ? F("FAILED (still 1)") : F("SUCCESS (now 0)"));

  // Step 5: Wait for subclock oscillator stabilization
  // Typical stabilization time for 32.768kHz crystal is ~2.5 seconds
  Serial.println(F("Waiting 2.5s for SOSC stabilization..."));
  delay(2500);

  Serial.println(F("SOSC initialization complete\n"));

  // Start the 'real time' clock + Serial, CAN, & SD communication
  // RTC.begin() will now use the configured SUBCLK source (if RTC.cpp has SUBCLK enabled)
  RTC.begin();

  cli.begin();
  CAN.clearError();
  canReady = CAN.begin(CANSpeedArray[CANSpeed]);

  if (StatusConfig::TERMINATION_SENSE_PIN >= 0) {
    pinMode(StatusConfig::TERMINATION_SENSE_PIN, INPUT);
  }
  pinPeripheral(StatusConfig::VBATT_SENSE_PIN, IOPORT_CFG_ANALOG_ENABLE);
  vbattADC.cfg.resolution = ADC_RESOLUTION_12_BIT;
  vbattADC.cfg.mode = ADC_MODE_SINGLE_SCAN;
  vbattADC.cfg_extend.adc_vref_control = ADC_VREF_CONTROL_AVCC0_AVSS0;
  vbattADC.channel_cfg.scan_mask = 1UL << 3;
  vbattADCReady = R_ADC_Open(&vbattADC.ctrl, &vbattADC.cfg) == FSP_SUCCESS;
  if (vbattADCReady) {
    vbattADCReady = R_ADC_ScanCfg(&vbattADC.ctrl, &vbattADC.channel_cfg) == FSP_SUCCESS;
  }
  sdStartupMillis = millis();
  for (uint8_t attempt = 1; attempt <= SD_STARTUP_MAX_ATTEMPTS; ++attempt) {
    sdStartupAttempts = attempt;
    sdStartupReady = sd.begin(CHIP_SELECT, SD_SCK_MHZ(sdSpiClockMHz));
    sdStartupErrorCode = sd.sdErrorCode();
    sdStartupErrorData = sd.sdErrorData();
    Serial.print(F("SD startup attempt "));
    Serial.print(attempt);
    Serial.print(F("/"));
    Serial.print(SD_STARTUP_MAX_ATTEMPTS);
    Serial.println(sdStartupReady ? F(": OK") : F(": FAILED"));
    if (sdStartupReady) {
      break;
    }
    if (attempt == 1) {
      sdFirstErrorCode = sdStartupErrorCode;
      sdFirstErrorData = sdStartupErrorData;
    }
    sd.initErrorPrint(&Serial);
    if (attempt < SD_STARTUP_MAX_ATTEMPTS) {
      delay(SD_STARTUP_RETRY_DELAY_MS);
    }
  }
  sdStartupDurationMillis = millis() - sdStartupMillis;
  if(sdStartupReady){
    pixels.setPixelColor(SDLED, pixels.Color(0, neoPixelsBrightness, 0));
  } else{
    pixels.setPixelColor(SDLED, pixels.Color(neoPixelsBrightness, 0, 0));
  }
  Serial.print(F("SD startup: "));
  Serial.println(sdStartupReady ? F("OK") : F("FAILED (run SD for diagnostics)"));
  sdReady = sdStartupReady;
  if (sdReady) {
    sdCardIdentityValid = sd.card()->readCID(&sdCardIdentity);
    if (!sdCardIdentityValid) {
      markSDUnavailable("Startup card check");
    }
  }
  sdLastServiceMillis = millis();

  // Update white/blacklist from EEPROM
  updateList(whitelist, whitelistIndex);
  updateList(blacklist, blacklistIndex);
  loadLegacyRules();
  loadSavedFilterRules();

  // Update filter states from EEPROM
  clearFilters();
  booting = false;
  updateStatusLEDs();
}

void loop() {
  logCAN();
  if (savvyCAN) {
    slcanProcess();
  } else {
    cli.process();
  }
  static bool wasBridge = false;
  if (wasBridge && !savvyCAN) {
    canReady = CAN.begin(CANSpeedArray[CANSpeed]);
    loggingEnabled = loggingBeforeBridge;
    sdLastSyncMillis = millis();
  }
  wasBridge = savvyCAN;
  logCAN();
  if (!savvyCAN) {
    serviceSD();
    flushBufferToSD();
  }
  currentMillis = millis();
  handleButtons();

  // Update the NeoPixel values
  if(currentMillis - previousMillis > neoPixelsDelay){
    previousMillis = currentMillis;
    updateStatusLEDs();
  }

}

bool readVBATT() {
  if (!vbattADCReady || R_ADC_ScanStart(&vbattADC.ctrl) != FSP_SUCCESS) {
    return false;
  }
  const unsigned long started = micros();
  adc_status_t status;
  do {
    if (R_ADC_StatusGet(&vbattADC.ctrl, &status) != FSP_SUCCESS || micros() - started >= 2000) {
      return false;
    }
  } while (status.state == ADC_STATE_SCAN_IN_PROGRESS);
  uint16_t raw = 0;
  if (R_ADC_Read(&vbattADC.ctrl, ADC_CHANNEL_3, &raw) != FSP_SUCCESS) {
    return false;
  }
  vbattRaw = raw;
  vbattVolts = vbattRaw * StatusConfig::VBATT_VOLTS_PER_COUNT;
  return true;
}

void updateStatusLEDs() {
  const unsigned long now = millis();
  const uint32_t off = pixels.Color(0, 0, 0);
  const uint32_t red = pixels.Color(neoPixelsBrightness, 0, 0);
  const uint32_t green = pixels.Color(0, neoPixelsBrightness, 0);
  const uint32_t blue = pixels.Color(0, 0, neoPixelsBrightness);
  const uint32_t white = pixels.Color(neoPixelsBrightness, neoPixelsBrightness, neoPixelsBrightness);
  int canErrorCode = 0;
  const bool canFault = !canReady || CAN.isError(canErrorCode);
  const bool receiving = canMessageReceived && now - lastCANReceiveMillis < StatusConfig::CANTIMEOUT;
  const bool recording = bufferIndex > 0 ||
      (logWriteCompleted && now - lastLogWriteMillis < StatusConfig::CANTIMEOUT);
  vbattValid = readVBATT();

  if (menuActive) {
    const uint32_t amber = pixels.Color(neoPixelsBrightness / 2, neoPixelsBrightness / 6, 0);
    // Selected LED shows the current value: CAN 125k red, 250k blue, 500k white, 1000k green
    const uint32_t canSpeedColors[] = {red, blue, white, green};
    for (int i = 0; i < numNeoPixels; i++) {
      pixels.setPixelColor(i, amber);
    }
    if (menuSelection == MENU_CAN_SPEED) {
      pixels.setPixelColor(errorLED, canSpeedColors[CANSpeed]);
    } else if (menuSelection == MENU_TERMINATION) {
      pixels.setPixelColor(terminationLED, terminationEnabled ? green : red);
    } else if (menuSelection == MENU_LOGGING) {
      pixels.setPixelColor(recordingLED, loggingEnabled ? green : red);
    }
    pixels.show();
    return;
  }

  pixels.setPixelColor(SDLED, sdReady ? green : red);
  pixels.setPixelColor(recordingLED, savvyCAN ? off :
      (!sdReady || canFault || logBufferOverflow) ? red : recording ? green : white);
  pixels.setPixelColor(errorLED, canFault ? red : receiving ? green : blue);
  pixels.setPixelColor(terminationLED, terminationEnabled ? white : off);
  if (StatusConfig::TERMINATION_SENSE_PIN >= 0) {
    pixels.setPixelColor(terminationLED,
        digitalRead(StatusConfig::TERMINATION_SENSE_PIN) == StatusConfig::TERMINATION_ACTIVE_LEVEL ? white : off);
  }
  pixels.setPixelColor(pwrLED, booting ? pixels.Color(neoPixelsBrightness, 0, neoPixelsBrightness) : !vbattValid ? off :
      vbattVolts > StatusConfig::VEHICLE_POWER_THRESHOLD_VOLTS ? green : blue);
  pixels.show();
}

void setTermination(bool enabled) {
  terminationEnabled = enabled;
  digitalWrite(StatusConfig::TERMINATION_CONTROL_PIN,
      enabled ? StatusConfig::TERMINATION_ON_LEVEL : !StatusConfig::TERMINATION_ON_LEVEL);
}

ButtonEvent readButton(Button &button) {
  const unsigned long now = millis();
  const bool raw = digitalRead(button.pin) == LOW;
  if (raw != button.rawPressed) {
    button.rawPressed = raw;
    button.rawChangedMillis = now;
  }
  if (raw != button.pressed && now - button.rawChangedMillis >= StatusConfig::BUTTON_DEBOUNCE_MS) {
    button.pressed = raw;
    if (raw) {
      button.pressedMillis = now;
      button.longHandled = false;
    } else if (!button.longHandled) {
      return BUTTON_SHORT;
    }
  }
  if (button.pressed && !button.longHandled && now - button.pressedMillis >= StatusConfig::BUTTON_LONG_PRESS_MS) {
    button.longHandled = true;
    return BUTTON_LONG;
  }
  return BUTTON_NONE;
}

void handleButtons() {
  if (savvyCAN) return;
  const ButtonEvent button1Event = readButton(button1);
  const ButtonEvent button2Event = readButton(button2);
  if (button2Event == BUTTON_LONG) {
    menuActive = !menuActive;
    menuSelection = MENU_CAN_SPEED;
    updateStatusLEDs();
    return;
  }
  if (!menuActive) {
    return;
  }
  if (button1Event != BUTTON_NONE) {
    menuSelection = (menuSelection + 1) % MENU_ITEM_COUNT;
    updateStatusLEDs();
  }
  if (button2Event == BUTTON_SHORT) {
    if (menuSelection == MENU_CAN_SPEED) {
      CANSpeed = (CANSpeed + 1) % 4;
      updateCANSpeed();
    } else if (menuSelection == MENU_TERMINATION) {
      setTermination(!terminationEnabled);
      EEPROM.update(terminationIndex, terminationEnabled);
      updateStatusLEDs();
    } else if (menuSelection == MENU_LOGGING) {
      if (loggingEnabled) {
        stopLogging();
      } else {
        startLogging();
      }
      updateStatusLEDs();
    }
  }
}

void markSDUnavailable(const char *reason) {
  sdLastFailure = reason;
  sdLastErrorCode = sd.sdErrorCode();
  sdLastErrorData = sd.sdErrorData();
  if (sdReady) {
    sdFaultLatched = true;
  }
  sdReady = false;
  sdCardIdentityValid = false;
  sdUnconfirmedMessages += sdPendingMessages;
  sdDroppedMessages += sdPendingMessages;
  sdPendingMessages = 0;
  sdLastServiceMillis = millis();
  updateStatusLEDs();
}

void serviceSD() {
  if (sdFaultLatched) {
    return;
  }
  const unsigned long interval = sdReady ? SD_HEALTH_INTERVAL_MS : SD_RUNTIME_RETRY_MS;
  if (millis() - sdLastServiceMillis < interval) {
    return;
  }
  if (!sdReady) {
    if (savvyCAN) {
      return;
    }
    ++sdRuntimeAttempts;
    if (!sd.begin(CHIP_SELECT, SD_SCK_MHZ(sdSpiClockMHz))) {
      markSDUnavailable("Runtime mount");
      return;
    }
    if (!sd.card()->readCID(&sdCardIdentity)) {
      markSDUnavailable("Recovery card check");
      return;
    }
    sdCardIdentityValid = true;
    sdReady = true;
    updateStatusLEDs();
  } else {
    cid_t identity;
    if (!sd.card()->readCID(&identity)) {
      markSDUnavailable("Card check (removed or communication failed)");
      return;
    }
    if (sdCardIdentityValid && memcmp(&identity, &sdCardIdentity, sizeof(identity)) != 0) {
      markSDUnavailable("Card changed; remount required");
      return;
    }
    sdCardIdentity = identity;
    sdCardIdentityValid = true;
  }
  sdLastServiceMillis = millis();
}

void clearFilters(){
  CAN.end();
  CAN.setFilterMask_Extended(0x0);
  CAN.setFilterMask_Standard(0x0);
  CAN.clearError();
  canMessageReceived = false;
  canReady = CAN.begin(CANSpeedArray[CANSpeed]);
  updateStatusLEDs();
}

bool configureBridgeCAN(uint32_t bitrate, bool listenOnly) {
  if (listenOnly) return false;
  CAN.end();
  CAN.setFilterMask_Extended(0);
  CAN.setFilterMask_Standard(0);
  CAN.clearError();
  canReady = CAN.begin(bitrate);
  return canReady;
}

void closeBridgeCAN() {
  CAN.end();
  canReady = false;
}

bool sendBridgeCAN(uint32_t identifier, bool extended, uint8_t length, const uint8_t *data) {
  if (!canReady || length > 8) return false;
  const CanMsg frame(extended ? CanExtendedId(identifier) : CanStandardId(identifier), length, data);
  return CAN.write(frame) > 0;
}

void enterBridgeMode() {
  loggingBeforeBridge = loggingEnabled;
  loggingEnabled = false;
  while (logQueue.size() && sdReady) flushBufferToSD();
  if (sdReady && dataFile.isOpen()) {
    finishLogSegment();
  }
  ++fileNum;
  savvyCAN = true;
  slcanBegin();
}

void stopLogging() {
  loggingEnabled = false;
  while (logQueue.size() && sdReady) flushBufferToSD();
  if (sdReady && dataFile.isOpen()) {
    finishLogSegment();
  }
}

void startLogging() {
  if (!loggingEnabled) ++fileNum;
  loggingEnabled = true;
}

void setFilters(){
  clearFilters();
}

// Read the white/blacklist from the serial monitor
void readList(char list[NUM_ROWS][NUM_COLS]){
  int serialIndex = 0;
  recvWithStartEndMarkers();
  for(int i = 0; i < NUM_ROWS; i++){
    for(int j = 0; j < NUM_COLS; j++){
      list[i][j] = receivedChars[serialIndex];
      serialIndex++;
    }
  }
}

// Write the white/blacklist to the serial monitor
void writeList(char list[NUM_ROWS][NUM_COLS]){
  txDelay = true;
  for(int i = 0; i < NUM_ROWS; i++){
    for(int j = 0; j < NUM_COLS; j++){
      Serial.print(list[i][j]);
    }
    Serial.print("*");
  }
  Serial.print(">");
}

// Burn the white/blacklist to the EEPROM
void burnList(char list[NUM_ROWS][NUM_COLS], int index){
  int originalIndex = index;
  for(int i = 0; i < NUM_ROWS; i++){
    for(int j = 0; j < NUM_COLS; j++){
      EEPROM.update(index, list[i][j]);
      index++;
    }
    index = index + NUM_COLS;
  }
  updateList(list, originalIndex);
}

// Update the white/blacklist from the EEPROM
void updateList(char list[NUM_ROWS][NUM_COLS], int index){
  for(int i = 0; i < NUM_ROWS; i++){
    for(int j = 0; j < NUM_COLS; j++){
      list[i][j] = EEPROM.read(index);
      index++;
    }
    index = index + NUM_COLS;
  }
}

// Update the CANBus speed
void updateCANSpeed(){
  CAN.end();
  CAN.clearError();
  canMessageReceived = false;
  canReady = CAN.begin(CANSpeedArray[CANSpeed]);
  updateStatusLEDs();
  EEPROM.update(CANSpeedIndex, CANSpeed);
}

// Check if the CAN msg passes the blacklist
void loadLegacyRules() {
  for (int row = 0; row < NUM_ROWS; ++row) {
    for (int kind = 0; kind < 2; ++kind) {
      char value[5] = {};
      memcpy(value, kind ? blacklist[row] : whitelist[row], 4);
      const bool standard = value[3] == '-';
      if (standard) value[3] = '\0';
      char *end = nullptr;
      const uint32_t identifier = strtoul(value, &end, 16);
      FilterRule &rule = kind ? blacklistRules[row] : whitelistRules[row];
      rule.identifier = identifier;
      rule.extended = !standard;
      rule.active = end != value && *end == '\0' && identifier <= (standard ? 0x7FFUL : 0x1FFFFFFFUL);
    }
  }
}

void saveFilterRules(bool blacklistList) {
  const FilterRule *rules = blacklistList ? blacklistRules : whitelistRules;
  const int base = blacklistList ? 160 : 96;
  EEPROM.update(base, 0);
  for (int row = 0; row < NUM_ROWS; ++row) {
    const int offset = base + 1 + row * 5;
    for (int octet = 0; octet < 4; ++octet) {
      EEPROM.update(offset + octet, (rules[row].identifier >> (8 * octet)) & 0xFF);
    }
    EEPROM.update(offset + 4, (rules[row].active ? 1 : 0) | (rules[row].extended ? 2 : 0));
  }
  EEPROM.update(base, 0xA5);
}

void loadSavedFilterRules() {
  for (int kind = 0; kind < 2; ++kind) {
    const int base = kind ? 160 : 96;
    if (EEPROM.read(base) != 0xA5) continue;
    FilterRule *rules = kind ? blacklistRules : whitelistRules;
    for (int row = 0; row < NUM_ROWS; ++row) {
      const int offset = base + 1 + row * 5;
      rules[row].identifier = 0;
      for (int octet = 0; octet < 4; ++octet) {
        rules[row].identifier |= static_cast<uint32_t>(EEPROM.read(offset + octet)) << (8 * octet);
      }
      const uint8_t flags = EEPROM.read(offset + 4);
      rules[row].extended = (flags & 2) != 0;
      rules[row].active = (flags & 1) && flags <= 3 &&
          rules[row].identifier <= (rules[row].extended ? 0x1FFFFFFFUL : 0x7FFUL);
    }
  }
}

bool checkBlacklist(CanMsg msg){
  const bool extended = msg.isExtendedId();
  const uint32_t identifier = extended ? msg.getExtendedId() : msg.getStandardId();
  const FilterRule *rules = listState ? whitelistRules : blacklistRules;
  for (int row = 0; row < NUM_ROWS; ++row) {
    if (matchesRule(rules[row], identifier, extended)) return listState != 0;
  }
  return listState == 0;
}

// Log new CAN message to the char buffer
void logCAN(){
  for (unsigned int drained = 0; drained < 32; ++drained) {
    noInterrupts();
    if (!CAN.available()) {
      interrupts();
      break;
    }
    CanMsg const MSG = CAN.read();
    interrupts();
    lastCANReceiveMillis = millis();
    canMessageReceived = true;

    bool filterPass = true;
    if(filterState == 1 && !savvyCAN){
      filterPass = checkBlacklist(MSG);
    }

    if(filterPass && !savvyCAN && loggingEnabled){
      LoggedFrame frame = {};
      frame.timestampMillis = lastCANReceiveMillis;
      frame.identifier = MSG.isExtendedId() ? MSG.getExtendedId() : MSG.getStandardId();
      frame.extended = MSG.isExtendedId();
      frame.length = MSG.data_length;
      memcpy(frame.data, MSG.data, sizeof(frame.data));
      if (!logQueue.push(frame)) {
        ++sdDroppedMessages;
        ++sdQueueOverflows;
        logBufferOverflow = true;
        continue;
      }
      bufferIndex = logQueue.size();
      if (logQueue.size() > sdQueueHighWater) sdQueueHighWater = logQueue.size();
    } else if (savvyCAN){
      if (slcanCanRead()) {
        slcanReceive(MSG.isExtendedId() ? MSG.getExtendedId() : MSG.getStandardId(),
            MSG.isExtendedId(), MSG.data_length, MSG.data);
      }
    }
  }
}

bool syncLogSegment() {
  const unsigned long started = micros();
  const bool confirmed = LogPolicy::sync(dataFile, sdSyncedBytes);
  sdLastSyncMicros = micros() - started;
  if (sdLastSyncMicros > sdMaxSyncMicros) sdMaxSyncMicros = sdLastSyncMicros;
  if (!confirmed) {
    markSDUnavailable("Log sync (unconfirmed rows)");
    return false;
  }
  sdPendingMessages = 0;
  sdLastSyncMillis = millis();
  logWriteCompleted = true;
  lastLogWriteMillis = millis();
  return true;
}

bool finishLogSegment() {
  if (!syncLogSegment()) return false;
  if (!dataFile.close()) {
    markSDUnavailable("Segment close");
    return false;
  }
  return true;
}

void flushBufferToSD() {
  if (!sdReady || savvyCAN) {
    return;
  }
  if (bufferIndex == 0 && sdPendingMessages == 0) {
    return;
  }
  if (!dataFile.isOpen()) {
    segmentBase = String(fileCount) + "_" + String(fileNum);
    RTCTime logTime;
    if (RTC.isRunning() && RTC.getTime(logTime)) {
      const int year = logTime.getYear();
      const int month = static_cast<int>(logTime.getMonth()) + 1;
      const int day = logTime.getDayOfMonth();
      const int hour = logTime.getHour();
      const int minute = logTime.getMinutes();
      const int second = logTime.getSeconds();
      const int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      const bool leapYear = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
      if (year > 2000 && year <= 2099 && month >= 1 && month <= 12 &&
          day >= 1 && day <= daysInMonth[month - 1] + (month == 2 && leapYear ? 1 : 0) &&
          hour >= 0 && hour <= 23 && minute >= 0 && minute <= 59 && second >= 0 && second <= 59) {
        char timestamp[21];
        snprintf(timestamp, sizeof(timestamp), "_%04d-%02d-%02d_%02d-%02d-%02d",
            year, month, day, hour, minute, second);
        segmentBase += timestamp;
      }
    }
    const String requestedBase = segmentBase;
    unsigned int suffix = 0;
    while (sd.exists((segmentBase + ".part").c_str()) || sd.exists((segmentBase + ".idx").c_str()) ||
        sd.exists((segmentBase + ".csv").c_str())) {
      segmentBase = requestedBase + "_" + String(++suffix);
    }
    if (!dataFile.open((segmentBase + ".csv").c_str(), O_WRONLY | O_CREAT | O_EXCL)) {
      markSDUnavailable("Log file open");
      return;
    }
    sdSyncedBytes = 0;
    segmentStartedMillis = millis();
    const char *header = "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8";
    if (dataFile.println(header) != strlen(header) + 2 || !syncLogSegment()) {
      if (sdReady) markSDUnavailable("Segment header");
      return;
    }
  }
  // Flush buffer to the SD card
  for (unsigned int written = 0; written < 8 && logQueue.size(); ++written) {
    const LoggedFrame &frame = logQueue.front();
    char line[MSG_LENGTH];
    if (!formatLogRecord(line, sizeof(line), frame.timestampMillis, frame.identifier,
        frame.extended, frame.length, frame.data)) {
      ++sdDroppedMessages;
      logQueue.pop();
      bufferIndex = logQueue.size();
      continue;
    }
    if (LogPolicy::needsRotation(dataFile.curPosition(), strlen(line) + 2,
        millis() - segmentStartedMillis)) {
      if (finishLogSegment()) ++fileNum;
      return;
    }
    ++sdPendingMessages;
    if (dataFile.println(line) != strlen(line) + 2 || dataFile.getWriteError()) {
      logQueue.pop();
      bufferIndex = logQueue.size();
      markSDUnavailable("Log write (unconfirmed rows)");
      return;
    }
    logQueue.pop();
    bufferIndex = logQueue.size();
    logCAN();
  }
  if (sdPendingMessages && millis() - sdLastSyncMillis >= LogPolicy::SyncMillis) {
    if (!syncLogSegment()) return;
  }
  if (logQueue.size() < BUFFER_SIZE) logBufferOverflow = false;
}

// Receive data from the serial monitor bracketed by '<>'
// Credit: https://forum.arduino.cc/t/serial-input-basics-updated/382007
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= NUM_CHARS) {
          ndx = NUM_CHARS - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
  newData = false;
}