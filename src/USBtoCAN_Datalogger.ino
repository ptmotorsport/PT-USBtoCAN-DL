/*
 * PT Motorsport AU - USB to CAN DL
 *
 * Original firmware developed by Tessa Shea https://github.com/MythicalSheep
 * while working at PT Motorsport AU.
 *
 * Subsequent development and maintenance:
 * PT Motorsport AU
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

// NeoPixel variables
int pwrLED = 0;
int txLED = 1;
int rxLED = 2;
int errorLED = 3;
int SDLED = 4;
int neoPixelsPin = 3;
int numNeoPixels = 5;
int neoPixelsDelay = 500;
int neoPixelsBrightness = 25;
boolean txDelay = false;
boolean rxDelay = false;
boolean falseInput = false;
Adafruit_NeoPixel pixels(numNeoPixels, neoPixelsPin, NEO_GRB + NEO_KHZ800);

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

// Timer variables
RTCTime currentTime;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

// SD card variables
SdFat sd;
int fileCount;
FsFile dataFile;
int fileNum = 0;
const int CHIP_SELECT = 10;

// EEPROM variables
int filterIndex = 0;
int fileCountIndex = 1;
int CANSpeedIndex = 2;
int listStateIndex = 3;
int whitelistIndex = 4;
int blacklistIndex = whitelistIndex + NUM_ROWS*NUM_COLS;

// CANSpeed variables
byte CANSpeed;
int CANSpeedArray[4] = {125000, 250000, 500000, 1000000};

// SD buffer variables
int bufferIndex = 0;
const int MSG_LENGTH = 100;
const int BUFFER_SIZE = 150;
char msgBuffer[BUFFER_SIZE][MSG_LENGTH];

void setup() {
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
  if(CAN.begin(CANSpeedArray[CANSpeed])){
    pixels.setPixelColor(errorLED, pixels.Color(0, 0, 0));
  } else{
    pixels.setPixelColor(errorLED, pixels.Color(neoPixelsBrightness, 0, 0));
  }

  // Start the NeoPixels
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(pwrLED, pixels.Color(0, neoPixelsBrightness, 0));
  if(sd.begin(CHIP_SELECT, SD_SCK_MHZ(50))){
    pixels.setPixelColor(SDLED, pixels.Color(0, neoPixelsBrightness, 0));
  } else{
    pixels.setPixelColor(SDLED, pixels.Color(neoPixelsBrightness, 0, 0));
  }

  // Update white/blacklist from EEPROM
  updateList(whitelist, whitelistIndex);
  updateList(blacklist, blacklistIndex);

  // Update filter states from EEPROM
  if(listState == 1 && filterState == 1){
    setFilters();
  } else{
    clearFilters();
  }
}

void loop() {
  // Get the current time & log available CAN messages
  RTC.getTime(currentTime);
  currentMillis = millis();
  logCAN();

  // Update the NeoPixel values
  if(currentMillis - previousMillis > neoPixelsDelay){
    if(txDelay){
      pixels.setPixelColor(txLED, pixels.Color(0, neoPixelsBrightness, 0));
    } else{
      pixels.setPixelColor(txLED, pixels.Color(0, 0, 0));
    }
    if(rxDelay){
      pixels.setPixelColor(rxLED, pixels.Color(0, neoPixelsBrightness, 0));
    } else{
      pixels.setPixelColor(rxLED, pixels.Color(0, 0, 0));
    }
    txDelay = false;
    rxDelay = false;
    previousMillis = currentMillis;
    if(Serial.dtr()){} // SavvyCAN check
    pixels.show();
  }

  // Check for serial commands from CLI
  cli.process();
}

void clearFilters(){
  CAN.end();
  CAN.setFilterMask_Extended(0x0);
  CAN.setFilterMask_Standard(0x0);
  if(CAN.begin(CANSpeedArray[CANSpeed])){
    pixels.setPixelColor(errorLED, pixels.Color(0, 0, 0));
  } else{
    pixels.setPixelColor(errorLED, pixels.Color(neoPixelsBrightness, 0, 0));
  }
}

void setFilters(){
  CAN.end();
  int numMailboxes = 5;
  CAN.setFilterMask_Extended(CAN_FILTER_MASK_EXTENDED);
  CAN.setFilterMask_Standard(CAN_FILTER_MASK_STANDARD);

  // Clear previous filters
  for (int mailbox = 0; mailbox < numMailboxes; mailbox++) {
    CAN.setFilterId_Extended(mailbox, 0);
  }
  for (int mailbox = 0; mailbox < numMailboxes; mailbox++) {
    CAN.setFilterId_Standard(mailbox, 0);
  }

  // Set new filters
  int standardIndex = 0;
  int extendedIndex = 0;
  char standardArray[4];
  char extendedArray[5];
  for (int mailbox = 0; mailbox < numMailboxes; mailbox++){
    if(whitelist[mailbox][3] == '-'){
      snprintf(standardArray, sizeof(standardArray), "%c%c%c", whitelist[mailbox][0], whitelist[mailbox][1], whitelist[mailbox][2]);
      standardArray[3] = '\0'; // Null-terminate
      uint32_t canId = strtoul(standardArray, NULL, 16);
      CAN.setFilterId_Standard(standardIndex, canId);
      standardIndex++;
    } else {
      snprintf(extendedArray, sizeof(extendedArray), "%c%c%c%c", whitelist[mailbox][0], whitelist[mailbox][1], whitelist[mailbox][2], whitelist[mailbox][3]);
      extendedArray[4] = '\0'; // Null-terminate
      uint32_t canId = strtoul(extendedArray, NULL, 16);
      CAN.setFilterId_Extended(extendedIndex, canId);
      extendedIndex++;
    }
  }
  if(CAN.begin(CANSpeedArray[CANSpeed])){
    pixels.setPixelColor(errorLED, pixels.Color(0, 0, 0));
  } else{
    pixels.setPixelColor(errorLED, pixels.Color(neoPixelsBrightness, 0, 0));
  }
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
  if(CAN.begin(CANSpeedArray[CANSpeed])){
    pixels.setPixelColor(errorLED, pixels.Color(0, 0, 0));
  } else{
    pixels.setPixelColor(errorLED, pixels.Color(neoPixelsBrightness, 0, 0));
  }
  EEPROM.update(CANSpeedIndex, CANSpeed);
}

// Check if the CAN msg passes the blacklist
bool checkBlacklist(CanMsg msg){
  bool filterPass = true;
  char msgIdStr[NUM_COLS + 1];
  snprintf(msgIdStr, sizeof(msgIdStr), "%03X", msg.id);
  for(int i = 0; i < NUM_ROWS; i++) {
    bool match = true;
    if(blacklist[i][3] == '-'){
      for(int j = 0; j < NUM_COLS - 1; j++) {
        if(blacklist[i][j] != msgIdStr[j]) {
          match = false;
          break;
        }
      }
    } else {
      for(int j = 0; j < NUM_COLS; j++) {
        if(blacklist[i][j] != msgIdStr[j]) {
          match = false;
          break;
        }
      }
    }
    if(match){
      filterPass = false;
      break;
    } else{
    }
  }
  return filterPass;
}

// Log new CAN message to the char buffer
void logCAN(){
  if (CAN.available()){
    CanMsg const MSG = CAN.read();

    bool filterPass = true;
    if(filterState == 1 && listState == 0){
      filterPass = checkBlacklist(MSG);
    }

    if(filterPass && !savvyCAN){
      char line[MSG_LENGTH];
      int len = snprintf(line, MSG_LENGTH, "%lu,%03X,%d,Rx,0,%d", currentMillis, MSG.id, MSG.isExtendedId(), MSG.data_length);
      for (int i = 0; i < byte(MSG.data_length); i++){
        if (i == 0) {
          len += snprintf(line + len, MSG_LENGTH - len, ",%02X", MSG.data[i]);
        } else {
          len += snprintf(line + len, MSG_LENGTH - len, ",%02X", MSG.data[i]);
        }
      }
      strncpy(msgBuffer[bufferIndex], line, MSG_LENGTH);
      msgBuffer[bufferIndex][MSG_LENGTH-1] = '\0';
      if (++bufferIndex >= BUFFER_SIZE){
        flushBufferToSD();
      }
    } else if (savvyCAN){
      Serial.print("t");
      Serial.print(MSG.id, HEX);
      Serial.print(MSG.data_length, HEX);

      for(int i = 0; i < byte(MSG.data_length); i++){
        if(byte(MSG.data[i]) < 0x10){
          Serial.print(0);
        }
        Serial.print(MSG.data[i], HEX);
      }
      Serial.print("\r");
    }
  }
}

// Flush the contents of the char buffer to the SD card
void flushBufferToSD() {
  // Create and open a new file
  String tempFileName = String(fileCount) + "_" + String(fileNum) + ".txt";
  if (!dataFile.isOpen()) {
    dataFile = sd.open(tempFileName, O_WRONLY | O_CREAT | O_APPEND);
  }
  // Print file header
  if(dataFile.size() == 0){
    dataFile.println("Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8");
  }
  // Flush buffer to the SD card
  for (int i = 0; i < bufferIndex; i++) {
    dataFile.println(msgBuffer[i]);
  }
  dataFile.sync();
  bufferIndex = 0;
  // Check if file size exceeds maximum
  if(dataFile.size() > 100000000){
    fileNum++;
  }
  dataFile.close();
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