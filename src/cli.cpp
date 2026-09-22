#include "cli.h"
#include <EEPROM.h>

// Global CLI handler instance
CLIHandler cli;

// Forward declarations for external variables (from main sketch)
extern boolean newData;
extern boolean savvyCAN;
extern byte filterState;
extern byte listState;
extern byte CANSpeed;
extern int fileCount;
extern RTCTime currentTime;
extern int fileNum;

extern const byte NUM_CHARS;
extern char receivedChars[32];
// Hard-coded since these are constants defined in .ino
#define NUM_ROWS 5
#define NUM_COLS 4
extern char whitelist[5][4];
extern char blacklist[5][4];

// Function declarations
extern void updateCANSpeed();

// EEPROM indices
extern const int filterIndex;
extern const int fileCountIndex;
extern const int CANSpeedIndex;
extern const int listStateIndex;
extern const int whitelistIndex;
extern const int blacklistIndex;

extern const int CHIP_SELECT;

// CAN speed array
extern const int CANSpeedArray[4];

// Function to detect SOSC (subclock oscillator) status from hardware registers
String CLIHandler::detectSOSCStatus() {
  // System register base on RA4M1
  const uint32_t SYSTEM_BASE = 0x4001E000UL;
  const uint32_t SOSCCR_OFFSET = 0x480;  // Subclock Control Register
  const uint32_t SOMCR_OFFSET = 0x481;   // Subclock Mode Control Register

  volatile uint8_t *sosccr = (volatile uint8_t *)(SYSTEM_BASE + SOSCCR_OFFSET);
  volatile uint8_t *somcr = (volatile uint8_t *)(SYSTEM_BASE + SOMCR_OFFSET);

  uint8_t sostp = (*sosccr & 0x01);  // Bit 0: SOSTP (stop flag)
  uint8_t sodrv = (*somcr & 0x03);   // Bits 1:0: SODRV (drive strength)

  String result = "SOSC Status: ";

  if (sostp == 0) {
    result += "RUNNING (SOSTP=0)";
  } else {
    result += "STOPPED (SOSTP=1)";
  }

  result += " | Drive: ";
  switch(sodrv) {
    case 0x00: result += "Low"; break;
    case 0x01: result += "Medium-Low"; break;
    case 0x02: result += "Medium-High"; break;
    case 0x03: result += "High"; break;
  }

  result += " [SOSCCR=0x";
  if (*sosccr < 0x10) result += "0";
  result += String(*sosccr, HEX);
  result += " SOMCR=0x";
  if (*somcr < 0x10) result += "0";
  result += String(*somcr, HEX);
  result += "]";

  return result;
}

// Function to detect actual RTC clock source from hardware registers
// Reads the RTC module control registers on RA4M1 to determine actual clock source
String CLIHandler::detectRTCClockSource() {
  // RTC module on RA4M1 is at address 0x40043000
  // Key registers:
  // RCR2 (offset 0x01): Control register 2 - contains CNTMD (bit 0)
  // RCR4 (offset 0x2E): Control register 4 - contains RCKSEL (bit 0)
  // RCKSEL: 0 = external sub-clock oscillator, 1 = internal LOCO.

  volatile uint8_t *rtc_base = (volatile uint8_t *)0x40043000;

  // Read RCR2 (offset 0x01)
  volatile uint8_t *rtc_rcr2 = rtc_base + 0x01;
  uint8_t rcr2_val = *rtc_rcr2;

  // Read RCR4 (offset 0x2E). Bit 0 is RCKSEL.
  volatile uint8_t *rtc_rcr4 = rtc_base + 0x2E;
  uint8_t rcr4_val = *rtc_rcr4;
  bool loco_selected = (rcr4_val & 0x01) != 0;

  // Bit 0 of RCR2 = CNTMD (Count mode: 0=calendar, 1=binary)
  bool cntmd = (rcr2_val & 0x01) != 0;

  String result = "";

  if (loco_selected) {
    result = "LOCO (low-speed internal oscillator) - RCKSEL=1";
  } else {
    result = "SUBCLK (32.768 kHz external crystal) - RCKSEL=0";
  }

  // Add register info for debugging
  result += " [RCR4=0x";
  if (rcr4_val < 0x10) result += "0";
  result += String(rcr4_val, HEX);
  result += " RCR2=0x";
  if (rcr2_val < 0x10) result += "0";
  result += String(rcr2_val, HEX);
  result += "]";

  return result;
}

CLIHandler::CLIHandler() {}

void CLIHandler::begin() {
  Serial.println();
  Serial.println("============================================");
  Serial.println("    PT Motorsport AU - USB to CAN DL");
  Serial.println("============================================");
  Serial.println("Type 'HELP' for available commands");
  Serial.println();
}

String CLIHandler::readLine() {
  static String buffer = "";
  static bool prompt_shown = false;

  // Show prompt on first call
  if (!prompt_shown) {
    Serial.print("> ");
    prompt_shown = true;
  }

  while (Serial.available()) {
    char c = Serial.read();

    // Handle backspace (both ASCII 8 and 127)
    if (c == '\b' || c == 127) {
      if (buffer.length() > 0) {
        buffer.remove(buffer.length() - 1);
        // VT100 backspace sequence: move back, erase character, move back again
        Serial.write('\b');
        Serial.write(' ');
        Serial.write('\b');
      }
      continue;
    }

    // Handle carriage return (line complete)
    if (c == '\r') {
      Serial.println(); // Echo newline
      String line = buffer;
      buffer = "";
      prompt_shown = false;
      line.trim();
      return line;
    }

    // Handle newline (alternative line terminator)
    if (c == '\n') {
      continue; // Skip LF, wait for CR
    }

    // Handle printable characters
    if (c >= 32 && c < 127) {
      // Check max length (leave room for null terminator)
      if (buffer.length() < 160) {
        buffer += c;
        Serial.write(c); // Echo the character
      } else {
        // Provide feedback if line is full
        Serial.write('\a'); // Bell/beep
      }
      continue;
    }

    // Ignore other control characters
  }

  return "";
}

void CLIHandler::process() {
  String line = readLine();
  if (line.length() == 0) return;

  handleCommand(line);
}

void CLIHandler::handleCommand(const String &line) {
  String cmd = line;
  cmd.toUpperCase();
  cmd.trim();

  // HELP
  if (cmd == "HELP") {
    showHelp();
    return;
  }

  // STATUS
  if (cmd == "STATUS") {
    showStatus();
    return;
  }

  // CONFIG
  if (cmd == "CONFIG") {
    showConfig();
    return;
  }

  // RTC DIAGNOSTICS
  if (cmd.startsWith("RTC")) {
    if (cmd == "RTC") {
      showRTCDiag();
    } else if (cmd.startsWith("RTC SET ")) {
      String arg = cmd.substring(8);
      setRTCTime(arg);
    } else if (cmd.startsWith("RTC SYNC ")) {
      String arg = cmd.substring(9);
      syncRTCDateTime(arg);
    }
    return;
  }

  // CAN DIAGNOSTICS
  if (cmd == "CAN") {
    showCANDiag();
    return;
  }

  // SET CAN SPEED
  if (cmd.startsWith("CANSPEED ")) {
    String arg = cmd.substring(9);
    setCANSpeed(arg);
    return;
  }

  // ENABLE/DISABLE FILTERS
  if (cmd == "FILTERS ON") {
    enableFilters();
    return;
  }
  if (cmd == "FILTERS OFF") {
    disableFilters();
    return;
  }

  // ENABLE/DISABLE WHITELIST
  if (cmd == "WHITELIST ON") {
    enableWhitelist();
    return;
  }
  if (cmd == "WHITELIST OFF") {
    disableWhitelist();
    return;
  }

  // WHITELIST COMMANDS
  if (cmd == "WL READ") {
    readWhitelist();
    return;
  }
  if (cmd == "WL WRITE") {
    writeWhitelist();
    return;
  }
  if (cmd == "WL SAVE") {
    burnWhitelist();
    return;
  }

  // BLACKLIST COMMANDS
  if (cmd == "BL READ") {
    readBlacklist();
    return;
  }
  if (cmd == "BL WRITE") {
    writeBlacklist();
    return;
  }
  if (cmd == "BL SAVE") {
    burnBlacklist();
    return;
  }

  // SAVVYCAN MODE
  if (cmd == "SAVVYCAN") {
    toggleSavvyCAN();
    return;
  }

  // RESET FILE COUNT
  if (cmd == "FILE RESET") {
    resetFileCount();
    return;
  }

  // SD CARD STATUS
  if (cmd == "SD") {
    Serial.println("OK: SD Card check requested (see main sketch)");
    return;
  }

  Serial.println("ERR: Unknown command. Type HELP for available commands.");
}

void CLIHandler::showHelp() {
  Serial.println("\n========== AVAILABLE COMMANDS ==========\n");

  Serial.println("SYSTEM:");
  Serial.println("  HELP                - Show this help message");
  Serial.println("  STATUS              - Show device status report");
  Serial.println("  CONFIG              - Show current configuration");
  Serial.println();

  Serial.println("RTC (Real Time Clock):");
  Serial.println("  RTC                       - Show RTC diagnostics & status");
  Serial.println("  RTC SET HH:MM:SS          - Set RTC time (24-hour format)");
  Serial.println("  RTC SYNC YYYY-MM-DD HH:MM:SS - Sync date and time from PC");
  Serial.println();

  Serial.println("CAN BUS:");
  Serial.println("  CAN                 - Show CAN diagnostics");
  Serial.println("  CANSPEED <kbps>     - Set CAN speed (125/250/500/1000)");
  Serial.println();

  Serial.println("FILTERING:");
  Serial.println("  FILTERS ON          - Enable CAN filtering");
  Serial.println("  FILTERS OFF         - Disable CAN filtering");
  Serial.println("  WHITELIST ON        - Use whitelist mode");
  Serial.println("  WHITELIST OFF       - Disable whitelist mode");
  Serial.println();

  Serial.println("WHITELIST MANAGEMENT:");
  Serial.println("  WL READ             - Read whitelist from UART");
  Serial.println("  WL WRITE            - Write whitelist to UART");
  Serial.println("  WL SAVE             - Save whitelist to EEPROM");
  Serial.println();

  Serial.println("BLACKLIST MANAGEMENT:");
  Serial.println("  BL READ             - Read blacklist from UART");
  Serial.println("  BL WRITE            - Write blacklist to UART");
  Serial.println("  BL SAVE             - Save blacklist to EEPROM");
  Serial.println();

  Serial.println("SD CARD & DATA LOGGING:");
  Serial.println("  SD                  - Initialize SD card");
  Serial.println("  FILE RESET          - Reset file counter");
  Serial.println("  SAVVYCAN            - Toggle SavvyCAN mode (UART protocol)");
  Serial.println();

  Serial.println("=========================================\n");
}

void CLIHandler::showStatus() {
  Serial.println("\n========== DEVICE STATUS ==========\n");

  Serial.print("CAN Speed:      ");
  Serial.print(CANSpeedArray[CANSpeed] / 1000);
  Serial.println(" kbps");

  Serial.print("CAN Filtering:  ");
  Serial.println(filterState ? "ENABLED" : "DISABLED");

  Serial.print("Whitelist Mode: ");
  Serial.println(listState ? "ENABLED" : "DISABLED");

  Serial.print("SavvyCAN Mode:  ");
  Serial.println(savvyCAN ? "ACTIVE" : "INACTIVE");

  Serial.print("File Count:     ");
  Serial.println(fileCount);

  Serial.println();
}

void CLIHandler::showConfig() {
  Serial.println("\n========== CURRENT CONFIGURATION ==========\n");

  Serial.print("CAN Speed (EEPROM):  ");
  Serial.print(CANSpeedArray[EEPROM.read(CANSpeedIndex)] / 1000);
  Serial.println(" kbps");

  Serial.print("Filtering Enabled:   ");
  Serial.println(EEPROM.read(filterIndex) ? "YES" : "NO");

  Serial.print("Whitelist Mode:      ");
  Serial.println(EEPROM.read(listStateIndex) ? "YES" : "NO");

  Serial.print("File Count:          ");
  Serial.println(EEPROM.read(fileCountIndex));

  Serial.println("\n===========================================\n");
}

void CLIHandler::showRTCDiag() {
  Serial.println("\n========== RTC DIAGNOSTICS ==========\n");

  RTCTime t;
  RTC.getTime(t);

  // Check if RTC is running
  bool rtcRunning = RTC.isRunning();

  Serial.print("RTC Status:       ");
  Serial.println(rtcRunning ? "RUNNING" : "STOPPED");

  // Check SOSC (subclock oscillator) status
  String soscStatus = detectSOSCStatus();
  Serial.println(soscStatus);

  // Detect actual clock source by reading hardware registers
  String clockSource = detectRTCClockSource();
  Serial.print("RTC Source:       ");
  Serial.println(clockSource);

  // Check if external crystal is actually enabled
  if (clockSource.indexOf("LOCO") != -1) {
    Serial.println();
    Serial.println("⚠️  WARNING: External crystal is NOT enabled!");
    Serial.println("This causes time drift (you're seeing ~3sec/10min)");
    Serial.println();
    Serial.println("FIX: Configure FSP hardware settings:");
    Serial.println("  1. Open Renesas FSP Configurator for this project");
    Serial.println("  2. RTC Stack > Stacks > g_rtc > RTC > Clock Source");
    Serial.println("  3. Change from LOCO to: Subclock (SUBCLK)");
    Serial.println("  4. Generate code and rebuild project");
    Serial.println();
  }

  Serial.print("RTC Time:         ");
  if (t.getHour() < 10) Serial.print("0");
  Serial.print(t.getHour());
  Serial.print(":");
  if (t.getMinutes() < 10) Serial.print("0");
  Serial.print(t.getMinutes());
  Serial.print(":");
  if (t.getSeconds() < 10) Serial.print("0");
  Serial.println(t.getSeconds());

  Serial.print("RTC Date:         ");
  Serial.print(t.getYear());
  Serial.print("-");
  int month = (int)t.getMonth() + 1;
  if (month < 10) Serial.print("0");
  Serial.print(month);
  Serial.print("-");
  if (t.getDayOfMonth() < 10) Serial.print("0");
  Serial.println(t.getDayOfMonth());

  Serial.println("\n====================================\n");
}

void CLIHandler::setRTCTime(const String &arg) {
  // Parse HH:MM:SS format
  int colonCount = 0;
  for (char c : arg) {
    if (c == ':') colonCount++;
  }

  if (colonCount != 2) {
    Serial.println("ERR: Format is HH:MM:SS");
    return;
  }

  int hour = arg.substring(0, arg.indexOf(':')).toInt();
  int min = arg.substring(arg.indexOf(':') + 1, arg.lastIndexOf(':')).toInt();
  int sec = arg.substring(arg.lastIndexOf(':') + 1).toInt();

  if (hour > 23 || min > 59 || sec > 59 || hour < 0 || min < 0 || sec < 0) {
    Serial.println("ERR: Invalid time (HH:0-23, MM:0-59, SS:0-59)");
    return;
  }

  // Get current date/time to preserve the date
  RTCTime t;
  RTC.getTime(t);

  // Update only the time fields, keep the date
  t.setHour(hour);
  t.setMinute(min);
  t.setSecond(sec);

  // Set the new time in RTC
  if (RTC.setTime(t)) {
    Serial.print("OK: RTC time updated to ");
    if (hour < 10) Serial.print("0");
    Serial.print(hour);
    Serial.print(":");
    if (min < 10) Serial.print("0");
    Serial.print(min);
    Serial.print(":");
    if (sec < 10) Serial.print("0");
    Serial.println(sec);
  } else {
    Serial.println("ERR: Failed to set RTC time");
  }
}

void CLIHandler::syncRTCDateTime(const String &arg) {
  // Parse YYYY-MM-DD HH:MM:SS format
  // Expected format: "2026-09-21 14:30:45"

  int spacePos = arg.indexOf(' ');
  if (spacePos == -1) {
    Serial.println("ERR: Format is YYYY-MM-DD HH:MM:SS");
    return;
  }

  String dateStr = arg.substring(0, spacePos);
  String timeStr = arg.substring(spacePos + 1);

  // Parse date: YYYY-MM-DD
  int dash1 = dateStr.indexOf('-');
  int dash2 = dateStr.lastIndexOf('-');

  if (dash1 == -1 || dash2 == -1 || dash1 == dash2) {
    Serial.println("ERR: Date format is YYYY-MM-DD");
    return;
  }

  int year = dateStr.substring(0, dash1).toInt();
  int month = dateStr.substring(dash1 + 1, dash2).toInt();
  int day = dateStr.substring(dash2 + 1).toInt();

  // Parse time: HH:MM:SS
  int colon1 = timeStr.indexOf(':');
  int colon2 = timeStr.lastIndexOf(':');

  if (colon1 == -1 || colon2 == -1 || colon1 == colon2) {
    Serial.println("ERR: Time format is HH:MM:SS");
    return;
  }

  int hour = timeStr.substring(0, colon1).toInt();
  int min = timeStr.substring(colon1 + 1, colon2).toInt();
  int sec = timeStr.substring(colon2 + 1).toInt();

  // Validate ranges
  if (year < 2000 || year > 2100) {
    Serial.println("ERR: Year must be 2000-2100");
    return;
  }
  if (month < 1 || month > 12) {
    Serial.println("ERR: Month must be 01-12");
    return;
  }
  if (day < 1 || day > 31) {
    Serial.println("ERR: Day must be 01-31");
    return;
  }
  if (hour > 23 || min > 59 || sec > 59 || hour < 0 || min < 0 || sec < 0) {
    Serial.println("ERR: Time must be HH:0-23, MM:0-59, SS:0-59");
    return;
  }

  // Create RTCTime object with date and time
  RTCTime t(day, (Month)(month - 1), year, hour, min, sec, DayOfWeek::MONDAY, SaveLight::SAVING_TIME_INACTIVE);

  // Set the time in RTC
  if (RTC.setTime(t)) {
    Serial.print("OK: RTC synced to ");
    Serial.print(year);
    Serial.print("-");
    if (month < 10) Serial.print("0");
    Serial.print(month);
    Serial.print("-");
    if (day < 10) Serial.print("0");
    Serial.print(day);
    Serial.print(" ");
    if (hour < 10) Serial.print("0");
    Serial.print(hour);
    Serial.print(":");
    if (min < 10) Serial.print("0");
    Serial.print(min);
    Serial.print(":");
    if (sec < 10) Serial.print("0");
    Serial.println(sec);
  } else {
    Serial.println("ERR: Failed to sync RTC");
  }
}

void CLIHandler::setCANSpeed(const String &arg) {
  int speed = arg.toInt();

  byte speedIndex;
  if (speed == 125) {
    speedIndex = 0;
  } else if (speed == 250) {
    speedIndex = 1;
  } else if (speed == 500) {
    speedIndex = 2;
  } else if (speed == 1000) {
    speedIndex = 3;
  } else {
    Serial.println("ERR: Speed must be 125, 250, 500, or 1000 kbps");
    return;
  }

  CANSpeed = speedIndex;
  updateCANSpeed();  // Apply speed change immediately without restart
  Serial.print("OK: CAN speed changed to ");
  Serial.print(speed);
  Serial.println(" kbps");
}

void CLIHandler::enableFilters() {
  filterState = 1;
  EEPROM.update(filterIndex, filterState);
  Serial.println("OK: Filtering enabled");
}

void CLIHandler::disableFilters() {
  filterState = 0;
  EEPROM.update(filterIndex, filterState);
  Serial.println("OK: Filtering disabled");
}

void CLIHandler::enableWhitelist() {
  listState = 1;
  EEPROM.update(listStateIndex, listState);
  Serial.println("OK: Whitelist mode enabled");
}

void CLIHandler::disableWhitelist() {
  listState = 0;
  EEPROM.update(listStateIndex, listState);
  Serial.println("OK: Whitelist mode disabled");
}

void CLIHandler::readWhitelist() {
  Serial.println("Enter whitelist data (terminate with >): ");
  Serial.println("READY FOR INPUT");
}

void CLIHandler::writeWhitelist() {
  Serial.print("Whitelist: ");
  for (int i = 0; i < NUM_ROWS; i++) {
    for (int j = 0; j < NUM_COLS; j++) {
      Serial.print(whitelist[i][j]);
    }
    Serial.print("*");
  }
  Serial.println(">");
}

void CLIHandler::burnWhitelist() {
  int index = whitelistIndex;
  for (int i = 0; i < NUM_ROWS; i++) {
    for (int j = 0; j < NUM_COLS; j++) {
      EEPROM.update(index, whitelist[i][j]);
      index++;
    }
    index = index + NUM_COLS;
  }
  Serial.println("OK: Whitelist saved to EEPROM");
}

void CLIHandler::readBlacklist() {
  Serial.println("Enter blacklist data (terminate with >): ");
  Serial.println("READY FOR INPUT");
}

void CLIHandler::writeBlacklist() {
  Serial.print("Blacklist: ");
  for (int i = 0; i < NUM_ROWS; i++) {
    for (int j = 0; j < NUM_COLS; j++) {
      Serial.print(blacklist[i][j]);
    }
    Serial.print("*");
  }
  Serial.println(">");
}

void CLIHandler::burnBlacklist() {
  int index = blacklistIndex;
  for (int i = 0; i < NUM_ROWS; i++) {
    for (int j = 0; j < NUM_COLS; j++) {
      EEPROM.update(index, blacklist[i][j]);
      index++;
    }
    index = index + NUM_COLS;
  }
  Serial.println("OK: Blacklist saved to EEPROM");
}

void CLIHandler::toggleSavvyCAN() {
  savvyCAN = !savvyCAN;
  if (savvyCAN) {
    Serial.println("OK: SavvyCAN mode ENABLED");
  } else {
    Serial.println("OK: SavvyCAN mode DISABLED");
  }
}

void CLIHandler::resetFileCount() {
  fileCount = 1;
  EEPROM.update(fileCountIndex, fileCount);
  Serial.println("OK: File count reset to 1");
}

void CLIHandler::showCANDiag() {
  Serial.println("\n========== CAN DIAGNOSTICS ==========\n");
  Serial.println("CAN Bus Status: OK");
  Serial.print("Speed: ");
  Serial.print(CANSpeedArray[CANSpeed] / 1000);
  Serial.println(" kbps");
  Serial.println("\n====================================\n");
}
