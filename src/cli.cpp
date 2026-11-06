#include "cli.h"
#include <EEPROM.h>
#include <SdFat.h>
#include <SPI.h>
#include <Arduino_CAN.h>
#include "status_config.h"
#include "bounded_console.h"
#include "can_filter.h"
#include "slcan_protocol.h"
#include "slcan_bridge.h"
#include "usb_connection.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {

class BoundedConsole : public Print {
public:
  using Print::write;
  size_t write(uint8_t value) override { return output_.write(value); }
  size_t write(const uint8_t *data, size_t length) override {
    return output_.write(data, length);
  }
  explicit operator bool() { return usbSerialConnected(); }
  int available() { return Serial.available(); }
  int read() { return Serial.read(); }
  void drain() {
    if (usbSerialConnected()) output_.drain(Serial, 64);
  }
  bool pending() const { return !output_.empty(); }
  void discard() { output_.clear(); }
  uint32_t droppedBytes() const { return output_.droppedBytes(); }

private:
  BoundedConsoleBuffer output_;
};

BoundedConsole console;
bool bridgeReturnReportPending = false;

}

#undef Serial
#define Serial console

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
extern FilterRule whitelistRules[5];
extern FilterRule blacklistRules[5];

// Function declarations
extern void updateCANSpeed();
extern void clearFilters();
extern void enterBridgeMode();
extern void stopLogging();
extern void startLogging();
extern bool loggingEnabled;
extern uint32_t slcanDroppedFrames();
extern void saveFilterRules(bool blacklist);

// EEPROM indices
extern const int filterIndex;
extern const int fileCountIndex;
extern const int CANSpeedIndex;
extern const int listStateIndex;

extern const int CHIP_SELECT;
extern SdFat sd;
extern uint32_t sdSpiClockMHz;
extern bool sdStartupReady;
extern unsigned long sdStartupMillis;
extern unsigned long sdStartupDurationMillis;
extern uint8_t sdStartupErrorCode;
extern uint8_t sdStartupErrorData;
extern const uint8_t SD_STARTUP_MAX_ATTEMPTS;
extern const unsigned long SD_STARTUP_RETRY_DELAY_MS;
extern uint8_t sdStartupAttempts;
extern uint8_t sdFirstErrorCode;
extern uint8_t sdFirstErrorData;
extern bool sdReady;
extern bool sdFaultLatched;
extern uint32_t sdRuntimeAttempts;
extern uint32_t sdDroppedMessages;
extern uint32_t sdQueueOverflows;
extern uint32_t sdQueueHighWater;
extern uint32_t sdUnconfirmedMessages;
extern uint32_t sdPendingMessages;
extern uint32_t sdSyncedBytes;
extern uint32_t sdLastSyncMicros;
extern uint32_t sdMaxSyncMicros;
extern uint8_t sdLastErrorCode;
extern uint8_t sdLastErrorData;
extern const char *sdLastFailure;
extern int bufferIndex;
extern bool canReady;
extern bool canMessageReceived;
extern unsigned long lastCANReceiveMillis;
extern int vbattRaw;
extern float vbattVolts;
extern bool vbattValid;

namespace {

bool parseFilterNumber(const char *text, int base, unsigned long maximum,
                       unsigned long &value) {
  const char *digits = text;
  if (base == 16 && digits[0] == '0' && (digits[1] == 'X' || digits[1] == 'x')) {
    digits += 2;
  }
  if (*digits == '\0') return false;
  for (const char *digit = digits; *digit; ++digit) {
    if (*digit >= '0' && *digit <= '9') continue;
    if (base == 16 && ((*digit >= 'A' && *digit <= 'F') ||
                       (*digit >= 'a' && *digit <= 'f'))) continue;
    return false;
  }
  errno = 0;
  char *end = nullptr;
  const unsigned long parsed = strtoul(text, &end, base);
  if (errno == ERANGE || end == text || *end != '\0' || parsed > maximum) return false;
  value = parsed;
  return true;
}

void editFilterRule(const String &arguments, FilterRule (&rules)[5]) {
  char action[161] = {};
  char slotToken[161] = {};
  char typeToken[161] = {};
  char identifierToken[161] = {};
  char extra[2] = {};
  const int tokens = sscanf(arguments.c_str(), "%160s %160s %160s %160s %1s",
                            action, slotToken, typeToken, identifierToken, extra);
  const bool setting = strcmp(action, "SET") == 0;
  const bool clearing = strcmp(action, "CLEAR") == 0;
  unsigned long slot = 0;
  if ((!setting && !clearing) || tokens != (setting ? 4 : 2) ||
      !parseFilterNumber(slotToken, 10, 5, slot) || slot == 0) {
    Serial.println("ERR: Use WL/BL SET <1..5> STD|EXT <hex-id> or WL/BL CLEAR <1..5>");
    return;
  }
  if (clearing) {
    rules[slot - 1] = FilterRule{};
    Serial.println("OK: Rule cleared; effective immediately");
    return;
  }
  const bool extended = strcmp(typeToken, "EXT") == 0;
  unsigned long identifier = 0;
  if ((!extended && strcmp(typeToken, "STD") != 0) ||
      !parseFilterNumber(identifierToken, 16, extended ? 0x1FFFFFFFUL : 0x7FFUL, identifier)) {
    Serial.println("ERR: STD ID must be 0..7FF; EXT ID must be 0..1FFFFFFF (hex)");
    return;
  }
  FilterRule rule;
  rule.identifier = static_cast<uint32_t>(identifier);
  rule.extended = extended;
  rule.active = true;
  rules[slot - 1] = rule;
  Serial.println("OK: Rule set; effective immediately (use SAVE to persist)");
}

void showFilterRules(const char *name, const FilterRule (&rules)[5]) {
  Serial.println(name);
  for (size_t slot = 0; slot < 5; ++slot) {
    const FilterRule &rule = rules[slot];
    char identifier[9];
    snprintf(identifier, sizeof(identifier), rule.extended ? "%08lX" : "%03lX",
             static_cast<unsigned long>(rule.identifier));
    Serial.print(slot + 1);
    Serial.print(rule.active ? " ACTIVE   " : " INACTIVE ");
    Serial.print(rule.extended ? "EXT 0x" : "STD 0x");
    Serial.println(identifier);
  }
}

}

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
}

String CLIHandler::readLine() {
  static String buffer = "";

  for (size_t processed = 0; processed < 64 && Serial.available() > 0; ++processed) {
    const int character = Serial.read();
    if (character < 0) break;
    const char c = static_cast<char>(character);

    // Handle backspace (both ASCII 8 and 127)
    if (c == '\b' || c == 127) {
      if (buffer.length() > 0) {
        buffer.remove(buffer.length() - 1);
      }
      continue;
    }

    // Handle carriage return (line complete)
    if (c == '\r' || c == '\n') {
      String line = buffer;
      buffer = "";
      line.trim();
      return line;
    }

    // Handle printable characters
    if (c >= 32 && c < 127) {
      // Check max length (leave room for null terminator)
      if (buffer.length() < 160) {
        buffer += c;
      }
      continue;
    }

    // Ignore other control characters
  }

  return "";
}

void CLIHandler::process() {
  const bool connected = static_cast<bool>(Serial);
  if (savvyCAN) {
    Serial.discard();
    return;
  }
  Serial.drain();
  if (!connected || Serial.pending()) return;
  if (bridgeReturnReportPending) {
    bridgeReturnReportPending = false;
    Serial.print(F("\r\nReturned to CLI. SLCAN dropped frames: "));
    Serial.println(slcanDroppedFrames());
    Serial.drain();
    return;
  }
  String line = readLine();
  if (Slcan::isConnectionCommand(line.c_str(), line.length())) {
    Serial.discard();
    bridgeReturnReportPending = true;
    enterBridgeMode();
    slcanInitialCommand(line.c_str(), line.length());
    return;
  }
  if (line.length() != 0) handleCommand(line);
  if (!savvyCAN) Serial.drain();
}

void CLIHandler::handleCommand(const String &line) {
  String cmd = line;
  cmd.toUpperCase();
  cmd.trim();

  if (cmd == "LOG STOP") {
    stopLogging();
    Serial.println(sdReady ? F("OK: Logging stopped; queued rows drained and file closed") :
        F("ERR: Logging stopped; SD fault, queued rows may remain in RAM"));
    return;
  }
  if (cmd == "LOG START") {
    startLogging();
    Serial.println(F("OK: Logging enabled"));
    return;
  }

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
  if (cmd == "WL" || cmd.startsWith("WL ")) {
    editFilterRule(cmd.substring(2), whitelistRules);
    return;
  }
  if (cmd == "BL" || cmd.startsWith("BL ")) {
    editFilterRule(cmd.substring(2), blacklistRules);
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
    showSDDiag();
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
  Serial.println("  Filter flags and rule edits take effect immediately; no restart needed.");
  Serial.println();

  Serial.println("WHITELIST MANAGEMENT:");
  Serial.println("  WL SET <1..5> STD|EXT <hex-id> - Set and activate a rule");
  Serial.println("  WL CLEAR <1..5>      - Clear a rule");
  Serial.println("  WL READ / WL WRITE  - Show current numeric whitelist rules");
  Serial.println("  WL SAVE             - Save whitelist to EEPROM");
  Serial.println();

  Serial.println("BLACKLIST MANAGEMENT:");
  Serial.println("  BL SET <1..5> STD|EXT <hex-id> - Set and activate a rule");
  Serial.println("  BL CLEAR <1..5>      - Clear a rule");
  Serial.println("  BL READ / BL WRITE  - Show current numeric blacklist rules");
  Serial.println("  BL SAVE             - Save blacklist to EEPROM");
  Serial.println("  Hex ID limits: STD 7FF, EXT 1FFFFFFF (optional 0x prefix)");
  Serial.println();

  Serial.println("SD CARD & DATA LOGGING:");
  Serial.println("  LOG STOP / LOG START - Drain and close / resume logging");
  Serial.println("  SD                  - Show SD startup and error diagnostics");
  Serial.println("  FILE RESET          - Reset file counter");
  Serial.println("  SAVVYCAN            - Toggle SavvyCAN mode (UART protocol)");
  Serial.println();

  Serial.println("=========================================\n");
}

void CLIHandler::showSDDiag() {
  const uint8_t currentErrorCode = sd.sdErrorCode();
  const uint8_t currentErrorData = sd.sdErrorData();

  Serial.println(F("\n========== SD DIAGNOSTICS ==========\n"));
  Serial.print(F("Live SD state:     "));
  Serial.println(sdReady ? F("READY (SD LED green)") : F("UNAVAILABLE (SD LED red)"));
  Serial.print(F("Logging enabled:   "));
  Serial.println(loggingEnabled ? F("YES") : F("NO"));
  Serial.print(F("Queue high water:  "));
  Serial.println(sdQueueHighWater);
  Serial.print(F("Runtime recovery:  "));
  if (sdFaultLatched) {
    Serial.println(F("STOPPED after runtime fault; inspect card, then reboot"));
  } else {
    Serial.println(savvyCAN ? F("Paused in SavvyCAN mode") : F("Retry every 2000 ms until first successful mount"));
  }
  Serial.println(F("Card health check: Every 1000 ms while ready"));
  Serial.print(F("Runtime attempts:  "));
  Serial.println(sdRuntimeAttempts);
  Serial.print(F("Buffered messages: "));
  Serial.print(bufferIndex);
  Serial.println(F(" / 512"));
  Serial.print(F("Dropped messages:  "));
  Serial.println(sdDroppedMessages);
  Serial.print(F("SD queue overflows: "));
  Serial.println(sdQueueOverflows);
  Serial.print(F("Unconfirmed messages: "));
  Serial.println(sdUnconfirmedMessages);
  Serial.print(F("Pending unsynced messages: "));
  Serial.println(sdPendingMessages);
  Serial.print(F("Last synced bytes: "));
  Serial.println(sdSyncedBytes);
  Serial.println(F("Segments: direct CSV, 4 MiB / 5 min; sync every 1000 ms"));
  Serial.print(F("Last sync duration (us): "));
  Serial.println(sdLastSyncMicros);
  Serial.print(F("Max sync duration (us): "));
  Serial.println(sdMaxSyncMicros);
  Serial.print(F("CLI output overflow (dropped bytes): "));
  Serial.println(Serial.droppedBytes());
  Serial.println(F("CAN library RX losses: unmeasured"));
  Serial.print(F("SLCAN dropped frames: "));
  Serial.println(slcanDroppedFrames());
  Serial.print(F("Last runtime fault: "));
  Serial.println(sdLastFailure);
  Serial.print(F("Saved fault error: 0x"));
  Serial.print(sdLastErrorCode, HEX);
  Serial.print(F(" / data=0x"));
  Serial.print(sdLastErrorData, HEX);
  if (sdLastErrorCode) {
    Serial.print(F(" / "));
    printSdErrorSymbol(&Serial, sdLastErrorCode);
  }
  Serial.println();
  Serial.print(F("Startup result:    "));
  Serial.println(sdStartupReady ? F("OK (historical)") : F("FAILED (historical)"));
  Serial.print(F("Startup attempts:  "));
  Serial.print(sdStartupAttempts);
  Serial.print(F(" / "));
  Serial.println(SD_STARTUP_MAX_ATTEMPTS);
  Serial.print(F("Retry delay:       "));
  Serial.print(SD_STARTUP_RETRY_DELAY_MS);
  Serial.println(F(" ms"));
  Serial.print(F("Init at uptime:    "));
  Serial.print(sdStartupMillis);
  Serial.println(F(" ms"));
  Serial.print(F("Total init/retry:  "));
  Serial.print(sdStartupDurationMillis);
  Serial.println(F(" ms"));
  Serial.print(F("Current uptime:    "));
  Serial.print(millis());
  Serial.println(F(" ms"));
  Serial.print(F("SPI requested max: "));
  Serial.print(sdSpiClockMHz);
  Serial.println(F(" MHz (not a measured clock)"));
  Serial.print(F("SPI CRC checking:  "));
  Serial.println(USE_SD_CRC ? F("ENABLED (commands and data)") : F("DISABLED"));
  Serial.print(F("SPI transfer mode: "));
  Serial.println(USE_SPI_ARRAY_TRANSFER == 0 ? F("BYTE (bulk path bypassed)") : F("BULK"));
  Serial.print(F("SPI pins:          CS="));
  Serial.print(CHIP_SELECT);
  Serial.print(F(" SCK="));
  Serial.print(SCK);
  Serial.print(F(" MOSI="));
  Serial.print(MOSI);
  Serial.print(F(" MISO="));
  Serial.println(MISO);

  if (sdStartupAttempts > 1) {
    Serial.print(F("First failure:     0x"));
    Serial.print(sdFirstErrorCode, HEX);
    Serial.print(F(" / data=0x"));
    Serial.print(sdFirstErrorData, HEX);
    if (sdFirstErrorCode) {
      Serial.print(F(" / "));
      printSdErrorSymbol(&Serial, sdFirstErrorCode);
    }
    Serial.println();
  }
  Serial.print(F("Final init error:  0x"));
  Serial.print(sdStartupErrorCode, HEX);
  Serial.print(F(" / data=0x"));
  Serial.print(sdStartupErrorData, HEX);
  if (sdStartupErrorCode) {
    Serial.print(F(" / "));
    printSdErrorSymbol(&Serial, sdStartupErrorCode);
  }
  Serial.println();
  Serial.print(F("Current error:     0x"));
  Serial.print(currentErrorCode, HEX);
  Serial.print(F(" / data=0x"));
  Serial.print(currentErrorData, HEX);
  if (currentErrorCode) {
    Serial.print(F(" / "));
    printSdErrorSymbol(&Serial, currentErrorCode);
  }
  Serial.println();

  Serial.print(F("Filesystem:        "));
  if (sdReady) {
    sd.printFatType(&Serial);
    Serial.println();
  } else {
    Serial.println(sdFaultLatched ? F("Fault latched; reboot required") : F("Unavailable; awaiting mount"));
  }
  Serial.println(F("\nRead-only report; no reinitialization or write test performed."));
  Serial.println(F("Startup result is retained; current errors may reflect later logging."));
  if (sdFaultLatched) {
    Serial.println(F("No further SD access this boot; automatic retries could worsen corruption."));
    Serial.println(F("Preserve/image the card before repair or formatting. Reboot does not repair FAT."));
  }
  if (!sdReady) {
    if (sdStartupErrorCode == SD_CARD_ERROR_CMD0) {
      Serial.println(F("CMD0: card did not enter SPI idle; check card power, CS and SPI wiring."));
    }
    Serial.println(F("Compare reports after separate USB-only and vehicle-power cold boots."));
    Serial.println(F("Check SD 3.3V at the socket during startup for slow rise, dips or ripple."));
    Serial.println(F("Check common ground, SPI signal levels and regulator decoupling."));
  }
  Serial.println(F("SD supply voltage is not measured by this report."));
  Serial.println(F("\n====================================\n"));
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

  Serial.print(F("Termination LED input: "));
  Serial.println(StatusConfig::TERMINATION_SENSE_PIN >= 0 ? F("CONFIGURED") : F("UNCONFIGURED (LED off)"));
    Serial.print(F("VBATT (P003):          "));
    if (vbattValid) {
      Serial.print(vbattVolts, 2);
      Serial.print(F(" V / ADC="));
      Serial.println(vbattRaw);
    } else {
      Serial.println(F("ADC READ FAILED"));
    }
    Serial.print(F("Power indication:      "));
    Serial.println(!vbattValid ? F("UNKNOWN (LED off)") : vbattVolts > StatusConfig::VEHICLE_POWER_THRESHOLD_VOLTS ?
      F("VEHICLE (>6 V)") : F("USB (VBATT <=6 V inferred)"));
  Serial.println(F("Recording trigger:     Filtered CAN frames (no parameter trigger configured)"));
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
  writeWhitelist();
}

void CLIHandler::writeWhitelist() {
  showFilterRules("Whitelist:", whitelistRules);
}

void CLIHandler::burnWhitelist() {
  saveFilterRules(false);
  Serial.println("OK: Whitelist saved to EEPROM");
}

void CLIHandler::readBlacklist() {
  writeBlacklist();
}

void CLIHandler::writeBlacklist() {
  showFilterRules("Blacklist:", blacklistRules);
}

void CLIHandler::burnBlacklist() {
  saveFilterRules(true);
  Serial.println("OK: Blacklist saved to EEPROM");
}

void CLIHandler::toggleSavvyCAN() {
  Serial.discard();
  bridgeReturnReportPending = true;
  enterBridgeMode();
}

void CLIHandler::resetFileCount() {
  const bool resume = loggingEnabled;
  stopLogging();
  if (!sdReady) {
    Serial.println(F("ERR: File counter unchanged; SD close/drain failed"));
    return;
  }
  fileCount = 1;
  EEPROM.update(fileCountIndex, fileCount);
  if (resume) startLogging();
  Serial.println("OK: File count reset to 1");
}

void CLIHandler::showCANDiag() {
  Serial.println("\n========== CAN DIAGNOSTICS ==========\n");
  int errorCode = 0;
  const bool error = CAN.isError(errorCode);
  Serial.print(F("CAN Bus Status: "));
  Serial.println(!canReady || error ? F("ERROR") : F("OK"));
  Serial.print(F("CAN error code: "));
  Serial.println(errorCode, HEX);
  Serial.print(F("CANTIMEOUT: "));
  Serial.print(StatusConfig::CANTIMEOUT);
  Serial.println(F(" ms"));
  Serial.print(F("Receive state: "));
  Serial.println(canMessageReceived && millis() - lastCANReceiveMillis < StatusConfig::CANTIMEOUT ?
      F("ACTIVE") : F("IDLE / no recent received messages"));
  Serial.print("Speed: ");
  Serial.print(CANSpeedArray[CANSpeed] / 1000);
  Serial.println(" kbps");
  Serial.println("\n====================================\n");
}
