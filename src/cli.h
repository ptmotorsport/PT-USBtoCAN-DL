#pragma once

#include <Arduino.h>
#include <RTC.h>

// CLI command handler
class CLIHandler {
public:
  CLIHandler();
  void begin();
  void process();

private:
  // Command parsing
  String readLine();
  void handleCommand(const String &line);

  // Menu functions
  void showHelp();
  void showStatus();
  void showConfig();
  void showRTCDiag();
  void showCANDiag();

  // RTC utilities
  String detectRTCClockSource();
  String detectSOSCStatus();

  // RTC functions
  void setRTCTime(const String &arg);
  void syncRTCDateTime(const String &arg);

  // CAN functions
  void setCANSpeed(const String &arg);
  void setCANMode(const String &arg);
  void enableFilters();
  void disableFilters();
  void enableWhitelist();
  void disableWhitelist();

  // Filter list functions
  void readWhitelist();
  void writeWhitelist();
  void burnWhitelist();
  void readBlacklist();
  void writeBlacklist();
  void burnBlacklist();

  // SavvyCAN mode
  void toggleSavvyCAN();
  void resetFileCount();

  // Configuration print
  void printBanner();
};

extern CLIHandler cli;
