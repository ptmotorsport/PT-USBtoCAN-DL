#include "gps_monitor.h"
#include <assert.h>
#include <stdio.h>
#include <string>

namespace {

std::string nmea(const std::string &payload) {
  uint8_t checksum = 0;
  for (char c : payload) checksum ^= static_cast<uint8_t>(c);
  char suffix[8];
  snprintf(suffix, sizeof(suffix), "*%02X\r\n", checksum);
  return "$" + payload + suffix;
}

void feed(GPSMonitor &monitor, const std::string &text, uint32_t now = 100) {
  for (unsigned char c : text) monitor.feed(c, now);
}

void testNoFixCommunication() {
  GPSMonitor monitor;
  assert(monitor.bytesReceived == 0 && monitor.validSentences == 0);
  const std::string gga = nmea("GPGGA,,,,,,0,00,99.99,,,,,,");
  const std::string rmc = nmea("GPRMC,,V,,,,,,,,,,N");
  feed(monitor, gga, 100);
  feed(monitor, rmc, 200);
  assert(monitor.bytesReceived == gga.size() + rmc.size());
  assert(monitor.validSentences == 2);
  assert(monitor.lastByteMillis == 200 && monitor.lastValidMillis == 200);
  assert(std::string(monitor.lastSentence) == rmc.substr(0, rmc.size() - 2));
  assert(!monitor.partialSentence());
  assert(monitor.checksumErrors == 0 && monitor.malformedSentences == 0);
  const std::string gn = nmea("GNGGA,,,,,,0,00,99.99,,,,,,");
  feed(monitor, gn);
  assert(std::string(monitor.lastSentence) == gn.substr(0, gn.size() - 2));
  assert(monitor.validSentences == 3);
}

void testInvalidInputAndRecovery() {
  GPSMonitor monitor;
  feed(monitor, "garbage without a sentence\r\n");
  assert(monitor.validSentences == 0 && monitor.malformedSentences == 0);
  std::string bad = nmea("GPRMC,,V,,,,,,,,,,N");
  bad[7] = '1';
  feed(monitor, bad);
  assert(monitor.checksumErrors == 1 && monitor.validSentences == 0);
  const char *malformed[] = {
    "$GPGGA,no checksum\r\n", "$GPGGA,*ZZ\n", "$GPGGA,*00extra\n",
    "$*00\n", "$GPGGA,*0\n", "$GPGGA,*000\n", "$GPGGA,*\n"
  };
  for (const char *text : malformed) feed(monitor, text);
  assert(monitor.malformedSentences == 7);
  feed(monitor, "$GPGGA,broken");
  assert(monitor.partialSentence());
  assert(strcmp(monitor.lastSentence, "$GPGGA,broken") == 0);
  feed(monitor, nmea("GPRMC,,V,,,,,,,,,,N"));
  assert(monitor.malformedSentences == 8 && monitor.validSentences == 1);
  feed(monitor, "$GPGGA," + std::string(200, '1') + "\r\n");
  assert(monitor.overlongSentences == 1 && monitor.malformedSentences == 9);
  assert(strlen(monitor.lastSentence) == GPSMonitor::SentenceCapacity - 1);
  feed(monitor, std::string("$GPGGA,") + char(0) + "*00\n");
  assert(monitor.malformedSentences == 10);
  feed(monitor, nmea("GPVTG,,,,,,,,,N"), UINT32_MAX - 10);
  assert(monitor.validSentences == 2);
  assert(monitor.lastValidMillis == UINT32_MAX - 10);
  assert(static_cast<uint32_t>(20 - monitor.lastValidMillis) == 31);
}

void testBoundariesAndFragments() {
  GPSMonitor monitor;
  // Total stored length = '$' + payload + '*xx' = capacity - 1.
  const std::string maximum = nmea("GPTXT," + std::string(
      GPSMonitor::SentenceCapacity - 11, 'a'));
  feed(monitor, maximum.substr(0, 10));
  assert(monitor.validSentences == 0 && monitor.partialSentence());
  feed(monitor, maximum.substr(10));
  assert(monitor.validSentences == 1 && monitor.overlongSentences == 0);
  assert(!monitor.partialSentence());
  assert(strlen(monitor.lastSentence) == GPSMonitor::SentenceCapacity - 1);
  std::string tooLong = nmea("GPTXT," + std::string(
      GPSMonitor::SentenceCapacity - 10, 'a'));
  feed(monitor, tooLong);
  assert(monitor.validSentences == 1 && monitor.overlongSentences == 1);
  std::string lowercase = nmea("GPRMC,,V,,,,,,,,,,N");
  for (size_t i = lowercase.size() - 4; i < lowercase.size() - 2; ++i) {
    if (lowercase[i] >= 'A' && lowercase[i] <= 'F') lowercase[i] += 'a' - 'A';
  }
  feed(monitor, lowercase);
  assert(monitor.validSentences == 2);
  monitor = GPSMonitor{};
  assert(monitor.bytesReceived == 0 && monitor.lastSentence[0] == '\0');
}

}

int main() {
  testNoFixCommunication();
  testInvalidInputAndRecovery();
  testBoundariesAndFragments();
  puts("gps_monitor_test: PASS");
}
