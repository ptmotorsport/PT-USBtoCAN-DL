#include "usb_connection.h"
#include <assert.h>
#include <stdio.h>

MockSerial Serial;

struct DtrTransport {
  bool dtrHigh = false;
  bool ignoreDtr = false;
  unsigned int serviced = 0;
  bool dtr() { ignoreDtr = true; return dtrHigh; }
  explicit operator bool() { ++serviced; return dtrHigh || ignoreDtr; }
  bool canRead() const { return dtrHigh || ignoreDtr; }
};

int main() {
  DtrTransport transport;
  assert(!transport.canRead());
  assert(serviceUsbConnection(transport, []() { return true; }));
  assert(transport.canRead());
  assert(!transport.dtrHigh);
  assert(transport.serviced == 1);
  assert(!serviceUsbConnection(transport, []() { return false; }));
  assert(transport.serviced == 2);
  transport.dtrHigh = true;
  assert(serviceUsbConnection(transport, []() { return true; }));
  assert(!serviceUsbConnection(transport, []() { return false; }));
  puts("PASS: USB mount state, DTR-low receive enable and background servicing");
}