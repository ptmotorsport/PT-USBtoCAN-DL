#pragma once
#include <Arduino.h>

#if defined(ARDUINO_MINIMA)
#include "tusb.h"
#endif

template <typename Transport, typename Mounted>
bool serviceUsbConnection(Transport &transport, Mounted mounted) {
  transport.dtr();
  static_cast<void>(static_cast<bool>(transport));
  return mounted();
}

inline bool usbSerialConnected() {
#if defined(ARDUINO_MINIMA)
  return serviceUsbConnection(Serial, []() { return tud_mounted(); });
#else
  return static_cast<bool>(Serial);
#endif
}