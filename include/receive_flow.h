#pragma once

#include <stddef.h>

inline bool receiveMustWait(bool logging, bool drainingStoppedFrames, bool bridge,
    bool storageReady, size_t queued, size_t capacity) {
  return (logging || drainingStoppedFrames) && !bridge && storageReady && queued >= capacity;
}
