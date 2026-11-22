#pragma once

#include <stdint.h>

namespace LogFileCreation {
template <typename Operations>
bool create(Operations &operations, bool rollover) {
  uint32_t suffix = 0;
  for (;;) {
    operations.selectName(suffix);
    if (rollover) return operations.openExclusive();

    bool conflict = false;
    if (!operations.nameConflict(conflict)) return false;
    if (!conflict) return operations.openExclusive();
    if (suffix == UINT32_MAX) {
      operations.namesExhausted();
      return false;
    }
    ++suffix;
  }
}
}
