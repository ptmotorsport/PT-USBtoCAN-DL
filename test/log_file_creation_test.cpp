#include "../include/log_file_creation.h"
#include <cassert>
#include <cstdint>
#include <cstdio>

struct FakeOperations {
  uint32_t selected = 0;
  unsigned int checks = 0;
  unsigned int opens = 0;
  unsigned int conflicts = 0;
  bool checkSucceeds = true;
  bool openSucceeds = true;
  bool exhausted = false;

  void selectName(uint32_t suffix) { selected = suffix; }
  bool nameConflict(bool &conflict) {
    ++checks;
    conflict = selected < conflicts;
    return checkSucceeds;
  }
  bool openExclusive() { ++opens; return openSucceeds; }
  void namesExhausted() { exhausted = true; }
};

int main() {
  FakeOperations first;
  first.conflicts = 2;
  assert(LogFileCreation::create(first, false));
  assert(first.selected == 2 && first.checks == 3 && first.opens == 1);

  FakeOperations rollover;
  rollover.conflicts = 2;
  assert(LogFileCreation::create(rollover, true));
  assert(rollover.selected == 0 && rollover.checks == 0 && rollover.opens == 1);

  FakeOperations checkFailure;
  checkFailure.checkSucceeds = false;
  assert(!LogFileCreation::create(checkFailure, false));
  assert(checkFailure.checks == 1 && checkFailure.opens == 0);

  FakeOperations openFailure;
  openFailure.openSucceeds = false;
  assert(!LogFileCreation::create(openFailure, true));
  assert(openFailure.checks == 0 && openFailure.opens == 1);
  puts("PASS: new-log checks, rollover fast path and failures");
}
