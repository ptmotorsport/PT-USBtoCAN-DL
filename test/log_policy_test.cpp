#include "log_policy.h"
#include <assert.h>
#include <stdio.h>

struct MockFile {
  bool succeeds = true;
  uint32_t position = 123;
  unsigned int calls = 0;
  bool sync() { ++calls; return succeeds; }
  uint32_t curPosition() const { return position; }
};

int main() {
  using namespace LogPolicy;
  assert(!needsRotation(SegmentBytes - 100, 100, 0));
  assert(needsRotation(SegmentBytes - 100, 101, 0));
  assert(needsRotation(SegmentBytes + 1, 0, 0));
  assert(!needsRotation(100, 50, SegmentMillis - 1));
  assert(needsRotation(100, 50, SegmentMillis));
  MockFile file;
  uint32_t confirmed = 0;
  assert(sync(file, confirmed));
  assert(confirmed == 123 && file.calls == 1);
  file.position = 456;
  file.succeeds = false;
  assert(!sync(file, confirmed));
  assert(confirmed == 123 && file.calls == 2);
  file.succeeds = true;
  assert(sync(file, confirmed) && confirmed == 456);
  puts("PASS: direct CSV rotation boundaries and successful-sync accounting");
}