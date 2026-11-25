#include "../include/tach_period.h"
#include <assert.h>
#include <stdio.h>

int main() {
  TachPeriod tach;
  assert(tach.period(0) == 0);
  tach.edge(1000);
  assert(tach.period(1000) == 0);
  tach.edge(31000);
  assert(tach.period(31000) == 30000);  // 100 Hz, 6000 RPM.
  assert(TachConfig::TimeoutTicks == 900000);
  assert(tach.period(930999) == 30000);
  assert(tach.period(931000) == 0);
  tach.edge(940000);
  assert(tach.period(940000) == 0);
  tach.edge(970000);
  assert(tach.period(970000) == 30000);

  TachPeriod wrap;
  wrap.edge(TachConfig::CounterModulus - 10000);
  wrap.edge(20000);
  assert(wrap.period(20000) == 30000);
  assert(wrap.period(920000) == 0);

  TachPeriod longGap;
  longGap.edge(1000);
  longGap.edge(901000);
  assert(longGap.period(901000) == 0);
  longGap.edge(931000);
  assert(longGap.period(931000) == 30000);
  longGap.edge(931000);
  assert(longGap.period(931000) == 0);

  TachPeriod lowRpm;
  lowRpm.edge(1000);
  lowRpm.edge(601000);
  assert(lowRpm.period(601000) == 600000);  // 5 Hz, 300 RPM.
  puts("PASS: tach startup, period, wraparound, timeout and reacquisition");
}
