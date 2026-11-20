#include "sd_card_detect.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

int main() {
  SDCardDetect detect;
  assert(detect.update(true, 0) == SDCardDetect::PRESENT);
  assert(detect.update(true, 1000) == SDCardDetect::PRESENT);
  assert(detect.update(false, 1001) == SDCardDetect::ABSENT);
  assert(detect.update(true, 1002) == SDCardDetect::SETTLING);
  assert(detect.update(true, 1101) == SDCardDetect::SETTLING);
  assert(detect.update(true, 1102) == SDCardDetect::REINSERTED);
  assert(detect.update(true, 1103) == SDCardDetect::PRESENT);

  assert(detect.update(false, 2000) == SDCardDetect::ABSENT);
  assert(detect.update(true, 2001) == SDCardDetect::SETTLING);
  assert(detect.update(false, 2050) == SDCardDetect::ABSENT);
  assert(detect.update(true, 2051) == SDCardDetect::SETTLING);
  assert(detect.update(true, 2150) == SDCardDetect::SETTLING);
  assert(detect.update(true, 2151) == SDCardDetect::REINSERTED);

  SDCardDetect emptyStartup;
  assert(emptyStartup.update(false, 0) == SDCardDetect::ABSENT);
  assert(emptyStartup.update(false, 5000) == SDCardDetect::ABSENT);
  assert(emptyStartup.update(true, 6000) == SDCardDetect::SETTLING);
  assert(emptyStartup.update(true, 6100) == SDCardDetect::REINSERTED);

  assert(detect.update(false, ULONG_MAX - 100) == SDCardDetect::ABSENT);
  assert(detect.update(true, ULONG_MAX - 50) == SDCardDetect::SETTLING);
  assert(detect.update(true, 48) == SDCardDetect::SETTLING);
  assert(detect.update(true, 49) == SDCardDetect::REINSERTED);
  puts("PASS: SD removal, stable reinsertion, bounce, empty startup and timer rollover");
}
