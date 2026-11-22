#include "../include/receive_flow.h"
#include "../include/frame_queue.h"
#include "../lib/Arduino_CAN/src/MeasuredCanRing.h"
#include <assert.h>
#include <stdio.h>

int main() {
  assert(receiveMustWait(true, false, false, true, 4, 4));
  assert(receiveMustWait(false, true, false, true, 4, 4));
  assert(!receiveMustWait(false, false, false, true, 4, 4));
  assert(!receiveMustWait(true, false, true, true, 4, 4));
  assert(!receiveMustWait(true, false, false, false, 4, 4));
  assert(!receiveMustWait(true, false, false, true, 3, 4));

  FrameQueue<4> queue;
  MeasuredCanRing<LoggedFrame, 4> ring;
  for (unsigned i = 0; i < 8; ++i) {
    LoggedFrame frame = {};
    frame.identifier = i;
    if (i < 4) assert(queue.push(frame));
    else ring.enqueue(frame);
  }
  size_t remaining = ring.available();
  for (unsigned expected = 0; expected < 8; ++expected) {
    if (queue.size()) {
      assert(queue.front().identifier == expected);
      queue.pop();
    } else {
      assert(false);
    }
    if (remaining && !receiveMustWait(false, true, false, true, queue.size(), 4)) {
      assert(queue.push(ring.dequeue()));
      --remaining;
      if (expected == 0) {
        LoggedFrame afterStop = {};
        afterStop.identifier = 99;
        ring.enqueue(afterStop);
      }
    }
  }
  assert(remaining == 0 && queue.size() == 0);
  assert(ring.available() == 1 && ring.dequeue().identifier == 99);
  puts("PASS: queue backpressure and bounded stop snapshot preserve FIFO");
}
