#include "../include/frame_queue.h"
#include "../include/can_filter.h"
#include <assert.h>
#include <stdio.h>

int main() {
  FrameQueue<4> queue;
  LoggedFrame frame = {};
  for (uint32_t index = 0; index < 4; ++index) {
    frame.identifier = index;
    assert(queue.push(frame));
  }
  assert(!queue.push(frame));
  assert(queue.size() == 4);
  for (uint32_t index = 0; index < 1000; ++index) {
    assert(queue.front().identifier == index);
    queue.pop();
    frame.identifier = index + 4;
    assert(queue.push(frame));
    assert(queue.size() == 4);
  }
  for (uint32_t index = 1000; index < 1004; ++index) {
    assert(queue.front().identifier == index);
    queue.pop();
  }
  assert(queue.size() == 0);
  queue.pop();
  FrameQueue<448> sensorBranchQueue;
  for (uint32_t index = 0; index < 448; ++index) {
    frame.identifier = index;
    assert(sensorBranchQueue.push(frame));
  }
  assert(sensorBranchQueue.size() == 448);
  assert(!sensorBranchQueue.push(frame));
  for (uint32_t index = 0; index < 896; ++index) {
    assert(sensorBranchQueue.front().identifier == index);
    sensorBranchQueue.pop();
    frame.identifier = index + 448;
    assert(sensorBranchQueue.push(frame));
  }
  for (uint32_t index = 896; index < 1344; ++index) {
    assert(sensorBranchQueue.front().identifier == index);
    sensorBranchQueue.pop();
  }
  assert(sensorBranchQueue.size() == 0);
  FilterRule rule;
  rule.identifier = 0x18DAF110;
  rule.extended = true;
  rule.active = true;
  assert(matchesRule(rule, 0x18DAF110, true));
  assert(!matchesRule(rule, 0x18DAF110, false));
  assert(!matchesRule(rule, 0x18DAF111, true));
  rule.active = false;
  assert(!matchesRule(rule, 0x18DAF110, true));
  puts("PASS: ring overflow, incremental capacity release, wraparound, FIFO order and full-width filters");
}