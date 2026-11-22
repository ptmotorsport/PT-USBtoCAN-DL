#include "../lib/Arduino_CAN/src/MeasuredCanRing.h"
#include <assert.h>
#include <stdio.h>

int main() {
  MeasuredCanRing<int, 3> ring;
  assert(ring.available() == 0 && ring.highWater() == 0 && ring.drops() == 0);
  ring.enqueue(10);
  ring.enqueue(20);
  ring.enqueue(30);
  ring.enqueue(40);
  assert(ring.available() == 3 && ring.highWater() == 3 && ring.drops() == 1);
  assert(ring.dequeue() == 10);
  ring.enqueue(50);
  assert(ring.dequeue() == 20);
  assert(ring.dequeue() == 30);
  assert(ring.dequeue() == 50);
  assert(ring.dequeue() == 0);
  assert(ring.highWater() == 3 && ring.drops() == 1);
  MeasuredCanRing<int, 256> large;
  for (int i = 0; i < 256; ++i) large.enqueue(i);
  large.enqueue(256);
  assert(large.available() == 256 && large.highWater() == 256 && large.drops() == 1);
  for (int i = 0; i < 256; ++i) assert(large.dequeue() == i);
  puts("PASS: receive ring overflow accounting and wraparound");
}
