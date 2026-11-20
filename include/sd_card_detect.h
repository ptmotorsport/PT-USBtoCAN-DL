#pragma once

class SDCardDetect {
public:
  enum State { ABSENT, SETTLING, PRESENT, REINSERTED };
  static constexpr unsigned long InsertionSettleMillis = 100;

  State update(bool present, unsigned long now) {
    if (!present) {
      removalObserved_ = true;
      insertionPending_ = false;
      return ABSENT;
    }
    if (!removalObserved_) return PRESENT;
    if (!insertionPending_) {
      insertionPending_ = true;
      insertionMillis_ = now;
    }
    if (now - insertionMillis_ < InsertionSettleMillis) return SETTLING;
    removalObserved_ = false;
    insertionPending_ = false;
    return REINSERTED;
  }

private:
  bool removalObserved_ = false;
  bool insertionPending_ = false;
  unsigned long insertionMillis_ = 0;
};
