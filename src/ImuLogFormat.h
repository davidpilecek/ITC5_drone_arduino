#pragma once

#include <stdint.h>

namespace itc {

// Fixed-size binary records avoid formatting CSV strings in the control loop.
// Units and field order are shared with tools/imu_log_to_csv.py.
struct ImuLogRecord {
  uint32_t timestampUs = 0; // Application's IMU-read timestamp; wraps after ~71 min.
  uint32_t sequence = 0;    // Missing numbers reveal dropped/skipped samples.
  float accelG[3] = {};
  float gyroDps[3] = {};
  float magUt[3] = {};
  float attitudeDeg[3] = {}; // Roll, pitch, yaw from the existing estimator.
  uint32_t flags = 0; // bit 0 record present, 1 mag valid, 2 mag calibrated, 3 attitude valid
  uint32_t reserved = 0;
};
static_assert(sizeof(float) == 4 && sizeof(ImuLogRecord) == 64,
              "The IMU log format requires 32-bit floats and 64-byte records.");

// One sector describes the file. Each following sector contains eight records.
// Partial sectors are padded with records whose flags are zero.
struct ImuLogHeader {
  char magic[8] = {'I', 'T', 'C', 'I', 'M', 'U', '1', '\0'};
  uint32_t version = 1;
  uint32_t recordBytes = sizeof(ImuLogRecord);
  uint32_t samplePeriodUs = 0; // 0 = event-driven; use record timestamps for timing.
  uint32_t capacity = 0;
  uint8_t reserved[488] = {};
};
static_assert(sizeof(ImuLogHeader) == 512, "The header must fill one SD sector.");

// Main-loop-only ring queue. Its storage is supplied by the logger, with no heap.
class ImuLogQueue {
 public:
  ImuLogQueue(ImuLogRecord* storage, unsigned capacity)
      : storage_(storage), capacity_(capacity) {}

  void clear() { head_ = tail_ = count_ = 0; }
  unsigned size() const { return count_; }
  bool push(const ImuLogRecord& record) {
    if (count_ == capacity_) {
      return false; // Drop the newest sample; never overwrite pending data.
    }
    storage_[head_] = record;
    head_ = (head_ + 1) % capacity_;
    ++count_;
    return true;
  }
  const ImuLogRecord& peek(unsigned offset) const {
    return storage_[(tail_ + offset) % capacity_];
  }
  void discard(unsigned count) {
    tail_ = (tail_ + count) % capacity_;
    count_ -= count;
  }

 private:
  ImuLogRecord* storage_;
  unsigned capacity_;
  unsigned head_ = 0;
  unsigned tail_ = 0;
  unsigned count_ = 0;
};

} // namespace itc
