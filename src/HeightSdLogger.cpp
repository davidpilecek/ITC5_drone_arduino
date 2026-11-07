#include "HeightSdLogger.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

namespace itc {

void HeightSdLogger::begin() {
  requestStart(); // Actual card access waits until service() is allowed.
}

void HeightSdLogger::requestStart() {
  if (!fileOpen_ && !failed_) startRequested_ = true;
}

void HeightSdLogger::requestStop() {
  startRequested_ = false;
  recording_ = false;
  stopping_ = fileOpen_;
}

void HeightSdLogger::sampleTof(const HeightLogRecord& reading) {
  if (!recording_) return;
  const uint32_t sequence = sequence_++;
  if (count_ == CAPACITY) {
    ++dropped_;
    return; // Do not wait for storage or overwrite a queued observation.
  }

  HeightLogRecord& record = records_[head_];
  record = reading;
  record.sequence = sequence;
  head_ = (head_ + 1) % CAPACITY;
  ++count_;
}

bool HeightSdLogger::openNewFile() {
  if (!cardStarted_) {
    cardStarted_ = sd_.begin(SdioConfig(FIFO_SDIO));
    if (!cardStarted_) return false;
  }
  bool opened = false;
  for (unsigned index = 0; index < 10000; ++index) {
    snprintf(filename_, sizeof(filename_), "TOF%04u.CSV", index);
    if (!sd_.exists(filename_)) {
      opened = file_.open(filename_, O_WRONLY | O_CREAT | O_EXCL);
      break;
    }
  }
  if (!opened) return false;
  fileOpen_ = true;
  const char header[] = "timestamp_us,sequence,distance_mm,range_status,i2c_status,signal_mcps,ambient_mcps\n";
  if (file_.write(header, sizeof(header) - 1) != sizeof(header) - 1 || !file_.sync()) {
    return false;
  }
  head_ = tail_ = count_ = 0;
  sequence_ = dropped_ = 0;
  previousSyncMs_ = millis();
  stopping_ = false;
  recording_ = true;
  return true;
}

void HeightSdLogger::fail() {
  failed_ = true;
  recording_ = false;
  startRequested_ = stopping_ = false;
  // No automatic retries after a failed/removed card; fix it and reboot.
}

void HeightSdLogger::service(bool safeToAccessCard) {
  // This gate prevents even isBusy()/sync()/close() when access is prohibited.
  if (!safeToAccessCard || failed_) return;
  if (startRequested_) {
    startRequested_ = false;
    if (!openNewFile()) fail();
    return;
  }
  if (!fileOpen_ || file_.isBusy()) return;

  if (count_ != 0) {
    const HeightLogRecord& record = records_[tail_];
    char row[192];
    const int length = snprintf(row, sizeof(row),
        "%lu,%lu,%u,%u,%u,%.4f,%.4f\n",
        (unsigned long)record.timestampUs, (unsigned long)record.sequence,
        unsigned(record.distanceMm), unsigned(record.status), unsigned(record.i2cStatus),
        record.signalMcps, record.ambientMcps);
    if (length <= 0 || length >= int(sizeof(row))
        || file_.write(row, size_t(length)) != size_t(length)) {
      fail();
      return;
    }
    tail_ = (tail_ + 1) % CAPACITY;
    --count_; // Only discard after a successful row write.
  } else if (stopping_) {
    if (!file_.close()) fail();
    fileOpen_ = false;
    stopping_ = false;
  } else if (uint32_t(millis() - previousSyncMs_) >= 1000) {
    if (!file_.sync()) fail();
    previousSyncMs_ = millis();
  }
}

} // namespace itc
