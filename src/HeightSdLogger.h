#pragma once

#include <SdFat.h>
#include "HeightLogFormat.h"

namespace itc {

// A small CSV logger for the STANDALONE height-sensor example.
// It may block on SD: do not put it into flight control without a timing design.
class HeightSdLogger {
 public:
  void begin();
  void requestStart();
  void requestStop();
  void sampleTof(const HeightLogRecord& reading);
  void service(bool safeToAccessCard);

  bool recording() const { return recording_; }
  bool failed() const { return failed_; }
  bool closed() const { return !fileOpen_ && !startRequested_ && !failed_; }
  unsigned queuedSamples() const { return count_; }
  uint32_t droppedSamples() const { return dropped_; }
  const char* filename() const { return filename_; }

 private:
  static constexpr unsigned CAPACITY = 64;
  HeightLogRecord records_[CAPACITY];
  unsigned head_ = 0;
  unsigned tail_ = 0;
  unsigned count_ = 0;
  uint32_t sequence_ = 0;
  uint32_t dropped_ = 0;
  bool recording_ = false;
  bool startRequested_ = false;
  bool stopping_ = false;
  bool fileOpen_ = false;
  bool failed_ = false;
  bool cardStarted_ = false;
  uint32_t previousSyncMs_ = 0;
  char filename_[13] = {};
  SdFs sd_;
  FsFile file_;

  bool openNewFile();
  void fail();
};

} // namespace itc
