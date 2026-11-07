#pragma once

#include <SdFat.h>
#include "Config.h"
#include "Types.h"
#include "ImuLogFormat.h"

namespace itc {

// Use ONE logger per firmware image: its 256 KiB queue lives in Teensy's RAM2.
// sample() only copies RAM. service() owns ALL card/filesystem operations.
class ImuSdLogger {
 public:
  ImuSdLogger();
  // periodUs=0 logs every supplied observation; caller must supply fresh reads.
  // The flight application keeps the original configured period by default.
  void begin(bool enabled = config::SD_IMU_LOG_ENABLED,
             uint32_t periodUs = config::SD_IMU_SAMPLE_PERIOD_US);
  void sample(uint32_t sampleTimeUs, const ImuSample& imu, const Attitude& attitude);

  // Caller must allow this only while disarmed AND the radio arm switch is OFF.
  // Card operations can block: this is intentionally NOT an in-flight SD writer.
  void service(uint32_t nowMs, bool safeToAccessCard);
  void requestStart(); // Opening the next file is deferred to a safe service call.
  void requestStop();  // Stop capturing, then drain/sync/close when safe.

  bool recording() const { return recording_; }
  bool failed() const { return failed_; }
  bool closed() const { return !fileOpen_ && !startRequested_ && !failed_; }
  unsigned queuedSamples() const { return queue_.size(); }
  uint32_t droppedSamples() const { return droppedSamples_; }
  uint32_t maximumIoUs() const { return maximumIoUs_; }
  const char* filename() const { return filename_; }

 private:
  SdFs sd_;
  FsFile file_;
  ImuLogQueue queue_;
  // One sector staged from the queue; dequeue only after a successful write.
  ImuLogRecord sector_[8];
  bool enabled_ = false;
  bool startRequested_ = false;
  bool stopping_ = false;
  bool recording_ = false;
  bool failed_ = false;
  bool fileOpen_ = false;
  bool cardStarted_ = false;
  uint32_t previousSampleUs_ = 0;
  uint32_t samplePeriodUs_ = config::SD_IMU_SAMPLE_PERIOD_US;
  uint32_t previousWriteMs_ = 0;
  uint32_t previousSyncMs_ = 0;
  uint32_t sequence_ = 0;
  uint32_t droppedSamples_ = 0;
  uint32_t maximumIoUs_ = 0;
  char filename_[13] = {};

  bool openNewFile();
  void fail();
  void measureIo(uint32_t startedUs);
  void report(const char* event);
};

} // namespace itc
