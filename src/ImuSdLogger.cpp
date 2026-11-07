#include "ImuSdLogger.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

namespace itc {
namespace {
// RAM2 keeps the buffer out of the fast RAM1 used by control and the stack.
// No DMA channel from the motor driver is allocated by this FIFO-SDIO logger.
#if defined(__IMXRT1062__)
DMAMEM ImuLogRecord sampleStorage[config::SD_IMU_BUFFER_SAMPLES];
#else
ImuLogRecord sampleStorage[config::SD_IMU_BUFFER_SAMPLES];
#endif
static_assert(config::SD_IMU_BUFFER_SAMPLES >= 8, "Need at least one sector of samples.");
static_assert(config::SD_IMU_SAMPLE_PERIOD_US > 0, "Log sample period must be positive.");
static_assert(sizeof(sampleStorage) <= 256 * 1024, "Keep the log queue within 256 KiB of RAM2.");
}

ImuSdLogger::ImuSdLogger() : queue_(sampleStorage, config::SD_IMU_BUFFER_SAMPLES) {}

void ImuSdLogger::begin(bool enabled, uint32_t periodUs) {
  samplePeriodUs_ = periodUs;
  enabled_ = enabled;
  if (enabled_) {
    requestStart(); // Does not touch the card here.
  }
}

void ImuSdLogger::requestStart() {
  if (enabled_ && !failed_ && !fileOpen_ && !recording_) {
    startRequested_ = true;
  }
}

void ImuSdLogger::requestStop() {
  startRequested_ = false;
  recording_ = false;
  stopping_ = fileOpen_;
}

void ImuSdLogger::sample(uint32_t sampleTimeUs, const ImuSample& imu, const Attitude& attitude) {
  if (!recording_) {
    return;
  }
  const uint32_t elapsedUs = uint32_t(sampleTimeUs - previousSampleUs_);
  if (samplePeriodUs_ != 0 && elapsedUs < samplePeriodUs_) {
    return;
  }
  previousSampleUs_ = sampleTimeUs;
  // We log the real new observation once. Do not invent catch-up measurements.
  const uint32_t skipped = samplePeriodUs_ == 0 ? 0 : elapsedUs / samplePeriodUs_ - 1;
  sequence_ += skipped;
  droppedSamples_ += skipped;

  ImuLogRecord record;
  record.timestampUs = sampleTimeUs;
  record.sequence = sequence_++;
  for (unsigned axis = 0; axis < 3; ++axis) {
    record.accelG[axis] = imu.accelG[axis];
    record.gyroDps[axis] = imu.gyroDps[axis];
    record.magUt[axis] = imu.magValid ? imu.magUt[axis] : NAN;
  }
  record.attitudeDeg[0] = attitude.rollDeg;
  record.attitudeDeg[1] = attitude.pitchDeg;
  record.attitudeDeg[2] = attitude.yawDeg;
  record.flags = 1u | (imu.magValid ? 2u : 0u) | (imu.magCalibrated ? 4u : 0u)
      | (attitude.valid ? 8u : 0u);
  if (!queue_.push(record)) {
    ++droppedSamples_;
  }
}

bool ImuSdLogger::openNewFile() {
  if (!cardStarted_) {
    // Built-in SD slot uses its own 4-bit SDIO bus, not the IMU's SPI1 bus.
    cardStarted_ = sd_.begin(SdioConfig(FIFO_SDIO));
    if (!cardStarted_) {
      return false;
    }
  }
  bool opened = false;
  for (unsigned index = 0; index < 10000; ++index) {
    snprintf(filename_, sizeof(filename_), "IMU%04u.BIN", index);
    if (!sd_.exists(filename_)) {
      // Exclusive creation prevents overwriting an existing recording.
      opened = file_.open(filename_, O_WRONLY | O_CREAT | O_EXCL);
      break;
    }
  }
  if (!opened) {
    return false;
  }
  fileOpen_ = true;
  ImuLogHeader header;
  header.samplePeriodUs = samplePeriodUs_;
  header.capacity = config::SD_IMU_BUFFER_SAMPLES;
  if (file_.write(&header, sizeof(header)) != sizeof(header) || !file_.sync()) {
    return false;
  }
  queue_.clear();
  sequence_ = 0;
  droppedSamples_ = 0;
  previousSampleUs_ = micros();
  previousWriteMs_ = previousSyncMs_ = millis();
  recording_ = true;
  stopping_ = false;
  return true;
}

void ImuSdLogger::fail() {
  failed_ = true;
  recording_ = false;
  startRequested_ = false;
  stopping_ = false;
  report("failed; recording stopped until reboot");
  // Do not repeatedly retry a failed/removed card. Reboot after fixing it.
  // Avoid another possibly blocking filesystem call just to handle an error.
}

void ImuSdLogger::report(const char* event) {
  char line[180];
  const int length = snprintf(line, sizeof(line),
      "SD IMU %s: file=%s queued=%u dropped=%lu max_io_us=%lu\n",
      event, filename_, queue_.size(), (unsigned long)droppedSamples_,
      (unsigned long)maximumIoUs_);
  if (Serial && length > 0 && length < int(sizeof(line))
      && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
  }
}

void ImuSdLogger::measureIo(uint32_t startedUs) {
  const uint32_t durationUs = uint32_t(micros() - startedUs);
  if (durationUs > maximumIoUs_) {
    maximumIoUs_ = durationUs;
  }
}

void ImuSdLogger::service(uint32_t nowMs, bool safeToAccessCard) {
  // This check must come BEFORE any SD call, including isBusy() and sync().
  if (!enabled_ || !safeToAccessCard || failed_) {
    return;
  }
  if (startRequested_) {
    startRequested_ = false;
    const uint32_t startedUs = micros();
    if (!openNewFile()) {
      fail();
    }
    measureIo(startedUs);
    if (recording_) report("started");
    return;
  }
  if (!fileOpen_) {
    return;
  }
  const uint32_t startedUs = micros();
  if (file_.isBusy()) {
    measureIo(startedUs);
    return;
  }

  const bool partialSectorDue = stopping_
      || uint32_t(nowMs - previousWriteMs_) >= config::SD_IMU_PARTIAL_WRITE_MS;
  if (queue_.size() >= 8 || (queue_.size() != 0 && partialSectorDue)) {
    // Write at most one 512-byte sector per service call, all while disarmed.
    const unsigned count = queue_.size() < 8 ? queue_.size() : 8;
    for (ImuLogRecord& padding : sector_) {
      padding = ImuLogRecord{}; // zero flags mark padding records
    }
    for (unsigned index = 0; index < count; ++index) {
      sector_[index] = queue_.peek(index);
    }
    if (file_.write(sector_, sizeof(sector_)) != sizeof(sector_)) {
      fail();
    } else {
      queue_.discard(count);
      previousWriteMs_ = nowMs;
    }
  } else if (stopping_ && queue_.size() == 0) {
    // close() synchronizes file metadata. Never close/sync while armed.
    if (!file_.close()) {
      fail();
    }
    fileOpen_ = false;
    stopping_ = false;
    if (!failed_) report("closed; safe to power off");
  } else if (uint32_t(nowMs - previousSyncMs_) >= config::SD_IMU_SYNC_MS) {
    if (!file_.sync()) {
      fail();
    }
    previousSyncMs_ = nowMs;
  }
  measureIo(startedUs);
}

} // namespace itc
