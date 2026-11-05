#include "Sensors.h"
#include "SafetyLogic.h"

namespace itc {

// ============================================================================
// PART 1: SparkFun ICM-20948 -- acceleration and rotation
// ============================================================================

Imu::Imu() = default;

bool Imu::begin() {
  started_ = false;
  hasSample_ = false;
  pinMode(config::IMU_CS, OUTPUT);
  digitalWrite(config::IMU_CS, HIGH);
  SPI1.setMISO(config::IMU_MISO);
  SPI1.setMOSI(config::IMU_MOSI);
  SPI1.setSCK(config::IMU_SCK);
  SPI1.begin();
  delay(100);
  // Same bus speed as the working standalone sketch.
  device_.begin(config::IMU_CS, SPI1, 100000);
  if (device_.status != ICM_20948_Stat_Ok) return false;
  ICM_20948_fss_t fullScale = {};
  fullScale.a = gpm2;
  fullScale.g = dps250;
  if (device_.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr,
      fullScale) != ICM_20948_Stat_Ok) return false;
  started_ = true;
  ImuSample firstSample;
  const uint32_t waitStartedMs = millis();
  while (!readFresh(firstSample)) {
    if (uint32_t(millis() - waitStartedMs) >= 100) {
      started_ = false;
      return false;
    }
    delay(1);
  }
  return true;
}

bool Imu::readFresh(ImuSample& measurement) {
  if (!started_ || !device_.dataReady()
      || device_.status != ICM_20948_Stat_Ok) return false;
  device_.getAGMT();
  if (device_.status != ICM_20948_Stat_Ok) return false;
  // SparkFun getters return mg and degrees/s, independent of full-scale range.
  const float accelG[3] = {
      device_.accX() * 0.001f, device_.accY() * 0.001f, device_.accZ() * 0.001f};
  const float gyroDps[3] = {device_.gyrX(), device_.gyrY(), device_.gyrZ()};
  for (unsigned axis = 0; axis < 3; ++axis) {
    measurement.accelG[axis] = accelG[axis] - config::ACCEL_BIAS_G[axis];
    measurement.gyroDps[axis] = gyroDps[axis] - config::GYRO_BIAS_DPS[axis];
  }
  // AK09916 ST1 bit 0 = data ready; ST2 bit 3 = magnetic overflow.
  // Magnetometer validity is separate from accelerometer/gyro validity.
  measurement.magValid = (device_.agmt.magStat1 & 0x01) != 0
      && (device_.agmt.magStat2 & 0x08) == 0;
  if (measurement.magValid) {
    const float raw[3] = {device_.magX(), device_.magY(), device_.magZ()};
    for (unsigned axis = 0; axis < 3; ++axis) {
      measurement.magRawUt[axis] = raw[axis];
      measurement.magUt[axis] = (raw[axis] - magCalibration_.biasUt[axis])
          * magCalibration_.scale[axis];
      measurement.magValid = measurement.magValid
          && isfinite(raw[axis]) && isfinite(measurement.magUt[axis]);
    }
    measurement.magCalibrated = measurement.magValid && magCalibration_.valid;
  }
  lastSample_ = measurement;
  lastSampleUs_ = micros();
  hasSample_ = true;
  return true;
}

ImuSample Imu::read() {
  ImuSample measurement;
  if (readFresh(measurement)) return measurement;
  // Reuse a recent reading when the control loop runs faster than the sensor.
  if (started_ && hasSample_ && uint32_t(micros() - lastSampleUs_) < 20000
      && (device_.status == ICM_20948_Stat_Ok
          || device_.status == ICM_20948_Stat_NoData)) {
    measurement = lastSample_;
    // No fresh AGMT transaction: do not advertise cached magnetic data as new.
    measurement.magValid = false;
    measurement.magCalibrated = false;
    return measurement;
  }
  // Nonfinite samples are rejected by the existing attitude estimator.
  for (unsigned axis = 0; axis < 3; ++axis) {
    measurement.accelG[axis] = NAN;
    measurement.gyroDps[axis] = NAN;
  }
  return measurement;
}

ImuCalibration Imu::calibrate(unsigned numberOfSamples) {
  ImuCalibration calibration;
  if (numberOfSamples == 0) {
    return calibration;
  }

  // Average many stationary readings. Existing offsets are added back because
  // this routine reports REPLACEMENT offsets, not corrections to the old ones.
  for (unsigned sampleNumber = 0; sampleNumber < numberOfSamples; ++sampleNumber) {
    ImuSample measurement;
    const uint32_t waitStartedMs = millis();
    // Average distinct updates, with a bounded wait on sensor failure.
    while (!readFresh(measurement)) {
      if (!started_ || uint32_t(millis() - waitStartedMs) >= 100) {
        for (unsigned axis = 0; axis < 3; ++axis) {
          calibration.accelBiasG[axis] = NAN;
          calibration.gyroBiasDps[axis] = NAN;
        }
        return calibration;
      }
      delay(1);
    }
    for (unsigned axis = 0; axis < 3; ++axis) {
      calibration.accelBiasG[axis] += measurement.accelG[axis] + config::ACCEL_BIAS_G[axis];
      calibration.gyroBiasDps[axis] += measurement.gyroDps[axis] + config::GYRO_BIAS_DPS[axis];
    }
  }

  for (unsigned axis = 0; axis < 3; ++axis) {
    calibration.accelBiasG[axis] /= numberOfSamples;
    calibration.gyroBiasDps[axis] /= numberOfSamples;
  }

  // A level stationary sensor should read +1 g on its Z axis, not zero.
  calibration.accelBiasG[2] -= 1.0f;
  return calibration;
}

bool Imu::setMagnetometerCalibration(const MagnetometerCalibration& calibration) {
  if (!calibration.valid) return false;
  for (unsigned axis = 0; axis < 3; ++axis) {
    if (!isfinite(calibration.biasUt[axis]) || !isfinite(calibration.scale[axis])
        || calibration.scale[axis] <= 0.0f) return false;
  }
  magCalibration_ = calibration;
  // Invalidate the cached result computed with the previous coefficients.
  lastSample_.magValid = false;
  lastSample_.magCalibrated = false;
  return true;
}

MagnetometerCalibration Imu::calibrateMagnetometer(uint32_t durationMs) {
  MagnetometerCalibration calibration;
  if (!started_ || durationMs < 1000 || durationMs > 300000) return calibration;
  float minimum[3] = {INFINITY, INFINITY, INFINITY};
  float maximum[3] = {-INFINITY, -INFINITY, -INFINITY};
  unsigned accepted = 0;
  const uint32_t startMs = millis();
  while (uint32_t(millis() - startMs) < durationMs) {
    ImuSample measurement;
    if (readFresh(measurement) && measurement.magValid) {
      for (unsigned axis = 0; axis < 3; ++axis) {
        const float value = measurement.magRawUt[axis];
        if (value < minimum[axis]) minimum[axis] = value;
        if (value > maximum[axis]) maximum[axis] = value;
      }
      ++accepted;
    }
    // Magnetometer updates more slowly than accel/gyro; avoid tight polling.
    delay(20);
  }
  if (accepted < 100) return calibration;
  float radius[3];
  float meanRadius = 0.0f;
  for (unsigned axis = 0; axis < 3; ++axis) {
    radius[axis] = (maximum[axis] - minimum[axis]) * 0.5f;
    // Reject stationary/poorly excited axes and degenerate fits.
    if (!isfinite(radius[axis]) || radius[axis] < 5.0f) return calibration;
    calibration.biasUt[axis] = (maximum[axis] + minimum[axis]) * 0.5f;
    meanRadius += radius[axis] / 3.0f;
  }
  for (unsigned axis = 0; axis < 3; ++axis) {
    calibration.scale[axis] = meanRadius / radius[axis];
  }
  calibration.valid = true;
  setMagnetometerCalibration(calibration);
  return calibration;
}

// ============================================================================
// PART 2: VL53L1X -- distance along the beam, then vertical height
// ============================================================================

bool RangeSensor::begin() {
  Wire1.setSCL(config::RANGE_SCL);
  Wire1.setSDA(config::RANGE_SDA);
  Wire1.begin();
  Wire1.setClock(config::RANGE_I2C_HZ);

  device_.setBus(&Wire1);
  device_.setTimeout(100);

  // Start the next configuration step only if the preceding one succeeded.
  state_.started = device_.init()
      && device_.setDistanceMode(VL53L1X::Long)
      && device_.setMeasurementTimingBudget(config::RANGE_PERIOD_MS * 1000);

  if (state_.started) {
    device_.startContinuous(config::RANGE_PERIOD_MS);
  }
  return state_.started;
}

const RangeState& RangeSensor::update(const Attitude& attitude, uint32_t currentTimeUs) {
  // 1. Decide whether the old result is still usable at this vehicle orientation.
  const bool orientationAcceptable = attitude.valid
      && fabsf(attitude.rollDeg) <= config::MAX_TILT_DEG
      && fabsf(attitude.pitchDeg) <= config::MAX_TILT_DEG;

  state_.valid = state_.started && accepted_ && orientationAcceptable
      && freshTimestamp(currentTimeUs, state_.acceptedUs, config::RANGE_STALE_US);

  // 2. Poll on a schedule. Returning the existing record is normal when no new
  // measurement is due; do not wait here for a 50 ms sensor conversion.
  const uint32_t timeSinceLastPollUs = uint32_t(currentTimeUs - previousPollTimeUs_);
  if (!state_.started || timeSinceLastPollUs < config::RANGE_POLL_US) {
    return state_;
  }
  previousPollTimeUs_ = currentTimeUs;

  const bool newMeasurementReady = device_.dataReady();
  if (device_.last_status != 0) {
    accepted_ = false;
    state_.valid = false;
    return state_;
  }
  if (!newMeasurementReady) {
    return state_;
  }

  // 3. Read the completed measurement and convert millimetres into metres.
  const uint16_t distanceMm = device_.read(false);
  state_.status = uint8_t(device_.ranging_data.range_status);
  state_.slantM = distanceMm * 0.001f;

  // Over level ground, tilting the downward sensor makes its beam path longer.
  // Multiplying by the two cosines estimates the vertical distance instead.
  state_.rawHeightM = state_.slantM
      * cosf(attitude.rollDeg * DEG_TO_RAD)
      * cosf(attitude.pitchDeg * DEG_TO_RAD);

  // 4. Reject bad status, inappropriate tilt and distances outside our envelope.
  const bool measurementAcceptable = device_.last_status == 0
      && !device_.timeoutOccurred()
      && device_.ranging_data.range_status == VL53L1X::RangeValid
      && orientationAcceptable
      && state_.slantM >= config::MIN_SLANT_M
      && state_.slantM <= config::MAX_SLANT_M
      && isfinite(state_.rawHeightM)
      && state_.rawHeightM >= 0
      && state_.rawHeightM <= config::MAX_HEIGHT_M;

  // 5. Smooth accepted readings. Restart the filter after invalid/stale data
  // rather than dragging an obsolete height into the new measurement.
  if (measurementAcceptable) {
    if (!state_.valid || !isfinite(state_.heightM)) {
      state_.heightM = state_.rawHeightM;
    } else {
      const float elapsedSeconds = uint32_t(currentTimeUs - previousFilterTimeUs_) * 1e-6f;
      const float filterWeight = elapsedSeconds / (config::HEIGHT_FILTER_TAU_S + elapsedSeconds);
      state_.heightM += filterWeight * (state_.rawHeightM - state_.heightM);
    }
    previousFilterTimeUs_ = currentTimeUs;
    state_.acceptedUs = currentTimeUs;
  }

  accepted_ = measurementAcceptable;
  state_.valid = measurementAcceptable;
  return state_;
}

} // namespace itc
