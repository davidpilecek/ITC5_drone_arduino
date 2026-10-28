#include "Sensors.h"
#include "SafetyLogic.h"

namespace itc {

// ============================================================================
// PART 1: MPU6500 -- acceleration and rotation
// ============================================================================

Imu::Imu() : device_(SPI1, config::IMU_CS) {
  // Construct the driver with its SPI bus and chip-select pin.
}

bool Imu::begin() {
  SPI1.setMISO(config::IMU_MISO);
  SPI1.setMOSI(config::IMU_MOSI);
  SPI1.setSCK(config::IMU_SCK);

  // Each call returns a negative number if configuration failed.
  if (device_.begin6Axis() < 0) {
    return false;
  }
  if (device_.setGyroRange(device_.GYRO_RANGE_250DPS) < 0) {
    return false;
  }
  if (device_.setAccelRange(device_.ACCEL_RANGE_2G) < 0) {
    return false;
  }
  if (device_.setSrd(0) < 0) {
    return false;
  }
  return true;
}

ImuSample Imu::read() {
  int16_t rawAcceleration[3];
  int16_t rawRotation[3];
  device_.getMotion6(
      &rawAcceleration[0], &rawAcceleration[1], &rawAcceleration[2],
      &rawRotation[0], &rawRotation[1], &rawRotation[2]);

  // The sensor supplies integer counts. Convert them into physical units,
  // then remove the measured stationary offset for each axis.
  ImuSample measurement;
  for (unsigned axis = 0; axis < 3; ++axis) {
    measurement.accelG[axis] = rawAcceleration[axis] / config::ACCEL_LSB_PER_G
        - config::ACCEL_BIAS_G[axis];
    measurement.gyroDps[axis] = rawRotation[axis] / config::GYRO_LSB_PER_DPS
        - config::GYRO_BIAS_DPS[axis];
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
    const ImuSample measurement = read();
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
