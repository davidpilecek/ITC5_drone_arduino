#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <VL53L1X.h>
#include "MPU9250/MPU9250.h"
#include "Types.h"
#include "Config.h"

namespace itc {

// Offsets measured while the IMU is stationary and level. Array order is X/Y/Z.
struct ImuCalibration {
  float accelBiasG[3] = {};
  float gyroBiasDps[3] = {};
};

// Reads raw acceleration and rotation, converts units and subtracts offsets.
// Estimating angles is a separate job, done by AttitudeEstimator.
class Imu {
 public:
  Imu();
  bool begin();
  ImuSample read();
  ImuCalibration calibrate(unsigned numberOfSamples = 12000);

 private:
  MPU9250 device_;
};

// Handles the distance sensor and corrects its reading for vehicle tilt.
class RangeSensor {
 public:
  bool begin();
  const RangeState& update(const Attitude& attitude, uint32_t currentTimeUs);

  const RangeState& state() const {
    return state_;
  }

 private:
  VL53L1X device_;
  RangeState state_;
  bool accepted_ = false;
  uint32_t previousPollTimeUs_ = 0;
  uint32_t previousFilterTimeUs_ = 0;
};

} // namespace itc
