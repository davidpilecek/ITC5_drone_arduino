#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <ICM_20948.h>
#include "Types.h"
#include "Config.h"

namespace itc {

// Offsets measured while the IMU is stationary and level. Array order is X/Y/Z.
struct ImuCalibration {
  float accelBiasG[3] = {};
  float gyroBiasDps[3] = {};
};

// Rotate through all orientations for magnetometer calibration (motors off).
// Diagonal scale correction approximates axis-aligned soft-iron distortion.
struct MagnetometerCalibration {
  float biasUt[3] = {};
  float scale[3] = {1.0f, 1.0f, 1.0f};
  bool valid = false;
};

// Reads raw acceleration and rotation, converts units and subtracts offsets.
// Estimating angles is a separate job, done by AttitudeEstimator.
class Imu {
 public:
  Imu();
  // Fast mode is for standalone measurement tests: configurable SPI, DLPF bypass.
  // Default false preserves the flight application's existing configuration.
  bool begin(bool fastMeasurements = false, uint32_t measurementSpiHz = 7000000);
  const char* initializationStep() const { return initializationStep_; }
  const char* statusString() { return device_.statusString(); }
  ImuSample read();
  // Returns false unless the device signals new data. Never returns the cache.
  // One ready event is NOT proof that every axis group updated together.
  bool readFresh(ImuSample& measurement);
  ImuCalibration calibrate(unsigned numberOfSamples = 12000);
  // Blocking setup-only routine; rotates vehicle by hand for durationMs.
  // Returns and applies a valid result; failure leaves old calibration intact.
  MagnetometerCalibration calibrateMagnetometer(uint32_t durationMs = 30000);
  bool setMagnetometerCalibration(const MagnetometerCalibration& calibration);
  MagnetometerCalibration magnetometerCalibration() const { return magCalibration_; }

 private:
  ICM_20948_SPI device_;
  const char* initializationStep_ = "not started";
  bool started_ = false;
  bool hasSample_ = false;
  uint32_t lastSampleUs_ = 0;
  ImuSample lastSample_;
  MagnetometerCalibration magCalibration_;
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
