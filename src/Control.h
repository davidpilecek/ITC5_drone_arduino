#pragma once

#include "Config.h"
#include "Types.h"
#include "HeightController.h"

namespace itc {

// This file describes the CONTROL module's public operations.
// Control.cpp performs only calculations: it never reads a device or drives an ESC.

// Estimates vehicle orientation from acceleration and angular-rate measurements.
class AttitudeEstimator {
 public:
  const Attitude& update(const ImuSample& measurement, float elapsedSeconds);

 private:
  Attitude state_;

  // A quaternion is a four-number representation of orientation.
  // These traditional mathematical names are used only in the advanced estimator.
  float q0 = 1;
  float q1 = 0;
  float q2 = 0;
  float q3 = 0;

  void integrate(float gx, float gy, float gz, float ax, float ay, float az, float dt);
};

// Calculates roll/pitch/yaw corrections from desired and measured motion.
class AttitudeController {
 public:
  Effort update(
      const Setpoints& desired,
      const Attitude& measured,
      float elapsedSeconds,
      bool resetIntegral);

  void reset(float yawRateError);

 private:
  float integralRoll_ = 0;
  float integralPitch_ = 0;
  float integralYaw_ = 0;
  float previousYawError_ = 0;
};

// Convert radio channels into requested angles, yaw rate and common throttle.
Setpoints mapCommands(const RcState& pilotCommands);

// Add/subtract attitude corrections to obtain four separate motor commands.
MotorCommands mixQuad(float collectiveThrottle, const Effort& corrections);

// The simpler bench mode sends equal, limited throttle to all four motors.
MotorCommands benchCommands(float collectiveThrottle, bool throttleAboveMinimum);

// May the height controller still use the latest accepted measurement?
bool usableHeight(const RangeState& heightReading, const Attitude& attitude, uint32_t currentTimeUs);

} // namespace itc
