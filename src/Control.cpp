// Estimator/controller equations derived from Nicholas Rehm's dRehmFlight.
// Original attribution and GPL terms are in COPYING.txt and THIRD_PARTY.md.
#include "Control.h"

namespace itc {

// ============================================================================
// PART 1: TRANSLATE THE PILOT'S STICKS INTO TARGETS
// ============================================================================

Setpoints mapCommands(const RcState& pilotCommands) {
  Setpoints desired;

  // Throttle goes from 1000..2000 to 0..1.
  desired.collective = clampValue((float(pilotCommands.throttleUs) - 1000) / 1000, 0, 1);

  // Angle sticks are centred on 1500: full left is -1, full right is +1.
  desired.rollDeg = clampValue((float(pilotCommands.rollUs) - 1500) / 500, -1, 1)
      * config::MAX_ROLL_DEG;
  desired.pitchDeg = clampValue((float(pilotCommands.pitchUs) - 1500) / 500, -1, 1)
      * config::MAX_PITCH_DEG;
  desired.yawDps = clampValue((float(pilotCommands.yawUs) - 1500) / 500, -1, 1)
      * config::MAX_YAW_DPS;

  return desired;
}

// ============================================================================
// PART 2: COMBINE COMMON THROTTLE AND ATTITUDE CORRECTIONS INTO FOUR MOTORS
// ============================================================================

MotorCommands mixQuad(float collectiveThrottle, const Effort& corrections) {
  // Same physical order everywhere: front-left, front-right, rear-left, rear-right.
  // Example: positive roll correction increases the two LEFT motor commands.
  const float mixedCommands[MOTOR_COUNT] = {
      collectiveThrottle - corrections.pitch + corrections.roll + corrections.yaw, // FL
      collectiveThrottle - corrections.pitch - corrections.roll - corrections.yaw, // FR
      collectiveThrottle + corrections.pitch + corrections.roll - corrections.yaw, // RL
      collectiveThrottle + corrections.pitch - corrections.roll + corrections.yaw  // RR
  };

  MotorCommands motorCommands;
  for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor) {
    if (!isfinite(mixedCommands[motor])) {
      motorCommands.valid = false;
      continue;
    }

    const float limitedCommand = clampValue(mixedCommands[motor], 0, 1);
    motorCommands.value[motor] = uint16_t(config::MOTOR_IDLE_COMMAND
        + limitedCommand * (2047 - config::MOTOR_IDLE_COMMAND));
  }
  return motorCommands;
}

MotorCommands benchCommands(float collectiveThrottle, bool throttleAboveMinimum) {
  MotorCommands motorCommands; // Starts with four zero commands.
  uint16_t benchCommand = 0;

  if (throttleAboveMinimum) {
    benchCommand = uint16_t(48 + collectiveThrottle * config::BENCH_MAX_COMMAND * (2047 - 48));
  }

  for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor) {
    motorCommands.value[motor] = benchCommand;
  }
  return motorCommands;
}

// ============================================================================
// PART 3: ATTITUDE CONTROLLER -- keep roll/pitch angles and yaw rate on target
// ============================================================================

void AttitudeController::reset(float yawRateError) {
  integralRoll_ = 0;
  integralPitch_ = 0;
  integralYaw_ = 0;
  previousYawError_ = yawRateError;
}

Effort AttitudeController::update(
    const Setpoints& desired,
    const Attitude& measured,
    float elapsedSeconds,
    bool resetIntegral) {

  const float rollError = desired.rollDeg - measured.rollDeg;
  const float pitchError = desired.pitchDeg - measured.pitchDeg;
  const float yawRateError = desired.yawDps - measured.filtered.gyroDps[2];

  // Integral terms remember persistent error. Clear them when control is inactive
  // or throttle is too low; otherwise accumulate error and keep it within limits.
  if (resetIntegral) {
    integralRoll_ = 0;
    integralPitch_ = 0;
    integralYaw_ = 0;
  } else {
    integralRoll_ = clampValue(integralRoll_ + rollError * elapsedSeconds,
        -config::ATTITUDE_I_LIMIT, config::ATTITUDE_I_LIMIT);
    integralPitch_ = clampValue(integralPitch_ + pitchError * elapsedSeconds,
        -config::ATTITUDE_I_LIMIT, config::ATTITUDE_I_LIMIT);
    integralYaw_ = clampValue(integralYaw_ + yawRateError * elapsedSeconds,
        -config::ATTITUDE_I_LIMIT, config::ATTITUDE_I_LIMIT);
  }

  // Roll/pitch damping comes directly from measured rotation speed.
  // Yaw instead controls a requested rotation rate, using its rate-error change.
  Effort corrections;
  corrections.roll = 0.01 * (config::ROLL_KP * rollError
      + config::ROLL_KI * integralRoll_
      - config::ROLL_KD * measured.filtered.gyroDps[0]);
  corrections.pitch = 0.01 * (config::PITCH_KP * pitchError
      + config::PITCH_KI * integralPitch_
      - config::PITCH_KD * measured.filtered.gyroDps[1]);
  corrections.yaw = 0.01 * (config::YAW_KP * yawRateError
      + config::YAW_KI * integralYaw_
      + config::YAW_KD * (yawRateError - previousYawError_) / elapsedSeconds);

  previousYawError_ = yawRateError;
  return corrections;
}

// ============================================================================
// PART 4: DECIDE WHETHER HEIGHT FEEDBACK CAN STILL BE USED
// ============================================================================

bool usableHeight(const RangeState& heightReading, const Attitude& attitude, uint32_t currentTimeUs) {
  // This permits a short hold of the LAST ACCEPTED height during a dropout.
  // It is intentionally different from RangeState.valid, which describes the latest result.
  return heightReading.started
      && isfinite(heightReading.heightM)
      && attitude.valid
      && fabsf(attitude.rollDeg) <= config::MAX_TILT_DEG
      && fabsf(attitude.pitchDeg) <= config::MAX_TILT_DEG
      && freshTimestamp(currentTimeUs, heightReading.acceptedUs, config::RANGE_STALE_US);
}

// ============================================================================
// PART 5: ATTITUDE ESTIMATOR -- first smooth readings, then fuse them
// ============================================================================

const Attitude& AttitudeEstimator::update(const ImuSample& measurement, float elapsedSeconds) {
  // A low-pass filter keeps part of the old value and part of the new value.
  for (unsigned axis = 0; axis < 3; ++axis) {
    state_.filtered.accelG[axis] = (1.0 - config::ACCEL_FILTER) * state_.filtered.accelG[axis]
        + config::ACCEL_FILTER * measurement.accelG[axis];
    state_.filtered.gyroDps[axis] = (1.0 - config::GYRO_FILTER) * state_.filtered.gyroDps[axis]
        + config::GYRO_FILTER * measurement.gyroDps[axis];
  }

  // These signs describe the existing sensor mounting convention.
  // Changing just one sign without checking the physical axes can reverse feedback.
  const float* acceleration = state_.filtered.accelG;
  const float* rotation = state_.filtered.gyroDps;
  integrate(rotation[0], -rotation[1], -rotation[2],
      -acceleration[0], acceleration[1], acceleration[2], elapsedSeconds);

  state_.valid = isfinite(state_.rollDeg) && isfinite(state_.pitchDeg) && isfinite(state_.yawDeg);
  return state_;
}

// ============================================================================
// ADVANCED MATH: QUATERNION UPDATE -- skip this on your first reading
// ============================================================================
// A quaternion stores orientation without some of the singularities of angles.
// Gyroscopes predict its motion; the gravity measurement corrects roll/pitch drift.
// There is no magnetometer here, so yaw has no absolute heading correction.
static float invSqrt(float value) {
  return 1.0f / sqrtf(value);
}

void AttitudeEstimator::integrate(float gx,float gy,float gz,float ax,float ay,float az,float dt) {
  // gx/gy/gz: angular rates; ax/ay/az: accelerations; dt: seconds elapsed.
  // These are the standard six-axis Madgwick quaternion equations.
  // The math and mounting signs are retained; do not tune individual coefficients.
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient descent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    // A perfectly aligned stationary sensor needs zero correction. Avoid 0 * infinity.
    const float gradientNorm = s0*s0 + s1*s1 + s2*s2 + s3*s3;
    recipNorm = gradientNorm > 0 ? invSqrt(gradientNorm) : 0; //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= config::AHRS_BETA * s0;
    qDot2 -= config::AHRS_BETA * s1;
    qDot3 -= config::AHRS_BETA * s2;
    qDot4 -= config::AHRS_BETA * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  state_.rollDeg = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  state_.pitchDeg = -asin(clampValue(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  state_.yawDeg = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees

}

} // namespace itc
