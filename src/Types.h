#pragma once

#include <stdint.h>
#include <math.h>

namespace itc {

// These records are the information passed between modules.
// They contain data; they do not read sensors or drive motors.
// Unit suffixes: Us = microseconds, Ms = milliseconds, M = metres,
//               Deg = degrees, Dps = degrees per second, G = gravity units.

// All motor arrays use this order. The pins are listed in Config.h.
enum class Motor : uint8_t {
  FrontLeft,
  FrontRight,
  RearLeft,
  RearRight
};
constexpr unsigned MOTOR_COUNT = 4;

// Keep a number between two limits. This does not change numbers inside them.
inline float clampValue(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

// One IMU reading. Array positions are X=0, Y=1, Z=2.
struct ImuSample {
  float accelG[3] = {};
  float gyroDps[3] = {};
};

// The estimator's best current description of the vehicle's orientation.
struct Attitude {
  float rollDeg = 0;
  float pitchDeg = 0;
  float yawDeg = 0;
  ImuSample filtered;
  bool valid = false;  // Finite calculation result; not a complete sensor health check.
};

// The pilot's six channels, converted from CRSF to a 1000..2000 convention.
// Safe starting values mean level commands, zero throttle and disarmed.
struct RcState {
  uint16_t rollUs = 1500;
  uint16_t pitchUs = 1500;
  uint16_t throttleUs = 1000;
  uint16_t yawUs = 1500;
  uint16_t armUs = 1000;
  uint16_t auxUs = 2000;
  uint32_t receivedUs = 0;  // When the last accepted receiver frame arrived.
  bool valid = false;
};

// What the controller is being asked to achieve.
struct Setpoints {
  float collective = 0;  // Common motor command, 0..1. Not a power or RPM percentage.
  float rollDeg = 0;
  float pitchDeg = 0;
  float yawDps = 0;      // Yaw uses a requested turning RATE, not a compass heading.
};

// The attitude controller's three corrections, before combining them into motors.
struct Effort {
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
};

struct MotorCommands {
  uint16_t value[MOTOR_COUNT] = {};  // DShot command for FL, FR, RL, RR.
  bool valid = true;
};

// NAN means "no usable number yet"; it must never be treated as zero height.
struct RangeState {
  float slantM = NAN;      // Distance along the sensor's beam.
  float rawHeightM = NAN;  // Vertical distance after correcting for tilt.
  float heightM = NAN;     // Smoothed vertical distance.
  uint32_t acceptedUs = 0;
  uint8_t status = 255;    // Sensor library's measurement status.
  bool started = false;
  bool valid = false;      // Is the most recent range result acceptable?
};

// One complete proposal sent from the main loop to the motor safety guard.
struct ControlFrame {
  MotorCommands commands;
  uint32_t heartbeatUs = 0;    // When this control cycle started.
  uint32_t heightSampleUs = 0;
  bool ready = false;
  bool heightReady = false;
};

// What the motor manager is ACTUALLY allowing, after applying safety checks.
struct MotorStatus {
  bool armed = false;
  uint16_t applied[MOTOR_COUNT] = {};
};

struct Timing {
  uint32_t maxLoopUs = 0;
  uint32_t maxRangeUs = 0;
  uint32_t overruns = 0;  // Counts control-cycle gaps longer than 1000 microseconds.
};

} // namespace itc
