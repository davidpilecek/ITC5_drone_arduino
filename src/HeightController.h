#pragma once

#include <math.h>
#include <stdint.h>
#include "Config.h"
#include "SafetyLogic.h"

namespace itc {
using namespace config;

// These checks run while compiling. They catch contradictory settings early.
static_assert(HEIGHT_HOVER_THROTTLE > 0 && HEIGHT_HOVER_THROTTLE < HEIGHT_THROTTLE_MAX,
              "Hover collective must be inside the controller output limits.");
static_assert(HEIGHT_THROTTLE_MAX <= 1 && HEIGHT_KP >= 0 && HEIGHT_KI >= 0 && HEIGHT_KD >= 0,
              "Height gains and normalized output bounds are invalid.");
static_assert(HEIGHT_SETPOINT_MAX_M <= MAX_HEIGHT_M && HEIGHT_SETPOINT_MAX_M > HEIGHT_MAX_ARM_READING_M,
              "The target ceiling must exceed resting clearance and stay inside the measurement envelope.");

// Small arithmetic helpers. They do not read hardware or remember state.
inline float heightClamp(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

inline float heightApproach(float currentValue, float targetValue, float maximumChange) {
  const float limitedChange = heightClamp(
      targetValue - currentValue, -maximumChange, maximumChange);
  return currentValue + limitedChange;
}

inline float heightRequestFromPulse(uint32_t throttlePulseUs) {
  // 1000 -> 0 m, 1500 -> 0.5 m, 2000 -> 1 m with the current settings.
  const float stickFraction = heightClamp(
      (float(throttlePulseUs) - 1000.0f) / 1000.0f, 0.0f, 1.0f);
  return stickFraction * HEIGHT_SETPOINT_MAX_M;
}

// The outer controller calculates COMMON THROTTLE from a requested height.
// It contains no Arduino calls, so its exact code can also run in desktop tests.
struct HeightController {
  enum Stage : uint8_t {
    WAITING = 0,   // Armed on the ground, waiting for a takeoff request.
    TRACKING = 1,  // Climbing, holding height or descending.
    LANDED = 2,    // Resting-height test completed; motors must remain stopped.
    FAULT = 3      // Invalid feedback; an arm-switch OFF action is needed to reset.
  };

  // Values shown in the console.
  Stage stage = WAITING;
  float requestedM = 0;    // What the pilot requested.
  float targetM = 0;       // The gradually changing target actually used by the controller.
  float groundM = NAN;     // Sensor height while resting when the vehicle was armed.
  float velocityMps = 0;   // Positive = climbing; negative = descending.
  float collective = 0;    // Calculated common throttle, in normalized command units.
  float integralTerm = 0;  // Memory of persistent height error.

  // Working memory retained between measurements.
  float previousHeightM = 0;
  uint32_t previousSampleUs = 0;
  uint32_t settlingSinceUs = 0;
  bool initialized = false;
  bool settling = false;

  bool faulted() const {
    return stage == FAULT;
  }

  bool tracking() const {
    return stage == TRACKING;
  }

  void reset() {
    stage = WAITING;
    requestedM = 0;
    targetM = 0;
    groundM = NAN;
    velocityMps = 0;
    collective = 0;
    integralTerm = 0;
    previousHeightM = 0;
    previousSampleUs = 0;
    settlingSinceUs = 0;
    initialized = false;
    settling = false;
  }

  float fail() {
    stage = FAULT;
    collective = 0;
    integralTerm = 0;
    return 0;
  }

  float update(
      uint32_t currentTimeUs,
      uint32_t measurementTimeUs,
      float measuredHeightM,
      float requestedHeightM,
      bool measurementUsable,
      bool armed,
      bool armSwitchOff) {

    // 1. The arm switch OFF action resets the controller, including any fault.
    if (armSwitchOff) {
      reset();
      return 0;
    }
    requestedM = heightClamp(requestedHeightM, 0, HEIGHT_SETPOINT_MAX_M);

    // 2. Never calculate an active motor output while disarmed.
    if (!armed) {
      collective = 0;
      integralTerm = 0;
      initialized = false;
      settling = false;
      if (!faulted()) {
        stage = WAITING;
        groundM = NAN;
      }
      return 0;
    }
    if (faulted()) {
      return 0;
    }

    // 3. A missing, invalid or old measurement cannot support height control.
    const bool measurementFresh = freshTimestamp(
        currentTimeUs, measurementTimeUs, RANGE_STALE_US);
    if (!measurementUsable || !isfinite(measuredHeightM)
        || !isfinite(requestedHeightM) || !measurementFresh) {
      return fail();
    }

    // 4. On first use, capture resting clearance. We ASSUME the vehicle is on
    // the ground when armed; the distance sensor cannot prove ground contact.
    if (!initialized) {
      if (measuredHeightM < MIN_SLANT_M || measuredHeightM > HEIGHT_MAX_ARM_READING_M) {
        return fail();
      }
      groundM = measuredHeightM;
      targetM = measuredHeightM;
      previousHeightM = measuredHeightM;
      previousSampleUs = measurementTimeUs;
      initialized = true;
    }

    if (stage == LANDED) {
      return 0;  // A fresh arm-switch cycle is required for another takeoff.
    }

    // 5. Do not start the motors just because the vehicle was armed.
    if (stage == WAITING) {
      if (requestedM <= groundM + HEIGHT_TAKEOFF_MARGIN_M) {
        previousSampleUs = measurementTimeUs;
        previousHeightM = measuredHeightM;
        velocityMps = 0;
        return 0;
      }
      targetM = measuredHeightM;
      stage = TRACKING;
    }

    // 6. Update only ONCE per new range sample. The main loop calls this much
    // faster than the range sensor supplies readings; repeated data is not new data.
    if (measurementTimeUs == previousSampleUs) {
      return collective;
    }
    const float sampleSeconds = uint32_t(measurementTimeUs - previousSampleUs) * 1e-6f;
    if (sampleSeconds <= 0 || sampleSeconds > RANGE_STALE_US * 1e-6f) {
      return fail();
    }
    previousSampleUs = measurementTimeUs;

    // Estimate vertical speed, then smooth it so sensor noise is less amplified.
    const float measuredSpeedMps = (measuredHeightM - previousHeightM) / sampleSeconds;
    previousHeightM = measuredHeightM;
    const float speedFilterWeight = sampleSeconds / (HEIGHT_VELOCITY_TAU_S + sampleSeconds);
    velocityMps += speedFilterWeight * (measuredSpeedMps - velocityMps);

    // 7. Move the target gradually. The sensor cannot physically reach zero
    // metres while the vehicle is resting on landing gear or hulls.
    const float reachableTargetM = heightClamp(requestedM, groundM, HEIGHT_SETPOINT_MAX_M);
    targetM = heightApproach(targetM, reachableTargetM, HEIGHT_TARGET_SLEW_MPS * sampleSeconds);
    const float heightErrorM = targetM - measuredHeightM;

    // 8. PI control plus vertical-speed damping:
    //    throttle = hover estimate + position correction + accumulated correction
    //               - upward-speed damping.
    const float candidateIntegral = heightClamp(
        integralTerm + HEIGHT_KI * heightErrorM * sampleSeconds,
        -HEIGHT_I_LIMIT, HEIGHT_I_LIMIT);
    const float throttleWithoutIntegral = HEIGHT_HOVER_THROTTLE
        + HEIGHT_KP * heightErrorM - HEIGHT_KD * velocityMps;

    // Limit both the throttle magnitude and how fast it changes.
    const float minimumAllowedThrottle = fmaxf(
        0, collective - HEIGHT_THROTTLE_SLEW_PER_S * sampleSeconds);
    const float maximumAllowedThrottle = fminf(
        HEIGHT_THROTTLE_MAX, collective + HEIGHT_THROTTLE_SLEW_PER_S * sampleSeconds);
    const float candidateThrottle = throttleWithoutIntegral + candidateIntegral;

    // Anti-windup: do not accumulate more error that pushes against an output
    // limit. Allow integration when it is inside the limits or helps it recover.
    const bool insideLimits = candidateThrottle <= maximumAllowedThrottle
        && candidateThrottle >= minimumAllowedThrottle;
    const bool helpsReduceHighOutput = candidateThrottle > maximumAllowedThrottle && heightErrorM < 0;
    const bool helpsRaiseLowOutput = candidateThrottle < minimumAllowedThrottle && heightErrorM > 0;
    if (insideLimits || helpsReduceHighOutput || helpsRaiseLowOutput) {
      integralTerm = candidateIntegral;
    }
    collective = heightClamp(
        throttleWithoutIntegral + integralTerm, minimumAllowedThrottle, maximumAllowedThrottle);

    // 9. Zero stick requests a descent, not an immediate motor cut in the air.
    // Stop only after a settled return near the captured resting height.
    const bool nearRestingHeight = requestedM <= HEIGHT_ZERO_REQUEST_M
        && targetM <= groundM + HEIGHT_LANDING_MARGIN_M
        && measuredHeightM <= groundM + HEIGHT_LANDING_MARGIN_M
        && fabsf(velocityMps) <= HEIGHT_LANDING_SPEED_MPS;

    if (!nearRestingHeight) {
      settling = false;
    } else if (!settling) {
      settling = true;
      settlingSinceUs = measurementTimeUs;
    } else if (uint32_t(measurementTimeUs - settlingSinceUs) >= HEIGHT_SETTLE_US) {
      stage = LANDED;
      collective = 0;
      integralTerm = 0;
    }

    if (!isfinite(collective) || !isfinite(velocityMps)) {
      return fail();
    }
    return collective;
  }
};

} // namespace itc
