#pragma once

#include <stdint.h>
#include "Config.h"

namespace itc {
using namespace config;

// Is a timestamp younger than the allowed age?
// Unsigned subtraction also works when the microsecond counter wraps around.
constexpr bool freshTimestamp(uint32_t now, uint32_t timestamp, uint32_t maximumAge) {
  return uint32_t(now - timestamp) < maximumAge;
}

// Remembers whether the pilot completed the required arm-switch sequence.
// This object only decides permission; it does not talk to the ESCs.
struct ArmGate {
  bool armed = false;
  bool sawDisarm = false;
  bool previousArmHigh = false;

  constexpr bool update(bool systemReady, uint16_t armChannel, uint16_t throttleChannel) {
    const bool armSwitchHigh = armChannel >= ARM_HIGH_US;

    if (!systemReady) {
      // Any readiness failure removes permission and forgets the previous sequence.
      armed = false;
      sawDisarm = false;
      previousArmHigh = armSwitchHigh;
    } else if (armChannel <= ARM_LOW_US) {
      // The pilot has deliberately put the arm switch in its OFF position.
      armed = false;
      sawDisarm = true;
      previousArmHigh = false;
    } else {
      // Arm only on a NEW low-to-high movement, with low throttle.
      if (armSwitchHigh && !previousArmHigh) {
        armed = sawDisarm && throttleChannel <= THROTTLE_LOW_US;
        sawDisarm = false;
      }

      // An ambiguous/middle switch position removes permission as well.
      if (!armSwitchHigh) {
        armed = false;
      }
      previousArmHigh = armSwitchHigh;
    }

    return armed;
  }
};

} // namespace itc
