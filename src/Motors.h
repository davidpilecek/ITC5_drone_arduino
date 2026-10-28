#pragma once

#include "Types.h"
#include "SafetyLogic.h"
#include "BidirectionalDShot/bidirectional_dshot.h"

namespace itc {

// The ONLY application module allowed to send commands to the ESC driver.
// The main loop proposes commands. This module checks permission independently.
class Motors {
 public:
  void begin();
  void publishReceiver(const RcState& pilotCommands);
  void publishControl(const ControlFrame& controlProposal);
  MotorStatus status() const;
  void readErpm(uint32_t electricalRpm[MOTOR_COUNT]);

 private:
  DshotManager driver_;
  ArmGate gate_;

  // Shared mailbox between the main loop and the timer interrupt.
  // "volatile" does NOT make a complete record atomic: the .cpp file also uses
  // short interrupt-disabled sections when copying these related values.
  volatile bool controlReady_ = false;
  volatile bool heightReady_ = false;
  volatile bool receiverValid_ = false;
  volatile bool armed_ = false;

  volatile uint32_t controlCycleStartedUs_ = 0;
  volatile uint32_t lastHeightSampleUs_ = 0;
  volatile uint32_t lastReceiverFrameUs_ = 0;
  volatile uint16_t armChannelUs_ = 1000;
  volatile uint16_t throttleChannelUs_ = 1000;

  volatile uint16_t requestedCommands_[MOTOR_COUNT] = {};
  volatile uint16_t appliedCommands_[MOTOR_COUNT] = {};

  // The driver expects a plain callback. This pointer identifies our one manager.
  static Motors* activeMotorManager_;
  static void onFrame();
};

} // namespace itc
