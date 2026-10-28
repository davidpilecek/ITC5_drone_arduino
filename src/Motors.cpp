#include "Motors.h"

namespace itc {

static_assert(DSHOT_ACTIVE_ESC == MOTOR_COUNT && DSHOT_BT_DURATION == 3333 && TX_WAIT == 1000,
              "Expected four-output DShot300 at 1 kHz.");
static_assert(DSHOT_FRAME_CALLBACK_API == 1 && DSHOT_ATOMIC_RPM_SNAPSHOT == 1,
              "Required driver APIs are missing.");

Motors* Motors::activeMotorManager_ = nullptr;

void Motors::begin() {
  activeMotorManager_ = this;
  for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor) {
    driver_.set_throttle_esc(motor, 0);
  }
  driver_.set_frame_callback(onFrame);
  driver_.start_tx();
}

// Copy one complete receiver snapshot without being interrupted halfway through.
void Motors::publishReceiver(const RcState& pilotCommands) {
  const uint32_t previousInterruptState = __get_primask();
  __disable_irq();

  receiverValid_ = pilotCommands.valid;
  lastReceiverFrameUs_ = pilotCommands.receivedUs;
  armChannelUs_ = pilotCommands.armUs;
  throttleChannelUs_ = pilotCommands.throttleUs;

  __set_primask(previousInterruptState);
}

void Motors::publishControl(const ControlFrame& controlProposal) {
  const uint32_t previousInterruptState = __get_primask();
  __disable_irq();

  controlReady_ = controlProposal.ready && controlProposal.commands.valid;
  heightReady_ = controlProposal.heightReady;
  controlCycleStartedUs_ = controlProposal.heartbeatUs;
  lastHeightSampleUs_ = controlProposal.heightSampleUs;
  for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor) {
    requestedCommands_[motor] = controlProposal.commands.value[motor];
  }

  // Restore the previous state, rather than blindly enabling interrupts.
  __set_primask(previousInterruptState);
}

MotorStatus Motors::status() const {
  MotorStatus snapshot;
  const uint32_t previousInterruptState = __get_primask();
  __disable_irq();

  snapshot.armed = armed_;
  for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor) {
    snapshot.applied[motor] = appliedCommands_[motor];
  }

  __set_primask(previousInterruptState);
  return snapshot;
}

void Motors::readErpm(uint32_t electricalRpm[MOTOR_COUNT]) {
  for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor) {
    electricalRpm[motor] = driver_.average_eRPM(motor);
  }
}

// ADVANCED: this callback runs inside the driver's 1 kHz timer interrupt.
// It must stay short: no sensor reads, USB printing or controller calculations.
void Motors::onFrame() {
  if (activeMotorManager_ == nullptr) {
    return;
  }
  Motors& motorManager = *activeMotorManager_;
  const uint32_t currentTimeUs = micros();

  bool systemReady = motorManager.controlReady_ && motorManager.receiverValid_
      && freshTimestamp(currentTimeUs, motorManager.lastReceiverFrameUs_, config::RC_TIMEOUT_US)
      && freshTimestamp(currentTimeUs, motorManager.controlCycleStartedUs_, config::CONTROL_TIMEOUT_US);

  if (ITC_OUTPUT_MODE == 3) {
    systemReady = systemReady && motorManager.heightReady_
        && freshTimestamp(currentTimeUs, motorManager.lastHeightSampleUs_, config::RANGE_STALE_US);
  }

  motorManager.armed_ = motorManager.gate_.update(
      systemReady, motorManager.armChannelUs_, motorManager.throttleChannelUs_);

  for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor) {
    if (ITC_OUTPUT_MODE != 0 && motorManager.armed_) {
      motorManager.appliedCommands_[motor] = motorManager.requestedCommands_[motor];
    } else {
      motorManager.appliedCommands_[motor] = 0;
    }
    motorManager.driver_.set_throttle_esc(motor, motorManager.appliedCommands_[motor]);
  }
}

} // namespace itc
