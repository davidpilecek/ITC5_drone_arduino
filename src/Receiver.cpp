#include "Receiver.h"
#include "SafetyLogic.h"

namespace itc {

Receiver* Receiver::active_ = nullptr;

Receiver::Receiver() : device_(&Serial5) {
  // Select the Teensy's Serial5 port; no communication starts until begin().
}

bool Receiver::begin() {
  active_ = this;
  device_.setRcChannelsCallback(onFrame);
  started_ = device_.begin();
  return started_;
}

// The CRSF library calls this function after decoding one complete channel frame.
void Receiver::onFrame(serialReceiverLayer::rcChannels_t* receivedFrame) {
  if (active_ == nullptr) {
    return;
  }
  Receiver& receiver = *active_;

  const uint8_t channelNumbers[6] = {
      config::RC_ROLL, config::RC_PITCH, config::RC_THROTTLE,
      config::RC_YAW, config::RC_ARM, config::RC_AUX
  };

  uint16_t convertedChannels[6];
  bool frameValid = !receivedFrame->failsafe;

  for (unsigned channel = 0; channel < 6; ++channel) {
    const uint16_t rawChannel = receivedFrame->value[channelNumbers[channel]];
    convertedChannels[channel] = receiver.device_.rcToUs(rawChannel);

    const bool rawValueInRange = rawChannel <= 2047;
    const bool convertedValueInRange = convertedChannels[channel] >= 800
        && convertedChannels[channel] <= 2200;
    frameValid = frameValid && rawValueInRange && convertedValueInRange;
  }

  receiver.state_.valid = frameValid;
  if (frameValid) {
    receiver.state_.rollUs = convertedChannels[0];
    receiver.state_.pitchUs = convertedChannels[1];
    receiver.state_.throttleUs = convertedChannels[2];
    receiver.state_.yawUs = convertedChannels[3];
    receiver.state_.armUs = convertedChannels[4];
    receiver.state_.auxUs = convertedChannels[5];
    receiver.state_.receivedUs = micros();
  }
}

RcState Receiver::update() {
  // The callback above runs during this call, in the main loop.
  if (started_) {
    device_.update();
  }

  const bool frameFresh = freshTimestamp(micros(), state_.receivedUs, config::RC_TIMEOUT_US);
  if (state_.valid && frameFresh) {
    RcState pilotCommands = state_;
    pilotCommands.rollUs = constrain(pilotCommands.rollUs, 1000, 2000);
    pilotCommands.pitchUs = constrain(pilotCommands.pitchUs, 1000, 2000);
    pilotCommands.throttleUs = constrain(pilotCommands.throttleUs, 1000, 2000);
    pilotCommands.yawUs = constrain(pilotCommands.yawUs, 1000, 2000);
    pilotCommands.auxUs = constrain(pilotCommands.auxUs, 1000, 2000);
    return pilotCommands;
  }

  // RcState's default values mean neutral angles, zero throttle, arm low, invalid link.
  return RcState{};
}

} // namespace itc
