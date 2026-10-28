#pragma once

#include <CRSFforArduino.hpp>
#include "Types.h"
#include "Config.h"

namespace itc {

// Owns CRSF communication. Other modules receive a named RcState record.
class Receiver {
 public:
  Receiver();
  bool begin();     // Start communication with the receiver.
  RcState update(); // Process waiting bytes and return the latest safe commands.

  bool started() const {
    return started_;
  }

 private:
  CRSFforArduino device_;
  RcState state_;
  bool started_ = false;

  // The library needs a plain callback function. active_ tells that function
  // which Receiver object to update. This project uses one receiver.
  static Receiver* active_;
  static void onFrame(serialReceiverLayer::rcChannels_t* receivedFrame);
};

} // namespace itc
