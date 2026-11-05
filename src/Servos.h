#pragma once

#include <Arduino.h>
#include <Servos.h>
#include "Types.h"
#include "Config.h"

namespace itc {

class Servo {
 public:
  Servo();
  bool begin();
 private:
};

} // namespace itc
