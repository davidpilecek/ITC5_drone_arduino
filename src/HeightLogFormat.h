#pragma once

#include <stdint.h>

namespace itc {

// A direct VL53L1X observation kept in RAM until the test writes its CSV row.
// Unlike ImuLogRecord, this is not a binary file format.
struct HeightLogRecord {
  uint32_t timestampUs = 0;
  uint32_t sequence = 0;
  uint16_t distanceMm = 0; // Exactly what VL53L1X::read(false) returned.
  uint8_t status = 255;
  uint8_t i2cStatus = 0;
  float signalMcps = 0;    // Driver-supplied signal count rate.
  float ambientMcps = 0;   // Driver-supplied ambient count rate.
};

} // namespace itc
