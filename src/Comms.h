#pragma once
#include <stdint.h>

namespace itc {
class MavLink{
 public:
  void begin();
  // true = queued to Serial8; false = UART busy, so the caller should retry.
  // Defaults keep the original learning examples source-compatible.
  bool sendHeartbeat(bool armed = false, uint8_t outputMode = 0, bool healthy = true);
  bool sendFloat(uint32_t time_boot_ms, const char *name, float value);
};
}
