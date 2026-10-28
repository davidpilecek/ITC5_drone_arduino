#pragma once
#include <stdint.h>

namespace itc {
class MavLink{
 public:
  void sendHeartbeat();
  void sendFloat(uint32_t time_boot_ms, const char *name, float value);
};
}
