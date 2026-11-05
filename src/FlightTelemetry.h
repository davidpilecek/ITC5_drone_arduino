#pragma once

#include <stdint.h>
#include "Comms.h"

namespace itc {

// Schedules diagnostic packets. It never reads sensors or controls a motor.
class FlightTelemetry {
 public:
  static constexpr unsigned MAX_VALUES = 56;
  void begin();
  bool snapshotDue(uint32_t nowMs) const;
  void beginSnapshot(uint32_t nowMs);

  // Literal names are limited to nine characters plus the terminating zero.
  // A compile-time check catches a too-long name while adding a new field.
  template<unsigned N>
  void addValue(const char (&name)[N], float value) {
    static_assert(N <= 10, "MAVLink diagnostic names must have at most 9 characters.");
    appendValue(name, value);
  }

  // Queues at most ONE packet per call. A full UART never blocks control work.
  void update(uint32_t nowMs, bool armed, uint8_t mode, bool healthy);

 private:
  struct NamedValue {
    char name[10] = {};
    float value = 0;
  };
  NamedValue values_[MAX_VALUES];
  unsigned numberOfValues_ = 0;
  unsigned nextValue_ = 0;
  uint32_t snapshotTimeMs_ = 0;
  uint32_t previousSnapshotMs_ = 0;
  uint32_t previousHeartbeatMs_ = 0;
  MavLink link_;

  void appendValue(const char* name, float value);
};

} // namespace itc
