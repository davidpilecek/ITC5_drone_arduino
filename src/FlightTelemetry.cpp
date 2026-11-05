#include "FlightTelemetry.h"
#include "Config.h"
#include <string.h>

namespace itc {

// Budget for the whole fixed queue, even if every packet uses all 30 bytes.
// UART 8N1 uses ten wire bits per byte. Reserve at least 25% for headroom.
static_assert(config::MAV_DIAGNOSTIC_PERIOD_MS > 0 && config::MAV_HEARTBEAT_PERIOD_MS > 0,
              "Telemetry periods must be positive.");
static_assert((uint64_t(FlightTelemetry::MAX_VALUES) * 30 * 1000 / config::MAV_DIAGNOSTIC_PERIOD_MS
              + 21ULL * 1000 / config::MAV_HEARTBEAT_PERIOD_MS) * 10
              <= config::MAV_BAUD * 3ULL / 4,
              "Diagnostic rate exceeds the UART budget. Increase the snapshot period.");

void FlightTelemetry::begin() {
  if (!config::MAV_ENABLED) {
    return;
  }
  link_.begin();
  // The first update sends a heartbeat; subsequent ones are one second apart.
  previousHeartbeatMs_ = uint32_t(0 - config::MAV_HEARTBEAT_PERIOD_MS);
}

bool FlightTelemetry::snapshotDue(uint32_t nowMs) const {
  // Finish the current snapshot first. Never accumulate an unbounded backlog.
  return config::MAV_ENABLED && nextValue_ >= numberOfValues_
      && uint32_t(nowMs - previousSnapshotMs_) >= config::MAV_DIAGNOSTIC_PERIOD_MS;
}

void FlightTelemetry::beginSnapshot(uint32_t nowMs) {
  snapshotTimeMs_ = nowMs;
  previousSnapshotMs_ = nowMs;
  numberOfValues_ = 0;
  nextValue_ = 0;
}

void FlightTelemetry::appendValue(const char* name, float value) {
  if (numberOfValues_ >= MAX_VALUES) {
    return;
  }
  NamedValue& field = values_[numberOfValues_++];
  memset(field.name, 0, sizeof(field.name));
  strncpy(field.name, name, sizeof(field.name) - 1);
  field.value = value;
}

void FlightTelemetry::update(uint32_t nowMs, bool armed, uint8_t mode, bool healthy) {
  if (!config::MAV_ENABLED) {
    return;
  }

  // Heartbeats take priority and report the latest state, not an old snapshot.
  if (uint32_t(nowMs - previousHeartbeatMs_) >= config::MAV_HEARTBEAT_PERIOD_MS) {
    if (link_.sendHeartbeat(armed, mode, healthy)) {
      previousHeartbeatMs_ = nowMs;
    }
    return;
  }

  if (nextValue_ < numberOfValues_) {
    const NamedValue& field = values_[nextValue_];
    if (link_.sendFloat(snapshotTimeMs_, field.name, field.value)) {
      ++nextValue_;
    }
  }
}

} // namespace itc
