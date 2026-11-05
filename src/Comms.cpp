#include "Comms.h"
#include "Config.h"
#include <Arduino.h>
#include <string.h>
// Use the MAVLink copy already bundled in this library, consistently.
#include "MAVLink/MAVLink.h"

namespace itc {

void MavLink::begin() {
  Serial8.begin(config::MAV_BAUD);
}

bool MavLink::sendHeartbeat(bool armed, uint8_t outputMode, bool healthy) {
  // Check room BEFORE packing. A busy UART means retry on a later loop pass.
  // This module is the only Serial8 writer in the main flight application.
  constexpr unsigned maximumLength = MAVLINK_MSG_ID_HEARTBEAT_LEN
      + MAVLINK_NUM_NON_PAYLOAD_BYTES;
  if (Serial8.availableForWrite() < int(maximumLength)) {
    return false;
  }

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t baseMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
  if (armed) {
    baseMode |= MAV_MODE_FLAG_SAFETY_ARMED;
  }
  const uint8_t state = !healthy ? MAV_STATE_CRITICAL
      : (armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
  mavlink_msg_heartbeat_pack(config::MAV_SYSTEM_ID, config::MAV_COMPONENT_ID, &msg,
      MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, baseMode, outputMode, state);
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial8.write(buf, len);
  return true; // Queued locally; this is not a wireless delivery acknowledgement.
}

bool MavLink::sendFloat(uint32_t time_boot_ms, const char* name, float value) {
  constexpr unsigned maximumLength = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN
      + MAVLINK_NUM_NON_PAYLOAD_BYTES;
  if (name == nullptr || Serial8.availableForWrite() < int(maximumLength)) {
    return false;
  }

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  char messageName[10] = {};
  strncpy(messageName, name, sizeof(messageName) - 1);
  mavlink_msg_named_value_float_pack(config::MAV_SYSTEM_ID, config::MAV_COMPONENT_ID,
      &msg, time_boot_ms, messageName, value);
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial8.write(buf, len);
  return true;
}

} // namespace itc
