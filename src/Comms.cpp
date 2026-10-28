#include "Comms.h"
#include <MAVLink.h>
#include <Arduino.h>
#include <string.h>

namespace itc {
void MavLink::sendHeartbeat(){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0, MAV_STATE_STANDBY);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.println("heartbeat sent");
  Serial8.write(buf, len);
}

void MavLink::sendFloat(uint32_t time_boot_ms, const char *name, float value){

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t msg;

  char messageName[10] = {};
  strncpy(messageName, name, sizeof(messageName) - 1);

  mavlink_msg_named_value_float_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, time_boot_ms, messageName, value);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial.println("value sent");
  Serial8.write(buf, len);

}

}
