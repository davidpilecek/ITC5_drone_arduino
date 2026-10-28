#include <Comms.h>

itc::MavLink Link1;

uint32_t previousUs;
uint32_t lastMillisMav;

void setup() {
  Serial.begin(115200);
  Serial8.begin(115200);
  lastMillisMav = millis();
}


void loop() {
  const uint32_t nowMav = millis();

  if (nowMav - lastMillisMav >= 1000){
    Link1.sendHeartbeat();
    Link1.sendFloat(nowMav, "test", 3.14f);
    lastMillisMav = nowMav;
  }

}
