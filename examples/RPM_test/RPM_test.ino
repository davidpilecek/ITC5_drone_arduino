

const int SENSOR_PIN = 22;  // Arduino pin connected to the OUT pin of the speed sensor

int lastState;          // the previous state of the sensor
int objectCount = 0;    // how many times the slot was blocked

void setup() {
  Serial.begin(9600);
  pinMode(SENSOR_PIN, INPUT);
  lastState = digitalRead(SENSOR_PIN);
}

void loop() {
  int state = digitalRead(SENSOR_PIN);

  if (state != lastState) {  // the state has changed
    if (state == LOW) {      // LOW = something is in the slot
      objectCount++;
      Serial.print("The slot is blocked - count: ");
      Serial.println(objectCount);
    } else {                 // HIGH = the slot is clear
      Serial.println("The slot is clear");
    }
    lastState = state;
  }
}
