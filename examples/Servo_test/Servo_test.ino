#include <Servo.h>

Servo servo;  // create Servo object to control a servo

int pos = 0;    // variable to store the servo position
int feedbackPin = A5;

// Calibration values
int minDegrees;
int maxDegrees;
int minFeedback;
int maxFeedback;
int tolerance = 2; // max feedback measurement error

int pulseUs = 1500;

void calibrate(Servo servo, int analogPin, int minPos, int maxPos)
{
  // Move to the minimum position and record the feedback value
  servo.write(minPos);
  minDegrees = minPos;
  delay(2000); // make sure it has time to get there and settle
  minFeedback = analogRead(analogPin);
  
  // Move to the maximum position and record the feedback value
  servo.write(maxPos);
  maxDegrees = maxPos;
  delay(2000); // make sure it has time to get there and settle
  maxFeedback = analogRead(analogPin);
}

void Seek(Servo servo, int analogPin, int pos)
{

  // Start the move...
  servo.write(pos);
  
  // Calculate the target feedback value for the final position
  int target = map(pos, minDegrees, maxDegrees, minFeedback, maxFeedback); 
  
  // Wait until it reaches the target
  while(abs(analogRead(analogPin) - target) > tolerance){} // wait...
}

int getPos(int analogPin)
{
  return map(analogRead(analogPin), minFeedback, maxFeedback, minDegrees, maxDegrees);
}

void setup() {
  Serial.begin(9600);
  Serial.println("begin");

  servo.attach(9, 500, 2500);  // attaches the servo on pin 9 to the Servo object
  servo.writeMicroseconds(pulseUs);

  // delay(1000);
  // Serial.println("delay");
  // calibrate(myservo, feedbackPin, 0, 180);  // calibrate for the 20-160 degree range

}

void loop() {

  // if (!Serial.available()) return;

  // char command = Serial.read();

  // if      (command == '+') pulseUs += 10;
  // else if (command == '-') pulseUs -= 10;
  // else if (command == 'c') pulseUs = 1500;
  // else return;
// 510
// 1640

  // pulseUs = constrain(pulseUs, 500, 2500);
  servo.writeMicroseconds(480);
  Serial.println(pulseUs);
  delay(2000);

  servo.writeMicroseconds(2500);
  Serial.println(pulseUs);
  delay(2000);
}
