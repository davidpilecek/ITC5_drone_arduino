#include <Comms.h>
#include <Sensors.h>
#include <Control.h>

// Including a header exposes class names. These lines create objects.
// The itc:: prefix means the class belongs to the project's namespace.
itc::Imu imu;
itc::AttitudeEstimator estimator;
itc::MavLink Link1;

uint32_t previousUs = 0;
uint32_t lastMillisMav = 0;

uint32_t previousSampleUs = 0;
uint32_t previousPrintMs = 0;

// Read frequently for estimation; print slowly so the output is readable.
constexpr uint32_t SAMPLE_PERIOD_US = 1000; // 1000 readings per second
constexpr uint32_t PRINT_PERIOD_MS = 100;  // 10 printed lines per second

void setup() {
  Serial.begin(itc::config::USB_BAUD);
  Serial8.begin(itc::config::MAV_BAUD);

  const uint32_t waitStartedMs = millis();
  while (!Serial && uint32_t(millis() - waitStartedMs) < 3000) {
    delay(10);
  }

  if (!imu.begin()) {
    while (true) {
      Serial.println("IMU initialization failed. Check SPI wiring and power.");
      delay(1000);
    }
  }

  Serial.println("IMU monitor: acceleration in g, angular rates in deg/s.");
  Serial.println("Angles use the drone estimator and existing Config.h offsets.");
  Serial.println("ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,yaw_deg");
  previousSampleUs = micros();
  lastMillisMav = millis();

}

void loop() {
  const uint32_t nowMav = millis();
  const uint32_t nowUs = micros();

  const uint32_t elapsedUs = uint32_t(nowUs - previousSampleUs);
  if (elapsedUs < SAMPLE_PERIOD_US) {
    return; // Arduino calls loop() again; no new sample is due yet.
  }
  previousSampleUs = nowUs;
  const float elapsedSeconds = elapsedUs * 1e-6f;

  // Reading the device and estimating angles are separate operations.
  const itc::ImuSample measurement = imu.read();
  const itc::Attitude& attitude = estimator.update(measurement, elapsedSeconds);

  if (nowMav - lastMillisMav >= PRINT_PERIOD_MS){
    // Link1.sendHeartbeat();
    Link1.sendFloat(nowMav, "roll", attitude.rollDeg);
    Link1.sendFloat(nowMav, "pitch", attitude.pitchDeg);
    Link1.sendFloat(nowMav, "yaw", attitude.yawDeg);
    lastMillisMav = nowMav;
  }

}
