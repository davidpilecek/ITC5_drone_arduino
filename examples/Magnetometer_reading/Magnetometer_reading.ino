// Standalone monitor: creates no motor objects or motor outputs.
#include <Sensors.h>
itc::Imu imu;
void setup() {
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && uint32_t(millis() - start) < 3000) delay(10);
  if (!imu.begin()) {
    Serial.println("IMU initialization failed.");
    while (true) delay(1000);
  }
  Serial.println("Mag calibration: rotate slowly through ALL orientations for 30 seconds.");
  Serial.println("Keep motors off and stay away from magnets and steel objects.");
  const auto calibration = imu.calibrateMagnetometer(30000);
  if (!calibration.valid) {
    Serial.println("Calibration failed. Repeat with greater rotation coverage.");
  } else {
    Serial.println("Calibration applied. Save these coefficients for future boots:");
    for (unsigned axis = 0; axis < 3; ++axis) {
      Serial.print("axis="); Serial.print(axis);
      Serial.print(" biasUt="); Serial.print(calibration.biasUt[axis], 6);
      Serial.print(" scale="); Serial.println(calibration.scale[axis], 6);
    }
  }
  Serial.println("mx_uT,my_uT,mz_uT,calibrated");
}
void loop() {
  const auto sample = imu.read();
  if (sample.magValid) {
    Serial.print(sample.magUt[0], 3); Serial.print(',');
    Serial.print(sample.magUt[1], 3); Serial.print(',');
    Serial.print(sample.magUt[2], 3); Serial.print(',');
    Serial.println(sample.magCalibrated);
  }
  delay(20);
}
