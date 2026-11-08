// Standalone IMU + attitude timing check. No receiver, motor or SD objects.
// Uses the same sensor, estimator, target period and SPI clock as the main app.
#include <Sensors.h>
#include <Control.h>

itc::Imu imu;
itc::AttitudeEstimator estimator;
itc::Timing timing;

// Defaults follow the main program. For a local comparison, replace only this
// sketch's IMU_SPI_HZ with 100000; restore it afterwards for the 1 MHz test.
constexpr uint32_t IMU_SPI_HZ = itc::config::IMU_SPI_HZ;
constexpr uint32_t SAMPLE_PERIOD_US = itc::config::CONTROL_PERIOD_US;
constexpr uint32_t PRINT_PERIOD_MS = 100; // Print at 10 Hz, not at every sample.

uint32_t previousSampleUs = 0;
uint32_t previousPrintMs = 0;
uint32_t rateWindowStartUs = 0;
uint32_t rateWindowSamples = 0;

void setup() {
  Serial.begin(itc::config::USB_BAUD);
  const uint32_t waitStartedMs = millis();
  while (!Serial && uint32_t(millis() - waitStartedMs) < 3000) delay(10);

  // Match FlightApp startup: continuous operation, internal DLPFs bypassed.
  if (!imu.begin(true, IMU_SPI_HZ)) {
    while (true) {
      Serial.print("IMU initialization failed at: ");
      Serial.println(imu.initializationStep());
      Serial.print("Driver status: ");
      Serial.println(imu.statusString());
      delay(1000);
    }
  }

  Serial.print("Target sample period us: ");
  Serial.println(SAMPLE_PERIOD_US);
  Serial.print("IMU SPI clock Hz: ");
  Serial.println(IMU_SPI_HZ);
  Serial.println("Angles are degrees. Timings are measured wall-clock microseconds.");
  Serial.println("Maxima and overruns are cumulative since startup; control_hz updates about once a second.");
  Serial.println("time_ms,roll_deg,pitch_deg,yaw_deg,control_hz,imu_us,estimator_us,max_imu_us,max_estimator_us,max_work_us,max_loop_us,overruns");
  previousSampleUs = micros();
  rateWindowStartUs = previousSampleUs;
  previousPrintMs = millis();
}

void loop() {
  // Same nonblocking timing gate as the main application. A late sample is
  // processed once; there is no burst of invented catch-up estimates.
  const uint32_t cycleStartUs = micros();
  const uint32_t elapsedUs = uint32_t(cycleStartUs - previousSampleUs);
  if (elapsedUs < SAMPLE_PERIOD_US) return;
  previousSampleUs = cycleStartUs;
  timing.maxLoopUs = max(timing.maxLoopUs, elapsedUs);

  ++rateWindowSamples;
  const uint32_t rateWindowUs = uint32_t(cycleStartUs - rateWindowStartUs);
  if (rateWindowUs >= 1000000) {
    timing.controlHz = rateWindowSamples * 1000000.0f / rateWindowUs;
    rateWindowStartUs = cycleStartUs;
    rateWindowSamples = 0;
  }

  // Read exactly once, then time the estimator separately. read() is the same
  // API as the main app and may reuse a recent cached reading if not ready.
  const uint32_t readStartUs = micros();
  const itc::ImuSample measurement = imu.read();
  const uint32_t imuUs = uint32_t(micros() - readStartUs);
  timing.maxImuReadUs = max(timing.maxImuReadUs, imuUs);

  const uint32_t estimatorStartUs = micros();
  const itc::Attitude& attitude = estimator.update(measurement, elapsedUs * 1e-6f);
  const uint32_t estimatorUs = uint32_t(micros() - estimatorStartUs);
  timing.maxEstimatorUs = max(timing.maxEstimatorUs, estimatorUs);

  const uint32_t nowMs = millis();
  if (uint32_t(nowMs - previousPrintMs) >= PRINT_PERIOD_MS) {
    previousPrintMs = nowMs;
    // One bounded write avoids per-column Serial.print waits. The work maxima
    // below describe COMPLETED earlier cycles; this cycle finishes afterwards.
    char line[320];
    const int length = snprintf(line, sizeof(line),
        "%lu,%.3f,%.3f,%.3f,%.1f,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
        (unsigned long)nowMs, attitude.rollDeg, attitude.pitchDeg, attitude.yawDeg,
        timing.controlHz, (unsigned long)imuUs, (unsigned long)estimatorUs,
        (unsigned long)timing.maxImuReadUs, (unsigned long)timing.maxEstimatorUs,
        (unsigned long)timing.maxWorkUs, (unsigned long)timing.maxLoopUs,
        (unsigned long)timing.overruns);
    if (Serial && length > 0 && length < int(sizeof(line))
        && Serial.availableForWrite() >= length) {
      Serial.write(reinterpret_cast<const uint8_t*>(line), length);
    }
  }

  // Measure the WHOLE completed work, including occasional formatting/output.
  // Do this even when no print was due or USB was disconnected.
  const uint32_t workUs = uint32_t(micros() - cycleStartUs);
  timing.maxWorkUs = max(timing.maxWorkUs, workUs);
  if (workUs > SAMPLE_PERIOD_US) ++timing.overruns;
}
