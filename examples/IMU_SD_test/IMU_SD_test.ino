// Standalone IMU + SD measurement at a configurable target recording rate.
// No motor/receiver subsystem. Internal digital low-pass filters stay bypassed.
#include <Sensors.h>
#include <Control.h>
#include <ImuSdLogger.h>

// Angle estimation costs processing time and is not needed to identify raw
// sensor noise. Set true if you also want roll/pitch/yaw in the recording.
constexpr bool ESTIMATE_ATTITUDE = true;
// Bus clock is separate from the recording rate. Start below the 7 MHz maximum.
// If startup still fails, use 100000 to match your earlier working sketch.
constexpr uint32_t IMU_SPI_HZ = 1000000;
static_assert(IMU_SPI_HZ > 0 && IMU_SPI_HZ <= 7000000,
              "ICM-20948 SPI clock must be positive and at most 7 MHz.");
// Change this one value to choose the acquisition rate. 1000 Hz = 1000 us.
// Examples: 100, 500, 1000, 2000, 4000. Choose a divisor of 1000000.
constexpr uint32_t SAMPLE_RATE_HZ = 1000;
static_assert(SAMPLE_RATE_HZ > 0 && SAMPLE_RATE_HZ <= 1000000,
              "Choose a positive acquisition rate up to 1000000 Hz.");
static_assert(1000000u % SAMPLE_RATE_HZ == 0,
              "Choose a rate with a whole-microsecond period.");
constexpr uint32_t SAMPLE_PERIOD_US = 1000000u / SAMPLE_RATE_HZ;
itc::Imu imu;
itc::AttitudeEstimator estimator;
itc::ImuSdLogger logger;
itc::ImuCalibration calibration;

uint32_t previousReadUs = 0;
uint32_t previousPrintMs = 0;
uint32_t windowReadings = 0;
uint32_t maximumGapUs = 0;
uint32_t previousSlotUs = 0;
uint32_t skippedSlots = 0;
uint32_t notReadyReads = 0;



void setup() {
  Serial.begin(itc::config::USB_BAUD);
  const uint32_t startedMs = millis();
  while (!Serial && uint32_t(millis() - startedMs) < 3000) delay(10);
  if (!imu.begin(true, IMU_SPI_HZ)) { // Keep digital filters bypassed.
    while (true) {
      Serial.print("IMU initialization failed at: ");
      Serial.println(imu.initializationStep());
      Serial.print("Driver status: ");
      Serial.println(imu.statusString());
      Serial.print("SPI clock Hz: ");
      Serial.println(IMU_SPI_HZ);
      delay(1000);
    }
  }
  // Serial.println("Starting Calibration ");

  // calibration = imu.calibrate(20000);
  // Serial.print("Calibration accel: ");
  // Serial.print(calibration.accelBiasG[0]);
  // Serial.print(calibration.accelBiasG[1]);
  // Serial.println(calibration.accelBiasG[2]);

  // Serial.print("Calibration gyro: ");
  // Serial.print(calibration.gyroBiasDps[0]);
  // Serial.print(calibration.gyroBiasDps[1]);
  // Serial.println(calibration.gyroBiasDps[2]);

  // The loop below controls the rate. Zero avoids a SECOND timing gate in the
  // logger, which could reject slightly early reads because of normal jitter.
  // Binary records and the RAM queue avoid CSV formatting for each observation.
  logger.begin(true, 0);
  Serial.println("IMU scheduled test: internal digital low-pass filters bypassed.");
  Serial.print("SPI clock Hz: ");
  Serial.println(IMU_SPI_HZ);
  Serial.print("Target acquisition Hz: ");
  Serial.println(SAMPLE_RATE_HZ);
  Serial.println("Sensor output: gyro nominal 9000 Hz, accel 4500 Hz, mag <=100 Hz.");
  Serial.println("Accel/mag can repeat between frames; ready does not mean all are new.");
  Serial.println(ESTIMATE_ATTITUDE ? "Angle estimation enabled." : "Raw capture: angle columns are NaN (attitude_valid=0).");
  Serial.println("S saves/stops; wait for closed=1. L starts a new file.");

  previousPrintMs = millis();
  previousSlotUs = micros();
}

void loop() {
  // Handle commands even when the sensor stops producing data.
  if (Serial.available()) {
    const int command = Serial.read();
    if (command == 'S') logger.requestStop();
    if (command == 'L') logger.requestStart();
  }

  // Attempt one new read per scheduled slot. Keep SD service and USB commands
  // running between slots; do not delay() or return from loop() while waiting.
  const uint32_t nowUs = micros();
  const uint32_t dueSlots = uint32_t(nowUs - previousSlotUs) / SAMPLE_PERIOD_US;
  const bool sampleDue = dueSlots != 0;
  if (sampleDue) {
    previousSlotUs += dueSlots * SAMPLE_PERIOD_US;
    // If SD blocked for several periods, skip old slots rather than inventing
    // historical samples by taking several immediate catch-up readings.
    skippedSlots += dueSlots - 1;
  }
  itc::ImuSample measurement;
  if (sampleDue && imu.readFresh(measurement)) {
    const uint32_t readUs = micros(); // Host read completion, not sensor time.
    const uint32_t elapsedUs = uint32_t(readUs - previousReadUs);
    itc::Attitude attitude;
    if (ESTIMATE_ATTITUDE && previousReadUs != 0) {
      attitude = estimator.update(measurement, elapsedUs * 1e-6f);
    } else {
      attitude.rollDeg = attitude.pitchDeg = attitude.yawDeg = NAN;
    }
    if (previousReadUs != 0 && elapsedUs > maximumGapUs) maximumGapUs = elapsedUs;
    previousReadUs = readUs;
    ++windowReadings;
    logger.sample(readUs, measurement, attitude);
  } else if (sampleDue) {
    ++notReadyReads; // No new reading OR a sensor/bus error; never copy the cache.
  }

  // SD can still stall this loop. The queue prevents queued data loss until full,
  // but cannot recover sensor conversions missed while the CPU is in an SD call.
  logger.service(millis(), true); // Safe only because this sketch has no motors.

  // Report once a second, rather than printing every measurement.
  const uint32_t nowMs = millis();
  const uint32_t windowMs = uint32_t(nowMs - previousPrintMs);
  if (windowMs < 1000) return;
  char line[224];
  const int length = snprintf(line, sizeof(line),
      "frames_hz=%.1f max_gap_us=%lu skipped_slots=%lu no_read=%lu rec=%u closed=%u failed=%u queued=%u queue_dropped=%lu max_io_us=%lu\n",
      windowReadings * 1000.0f / windowMs, (unsigned long)maximumGapUs,
      (unsigned long)skippedSlots, (unsigned long)notReadyReads,
      unsigned(logger.recording()), unsigned(logger.closed()), unsigned(logger.failed()),
      logger.queuedSamples(), (unsigned long)logger.droppedSamples(),
      (unsigned long)logger.maximumIoUs());
  if (Serial && length > 0 && length < int(sizeof(line)) && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
  }
  previousPrintMs = nowMs;
  windowReadings = maximumGapUs = 0;
  skippedSlots = notReadyReads = 0;
}
