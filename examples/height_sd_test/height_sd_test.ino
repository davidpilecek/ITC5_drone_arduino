// Standalone VL53L1X + SD test. Logs direct ToF readings without height processing.
// No IMU, attitude estimator, receiver or motor object is created.
#include <Wire.h>
#include <VL53L1X.h>
#include <Config.h>
#include <HeightSdLogger.h>

// Fastest supported setting: Short mode, 20 ms budget, nominal 50 Hz.
// Short mode reaches about 1.3 m under favorable conditions, NOT the full 2 m.
// Set false for tests up to 2 m: Long mode, minimum 33 ms budget (~30 Hz).
// Actual usable range depends on target/lighting; inspect range_status as well.
constexpr bool FAST_SHORT_MODE = true;

constexpr uint32_t TIMING_BUDGET_US = FAST_SHORT_MODE ? 20000 : 33000;
constexpr uint32_t MEASUREMENT_PERIOD_MS = FAST_SHORT_MODE ? 20 : 33;
constexpr uint32_t READY_POLL_US = 500; // Readiness polling, NOT measurement rate.

VL53L1X tof;
itc::HeightSdLogger logger;
itc::HeightLogRecord latest;
uint32_t previousPollUs = 0;
uint32_t previousPrintMs = 0;
uint32_t receivedReadings = 0;
uint32_t previousRateMs = 0;
uint32_t windowReadings = 0;
float measuredHz = 0;

void setup() {
  Serial.begin(itc::config::USB_BAUD);
  const uint32_t startedMs = millis();
  while (!Serial && uint32_t(millis() - startedMs) < 3000) delay(10);

  // 1. Set up the actual I2C bus: SCL16 and SDA17 from Config.h.
  Wire1.setSCL(itc::config::RANGE_SCL);
  Wire1.setSDA(itc::config::RANGE_SDA);
  Wire1.begin();
  Wire1.setClock(itc::config::RANGE_I2C_HZ);
  tof.setBus(&Wire1);
  tof.setTimeout(100);

  // 2. Request the shortest supported conversion for the selected distance mode.
  const bool ready = tof.init()
      && tof.setDistanceMode(FAST_SHORT_MODE ? VL53L1X::Short : VL53L1X::Long)
      && tof.setMeasurementTimingBudget(TIMING_BUDGET_US);
  if (!ready) {
    while (true) {
      Serial.println("VL53L1X initialization failed. Check Wire1 wiring and power.");
      delay(1000);
    }
  }
  tof.startContinuous(MEASUREMENT_PERIOD_MS);

  logger.begin(); // Enabled in this test, independently of the flight IMU logger.
  Serial.println("Raw ToF SD test: VL53L1X on Wire1 SCL16/SDA17, no correction/filter.");
  Serial.println(FAST_SHORT_MODE ? "Short mode: nominal 50 Hz; reduced range (~1.3 m)." : "Long mode: nominal ~30 Hz; check validity at 2 m.");
  Serial.println("SD files: TOF0000.CSV, TOF0001.CSV, ...; existing files are preserved.");
  Serial.println("S stops/saves. Wait for sd_closed=1 before power-off; L starts a new file.");
  Serial.println("time_ms,samples,distance_mm,range_status,i2c_status,signal_mcps,ambient_mcps,sd_rec,sd_closed,sd_failed,sd_queued,sd_dropped,measured_hz");
  previousPollUs = micros();
  previousRateMs = millis();
}

void loop() {
  // 3. Commands request work; they do not perform SD writes themselves.
  if (Serial.available()) {
    const int command = Serial.read();
    if (command == 'S') logger.requestStop();
    if (command == 'L') logger.requestStart();
  }

  // 4. Poll readiness, then log exactly ONE row per completed sensor read.
  // No RangeSensor wrapper, cosine correction, low-pass filter, or range limits.
  const uint32_t nowUs = micros();
  if (uint32_t(nowUs - previousPollUs) < READY_POLL_US) return;
  previousPollUs = nowUs;
  const bool newReadingReady = tof.dataReady();
  const uint8_t pollI2cStatus = tof.last_status;
  if (newReadingReady && pollI2cStatus == 0) {
    latest.distanceMm = tof.read(false); // Already ready: do not wait for conversion.
    latest.timestampUs = micros();      // Host read-completion time, not sensor time.
    latest.status = uint8_t(tof.ranging_data.range_status);
    latest.i2cStatus = tof.last_status;
    latest.signalMcps = tof.ranging_data.peak_signal_count_rate_MCPS;
    latest.ambientMcps = tof.ranging_data.ambient_count_rate_MCPS;
    ++receivedReadings;
    ++windowReadings;
    // Preserve readings even when range_status reports an invalid result.
    // Investigate/filter them later on the PC; the file stores what was returned.
    logger.sampleTof(latest);
  }

  // 5. Write one queued CSV row, or perform pending open/sync/close work.
  // Safe here because this sketch has no motor/arming subsystem. SD may block.
  logger.service(true);

  // 6. Show the latest reading/status at 10 Hz. Before the first read, samples=0
  // and distance_mm=0 is just an initial placeholder, not a logged measurement.
  const uint32_t nowMs = millis();
  const uint32_t rateWindowMs = uint32_t(nowMs - previousRateMs);
  if (rateWindowMs >= 1000) {
    measuredHz = windowReadings * 1000.0f / rateWindowMs;
    previousRateMs = nowMs;
    windowReadings = 0;
  }
  if (uint32_t(nowMs - previousPrintMs) < 100) return;
  previousPrintMs = nowMs;
  char line[224];
  const int length = snprintf(line, sizeof(line),
      "%lu,%lu,%u,%u,%u,%.4f,%.4f,%u,%u,%u,%u,%lu,%.1f\n",
      (unsigned long)nowMs, (unsigned long)receivedReadings,
      unsigned(latest.distanceMm), unsigned(latest.status),
      unsigned(pollI2cStatus != 0 ? pollI2cStatus : latest.i2cStatus),
      latest.signalMcps, latest.ambientMcps,
      unsigned(logger.recording()), unsigned(logger.closed()), unsigned(logger.failed()),
      logger.queuedSamples(), (unsigned long)logger.droppedSamples(), measuredHz);
  if (Serial && length > 0 && length < int(sizeof(line))
      && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
  }
}
