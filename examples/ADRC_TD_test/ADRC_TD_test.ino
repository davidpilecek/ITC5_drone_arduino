// Standalone reference-generator test: no receiver, sensors, SD or motor objects.
#include <Arduino.h>
#include <ADRC.h>

constexpr uint32_t PERIOD_US = 1000; // 1 kHz update schedule.
constexpr double PERIOD_S = PERIOD_US * 1e-6;
constexpr double ACCELERATION_PARAMETER = 3.0;
constexpr double SMOOTHING_HORIZON_S = 5.0 * PERIOD_S;

itc::TrackingDifferentiator td(
    PERIOD_S, ACCELERATION_PARAMETER, SMOOTHING_HORIZON_S);
uint32_t previousUpdateUs = 0;
uint32_t previousPrintMs = 0;
uint32_t startedMs = 0;

void setup() {
  Serial.begin(115200);
  const uint32_t waitStartedMs = millis();
  while (!Serial && uint32_t(millis() - waitStartedMs) < 3000) delay(10);
  if (!td.valid()) {
    Serial.println("TD configuration invalid.");
    return;
  }
  td.reset(0.0, 0.0);
  Serial.println("input,smoothed_reference,reference_rate");
  startedMs = millis();
  previousUpdateUs = micros();
}

void loop() {
  if (!td.valid()) return;
  const uint32_t nowUs = micros();
  if (uint32_t(nowUs - previousUpdateUs) < PERIOD_US) return;
  previousUpdateUs = nowUs;

  // Simulated command step after one second. No aircraft hardware is driven.
  const double input = uint32_t(millis() - startedMs) < 1000 ? 0.0 : 1.0;
  const itc::TDOutput shaped = td.update(input);
  if (!shaped.valid) return;

  // Print slowly; printing every update would disturb the 1 kHz test schedule.
  const uint32_t nowMs = millis();
  if (uint32_t(nowMs - previousPrintMs) < 100) return;
  previousPrintMs = nowMs;
  char line[128];
  const int length = snprintf(line, sizeof(line), "%.4f,%.6f,%.6f\n",
      input, shaped.ref, shaped.dref);
  if (Serial && length > 0 && length < int(sizeof(line))
      && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
  }
}
