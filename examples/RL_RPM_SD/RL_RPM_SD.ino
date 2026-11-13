// Teensy 4.1 single-motor bench test. Verify disarm and radio loss with props off.
// SD writes run only after the interrupt-side motor gate reports disarmed.
#include <Arduino.h>
#include <SD.h>
#include <Motors.h>
#include <Receiver.h>

using namespace itc;
constexpr unsigned RL = static_cast<unsigned>(Motor::RearLeft);
constexpr uint8_t opticalPin = 22;
constexpr uint32_t sampleUs = 50000;
constexpr size_t capacity = 3600;  // 180 s at 20 Hz; a full buffer ends the run.
static_assert(ITC_OUTPUT_MODE == 1 || ITC_OUTPUT_MODE == 2,
              "Bench example requires an enabled manual output mode (1 or 2).");
struct Sample {
  uint32_t ms, windowUs, pulses, erpm;
  uint16_t throttle, command;
  bool link, armed;
};
DMAMEM Sample samples[capacity];
size_t used = 0;
Motors motors;
Receiver receiver;
File logFile;
bool ready = false, recording = false;
volatile uint32_t pulseTotal = 0;
uint32_t priorCount = 0, priorUs = 0;


uint32_t lastMillis = 0;
uint32_t throttle = 0;

uint16_t rampThrottle(bool enabled) {
  static uint32_t startMs = 0;
  static bool running = false;
  constexpr uint32_t rampMs = 10000;
  constexpr uint16_t maximum =
      48 + uint16_t(1999 * config::BENCH_MAX_COMMAND);

  if (!enabled) {
    running = false;
    return 0;
  }

  if (!running) {
    startMs = millis();
    running = true;
  }

  const uint32_t elapsed = min(millis() - startMs, rampMs);
  return 48 + uint32_t(maximum - 48) * elapsed / rampMs;
}

void countBlade() { ++pulseTotal; }

bool openLog() {
  if (!SD.begin(BUILTIN_SDCARD)) return false;
  char name[20];
  for (unsigned i = 0; i < 10000; ++i) {
    snprintf(name, sizeof(name), "RL%04u.CSV", i);
    if (SD.exists(name)) continue;
    logFile = SD.open(name, FILE_WRITE);
    if (!logFile) return false;
    logFile.println("time_ms,window_us,link_valid,armed,throttle_us,rl_dshot,rl_erpm,optical_pulses,optical_rpm");
    logFile.flush();
    Serial.println(name);
    return !logFile.getWriteError();
  }
  return false;
}

void saveRun() {
  char row[120];
  for (size_t i = 0; i < used; ++i) {
    const Sample &s = samples[i];
    char erpm[16] = "";
    if (s.erpm != UINT32_MAX) snprintf(erpm, sizeof(erpm), "%lu", (unsigned long)s.erpm);
    const double rpm = 60000000.0 * s.pulses / (3.0 * s.windowUs);
    int n = snprintf(row, sizeof(row), "%lu,%lu,%u,%u,%u,%u,%s,%lu,%.2f\n",
      (unsigned long)s.ms, (unsigned long)s.windowUs, unsigned(s.link), unsigned(s.armed),
      unsigned(s.throttle), unsigned(s.command), erpm, (unsigned long)s.pulses, rpm);
    if (n <= 0 || size_t(n) >= sizeof(row) ||
        logFile.write((const uint8_t*)row, n) != size_t(n)) { ready = false; break; }
  }
  logFile.flush();
  if (logFile.getWriteError()) ready = false;
  Serial.println(ready ? "Run saved." : "SD write failed; motor locked out.");
  used = 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(opticalPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(opticalPin), countBlade, FALLING);
  motors.begin();
  ready = openLog() && receiver.begin();
  if (!ready) Serial.println("SD or CRSF initialization failed; motor locked out.");
}

void loop() {

  const RcState rc = receiver.update();
  ControlFrame frame{};
  frame.heartbeatUs = micros();
  frame.ready = ready && used < capacity;
  // if (rc.valid && rc.throttleUs > config::THROTTLE_LOW_US) {
  //   constexpr long maxCommand = 48 + long(1999 * config::BENCH_MAX_COMMAND);
  //   frame.commands.value[RL] = map(rc.throttleUs, config::THROTTLE_LOW_US + 1, 2000, 48, maxCommand);
  // }
  frame.commands.value[RL] = rampThrottle(
      rc.valid &&
      motors.status().armed &&
      rc.throttleUs > config::THROTTLE_LOW_US
  );
  motors.publishReceiver(rc);
  motors.publishControl(frame);
  const MotorStatus status = motors.status();
  if (!recording && !status.armed) return;

  noInterrupts();
  const uint32_t now = micros();
  const uint32_t count = pulseTotal;
  interrupts();
  uint32_t electrical[MOTOR_COUNT];
  if (!recording) {
    recording = true;
    priorUs = now;
    priorCount = count;
    motors.readErpm(electrical);  // Start a new telemetry averaging window.
    return;
  }
  const uint32_t dt = now - priorUs;
  if (used < capacity && dt && (dt >= sampleUs || !status.armed)) {
    motors.readErpm(electrical);
    samples[used++] = {millis(), dt, count - priorCount, electrical[RL],
                       rc.throttleUs, status.applied[RL], rc.valid, status.armed};
    priorUs = now;
    priorCount = count;
  }
  if (!status.armed) {
    frame.ready = false;
    motors.publishControl(frame);
    saveRun();
    recording = false;
  }
}
