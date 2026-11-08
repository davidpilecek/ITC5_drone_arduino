// Software-only ADRC demonstration. No receiver, sensor or motor objects.
#include <Arduino.h>
#include <ADRC.h>

constexpr uint32_t PERIOD_US = 1000;
constexpr double TS = PERIOD_US * 1e-6;
// Arbitrary demo plant: y'' = PLANT_GAIN*u + disturbance. NOT flight tuning.
constexpr double PLANT_GAIN = 1.0;

// Per-channel choice: true reproduces (k1*TD.ref + k2*TD.dref)/b0 feedforward.
// false preserves the original reference-only example.
constexpr bool USE_REFERENCE_RATE_FEEDFORWARD = false;
itc::AdrcController controller;
itc::TrackingDifferentiator td(TS,0.5,5.0*TS);
bool ready = false;
uint32_t previousUpdateUs = 0;
uint32_t previousPrintMs = 0;
uint32_t samples = 0;
double position = 0.0;
double velocity = 0.0;

void setup() {
  Serial.begin(115200);
  const uint32_t startMs = millis();
  while (!Serial && uint32_t(millis()-startMs)<3000) delay(10);
  const auto p=itc::makeADRC_D(5.0,5.0,PLANT_GAIN,TS);
  ready=controller.configure(p) && td.valid();
  if (!ready) { Serial.println("ADRC/TD configuration invalid."); return; }
  controller.reset(0.0,0.0);
  td.reset(0.0,0.0);
  Serial.println("time_s,command,reference,reference_rate,y,u_requested,u_lim,saturated");
  previousUpdateUs=micros();
}

void loop() {
  if (!ready) return;
  const uint32_t nowUs=micros();
  if (uint32_t(nowUs-previousUpdateUs)<PERIOD_US) return;
  previousUpdateUs=nowUs;

  // The simulated plant advances by TS each completed sample; USB timing is not
  // a physical plant measurement. It is a demonstration of the calculation flow.
  const double timeS=samples*TS;
  const double command=timeS<1.0 ? 0.0 : 1.0;
  const auto shaped=td.update(command);
  if (!shaped.valid) { ready=false; Serial.println("TD update invalid."); return; }
  const auto result = USE_REFERENCE_RATE_FEEDFORWARD
      ? controller.update(shaped.ref,shaped.dref,position,-2.0,2.0)
      : controller.update(shaped.ref,position,-2.0,2.0);
  if (!result.valid) { ready=false; Serial.println("ADRC update invalid."); return; }

  const uint32_t nowMs=millis();
  if (uint32_t(nowMs-previousPrintMs)>=50) {
    previousPrintMs=nowMs;
    char line[180];
    const int n=snprintf(line,sizeof(line),"%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u\n",
        timeS,command,shaped.ref,shaped.dref,position,result.requested,result.limited,
        unsigned(result.saturated));
    if (Serial && n>0 && n<int(sizeof(line)) && Serial.availableForWrite()>=n) {
      Serial.write(reinterpret_cast<const uint8_t*>(line),n);
    }
  }

  const double disturbance=timeS<6.0 ? 0.0 : 0.3;
  const double acceleration=PLANT_GAIN*result.limited+disturbance;
  position+=TS*velocity+0.5*TS*TS*acceleration;
  velocity+=TS*acceleration;
  ++samples;
}
