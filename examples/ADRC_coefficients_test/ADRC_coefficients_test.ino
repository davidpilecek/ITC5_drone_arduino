// Coefficient calculation only. No sensors, receiver or motor objects.
#include <Arduino.h>
#include <ADRC.h>

// EXAMPLE NUMBERS for comparing C++ with MATLAB, not flight tuning values.
constexpr double CONTROLLER_BANDWIDTH_RAD_S = 5.0;
constexpr double OBSERVER_RATIO = 5.0;
constexpr double NOMINAL_INPUT_GAIN = 1.0; // Replace for your model/units later.
constexpr double CONTROLLER_PERIOD_S = 0.001; // Coefficients for 1 kHz control.

void printCoefficient(const char* name, double value) {
  Serial.print(name);
  Serial.print(" = ");
  Serial.println(value, 12);
}

void setup() {
  Serial.begin(115200);
  // A bounded USB wait lets the sketch start even without an open monitor.
  const uint32_t startedMs = millis();
  while (!Serial && uint32_t(millis() - startedMs) < 3000) delay(10);

  const itc::AdrcCoefficients p = itc::makeADRC_D(
      CONTROLLER_BANDWIDTH_RAD_S, OBSERVER_RATIO,
      NOMINAL_INPUT_GAIN, CONTROLLER_PERIOD_S);
  if (!p.valid) {
    Serial.println("ADRC coefficients invalid: check inputs and numerical range.");
    return;
  }

  Serial.println("ADRC coefficients only; no controller outputs are generated.");
  printCoefficient("wc", p.wc);
  printCoefficient("kESO", p.kESO);
  printCoefficient("wo", p.wo);
  printCoefficient("b0", p.b0);
  printCoefficient("Ts", p.Ts);
  printCoefficient("zCL", p.zCL);
  printCoefficient("zESO", p.zESO);
  printCoefficient("k1", p.k1);
  printCoefficient("k2", p.k2);
  printCoefficient("alpha1", p.alpha1);
  printCoefficient("alpha2", p.alpha2);
  printCoefficient("alpha3", p.alpha3);
  printCoefficient("beta0", p.beta0);
  printCoefficient("beta1", p.beta1);
  printCoefficient("beta2", p.beta2);
  printCoefficient("gamma0", p.gamma0);
  printCoefficient("gamma1", p.gamma1);
  printCoefficient("gamma2", p.gamma2);
  printCoefficient("k1_over_b0", p.k1_over_b0);
}

void loop() {
  // Runtime controller update/state equations will be implemented separately.
}