#pragma once

namespace itc {

// C++ equivalent of the MATLAB structure returned by makeADRC_D().
// The names deliberately match MATLAB: p.beta0 becomes p.beta0 in C++ too.
// This stores coefficients only; it does not contain controller delay states.
struct AdrcCoefficients {
  // Check this before using the coefficients. False means invalid input or a
  // numerical overflow/underflow that prevents calculating finite coefficients.
  bool valid = false;

  // Tuning inputs and the derived observer bandwidth.
  double wc = 0.0;   // Controller bandwidth [rad/s], positive.
  double kESO = 0.0; // Observer/controller bandwidth ratio, positive.
  double wo = 0.0;   // Observer bandwidth [rad/s] = kESO * wc.
  double b0 = 0.0;   // Nominal plant input gain; nonzero, either sign allowed.
  double Ts = 0.0;   // Controller sampling period [seconds], positive.

  // Discrete pole locations.
  double zCL = 0.0;
  double zESO = 0.0;

  // Discrete state-feedback gains.
  double k1 = 0.0;
  double k2 = 0.0;

  // Dual-feedback transfer-function coefficients.
  double alpha1 = 0.0;
  double alpha2 = 0.0;
  double alpha3 = 0.0;
  double beta0 = 0.0;
  double beta1 = 0.0;
  double beta2 = 0.0;
  double gamma0 = 0.0;
  double gamma1 = 0.0;
  double gamma2 = 0.0;
  double k1_over_b0 = 0.0;
};

// Translate the supplied second-order discrete ADRC coefficient function.
// Call during setup(), or when tuning/period changes, rather than every tick.
// wc and kESO must be positive; b0 must be nonzero; Ts is in seconds.
// All inputs must be finite. Do not use returned coefficients unless valid=true.
// b0 must use the same input/output units as the plant you intend to control.
AdrcCoefficients makeADRC_D(double wc, double kESO, double b0, double Ts);

// ============================================================================
// Tracking differentiator -- smooth a requested reference before ADRC uses it.
// ============================================================================

struct TDOutput {
  double ref = 0.0;  // v1: smoothed reference, same units as the input.
  double dref = 0.0; // v2: reference rate, input units per second.
  bool valid = false;
};

class TrackingDifferentiator {
 public:
  // Ts: update period [s]. r0: acceleration parameter [input units/s^2].
  // h0: smoothing horizon [s]; commonly chosen as Ts or a multiple of Ts.
  // All three must be finite and positive. This check does not certify tuning.
  TrackingDifferentiator(double Ts, double r0, double h0);

  // Call once per Ts seconds with the requested, unmodified reference.
  // Invalid input or a nonfinite calculation returns valid=false and preserves
  // both states. The next finite input can be processed normally.
  TDOutput update(double input);

  // Start at rest, or at a known reference/rate to avoid a startup transition.
  // Invalid reset values return false and leave the previous states unchanged.
  bool reset(double initialRef = 0.0, double initialDref = 0.0);

  // valid() checks configuration; update().valid checks that particular update.
  bool valid() const { return configured_; }
  TDOutput state() const;

 private:
  double Ts_;
  double r0_;
  double h0_;
  double reference_ = 0.0; // v1 in the original equations.
  double rate_ = 0.0;      // v2 in the original equations.
  bool configured_ = false;

  static double sign(double value);
  static double fhan(double error, double rate, double r0, double h0);
};

} // namespace itc
