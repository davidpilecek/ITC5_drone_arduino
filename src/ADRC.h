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
// Second-order ADRC -- dual feedback, transposed direct form II.
// ============================================================================

struct AdrcOutput {
  double requested = 0.0; // u: controller request before the local limiter.
  double limited = 0.0;   // u_lim: value clipped and fed back by update().
  bool saturated = false;
  bool valid = false;     // Never apply a result whose valid flag is false.
};

// These are the three delay outputs in the diagram, NOT ESO state estimates.
struct AdrcState {
  double x1 = 0.0;
  double x2 = 0.0;
  double x3 = 0.0;
};

class AdrcController {
 public:
  AdrcController() = default;
  explicit AdrcController(const AdrcCoefficients& coefficients);

  // Copy precomputed coefficients and reset the three delays to zero.
  // A rejected configuration leaves the previous configuration/state unchanged.
  bool configure(const AdrcCoefficients& coefficients);

  // One complete sample: calculate u, clip it, feed back u_lim, advance delays.
  // Call once per coefficients.Ts seconds. Equal limits are allowed.
  // This convenience method assumes its limiter determines the plant input.
  AdrcOutput update(double reference, double measurement,
                    double minimumInput, double maximumInput);

  // Optional reference-rate feedforward, selectable independently per channel:
  // u = (k1/b0)*reference + (k2/b0)*referenceRate + feedback.
  // referenceRate is the TD's dref [reference units/s], NOT measured plant rate.
  // The four-argument overload above is equivalent to referenceRate=0.
  AdrcOutput update(double reference, double referenceRate, double measurement,
                    double minimumInput, double maximumInput);

  // For an external mixer/rate limiter, use these TWO calls instead of update:
  // 1. calculate(r, y, requested) -- reads old delays; stages y and feedback.
  // 2. commit(u_lim) -- advances delays using the input actually applied.
  // Call them once, in order, during the SAME sample. No actuator writes occur.
  // A pending sample must be committed or reset before calculating another one.
  bool calculate(double reference, double measurement, double& requested);
  bool calculate(double reference, double referenceRate,
                 double measurement, double& requested);
  bool commit(double appliedInput);

  // Initialize for a stationary operating point with reference=measurement.
  // reset() gives zero delays; reset(y0,u0) starts with a request of u0 when r=y0.
  // Assumes zero reference rate at reset. A later nonzero rate adds feedforward.
  // Also clears a pending sample. Invalid reset values leave everything intact.
  bool reset(double measurement = 0.0, double appliedInput = 0.0);

  bool valid() const { return configured_; }
  bool awaitingAppliedInput() const { return pending_; }
  double samplePeriod() const { return coefficients_.Ts; }
  AdrcState state() const { return state_; }

 private:
  AdrcCoefficients coefficients_;
  double referenceRateGain_ = 0.0; // k2/b0, computed once during configure().
  AdrcState state_;
  bool configured_ = false;
  bool pending_ = false;
  // Current-sample temporaries for calculate()/commit(), not extra delay states.
  double pendingMeasurement_ = 0.0;
  double pendingFeedback_ = 0.0;
};

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
