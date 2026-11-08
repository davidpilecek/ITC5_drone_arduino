#include "ADRC.h"
#include <cmath>

namespace itc {

// Fast sampling makes some terms almost equal before subtraction. Teensy 4.1
// uses 64-bit double; reject targets which would silently reduce this precision.
static_assert(sizeof(double) >= 8, "ADRC coefficients require 64-bit double.");

AdrcCoefficients makeADRC_D(double wc, double kESO, double b0, double Ts) {
  AdrcCoefficients p;
  p.wc = wc;
  p.kESO = kESO;
  p.b0 = b0;
  p.Ts = Ts;

  // Arduino firmware does not need exceptions here. Return valid=false when
  // the requested parameters cannot describe a usable coefficient calculation.
  if (!std::isfinite(wc) || !std::isfinite(kESO) || !std::isfinite(b0)
      || !std::isfinite(Ts) || wc <= 0.0 || kESO <= 0.0
      || b0 == 0.0 || Ts <= 0.0) {
    return p;
  }

  p.wo = kESO * wc;
  const double Ts2 = Ts * Ts;
  const double denominator = b0 * Ts2;
  if (!std::isfinite(p.wo) || !std::isfinite(Ts2) || Ts2 == 0.0
      || !std::isfinite(denominator) || denominator == 0.0) {
    return p;
  }

  // 1. Map continuous bandwidths to discrete pole locations.
  p.zCL = std::exp(-p.wc * Ts);
  p.zESO = std::exp(-p.wo * Ts);

  const double zCL = p.zCL;
  const double zESO = p.zESO;

  // Squared params
  const double zCL2 = zCL * zCL;
  const double zESO2 = zESO * zESO;
  const double zESO3 = zESO2 * zESO;
  const double onePlusCL2 = (1.0 + zCL) * (1.0 + zCL);
  const double onePlusESO3 = (1.0 + zESO) * (1.0 + zESO) * (1.0 + zESO);
  const double oneMinusCL2 = (1.0 - zCL) * (1.0 - zCL);

  // 2. Discrete state-feedback gains.
  p.k1 = oneMinusCL2 / Ts2;
  p.k2 = (4.0 - onePlusCL2) / (2.0 * Ts);
  const double common = onePlusCL2 * onePlusESO3;
  const double scale = 1.0 / denominator;

  // 3. Alpha coefficients.
  p.alpha1 = -3.0 * zESO;
  p.alpha2 = 3.0 * zESO2;
  p.alpha3 = -zESO3;

  // 4. Beta coefficients: kept in the same grouping as the MATLAB equations.
  p.beta0 = scale * (
      common / 4.0
      - 2.0 * (zCL2 * zESO3 + 2.0 * zCL + 3.0 * zESO - 2.0));

  p.beta1 = scale * (
      -common
      + 2.0 * onePlusCL2
      + 6.0 * (zCL2 * zESO3 + 2.0 * zCL * zESO
               + zESO2 + zESO - 1.0));

  p.beta2 = scale * (
      -common / 4.0
      + 2.0 * (-2.0 * zCL2 * zESO3 + 3.0 * zCL2 * zESO2
               + 2.0 * zCL * zESO3 + 1.0));

  // 5. Gamma coefficients and the final listed gain.
  p.gamma0 = common / 8.0 - zESO * (zCL2 * zESO2 + 3.0);
  p.gamma1 = -common / 8.0 + 3.0 * zESO2 + 1.0;
  p.gamma2 = zESO3 * (zCL2 - 1.0);
  p.k1_over_b0 = oneMinusCL2 / denominator;

  // valid describes the numerical calculation
  const double calculated[] = {
      p.zCL, p.zESO, p.k1, p.k2,
      p.alpha1, p.alpha2, p.alpha3,
      p.beta0, p.beta1, p.beta2,
      p.gamma0, p.gamma1, p.gamma2, p.k1_over_b0};
  for (double coefficient : calculated) {
    if (!std::isfinite(coefficient)) return p;
  }
  p.valid = true;
  return p;
}

// ============================================================================
// Han tracking differentiator -- reference and reference-rate generation.
// ============================================================================

TrackingDifferentiator::TrackingDifferentiator(double Ts, double r0, double h0)
    : Ts_(Ts), r0_(r0), h0_(h0) {
  if (!std::isfinite(Ts) || !std::isfinite(r0) || !std::isfinite(h0)
      || Ts <= 0.0 || r0 <= 0.0 || h0 <= 0.0) {
    return;
  }
  // fhan divides by d. Reject a configuration whose d cannot be represented.
  const double d = r0 * h0 * h0;
  configured_ = std::isfinite(d) && d > 0.0;
}

TDOutput TrackingDifferentiator::state() const {
  return {reference_, rate_, configured_};
}

bool TrackingDifferentiator::reset(double initialRef, double initialDref) {
  if (!configured_ || !std::isfinite(initialRef) || !std::isfinite(initialDref)) {
    return false;
  }
  reference_ = initialRef;
  rate_ = initialDref;
  return true;
}

TDOutput TrackingDifferentiator::update(double input) {
  if (!configured_ || !std::isfinite(input)) {
    return {reference_, rate_, false};
  }

  // fhan returns the acceleration used to move the reference toward the input.
  const double acceleration = fhan(reference_ - input, rate_, r0_, h0_);

  // Explicit Euler: BOTH expressions use the old states. Updating reference_
  // with the already-updated rate would change the original algorithm.
  const double nextReference = reference_ + Ts_ * rate_;
  const double nextRate = rate_ + Ts_ * acceleration;
  if (!std::isfinite(acceleration) || !std::isfinite(nextReference)
      || !std::isfinite(nextRate)) {
    return {reference_, rate_, false};
  }

  // Commit both states only after the whole update has succeeded.
  reference_ = nextReference;
  rate_ = nextRate;
  return {reference_, rate_, true};
}

double TrackingDifferentiator::sign(double value) {
  return (value > 0.0) - (value < 0.0);
}

double TrackingDifferentiator::fhan(double error, double rate, double r0, double h0) {
  // Keep the mathematical names from your original fhan implementation so the
  // switching equations remain easy to compare with the reference algorithm.
  const double d = r0 * h0 * h0;
  const double a0 = h0 * rate;
  const double y = error + a0;
  const double a1 = std::sqrt(d * (d + 8.0 * std::abs(y)));
  const double a2 = a0 + sign(y) * (a1 - d) / 2.0;
  const double sy = (sign(y + d) - sign(y - d)) / 2.0;
  const double a = (a0 + y - a2) * sy + a2;
  const double sa = (sign(a + d) - sign(a - d)) / 2.0;
  return -r0 * (a / d - sign(a)) * sa - r0 * sign(a);
}

} // namespace itc
