#pragma once

#include <algorithm>
#include <cmath>
#include <mutex>

#include "base_component.hpp"

class LinearAdrcSecondOrder : public BaseComponent {
public:
  /// Configuration for the second-order linear ADRC controller.
  struct Config {
    float b0;                   ///< Estimated control gain of the plant.
    float controller_bandwidth; ///< Closed-loop bandwidth used to derive the feedback gains.
    float observer_bandwidth;   ///< Observer bandwidth used for the linear extended state observer.
    float output_min;           ///< Minimum output command.
    float output_max;           ///< Maximum output command.
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Verbosity for the internal logger.
  };

  /// Observer and controller state.
  struct State {
    float measurement{0.0f};    ///< Most recent measured position.
    float reference{0.0f};      ///< Most recent reference position.
    float reference_rate{0.0f}; ///< Most recent reference rate.
    float z1{0.0f};             ///< Estimated plant position state.
    float z2{0.0f};             ///< Estimated plant rate state.
    float z3{0.0f};             ///< Estimated total disturbance state.
    float output{0.0f};         ///< Most recent controller output.
  };

  /// Construct the second-order linear ADRC controller.
  /// @param config Controller configuration.
  explicit LinearAdrcSecondOrder(const Config &config)
      : BaseComponent("LinearAdrcSecondOrder", config.log_level)
      , config_(config) {}

  /// Change the controller configuration.
  /// @param config New controller configuration.
  /// @param reset_state If true, reset the observer and control state after applying the config.
  void set_config(const Config &config, bool reset_state = true) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    config_ = config;
    if (reset_state) {
      clear();
    }
  }

  /// Reset the observer and controller state.
  /// @param measurement Initial measured position for the observer state.
  /// @param measurement_rate Initial measured rate estimate for the observer state.
  void clear(float measurement = 0.0f, float measurement_rate = 0.0f) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    state_.measurement = measurement;
    state_.reference = measurement;
    state_.reference_rate = 0.0f;
    state_.z1 = measurement;
    state_.z2 = measurement_rate;
    state_.z3 = 0.0f;
    state_.output = 0.0f;
  }

  /// Update the controller using an explicit reference rate.
  /// @param reference Desired position reference.
  /// @param reference_rate Desired reference rate.
  /// @param measurement Measured position.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float update(float reference, float reference_rate, float measurement, float dt) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (dt <= 0.0f) {
      return state_.output;
    }

    auto wo = std::max(config_.observer_bandwidth, detail::adrc_epsilon);
    auto beta1 = 3.0f * wo;
    auto beta2 = 3.0f * wo * wo;
    auto beta3 = wo * wo * wo;
    auto b0 = detail::safe_nonzero(config_.b0);
    auto error = state_.z1 - measurement;

    state_.measurement = measurement;
    state_.reference = reference;
    state_.reference_rate = reference_rate;
    state_.z1 += dt * (state_.z2 - beta1 * error);
    state_.z2 += dt * (state_.z3 - beta2 * error + b0 * state_.output);
    state_.z3 += dt * (-beta3 * error);

    auto wc = std::max(config_.controller_bandwidth, detail::adrc_epsilon);
    auto kp = wc * wc;
    auto kd = 2.0f * wc;
    auto u0 = kp * (reference - state_.z1) + kd * (reference_rate - state_.z2);
    state_.output = std::clamp((u0 - state_.z3) / b0, config_.output_min, config_.output_max);
    return state_.output;
  }

  /// Update the controller assuming a zero reference-rate command.
  /// @param reference Desired position reference.
  /// @param measurement Measured position.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float update(float reference, float measurement, float dt) {
    return update(reference, 0.0f, measurement, dt);
  }

  /// Update the controller assuming a zero reference-rate command.
  /// @param reference Desired position reference.
  /// @param measurement Measured position.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float operator()(float reference, float measurement, float dt) {
    return update(reference, measurement, dt);
  }

  /// Get the current controller state.
  /// @return Current state.
  State get_state() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return state_;
  }

  /// Get the active configuration.
  /// @return Current configuration.
  Config get_config() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return config_;
  }

protected:
  Config config_;
  State state_;
  mutable std::recursive_mutex mutex_;
};