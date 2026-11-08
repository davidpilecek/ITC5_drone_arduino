#pragma once

#include "Sensors.h"
#include "Receiver.h"
#include "Motors.h"
#include "Control.h"
#include "FlightTelemetry.h"
#include "ImuSdLogger.h"

namespace itc {

// The application connects the modules. It decides WHEN each module runs.
// It does not contain the detailed sensor drivers or control equations.
class FlightApp {
 public:
  void begin();   // Called once from Arduino setup().
  void update();  // Called repeatedly from Arduino loop().

 private:
  // Each object keeps the information it needs between calls.
  // The trailing '_' simply means "a variable belonging to this object".
  Imu imu_;
  RangeSensor range_;
  Receiver receiver_;
  Motors motors_;
  AttitudeEstimator estimator_;
  AttitudeController attitudeController_;
  HeightController heightController_;
  FlightTelemetry telemetry_;
  ImuSdLogger imuLogger_;

  // Copies of the calculations already performed in the current control cycle.
  // Logging these must never run a controller for a second time.
  Setpoints diagnosticSetpoints_;
  Effort diagnosticCorrections_;
  bool attitudeControlActive_ = false;

  bool imuReady_ = false;
  Timing timing_;

  uint32_t previousControlTimeUs_ = 0;
  uint32_t timingWindowStartUs_ = 0;
  uint32_t timingWindowCycles_ = 0;
  uint32_t previousRpmReadTimeUs_ = 0;
  uint32_t previousPrintTimeMs_ = 0;

  // 0xffffffff means that the ESC has not supplied a valid RPM reading.
  uint32_t electricalRpm_[MOTOR_COUNT] = {
      0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu
  };

  MotorCommands calculateMotorCommands(
      const RcState& pilotCommands,
      const Attitude& measuredAttitude,
      const MotorStatus& motorStatus,
      float elapsedSeconds,
      bool& controllerHealthy);

  void printCalibration(const ImuCalibration& calibration);
  void printStartupInformation();
  void printTelemetry(const RcState& pilotCommands, const Attitude& measuredAttitude);
  void captureMavlinkDiagnostics(
      uint32_t nowMs, const RcState& pilotCommands, const ImuSample& measurement,
      const Attitude& attitude, const MotorCommands& proposed, bool heightUsable);
};

} // namespace itc
