#include "FlightApp.h"

namespace itc {

// ============================================================================
// PART 1: STARTUP -- runs once
// ============================================================================

void FlightApp::begin() {
  Serial.begin(config::USB_BAUD);

  // Start sending STOP frames before doing anything that could take a long time.
  motors_.begin();

  imuReady_ = imu_.begin();
  if (!imuReady_) {
    Serial.println("IMU initialization failed; stop frames remain active.");
    while (true) {
      delay(1000);
    }
  }

  // Calibration is optional and allowed only in the zero-output test mode.
  if (config::CALIBRATE_IMU_ON_BOOT) {
    const ImuCalibration calibration = imu_.calibrate();
    printCalibration(calibration);
  }

  const bool rangeSensorStarted = range_.begin();
  if (rangeSensorStarted) {
    Serial.println("VL53L1X ready on Wire1");
  } else {
    Serial.println("Range unavailable: height invalid");
  }

  const bool receiverStarted = receiver_.begin();
  if (receiverStarted) {
    Serial.println("CRSF ready on Serial5");
  } else {
    Serial.println("Receiver failed: all motors stopped");
  }

  printStartupInformation();
  previousControlTimeUs_ = micros();
}

// ============================================================================
// PART 2: MAIN CYCLE -- read this function from top to bottom
// ============================================================================

void FlightApp::update() {
  // STEP 1 -- Read the radio on EVERY pass, even when no control update is due.
  const RcState pilotCommands = receiver_.update();
  motors_.publishReceiver(pilotCommands);

  // STEP 2 -- Wait until 500 microseconds have passed since the previous cycle.
  // Returning here does not stop Arduino: loop() immediately calls us again.
  const uint32_t cycleStartUs = micros();
  const uint32_t elapsedUs = uint32_t(cycleStartUs - previousControlTimeUs_);
  if (elapsedUs < config::CONTROL_PERIOD_US) {
    return;
  }
  previousControlTimeUs_ = cycleStartUs; 

  timing_.maxLoopUs = max(timing_.maxLoopUs, elapsedUs);
  if (elapsedUs > 1000) {
    ++timing_.overruns;
  }
  const float elapsedSeconds = elapsedUs * 1e-6f;

  // STEP 3 -- Read acceleration/rotation, then estimate the vehicle's angles.
  const ImuSample imuReading = imu_.read();
  const Attitude& measuredAttitude = estimator_.update(imuReading, elapsedSeconds);

  // STEP 4 -- Check whether the distance sensor has a new measurement.
  const uint32_t rangeReadStartUs = micros();
  const RangeState& heightReading = range_.update(measuredAttitude, rangeReadStartUs);
  const uint32_t timeAfterSensorsUs = micros();
  const uint32_t rangeReadDurationUs = uint32_t(timeAfterSensorsUs - rangeReadStartUs);
  timing_.maxRangeUs = max(timing_.maxRangeUs, rangeReadDurationUs);

  // Check age AFTER reading the sensors. A newly acquired reading must not be
  // compared with the earlier cycle-start time, or it could appear "from the future".
  const bool heightMeasurementUsable = usableHeight(
      heightReading, measuredAttitude, timeAfterSensorsUs);
  const MotorStatus motorStatus = motors_.status();

  // STEP 5 -- Run height feedback in mode 3, or preview it in mode 0.
  // Mode 0 calculates the same result for learning, but never spins the motors.
  if (ITC_OUTPUT_MODE == 0 || ITC_OUTPUT_MODE == 3) {
    const float requestedHeightM = heightRequestFromPulse(pilotCommands.throttleUs);
    const bool armSwitchOff = pilotCommands.armUs <= config::ARM_LOW_US;

    heightController_.update(
        timeAfterSensorsUs,
        heightReading.acceptedUs,
        heightReading.heightM,
        requestedHeightM,
        heightMeasurementUsable,
        motorStatus.armed,
        armSwitchOff);
  }

  // STEP 6 -- Choose the control path and calculate four proposed motor commands.
  bool controllerHealthy = true;
  const MotorCommands requestedMotorCommands = calculateMotorCommands(
      pilotCommands, measuredAttitude, motorStatus, elapsedSeconds, controllerHealthy);

  // STEP 7 -- Hand the complete proposal to the independent motor safety guard.
  // The guard, not this function, has final authority to send nonzero commands.
  ControlFrame controlProposal;
  controlProposal.commands = requestedMotorCommands;
  controlProposal.ready = imuReady_ && receiver_.started()
      && measuredAttitude.valid && controllerHealthy;
  controlProposal.heightReady = heightMeasurementUsable && !heightController_.faulted();
  controlProposal.heightSampleUs = heightReading.acceptedUs;

  // Keep the START time here: a slow/stalled sensor read must not make old work
  // appear fresh merely because it has finally finished.
  controlProposal.heartbeatUs = cycleStartUs;
  motors_.publishControl(controlProposal);

  // STEP 8 -- Read RPM and print diagnostics at slower rates than control.
  if (uint32_t(timeAfterSensorsUs - previousRpmReadTimeUs_) >= config::RPM_PERIOD_US) {
    previousRpmReadTimeUs_ = timeAfterSensorsUs;
    motors_.readErpm(electricalRpm_);
  }

  if (uint32_t(millis() - previousPrintTimeMs_) >= config::PRINT_PERIOD_MS) {
    previousPrintTimeMs_ = millis();
    printTelemetry(pilotCommands, measuredAttitude);
  }
}

// ============================================================================
// PART 3: WHAT EACH OUTPUT MODE DOES
// ============================================================================

MotorCommands FlightApp::calculateMotorCommands(
    const RcState& pilotCommands,
    const Attitude& measuredAttitude,
    const MotorStatus& motorStatus,
    float elapsedSeconds,
    bool& controllerHealthy) {

  // A default MotorCommands record contains four zero commands.
  MotorCommands motorCommands;
  if (ITC_OUTPUT_MODE == 0) {
    return motorCommands;
  }

  Setpoints desired = mapCommands(pilotCommands);

  if (ITC_OUTPUT_MODE == 1) {
    const bool throttleAboveMinimum = pilotCommands.throttleUs > config::THROTTLE_LOW_US;
    return benchCommands(desired.collective, throttleAboveMinimum);
  }

  // Modes 2 and 3 both stabilize attitude. Their source of common throttle differs.
  bool runAttitudeController = false;
  bool resetAttitudeIntegral = false;

  if (ITC_OUTPUT_MODE == 3) {
    desired.collective = heightController_.collective;
    controllerHealthy = !heightController_.faulted();

    const bool heightControllerActive = motorStatus.armed
        && heightController_.tracking() && controllerHealthy;
    runAttitudeController = heightControllerActive;
    resetAttitudeIntegral = !heightControllerActive;
  } else {
    runAttitudeController = motorStatus.armed
        && pilotCommands.throttleUs > config::THROTTLE_LOW_US;
    resetAttitudeIntegral = pilotCommands.throttleUs < 1060;
  }

  if (!runAttitudeController) {
    // Forget old accumulated errors so they do not carry into the next takeoff.
    const float yawRateError = desired.yawDps - measuredAttitude.filtered.gyroDps[2];
    attitudeController_.reset(yawRateError);
    return motorCommands;
  }

  const Effort attitudeCorrections = attitudeController_.update(
      desired, measuredAttitude, elapsedSeconds, resetAttitudeIntegral);
  return mixQuad(desired.collective, attitudeCorrections);
}

// ============================================================================
// PART 4: USB OUTPUT -- useful for diagnosis, not part of the controller
// ============================================================================

void FlightApp::printCalibration(const ImuCalibration& calibration) {
  Serial.println("Replacement ACCEL_BIAS_G and GYRO_BIAS_DPS: keep sensor level and still.");
  for (unsigned axis = 0; axis < 3; ++axis) {
    Serial.print(calibration.accelBiasG[axis], 6);
    Serial.print(axis == 2 ? '\n' : ',');
  }
  for (unsigned axis = 0; axis < 3; ++axis) {
    Serial.print(calibration.gyroBiasDps[axis], 6);
    Serial.print(axis == 2 ? '\n' : ',');
  }
}

void FlightApp::printStartupInformation() {
  Serial.print("ITC modular firmware, output mode ");
  Serial.println(ITC_OUTPUT_MODE);
  Serial.println("Motor order FL5 FR6 RL8 RR4; yaw has no absolute reference.");
  Serial.println(
      "time_ms,link,armed,throttle_us,roll_deg,pitch_deg,range_m,height_m,"
      "height_valid,range_status,rpm_FL,rpm_FR,rpm_RL,rpm_RR,"
      "cmd_FL,cmd_FR,cmd_RL,cmd_RR,max_loop_us,max_range_us,overruns,"
      "height_request_m,height_target_m,vz_mps,height_collective,height_stage,ground_m");
}

void FlightApp::printTelemetry(const RcState& pilotCommands, const Attitude& measuredAttitude) {
  const RangeState& heightReading = range_.state();
  const MotorStatus motorStatus = motors_.status();

  float shaftRpm[MOTOR_COUNT];
  for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor) {
    if (electricalRpm_[motor] == 0xffffffffu) {
      shaftRpm[motor] = NAN;
    } else {
      shaftRpm[motor] = float(electricalRpm_[motor]) / config::MOTOR_POLE_PAIRS;
    }
  }

  // Build one comma-separated line in memory, then send it only if USB has room.
  // The % placeholders describe the corresponding values listed underneath.
  // Keeping one buffered write avoids waiting on USB in the control cycle.
  char line[480];
  const int numberOfCharacters = snprintf(
      line, sizeof(line),
      "%lu,%u,%u,%u,%.2f,%.2f,%.3f,%.3f,%u,%u,"
      "%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%lu,%lu,%lu,"
      "%.3f,%.3f,%.3f,%.3f,%u,%.3f\n",
      (unsigned long)millis(),
      unsigned(pilotCommands.valid), unsigned(motorStatus.armed), unsigned(pilotCommands.throttleUs),
      measuredAttitude.rollDeg, measuredAttitude.pitchDeg,
      heightReading.slantM, heightReading.valid ? heightReading.heightM : NAN,
      unsigned(heightReading.valid), unsigned(heightReading.status),
      shaftRpm[0], shaftRpm[1], shaftRpm[2], shaftRpm[3],
      unsigned(motorStatus.applied[0]), unsigned(motorStatus.applied[1]),
      unsigned(motorStatus.applied[2]), unsigned(motorStatus.applied[3]),
      (unsigned long)timing_.maxLoopUs, (unsigned long)timing_.maxRangeUs,
      (unsigned long)timing_.overruns,
      heightController_.requestedM, heightController_.targetM, heightController_.velocityMps,
      heightController_.collective, unsigned(heightController_.stage), heightController_.groundM);

  const bool lineFitsBuffer = numberOfCharacters > 0 && numberOfCharacters < int(sizeof(line));
  if (Serial && lineFitsBuffer && Serial.availableForWrite() >= numberOfCharacters) {
    Serial.write(line, numberOfCharacters);
  }
}

} // namespace itc
