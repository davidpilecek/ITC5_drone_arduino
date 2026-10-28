#pragma once
#include <stdint.h>

// ============================================================================
// 1. CHOOSE WHAT THE FIRMWARE IS ALLOWED TO DO
// ============================================================================
// 0 = read sensors and preview control, but send ZERO to every motor (default).
// 1 = send the same manual throttle to all four motors, limited to a 10% command span.
// 2 = manual common throttle, with automatic roll/pitch/yaw stabilization.
// 3 = throttle stick requests 0..1 metre; height control calculates common throttle.
//
// These choices take effect when the sketch is compiled, not while it is running.
#ifndef ITC_OUTPUT_MODE
#define ITC_OUTPUT_MODE 0
#endif

#if ITC_OUTPUT_MODE < 0 || ITC_OUTPUT_MODE > 3
#error Select output mode 0, 1, 2 or 3.
#endif

// "itc::config" groups the project's settings under one name.
// "constexpr" means a value is fixed when the program is compiled.
namespace itc {
namespace config {

// ============================================================================
// 2. WIRING -- use FL, FR, RL, RR in EVERY motor array
// ============================================================================
constexpr uint8_t MOTOR_PINS[4] = {5, 6, 8, 4};

// MPU6500 on SPI1. Keep these consistent with your actual wiring.
constexpr uint8_t IMU_CS = 0;
constexpr uint8_t IMU_MISO = 1;
constexpr uint8_t IMU_MOSI = 26;
constexpr uint8_t IMU_SCK = 27;

// VL53L1X on Wire1. The CRSF receiver uses Serial5, RX21/TX20.
constexpr uint8_t RANGE_SCL = 16;
constexpr uint8_t RANGE_SDA = 17;
constexpr uint32_t RANGE_I2C_HZ = 400000;
constexpr uint32_t USB_BAUD = 500000;
constexpr uint32_t MAV_BAUD = 115200;

// ============================================================================
// 3. RADIO CHANNELS AND SAFETY TIMEOUTS
// ============================================================================
// Arrays start at zero: 0 means transmitter CH1, 1 means CH2, and so on.
constexpr uint8_t RC_ROLL = 0;
constexpr uint8_t RC_PITCH = 1;
constexpr uint8_t RC_THROTTLE = 2;
constexpr uint8_t RC_YAW = 3;
constexpr uint8_t RC_ARM = 4;
constexpr uint8_t RC_AUX = 5;

// Radio values are converted into a familiar 1000..2000 convention.
constexpr uint16_t ARM_HIGH_US = 1650;
constexpr uint16_t ARM_LOW_US = 1350;
constexpr uint16_t THROTTLE_LOW_US = 1050;

constexpr uint32_t RC_TIMEOUT_US = 100000;      // Stop after 0.10 s without valid radio data.
constexpr uint32_t CONTROL_TIMEOUT_US = 20000; // Stop if control work is 0.02 s old.
constexpr uint32_t RANGE_STALE_US = 200000;    // Height mode stops after 0.20 s without accepted height.

// ============================================================================
// 4. SCHEDULE -- Us means microseconds; Ms means milliseconds
// ============================================================================
constexpr uint32_t CONTROL_PERIOD_US = 500;   // Target: 2000 control updates per second.
constexpr uint32_t RANGE_POLL_US = 5000;      // Check for a new range reading 200 times/s.
constexpr uint32_t RANGE_PERIOD_MS = 50;     // Sensor produces 20 range readings/s.
constexpr uint32_t RPM_PERIOD_US = 10000;    // Collect RPM 100 times/s.
constexpr uint32_t PRINT_PERIOD_MS = 100;    // Print diagnostics 10 times/s.

// ============================================================================
// 5. IMU CONVERSION, CALIBRATION AND FILTERING
// ============================================================================
// The IMU driver is configured for +/-2 g and +/-250 degrees/s in Sensors.cpp.
// These conversion factors MUST agree with those hardware settings.
constexpr float ACCEL_LSB_PER_G = 16384.0f;
constexpr float GYRO_LSB_PER_DPS = 131.0f;

// Measured stationary offsets, in X/Y/Z order. Replace after calibration.
constexpr float ACCEL_BIAS_G[3] = {0, 0, 0};
constexpr float GYRO_BIAS_DPS[3] = {0, 0, 0};
constexpr bool CALIBRATE_IMU_ON_BOOT = false;

constexpr float ACCEL_FILTER = 0.2f;
constexpr float GYRO_FILTER = 0.17f;
constexpr float AHRS_BETA = 0.04f; // Strength of accelerometer correction in attitude estimation.

// ============================================================================
// 6. ATTITUDE CONTROL -- retained values; they still require physical tuning
// ============================================================================
constexpr float MAX_ROLL_DEG = 30;
constexpr float MAX_PITCH_DEG = 30;
constexpr float MAX_YAW_DPS = 160;
constexpr float ATTITUDE_I_LIMIT = 25;

constexpr float ROLL_KP = 0.2f;
constexpr float ROLL_KI = 0.3f;
constexpr float ROLL_KD = 0.05f;
constexpr float PITCH_KP = 0.2f;
constexpr float PITCH_KI = 0.3f;
constexpr float PITCH_KD = 0.05f;
constexpr float YAW_KP = 0.3f;
constexpr float YAW_KI = 0.05f;
constexpr float YAW_KD = 0.00015f;

// ============================================================================
// 7. MOTOR COMMANDS -- a command fraction is NOT a power or RPM fraction
// ============================================================================
constexpr float BENCH_MAX_COMMAND = 0.10f;
constexpr uint16_t MOTOR_IDLE_COMMAND = 100;
constexpr uint8_t MOTOR_POLE_PAIRS = 7; // Shaft RPM = electrical RPM / pole pairs.

// ============================================================================
// 8. HEIGHT MEASUREMENT -- all distances are in metres
// ============================================================================
// Height refers to the SENSOR optical centre, not the lowest point of the hull.
constexpr float MIN_SLANT_M = 0.04f;
constexpr float MAX_SLANT_M = 3.0f;
constexpr float MAX_HEIGHT_M = 2.0f;
constexpr float MAX_TILT_DEG = 30.0f;
constexpr float HEIGHT_FILTER_TAU_S = 0.10f;

// ============================================================================
// 9. HEIGHT CONTROLLER -- provisional settings, NOT a measured flight tune
// ============================================================================
constexpr float HEIGHT_SETPOINT_MAX_M = 1.0f;
constexpr float HEIGHT_HOVER_THROTTLE = 0.35f; // Initial guess, not a measured hover value.

constexpr float HEIGHT_KP = 0.15f;  // Common-throttle correction per metre of error.
constexpr float HEIGHT_KI = 0.025f; // Common-throttle correction per metre-second of accumulated error.
constexpr float HEIGHT_KD = 0.15f;  // Damping per metre/second of vertical speed.
constexpr float HEIGHT_I_LIMIT = 0.12f;

constexpr float HEIGHT_THROTTLE_MAX = 0.65f;
constexpr float HEIGHT_THROTTLE_SLEW_PER_S = 0.8f; // Limit how fast throttle may change.
constexpr float HEIGHT_TARGET_SLEW_MPS = 0.20f;    // Limit how fast the target height may change.
constexpr float HEIGHT_VELOCITY_TAU_S = 0.15f;

// Arm while resting on the ground. The initial reading becomes the landing reference.
constexpr float HEIGHT_MAX_ARM_READING_M = 0.50f;
constexpr float HEIGHT_TAKEOFF_MARGIN_M = 0.05f;
constexpr float HEIGHT_LANDING_MARGIN_M = 0.03f;
constexpr float HEIGHT_LANDING_SPEED_MPS = 0.08f;
constexpr float HEIGHT_ZERO_REQUEST_M = 0.01f;
constexpr uint32_t HEIGHT_SETTLE_US = 500000; // Remain near resting height for 0.5 s before stop.

// A mistake here should fail compilation rather than start motors during calibration.
static_assert(!CALIBRATE_IMU_ON_BOOT || ITC_OUTPUT_MODE == 0,
              "Calibrate only in sensor bench mode.");

} // namespace config
} // namespace itc
