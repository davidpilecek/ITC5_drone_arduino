# ITC modular firmware

Open **ITC_Firmware.ino** in this directory and select **Teensy 4.1 / USB Serial**. This is the new modular application. The earlier `dRehmFlight_ITC` folder and the separate RPM/logger sketches have not been modified by this refactor. No firmware was uploaded.

**New to programming? Start with [START_HERE.md](START_HERE.md).** Then read the comments in `ITC_Firmware.ino`, the numbered settings in `Config.h`, and the eight-step `FlightApp::update()` function. The longer [architecture guide](ARCHITECTURE.md) is a reference for later. The old height-control explanation remains applicable to the controller equations and limits, but the new code uses the structure described here.

## Files by responsibility

| File/module | Owns |
|---|---|
| `ITC_Firmware.ino` | Arduino entry points only |
| `Config.h` | Mode, pins, gains, calibration, timeouts and periods |
| `Types.h` | Named data records, units, canonical motor order |
| `FlightApp.h/.cpp` | Module instances, startup, schedule and CSV telemetry |
| `Receiver.h/.cpp` | CRSF parsing, channel mapping and link validity |
| `Sensors.h/.cpp` | MPU6500 and VL53L1X device access |
| `Control.h/.cpp` | Filtering/attitude estimation, angle control and quad mixing |
| `HeightController.h` | Existing height PI plus speed damping and landing state |
| `SafetyLogic.h` | Existing arming gate and wrap-safe age checks |
| `Motors.h/.cpp` | Interrupt-safe handoff, final output guard and RPM access |
| `src/` | Bundled low-level MPU and DShot drivers; provenance in `THIRD_PARTY.md` |

There is no dependency on the old `.ino` file, its global variables, or its named functions. The active numerical algorithms are retained with attribution; unused receiver types, nine-axis estimation, alternate controllers, servo declarations and legacy print functions are absent from this application.

## Hardware and modes

Motor order is **FL, FR, RL, RR**, with `config::MOTOR_PINS = {5, 6, 8, 4}`. The bundled adapter derives timer/DMA mappings from that pin list and rejects unsupported or duplicate assignments at compile time. Supported pin assignments are permutations of 4, 5, 6, 8; changing to another pin requires extending and checking the driver adapter.

CRSF uses Serial5, RX21/TX20. MPU6500 uses SPI1, CS0/MISO1/MOSI26/SCK27. VL53L1X uses Wire1, SCL16/SDA17. Use 3.3 V compatible signal levels; for the Pololu carrier, VIN=3.3 V avoids 5 V I2C pullups on the Teensy pins.

`Config.h` defaults to `ITC_OUTPUT_MODE=0`:

- **0:** sensor/height-controller preview; all motor commands zero.
- **1:** equal manual throttle on all four motors, 10% command-span limit.
- **2:** manual collective plus angle stabilization, full motor command range.
- **3:** CH3 requests 0–1 m sensor height; height feedback supplies collective.

CH1/2/3/4/5/6 mean roll/pitch/throttle/yaw/arm/auxiliary. CH5 high arms only after a low/high cycle with low throttle. Link loss after 100 ms or a control heartbeat older than 20 ms stops all motor commands. Mode 3 also requires usable height and an accepted height sample younger than 200 ms. Recovery requires a new arm cycle. These remain stop policies, not a validated automatic airborne recovery strategy.

Height mode retains its unmeasured hover estimate of 0.35, provisional gains, 0.20 m/s target ramp, 0.65 collective cap, captured resting clearance and range-based touchdown heuristic. A 1 m target limit is not a hard physical ceiling. Boat/servo transition control and horizontal position hold remain unimplemented.

## Timing and telemetry

The main application attempts an attitude/control update every 500 microseconds, polls the range sensor every 5 ms for new 20 Hz samples, reads RPM accumulators every 10 ms and prints the existing CSV fields every 100 ms at 500000 baud. DShot frames run independently at 1 kHz. I2C work can still exceed 500 microseconds; this refactor does not establish a hard real-time deadline.

Calibration: set `CALIBRATE_IMU_ON_BOOT=true` only in mode 0, keep the sensor still and level, and capture the printed replacement `ACCEL_BIAS_G`/`GYRO_BIAS_DPS` arrays. Copy them into `Config.h` and turn calibration off. Sensor range selection remains fixed at 2 g and 250 dps; its conversion constants must agree with `Imu::begin()`.

## Verification

Run `verify.ps1` in this folder to build the desktop checks and all four Teensy modes using the locally installed tools. Tests are outside the sketch in `firmware/tests/modular`, so Arduino does not link them into the flight firmware. Logs are written under `diagnostics/`.

Desktop checks compare the active estimator/controller equations with the previous implementation, re-run the height scenarios and arming assertions, check the mixer signs/order, and exercise the independent learning package. The two deliberate bug fixes are documented in `ARCHITECTURE.md`. Compilation and these tests do not validate physical mounting, motor rotation, sensor accuracy, loop timing or flight tuning.

Remove propellers for first hardware checks, start in mode 0, verify all named channels and measurement signs, and check the actual FL5/FR6/RL8/RR4 wiring before enabling outputs. Mode 1 is the next basic motor/RPM check. Mode 3 requires measured hover collective and a stable attitude setup before flight use.
