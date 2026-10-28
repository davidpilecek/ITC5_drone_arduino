# Start here: read the code as a sequence of jobs

You do not need to understand every C++ feature or every control equation before you can follow this program. Begin with the three files below. Leave the device drivers and advanced quaternion equations for later.

## 1. The entry point: ITC_Firmware.ino

```cpp
itc::FlightApp vehicle;

void setup() {
  vehicle.begin();
}

void loop() {
  vehicle.update();
}
```

Read this as: “Create the vehicle application. Start it once. Then repeatedly ask it to do its next piece of work.” Arduino calls `setup()` once and `loop()` repeatedly. The word `itc` is just our project's name group.

## 2. The settings: Config.h

This file is divided into numbered sections: output mode, wiring, radio, timing, IMU, attitude control, motors, height sensing and height control. Every setting has its own line. Keep mode 0 selected while learning: it calculates and prints results but sends zero motor commands.

An example:

```cpp
constexpr float HEIGHT_SETPOINT_MAX_M = 1.0f;
```

This declares a fixed decimal setting named `HEIGHT_SETPOINT_MAX_M`, with value 1 metre. `constexpr` means the value is fixed when compiling. The `f` tells C++ to use its `float` number type. Changing this file requires rebuilding and uploading before the vehicle uses the new setting; changing an editor tab does not change a running board.

Do not change the gains just to learn syntax. Use the separate LearningFilter example for experiments that do not affect vehicle behavior.

## 3. The main sequence: FlightApp.cpp

Find `FlightApp::update()`. It has eight numbered steps:

1. Read the pilot's radio commands.
2. Check whether it is time for another control calculation.
3. Read the IMU and estimate the vehicle's angles.
4. Read distance/height when a completed measurement is available.
5. Update height control, or preview it with motors disabled.
6. Calculate the four requested motor commands for the selected mode.
7. Give those commands to the motor safety guard.
8. Read RPM and print information at slower rates.

Read the numbered comments first. On your second pass, read the code between them. When you reach a function call, understand its job before opening its internal implementation.

For example:

```cpp
const RcState pilotCommands = receiver_.update();
```

Read this as: “Ask the receiver for its latest commands, store them in a record named `pilotCommands`, and do not modify that record in this scope.”

Then:

```cpp
const ImuSample imuReading = imu_.read();
const Attitude& measuredAttitude = estimator_.update(imuReading, elapsedSeconds);
```

Read this as: “Get one acceleration/rotation measurement. Give it, and the time since the last calculation, to the angle estimator.” The `&` gives us a view of the estimator's stored result instead of making another copy.

## A small vocabulary for this project

| Code | Plain meaning |
|---|---|
| `bool` | A true/false value |
| `float` | A number that can have a decimal part |
| `uint32_t` | A non-negative whole-number type used for timestamps here |
| `void` | This function does work but does not return a result value |
| `struct` | In this project, a record grouping related values |
| `class` | A definition of a component with its own memory and operations |
| `public` / `private` | Operations callers may use / details kept inside the component |
| `const` | This variable/view cannot be modified through this name |
| `if (...)` | Do the following work only when the condition is true |
| `return` | Finish this function call; the rest of the program continues |
| `for (...)` | Repeat work, such as once for each motor |
| `receiver_.update()` | Call the receiver object's `update` operation |
| `config::CONTROL_PERIOD_US` | The named setting inside our config group |
| `pilotCommands.throttleUs` | The throttle field inside the pilot-command record |
| `const Attitude&` | A read-only view of an existing attitude record |
| `bool& controllerHealthy` | A reference that the function may update for its caller |
| `imu_` | The trailing underscore is our convention for an object's own variable |
| `#include "Types.h"` | Make the definitions in that header available while compiling |

Time suffixes matter: `Us` means microseconds; `Ms` means milliseconds. Distance suffix `M` means metres. `Deg` means degrees, and `Dps` means degrees/second.

## Next: follow one value instead of reading every file

Choose ONE of these paths:

- **Throttle stick:** `Receiver::update()` → `mapCommands()` → `FlightApp::calculateMotorCommands()` → `Motors`.
- **Height request:** throttle channel → `heightRequestFromPulse()` → `HeightController::update()` → common throttle → motor mixer.
- **Roll correction:** desired roll minus measured roll → `AttitudeController::update()` → `mixQuad()` → four motor commands.

`Types.h` describes the records along those paths. Its comments explain each field and its units. You can return to it whenever a name is unfamiliar.

## Where to read the calculations

`Control.cpp` is arranged from simpler operations to advanced math:

1. Translate stick values into targets.
2. Combine common throttle and attitude corrections into four motor commands.
3. Calculate attitude corrections.
4. Check whether height feedback can still be used.
5. Filter IMU readings and estimate orientation.
6. Advanced quaternion equations at the end; skip these on your first reading.

`HeightController.h` has its own numbered stages. Its state names mean waiting, tracking height, landed and fault. The comments explain why zero stick means a descent request while airborne, and why the controller remembers previous measurements.

`SafetyLogic.h` is a short, self-contained next exercise: follow what happens when the arm switch goes low, then high. The timer-interrupt details in `Motors.cpp` are more advanced and are explicitly marked.

## Your first learning exercise

Without editing the flight code, explain these three lines to yourself:

```cpp
const float reachableTargetM = heightClamp(requestedM, groundM, HEIGHT_SETPOINT_MAX_M);
targetM = heightApproach(targetM, reachableTargetM, HEIGHT_TARGET_SLEW_MPS * sampleSeconds);
const float heightErrorM = targetM - measuredHeightM;
```

The first keeps the requested target within physically meaningful bounds. The second limits how far the target may move during this particular sample interval. The third measures how far below/above that ramped target we are.

After this feels comfortable, use `firmware/examples/LearningFilter` to practice making objects and changing parameters outside the aircraft code. The longer `ARCHITECTURE.md` remains available for file boundaries, headers, ownership and creating packages; it is a reference, not required first reading.
