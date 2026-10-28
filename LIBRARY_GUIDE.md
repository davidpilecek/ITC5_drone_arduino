# Using this local Arduino library

Install this folder as Documents/Arduino/libraries/ITC_Firmware.
Reusable headers and implementations live in src/. Each examples/ subfolder
is a separate program with its own setup() and loop(). Uploading one replaces
the program currently running on the Teensy.

## Start with IMU_test

Open examples/IMU_test/IMU_test.ino and select Teensy 4.1. It reads the MPU6500
and runs the existing attitude estimator. It does not initialize the range
sensor, receiver, or motor output. Its CSV columns include units in the header.
The USB baud setting comes from src/Config.h (currently 500000).
Yaw from this six-axis estimator is relative and can drift without a heading reference.
This example uses the existing calibration offsets; it does not measure new ones.

## How the pieces connect

1. #include <Sensors.h> exposes the declarations to the compiler.
2. itc::Imu imu; creates a sensor object in your sketch.
3. imu.begin() initializes its hardware from setup().
4. imu.read() returns one measurement from loop().
5. The sketch decides when to read, estimate, and print.

Headers describe operations and types. The corresponding .cpp files implement
them. Arduino compiles library .cpp files automatically; do not include .cpp
files in sketches. Driver includes are relative to their containing file.

Sensors.h still groups IMU and range sensor classes, so VL53L1X must be installed
to compile it, even though IMU_test never starts that sensor. Splitting those
classes into separate modules can be a later learning exercise.

## Full drone program

examples/Drone/Drone.ino creates FlightApp and runs the complete application.
Its output mode is selected in src/Config.h; the existing default is mode 0.
This packaging change does not change the controller or add MAVLink.

The older START_HERE.md, ARCHITECTURE.md and src/README.md describe the original
standalone sketch layout. Their application-level explanations still apply,
but paths now point into src/ and examples/Drone/.
The root verify.ps1 also targets the original repository layout and is retained
as reference; use the Arduino IDE to compile these installed examples.
