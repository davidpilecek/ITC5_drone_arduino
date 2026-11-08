# IMU and attitude-estimator timing test

This example now follows the main application's Config.h values:
CONTROL_PERIOD_US=1000 (1 kHz target) and IMU_SPI_HZ=1000000 (1 MHz bus).
It uses the same Imu::read() and AttitudeEstimator::update(), with internal
digital filters bypassed. Existing estimator software filtering remains active.
No FlightApp, receiver, motor, distance-sensor or SD objects are created.

## Run

Select Teensy 4.1, upload this example when ready, and open USB Serial Monitor
at Config.h's USB_BAUD (500000). Observe at least several seconds, including
the first completed one-second rate window. Nothing has been uploaded by the
assistant. These timings must be measured on your actual Teensy and wiring.

Output is CSV, printed at 10 Hz. It retains roll/pitch/yaw in degrees and adds:

| Column | Meaning |
| --- | --- |
| control_hz | Completed sampling starts per elapsed second, roughly 1 s window. Initially 0. |
| imu_us | Duration of this row's one IMU read, including driver overhead/interrupts. |
| estimator_us | Duration of this row's attitude estimator call. |
| max_imu_us | Largest IMU-read duration since startup. |
| max_estimator_us | Largest estimator duration since startup. |
| max_work_us | Largest complete work duration, including occasional print work. |
| max_loop_us | Largest sampling start-to-start interval since startup. |
| overruns | Number of completed work durations exceeding the target period. |

time_ms is time since boot. Maxima and overrun counts are cumulative. Because
printing happens during a cycle, its complete work duration appears in later
rows, after the cycle finishes. USB disconnection does not bypass timing updates.

## Interpreting results

At the 1 kHz target, expect control_hz near 1000 if the work fits consistently
within 1000 us. Compare read/estimator times with the whole-work maximum. Zero
overruns is not proof of zero jitter: inspect max_loop_us and control_hz too.
This counts scheduled estimator updates, not verified independent valid sensor
conversions. read() can reuse a recent sample, just as in the main program.

imu_us should be compared with the earlier 344 us estimate at 1 MHz. That estimate
counted 43 clocked bytes at 8 bits per byte; it was not a hardware measurement.
The estimate describes a normal full fresh read. A cached/not-ready path can be
shorter, while driver overhead and interrupts make a full read take longer.

To compare bus speeds without changing the main app, temporarily change THIS
sketch's IMU_SPI_HZ constant to 100000, compile/upload, and repeat. Restore it
to itc::config::IMU_SPI_HZ for the normal 1 MHz case. Keep the target period the
same. Slow reads may prevent reaching 1 kHz; the diagnostics should reveal that.

The example isolates IMU + estimation and print overhead. It does not prove the
complete flight loop meets 1 kHz when ToF, receiver, controllers, MAVLink and
optional SD operations are also present. Validate that separately in Drone.

All CSV columns share one stream. Timing/rate numbers are much larger than
angle noise, so use Serial Monitor or select columns in MATLAB rather than
expecting one automatically scaled Serial Plotter view to show them equally well.
