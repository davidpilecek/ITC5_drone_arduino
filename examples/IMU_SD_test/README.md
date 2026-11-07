# IMU SD measurement at a configurable recording rate

The example uses the same continuous SD logging approach as the original 100 Hz
example, now targeting 1000 readings/s. Internal accelerometer and gyro digital
low-pass filters remain bypassed. It does not add a hardware FIFO or RAM-only
capture stage. The main flight application and height example are unchanged.

## Change the rate

Set SAMPLE_RATE_HZ near the top of IMU_SD_test.ino. Examples:
- 100: original 100 Hz recording rate.
- 500: one attempt every 2000 microseconds.
- 1000: default, one attempt every 1000 microseconds.
- 2000: one attempt every 500 microseconds.
- 4000: one attempt every 250 microseconds.

This sets the host acquisition schedule, not the IMU's internal output rate.
Choose a rate with a whole-microsecond period. A requested rate is not a guarantee
that the sensor, driver and SD card can sustain it.

## Follow the data

1. imu.begin(true, IMU_SPI_HZ) uses a configurable SPI bus clock, initially 1 MHz,
   and continuous DLPF-bypass configuration. Full-scale ranges remain +/-2 g and
   +/-250 degrees/s. This bus clock is separate from SAMPLE_RATE_HZ.
2. The loop schedules read attempts using micros(). Advancing by whole periods
   avoids accumulating timing drift. USB commands and SD servicing continue
   between slots.
3. Each attempt calls readFresh(). A failed/not-ready attempt logs nothing;
   cached readings are not substituted. Missed schedule slots are skipped, not
   filled with invented catch-up samples.
4. logger.begin(true, 0) records every supplied observation. The loop controls
   the rate, so a second logger timing gate would unnecessarily reject readings
   because of normal jitter. Header samplePeriodUs=0 means event-driven; actual
   timing comes from record timestamps. Note SAMPLE_RATE_HZ with each experiment.
5. Binary records enter the existing 4096-entry RAM queue. SD service writes
   512-byte sectors while recording continues. At 1000 Hz, records require
   about 64 kB/s and the queue holds about 4.1 seconds of captured observations.

ESTIMATE_ATTITUDE=false is unchanged from the previous measurement sketch.
Angle columns contain NaN with attitude_valid=0. Set true to calculate angles;
this adds processing, while the recorded accel/gyro values still precede the
attitude estimator's software filters. Configured sensor biases are subtracted.

## Check the measurements

If initialization fails, the USB output now names the failed startup step and
prints the SparkFun driver status. Startup includes the magnetometer, so a
magnetometer error is distinct from an accel/gyro data-ready timeout. The former
7 MHz setting may not be reliable on every wiring setup. If 1 MHz still fails,
set IMU_SPI_HZ=100000 to match the earlier working sketch. Power-cycle the Teensy
and sensor together, then test again. A lower bus clock may reduce achieved
reading rate; frames_hz is the measurement to check, not the requested rate.

USB prints once per second:
- frames_hz: achieved successful reading rate, including while SD is stopped.
- max_gap_us: largest gap between successful reads in the reporting window.
- skipped_slots: scheduled slots missed because the loop ran late.
- no_read: scheduled attempts with no new reading or a sensor/bus error.
- queue_dropped: cumulative queue losses within the current recording.
- max_io_us: longest measured SD operation since boot.

Timestamp records describe host read completion, not sensor conversion times.
A zero queue_dropped count alone cannot prove lossless sensor acquisition.
SD calls may still block and cause skipped slots. No FIFO recovery is provided.
The gyro internally outputs nominally 9000 Hz, acceleration 4500 Hz and magnetic
measurements up to 100 Hz; a frame does not imply all sensor groups updated.
Unfiltered host subsampling can alias noise: interpret the file as the sampled
output of this configuration, not a universal intrinsic sensor noise density.

## Record a file

Insert the SD card before boot. S stops, drains and closes the recording; wait
for closed=1 and failed=0 before removing the card or power. L opens a new file
once closure finishes. Existing files are preserved. Convert IMUxxxx.BIN with
tools/imu_log_to_csv.py. The binary and CSV layouts remain unchanged.
