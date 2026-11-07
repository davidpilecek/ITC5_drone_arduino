# Optional IMU recording on Teensy 4.1's built-in SD slot

## The timing decision

SD card and filesystem calls can block unpredictably. Checking isBusy() or using
a buffer alone does not establish a hard execution-time bound. Therefore the
flight application makes NO SD calls while armed or while the radio arm switch
is above its OFF threshold. IMU samples go to a fixed RAM queue during that time.
Card operations, including opening, syncing and closing, wait until disarmed with
the arm switch OFF. No timer callback or interrupt accesses SD or formats text.
The motor driver's independent guard is retained without changes.

This removes SD I/O stalls from armed control; it is not a proof of zero timing
impact. Copying a sample takes CPU time, 256 KiB of RAM2 is reserved, and the exact
card/vehicle combination still needs on-device timing validation. SD calls may
cause cycle overruns while disarmed. The SD slot uses SDIO, separate from SPI1.
Do not move SD service into an interrupt, or change the safe-to-access test to
only check throttle. Disarmed PLUS arm-switch OFF is deliberate: the guard sees
the same OFF receiver snapshot throughout a blocking SD operation.

## First use: standalone test

The standalone IMU_SD_test now targets 1000 Hz (set SAMPLE_RATE_HZ): configurable
SPI clock (IMU_SPI_HZ, initially 1 MHz),
continuous operation, accelerometer/gyro DLPF bypass, and event-driven SD capture
(no 100 Hz logger gate). See examples/IMU_SD_test/README.md for the rate limits,
repeated accel/mag fields, timing diagnostics and optional angle estimation.
The flight application's default logging rate and sensor setup are unchanged.

1. Insert a FAT32 or exFAT card in the Teensy 4.1's built-in slot before power-on.
2. Open examples/IMU_SD_test/IMU_SD_test.ino, select Teensy 4.1 and upload when ready.
   This sketch creates no receiver or motor object. It explicitly enables logging.
3. Open USB Serial Monitor at Config.h's USB_BAUD (500000). After initialization,
   look for `SD IMU started: file=IMU0000.BIN ...`.
4. Move the IMU. Send uppercase `S` to stop sampling, drain queued records, and close.
   Wait for `SD IMU closed; safe to power off ...`, then power down and remove the card.
5. `L` requests a new recording with a new filename after the previous one closes.
   A missing/failed card stops the logger; fix it and reboot rather than retrying in flight.

This test can pause while writing to SD; it is for checking the sensor, card and
file format without motor control. It is not a flight timing benchmark.

## Enable in the drone application

Set `#define ITC_SD_IMU_LOG_ENABLED 1` near the top of src/Config.h and compile/upload
examples/Drone. The default is 0 (disabled). The derived setting SD_IMU_LOG_ENABLED
is then true. When enabled, a new file is requested at startup and opens
when disarmed with the arm switch OFF. Sampling starts after opening succeeds.
Logging records application IMU observations at up to 100 Hz (one every 10000 us),
not every 2 kHz control tick, and never reads the sensor a second time.

The 4096-record queue contains about 40.96 seconds at 100 Hz when no writes occur.
If a flight lasts longer, the newest log samples are dropped until space is free.
Flight control never waits for space. Sequence gaps and sd_drop expose lost
samples; timing gaps also count as dropped scheduled observations. Increasing
the period decreases rate and extends buffered duration. Capacity is capped at
256 KiB so the logger leaves RAM2 for the rest of the application.

After disarming and switching the arm control OFF, queued samples write to the
card in 512-byte sectors. Logging continues during that drain. Send USB `S` to
stop and close cleanly, and wait for the close message before power-off/removal.
Send `L` after closure to request a new file. Commands only request work: even
an `S` received while armed cannot trigger a card call until the safe condition.
Never remove the card while the logger is active. Changing an SD card requires
a reboot. A write failure halts recording without touching controller outputs.

Power loss before draining loses all armed samples still in RAM. Periodic sync
only runs disarmed, so abrupt power removal can also lose unsynced card data or
damage the current file. This initial version does NOT provide unlimited flight
logging or power-loss guarantees. That would require a separately validated
asynchronous storage design or an external logging device.

## What is recorded

Each fixed 64-byte binary record stores timestamp_us, sequence, accel XYZ [g],
gyro XYZ [deg/s], magnetometer XYZ [microteslas], roll/pitch/yaw [degrees], and
mag-valid, mag-calibrated and attitude-valid flags. These are the sensor module's
readings before estimator filtering; the angle estimates are from the existing
estimator. Cached driver observations may repeat. Invalid magnetometer values
are NaN. The microsecond timestamp marks completion of the application's IMU
read; it is not a hardware sensor timestamp. Yaw can drift in the six-axis estimator.

Files are uniquely named IMU0000.BIN through IMU9999.BIN. Existing files are never
overwritten. A 512-byte format header precedes sectors of eight records. Empty
slots in a partial sector have zero flags and are ignored by the converter.
Binary records keep string formatting out of flight control. Files are ordinary
files on the card; no raw sectors outside a file are accessed.

## Read on the PC

Run with Python 3 (no extra packages):

```
python tools/imu_log_to_csv.py IMU0000.BIN imu.csv
```

Use the actual paths to your copied BIN and desired CSV files. The converter
does not overwrite existing output. It handles timestamp wrap across one log,
skips padding and ignores an incomplete trailing record. CSV can be imported
into MATLAB, Python, or Excel. Missing sequence numbers identify gaps.

## Status and learning the structure

- FlightApp: hands the existing measurement to sample(), then decides whether
  card access is safe. USB S/L are reserved for the enabled logger.
- ImuLogFormat.h: on-disk data structure and bounded queue, with no Arduino calls.
- ImuSdLogger: sample() copies RAM; service() performs all filesystem operations.
- MAVLink sd_rec: recording samples (0/1); sd_drop: total scheduled samples lost;
  sd_queue: samples waiting for SD. A recording flag of 1 does not mean RAM is
  already persisted. 0 can mean disabled, waiting to open, stopped, or failed.
- USB start/failure/close events and the standalone test show file and I/O timing.
  maximumIoUs() records the longest observed SD service operation; it is not a guarantee.

Existing output mode, motor mapping, controller gains, MAVLink rate and arming
rules are preserved. The three SD diagnostics bring the MAVLink snapshot to
56 named values, still within its existing queue capacity and UART budget.

