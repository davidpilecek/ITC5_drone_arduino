# Direct ToF sensor collection on SD

Open height_sd_test.ino and select Teensy 4.1. This standalone sketch reads the
VL53L1X directly using its library on Wire1 (SCL16/SDA17). It does not create a
RangeSensor wrapper, IMU, attitude estimator, receiver or motor object. It applies
no tilt correction, height filter, ground offset, application range limits or
validity-based rejection. These are the sensor library's measurements, not raw
photon/histogram data from inside the chip. The main flight sensor path is unchanged.

## Your draft: what needed fixing

- tof is now a VL53L1X, not the flight RangeSensor. Its driver provides dataReady()
  and read(false), so the example no longer needs an Attitude or calls update().
- ImuSdLogger expects ImuSample records and a fixed binary IMU format. A ToF
  observation has different fields. HeightSdLogger::sampleTof() accepts your
  HeightLogRecord, now in HeightLogFormat.h, and writes a separate CSV format.
- The unfinished sampleHeight() in ImuSdLogger.cpp referred to undefined imu,
  attitude and HeightLogRecord members. It was removed to restore compilation;
  the existing IMU logger and its binary records retain their original behaviour.

## Read the sketch in six steps

1. setup() configures Wire1 with the same pins as the flight application.
2. It initializes the VL53L1X in Short mode with a 20 ms timing budget and period:
   nominally 50 readings/s. FAST_SHORT_MODE=false selects Long mode with the
   minimum 33 ms budget/period (~30 readings/s). Short mode has reduced range
   (~1.3 m in favorable conditions); use Long mode for your tests up to 2 m.
3. S/L commands request stop/start work from the logger.
4. Each poll checks dataReady(). When ready, read(false) returns the distance in
   millimetres. One record copies distance, range status, signal and ambient rates,
   and I2C status. Invalid range statuses are recorded too; no values are replaced
   by filtered/corrected heights or rejected by application limits.
5. sampleTof() copies to a 64-entry RAM queue; service(true) writes a CSV row or
   opens/syncs/closes a file. SD calls can block; this logger is for this standalone
   test and is not added to flight control. It is not a lossless sensor FIFO: a long
   card stall can cause missed sensor conversions, which are not counted as drops.
6. Serial Monitor shows latest values and logger status at 10 Hz. Repeated USB
   rows are normal; the file receives only new read events, not repeated polls.

Readiness is polled every 500 us. This improves response to a finished conversion;
it does not make the sensor produce 2000 distance measurements per second.
The final USB column, measured_hz, counts actual read events over roughly 1 s.
These include invalid range statuses; always inspect range_status during analysis.
SD stalls can reduce the achieved rate. The CSV format is unchanged.

Shorter timing budgets generally increase measurement scatter. Identify noise
using the same distance mode, timing budget, target and lighting as the intended
controller configuration; a 20 ms Short-mode log does not characterize 50 ms Long mode.

The measured distance is along the beam. For example, a returned 350 is logged
as 350 mm. It is not converted to vertical height when you tilt the sensor.

## Collect a file

Insert a FAT32/exFAT card into the built-in SD slot before boot. Upload this
example when ready, then open USB Serial Monitor at Config.h's USB_BAUD (500000).
It opens TOF0000.CSV, then TOF0001.CSV, etc., without overwriting existing files.
Logging starts after opening succeeds; readings before that are not recorded.

- S: stop adding records, drain queued rows, then close/sync the file.
- Wait for sd_closed=1 and sd_failed=0 before powering down/removing the card.
- L: request another file after closure. During recording it has no effect.
- Missing/failed card: sd_failed=1; sensor output continues on USB. Fix and reboot.

CSV columns are timestamp_us, sequence, distance_mm, range_status, i2c_status,
signal_mcps and ambient_mcps. MCPS means million counts per second. The timestamp
marks completion of the host read, not a hardware sensor timestamp; it wraps at
about 71 minutes. sequence counts observations offered to an active logger.
Range status is the Pololu library's numeric status; 0 means RangeValid.
Use the driver's status names when analysing invalid values. Signal/ambient
rates are printed to four decimal places; distance/status values are copied as integers.
sd_dropped counts queue overflow, not conversions missed while the CPU was busy.

Open the CSV directly in MATLAB, Python or Excel. No BIN converter is needed.
Close cleanly: abrupt power loss can lose buffered/unsynced data or damage the file.
Before the first sensor read, USB shows samples=0, range_status=255 and a zero
distance placeholder; that placeholder is not written as a sensor reading.
