# Flight diagnostics over DroneBridge

The full Drone example now starts Serial8 at Config.h's MAV_BAUD (115200).
The ESP32 bridge does not need a new protocol or a custom MAVLink dialect.
Open QGroundControl's MAVLink Inspector after connecting to the ground UART
converter. Look for system 1, component 1: HEARTBEAT and NAMED_VALUE_FLOAT.
Select/expand the named values; the exact presentation depends on QGC version.
These debug messages do not populate every normal flight-dashboard instrument.

## Follow the data through three small pieces

1. FlightApp reads sensors, runs control and publishes the motor proposal first.
   captureMavlinkDiagnostics() copies that existing information into named values.
   It never reads a sensor again or runs a controller a second time.
2. FlightTelemetry remembers a bounded snapshot and sends one packet per update.
   It schedules heartbeats at 1 Hz and complete diagnostic snapshots at 4 Hz.
   Every named value in one snapshot has the same time_boot_ms. Motor applied
   values are the guard's latest completed interrupt snapshot, so they can be
   one motor frame behind the just-published proposal.
3. Comms retains your pack -> serialize -> Serial8.write sequence. It checks
   availableForWrite() before packing and returns false when the UART is busy.
   The scheduler then retries that value next time. There is no flush(), delay(),
   heap queue, or per-packet USB print. Only this module writes Serial8 in Drone.

The control loop runs at its existing rate. Slow telemetry samples cannot show
all fast control dynamics; these are monitoring diagnostics, not a 2 kHz log.
If transmission falls behind, the next snapshot waits; no growing backlog forms.
The named values within a snapshot arrive progressively, not all at once.
Firmware compilation cannot prove wireless delivery or measure loop timing on hardware.

## Names and units

All names fit nine characters plus a trailing zero in the ten-byte name field.
Booleans and integer diagnostics are represented as floats for your sendFloat API.

| Names | Meaning / units |
|---|---|
| mode | Configured output mode: 0 preview, 1 bench, 2 attitude, 3 height |
| armed | Actual motor-guard armed state, 0 or 1; mode 0 still applies zero output |
| rc_ok, h_ok, att_ok | Receiver, usable height, finite attitude validity flags |
| range_m | Last distance along the range sensor beam, metres; may be old |
| height_m | Tilt-corrected, filtered vertical distance, metres; NaN if unusable |
| rng_stat, h_age_ms | Range-driver status and age of last accepted sample in ms |
| ax_g, ay_g, az_g | Sensor-axis acceleration in g, existing offsets applied |
| gx_dps, gy_dps, gz_dps | Sensor-axis angular rates, degrees/second |
| mx_uT, my_uT, mz_uT | Sensor module's magnetometer output, microteslas; NaN if invalid |
| mag_ok, mag_cal | Fresh/valid magnetometer reading and user-calibration flags |
| roll_deg, pitch_deg, yaw_deg | Estimated angles, degrees; current estimator uses six axes, so yaw can drift |
| erpm_FL, erpm_FR, erpm_RL, erpm_RR | Electrical RPM from ESC feedback, not shaft RPM; NaN for missing feedback |
| req_FL, req_FR, req_RL, req_RR | Proposed DShot command integers, before the motor guard |
| out_FL, out_FR, out_RL, out_RR | DShot command integers actually applied by the guard |
| sp_roll, sp_pitch | Pilot's requested roll/pitch angles, degrees |
| sp_yaw | Requested yaw rate, degrees/second |
| sp_thr | Collective command used by the selected path, normalized 0..1; mode 0 shows pilot demand |
| ctl_roll, ctl_pitch, ctl_yaw | Attitude-controller mixer corrections in normalized command units |
| ctl_on | Whether attitude control ran in this sampled cycle |
| h_req_m, h_tgt_m, ground_m | Requested height, slewed target and resting clearance, metres |
| vz_mps, h_thr, h_stage | Estimated vertical speed, height collective and height-controller stage |
| loop_us, range_us, overruns | Existing maximum cycle gap, maximum range-read time, and cycle-overrun count |
| rc_thr_us | Pilot's throttle channel in the 1000..2000 microsecond convention |

FL=5, FR=6, RL=8, RR=4. DShot commands are not measured RPM or power percentages.
The height-controller diagnostics are active in modes 0 and 3 only. Ground
clearance intentionally remains NaN before capture or after resetting/disarming.
In mode 0, req/out and attitude corrections remain zero by the existing design.
This integration does not implement a new preview attitude controller.

HEARTBEAT now reports the real armed flag, the output mode in custom_mode, and
STANDBY/ACTIVE/CRITICAL based on application readiness. This health indication
is based on existing software checks, not a complete vehicle-health assessment.
No incoming MAVLink commands, remote arming, parameter service, mission protocol,
or acknowledgements are implemented. CRSF remains the pilot input.

## Why four snapshots per second?

53 unsigned MAVLink 2 named-value packets use at most 53 * 30 * 4 = 6360 bytes/s,
plus roughly 21 bytes/s for the heartbeat. 115200 baud with 8N1 framing allows
11520 bytes/s in theory. This leaves room for bridge overhead and avoids queuing
53 messages together. No signed MAVLink transport is configured here.
The scheduler has capacity for 56 values and checks its worst-case UART budget
at compile time. Raising all 53 values to 10 Hz would exceed this UART's capacity.

## First hardware check

Keep the existing output-mode selection in mind (currently mode 2; this change
preserves it). With props removed and the radio disarmed, compile/upload Drone
when you are ready. Check heartbeat/armed/mode, then h_ok/height, IMU, and motors.
Disarmed out_* should be zero; missing eRPM may be NaN. Validate link continuity
and watch loop_us/overruns before relying on this with active motor output.

## Add another diagnostic yourself

Add one telemetry_.addValue("shortname", measurement) line to
captureMavlinkDiagnostics(). Choose a unique name of at most nine characters,
document its units here, and check the field count stays within 56. If adding
more than 56 fields, increase MAX_VALUES in FlightTelemetry.h, then choose a
period within the baud budget. The budget calculation uses that capacity automatically.
