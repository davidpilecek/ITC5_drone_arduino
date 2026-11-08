# Tracking differentiator in the ADRC module

Include ADRC.h to use either makeADRC_D() or TrackingDifferentiator. The public
declarations live in ADRC.h; the calculation bodies live in ADRC.cpp. The former
TD.cpp class definition has been moved into this pair, so sketches can now see
it without including a .cpp file. It is not wired into the flight controller.

## What the TD does

It converts a requested reference into a smooth reference and a reference rate.
For example, a requested roll angle can jump from 0 to 10 degrees, while the TD
generates a gradual motion toward 10 degrees and its rate in degrees/second.
It processes the reference command, not the measured gyro/accelerometer signal.

    itc::TrackingDifferentiator td(0.001, 3.0, 0.005);
    //                            Ts     r0     h0

    // During setup, after checking td.valid():
    td.reset(0.0, 0.0);

    // Once every Ts seconds:
    const itc::TDOutput shaped = td.update(requestedReference);
    if (shaped.valid) {
      // shaped.ref  : smoothed reference
      // shaped.dref : reference rate
    }

The example sketch supplies a simulated 0 -> 1 command after one second. It
targets 1 kHz updates and prints only at 10 Hz. It creates no motor objects.
Ts is a fixed discretization period; the update function does not measure time
or schedule itself. Missed loop deadlines change the reference's real-time
trajectory. As with the other modules, correct timing must be checked on hardware.

## Parameters and state

- Ts [s]: period between updates.
- r0 [reference units/s^2]: acceleration parameter controlling how rapidly the
  reference rate can change. The old default/example number 3.0 is not flight tuning.
- h0 [s]: smoothing horizon; commonly Ts or a multiple of Ts. Choose it together
  with r0 and Ts. Positive inputs alone do not guarantee suitable dynamics.
- reference_ is the old v1: the current smoothed reference.
- rate_ is the old v2: its rate of change.

For a height command measured in metres, ref is metres, dref is m/s, and r0 is
m/s^2. For an angle measured in degrees, the corresponding units are degrees,
degrees/s and degrees/s^2. Keep these consistent with the later ADRC plant gain.

## Update order

The original explicit-Euler equations are preserved:

    acceleration = fhan(reference - input, rate, r0, h0);
    nextReference = reference + Ts * rate;
    nextRate      = rate + Ts * acceleration;

Both expressions use the PREVIOUS states. Only after calculating and checking
both results does the implementation assign the new states. Using nextRate in
nextReference would implement a different discretization.

The original fhan switching expressions are unchanged. Comments explain their
role, while the mathematical names a0, a1, a2, sy and sa remain easy to compare
with the original reference code. All calculations continue to use double.

## Error handling

td.valid() describes configuration validity. Invalid/nonpositive parameters or
a zero/nonfinite fhan denominator are rejected. Each update also returns its
own valid flag. A nonfinite input/result leaves BOTH states unchanged, returns
valid=false, and permits a later valid input to recover. Do not silently treat
that rejected sample as a successful new reference when integrating the controller.

reset(initialRef, initialDref) returns false for invalid arguments/configuration
and preserves state. For a known startup setpoint, reset directly to that value
with zero rate if you want to avoid a transition from zero.

The TD is only a reference generator. It does not yet calculate the ADRC actuator
command, close the ESO feedback loop, or implement actuator saturation handling.
