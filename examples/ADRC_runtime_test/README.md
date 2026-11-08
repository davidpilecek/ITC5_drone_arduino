# Second-order ADRC: efficient dual-feedback runtime

This implements the supplied minimum-footprint diagram in ADRC.h/ADRC.cpp.
It is the OUTPUT-based dual-feedback transfer-function form, not the error-only
form: it accepts reference r and measurement y separately. The coefficients
still come from makeADRC_D(wc,kESO,b0,Ts). The TD and coefficient API are preserved.

The standalone example drives only a SOFTWARE plant. It creates no sensor,
receiver, mixer, ESC or motor objects. Example gains/limits are not flight tuning.

## Read the diagram as one control sample

Let x1, x2 and x3 be the outputs of its three delay blocks at sample k. Let
K = k1_over_b0. The upper summers calculate:

    f = x1 - beta0*y;
    u = K*r + f;

For channels using the optional TD reference-rate feedforward, replace only the
second line with:

    u = K*r + (k2/b0)*referenceRate + f;

This is equivalent to (k1*TD.ref + k2*TD.dref)/b0 + f in your Simulink diagram.
The alpha feedback still uses f, not this full u. The gamma path still receives
the full applied/limited input. k2/b0 is checked and precomputed in configure().

Here f is the feedback contribution. It is f, not the total u, which goes through
the alpha gains. Once the limiter/actuator processing has chosen u_lim[k], the
lower summers calculate the values for sample k+1:

    next1 = x2 - beta1*y + gamma0*u_lim - alpha1*f;
    next2 = x3 - beta2*y + gamma1*u_lim - alpha2*f;
    next3 =               gamma2*u_lim - alpha3*f;

All three use the OLD states and are committed together. The current limited
input therefore affects the NEXT output, preserving the explicit one-sample
delay in the control-input feedback path. Feeding a previous u_lim into these
next-state equations would insert an extra delay. Feeding raw u instead would
ignore limiting and change the observer feedback during saturation.

The shared runtime recurrence uses three delay values and ten scalar
multiplications per reference-only sample, plus additions, checks and limiting.
Nonzero reference-rate feedforward adds one multiplication and one addition. No
exponentials, dynamic allocations, sensor reads or actuator writes occur in it.
The split interface also stores two current-sample temporaries between calls;
these are not additional recursive delay states. x1/x2/x3 are filter states,
not directly interpretable ESO estimates of position, rate and disturbance.

## Configure once

    #include <ADRC.h>

    itc::AdrcController controller;

    // In setup(), with EXAMPLE parameters only:
    const auto p = itc::makeADRC_D(5.0, 5.0, 1.0, 0.001);
    const bool ready = controller.configure(p);

configure() copies coefficients and clears all three delays. It rejects invalid
coefficient sets and nonfinite values used by the runtime. On rejection, the
previous configuration and state remain unchanged; always check its return.
An optional constructor taking AdrcCoefficients is also available. valid()
describes configuration, not closed-loop stability or suitability for a vehicle.

## Option A: local output limits

Once every p.Ts seconds:

    const itc::AdrcOutput result = controller.update(r, y, uMin, uMax);
    if (result.valid) {
      // result.requested: raw controller request
      // result.limited:   clipped value, already fed back into the recurrence
      // result.saturated: whether clipping changed this sample's request
    }

For a channel which uses reference-rate feedforward, use the FIVE-argument call:

    const auto result = controller.update(r, referenceRate, y, uMin, uMax);

The four-argument form is unchanged and equals a reference rate of zero. Each
channel can choose independently, with no global controller mode. referenceRate
is the desired reference's rate, not the measured gyro/plant rate.

Finite uMin <= uMax is required; equal limits, including 0/0, are supported.
This convenience path assumes this limiter determines the plant input. Do not
add further limiting downstream and then pretend the earlier value was applied.

## Option B: external mixing, rate limits or actuator constraints

Use this INSTEAD of update(), not in addition to it:

    double requested = 0.0;
    if (controller.calculate(r, y, requested)) {
      // Determine the actual controller input after ALL external constraints.
      const double applied = /* result of your actuator processing */;
      const bool accepted = controller.commit(applied);
      // Handle !accepted; no delay states were committed on failure.
    }

calculate() reads the old states and stages y and f. commit() accepts the current
applied input and advances the three states exactly once. The two calls belong
to the same sample. Another calculate() is rejected while a sample is pending.
commit() without calculate() is rejected. awaitingAppliedInput() exposes this
condition for diagnostics. commit() never writes an actuator itself.

The external-limiter equivalent with rate feedforward is:

    double requested = 0.0;
    if (controller.calculate(r, referenceRate, y, requested)) {
      // Determine applied input after all constraints, then commit it as above.
    }

Neither path requires different alpha, beta or gamma coefficients. The entire
feedforward-plus-feedback request is limited; the rate term must not be added
after limiting. Nonfinite reference rates/commands are rejected without advancing
states. configure() rejects a zero/nonfinite b0 or nonfinite k2/b0 gain.

For an eventual quadrotor controller, applied must represent the realized
axis-level effort in the SAME units as u and b0, after mixing/limiting. It is not
an arbitrary motor's DShot number or its RPM. That mapping remains a separate
flight-integration task. This module has not been wired into FlightApp.

## Reset and startup

    controller.reset();          // Zero delays; suitable for a zero operating point.
    controller.reset(y0, u0);    // Initialize with r=y0 and first requested u=u0.

The second form initializes a stationary held operating point, subject to finite
precision. A later reference change can still change the feedforward request.
Reset assumes referenceRate=0. A nonzero reference rate immediately after reset
adds its feedforward contribution to the first request. The first zero-rate
request is initialized to u0; subsequent updates follow the rounded
coefficients. Near-unit poles and beta-coefficient cancellation can create small
DC residuals, so this does not promise bit-exact indefinite holding of u0. In the
20 rad/s, kESO=5, b0=1, Ts=1 ms held-input test, the one-second numerical drift
was about 5e-5 input units. No coefficient adjustment is silently applied here.
The reset output is not limited internally; the supplied u0 should be feasible.
For a pending calculation, a successful reset discards that pending sample.
Invalid reset arguments leave both state and pending information unchanged.

For inspection, controller.state() returns x1/x2/x3 and samplePeriod() returns Ts.
These do not read or reset any hardware.

## Rejected samples and timing

Nonfinite measurements/references, invalid local limits or numerical overflow
return failure. Do not use an AdrcOutput whose valid flag is false. Delay values
are updated only after all next-state calculations are finite. If commit fails,
the pending sample remains staged so you may retry with valid applied input or
explicitly reset; it is not silently treated as a successful completed sample.

There is no scheduler inside the class. Invoke one full sample every configured
Ts. The example targets 1 kHz, but actual hardware timing still needs measurement.
Missed deadlines do not trigger invented catch-up controller steps. These are
fixed-period transfer-function coefficients; arbitrary elapsed-time substitution
would not preserve the designed controller.

## Where the tracking differentiator belongs

    requested command -> TD -> shaped.ref -> ADRC reference input r
    measured output ---------------------> ADRC measurement input y

The original reference-only form uses shaped.ref. To reproduce your extended
Simulink feedforward diagram on selected channels, pass shaped.dref too:

    const auto result = controller.update(
        shaped.ref, shaped.dref, measurement, minimumInput, maximumInput);

For angle references in degrees, dref is degrees/s. Keep the measurement, TD,
plant gain and actuator-effort units consistent. The implementation does not
numerically differentiate noisy measurements to obtain this reference-rate input.
Keep TD/controller units and sampling periods consistent. Sensor filtering,
the attitude estimator, actuator mapping and plant dynamics are separate modules.

## Run the software-only example

Open ADRC_runtime_test.ino for Teensy 4.1. It prints CSV at 20 Hz while targeting
1 kHz calculation. The command steps from 0 to 1 after one simulated second.
The TD smooths it; ADRC requests an input limited to [-2,2]; the software plant
uses y''=u+d. A constant disturbance d=0.3 begins at simulated t=6 seconds.
time_s is the software plant's sample-count time. No aircraft hardware is driven.
Set USE_REFERENCE_RATE_FEEDFORWARD=true to enable the extra term in this example;
false preserves its previous behavior. The CSV now also includes reference_rate.

## Verification and source

The recurrence follows Eq. (8.41) and the efficient realization in Fig. 8.9 / the
appendix diagram in Herbst and Madonski, Active Disturbance Rejection Control,
Chapter 8: https://link.springer.com/chapter/10.1007/978-3-031-72687-3_8 .

Host tests compare against an independent direct-form-I history implementation,
exercise local and external limiting, verify call order and atomic state updates,
and simulate a saturated double-integrator plant with a constant disturbance.
The MATLAB check uses two independent filter() paths with a leading zero in
the u_lim numerator. Tests establish the software realization, not flight tuning
or vehicle performance. All actual closed-loop flight integration remains separate.
