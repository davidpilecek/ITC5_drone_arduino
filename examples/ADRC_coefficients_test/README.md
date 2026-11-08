# First ADRC module: coefficient generation

Open ADRC_coefficients_test.ino, select Teensy 4.1, compile and upload when you
choose to run it. The example prints coefficients once over USB Serial at
115200. It creates no sensor, receiver, estimator, controller-update or motor
objects. The example tuning numbers are for calculation checks, not flight use.

## File structure

- src/ADRC.h declares AdrcCoefficients and makeADRC_D(). It is the public interface:
  include <ADRC.h> in a sketch that needs ADRC coefficients.
- src/ADRC.cpp implements the equations. Arduino compiles this source as part of
  the library; the sketch does not include the .cpp file.
- This example shows how to call the function and inspect its returned fields.

The existing Control module, main flight code, and unfinished ADRC_test are preserved. This example only reproduces your coefficient calculation.
It does not yet evaluate a controller recurrence or calculate actuator commands.

## Calling it

    #include <ADRC.h>

    itc::AdrcCoefficients p;

    void setup() {
      p = itc::makeADRC_D(20.0, 5.0, 1.0, 0.001);
      if (!p.valid) {
        // Reject these parameters; do not use their coefficients.
        return;
      }
      // Access fields exactly as in MATLAB: p.zCL, p.beta0, p.gamma2, etc.
    }

AdrcCoefficients is the C++ equivalent of your MATLAB p structure. The function
returns it by value. The itc:: prefix selects the same namespace as your other
firmware modules. valid is the one extra field: it reports whether this numerical
calculation succeeded, not whether a physical closed loop will be stable.

## Inputs and units

- wc: positive controller bandwidth in radians/second. If starting with Hz,
  convert using wc = 2*pi*f_Hz before calling the function.
- kESO: positive observer/controller bandwidth ratio. wo = kESO*wc.
- b0: finite, nonzero nominal plant input gain. Its sign is preserved. Its units
  depend on your plant input and controlled output; degrees and radians require
  correspondingly different gains. This function does not estimate b0.
- Ts: positive sampling period in seconds. For 1 kHz control, Ts=0.001.

Inputs must be finite. Invalid inputs, a zero/overflowed denominator, or nonfinite
results leave valid=false. Check valid before using ANY returned coefficients;
on failure some fields may still contain defaults or intermediate results.

## Translation decisions

The field names and equation grouping follow the supplied MATLAB function.
C++ ^ is bitwise XOR, so squared/cubed terms use multiplication. Repeated powers
are stored in local aliases. std::exp() calculates the discrete poles.
The example uses descriptive tuning names because Arduino already defines B0
as a binary-number macro; a new variable named B0 would not compile.

Use double throughout. Some beta terms involve subtracting nearly equal values
before multiplication by 1/(b0*Ts^2). float can lose important digits here. The
module requires at least 64-bit double, as supported by the Teensy 4.1 toolchain.
Even double has finite precision; extreme tuning/sample-period values can remain
ill-conditioned. A finite result is not a guarantee of arbitrary numerical accuracy.

## Timing and the next implementation step

Calculate coefficients in setup(), or when you intentionally change tuning or
the controller period. They depend on tuning, not on each new sensor reading.
AdrcController must execute with the Ts these coefficients use.
Replacing Ts with irregular loop elapsed time would require a separate design.

AdrcController now implements the dual-feedback runtime, reset and limited-input
feedback in ADRC.h/ADRC.cpp. See ADRC_runtime_test for its equations and software-only
example. This coefficient-printing example still only calculates the parameters.


TrackingDifferentiator is now declared in ADRC.h and implemented in ADRC.cpp;
its former TD.cpp file is archived outside the library. See the separate
ADRC_TD_test example for reference smoothing. This example still only prints coefficients.



