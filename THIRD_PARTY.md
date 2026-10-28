# Source provenance

- `Control.cpp` retains the active six-axis Madgwick equations and angle-controller/mixer behavior derived from Nicholas Rehm's dRehmFlight Beta 1.3. The original credits include Arduino MadgwickAHRS and contributions by RcGroups user jihlein. The surrounding application has been replaced by the modular ITC implementation. `COPYING.txt` retains the original GNU GPL text.
- `src/MPU9250` is Brian R. Taylor / Bolder Flight Systems' 2017 driver, with the project's existing explicit six-axis MPU6500 adaptation. Original source copyright and GPL notices are retained.
- `src/BidirectionalDShot` derives from Jared Boyer's bidirectional DShot implementation and the project's existing DShot300, RPM snapshot, frame-callback and decoder guards. The local refactor adds translation from the central configured motor-pin order; pulse generation and decode logic are retained.
- CRSFforArduino and the Pololu VL53L1X library remain external installed dependencies. Their original notices/terms remain in their installed sources.

The code is not represented as an independently invented estimator or low-level driver. Removing legacy application functions does not remove the provenance of algorithms and code retained from them.
