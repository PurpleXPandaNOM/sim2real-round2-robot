# V4 changes from V3.1

- PREP/CAL only calibrates. START or idle BOOT starts an already-prepared robot with the 5-second countdown.
- Explicit HANDLE mode stops and invalidates pose before carrying; RECOVER supplies a known waypoint and heading. RESUME validates health and retries retained faults without a separate command.
- Successful standalone TURN/ROTATE/DRIVE tasks retain arming and arena pose. STOP, faults and completed routes disarm.
- Bounded RAM-only PWM and MINPWM commands; reported values in STATUS.
- NOISE 1/0 logs per-report accumulated absolute encoder travel, peak gyro rate, invalid samples and quiet time. No fabricated empirical threshold tuning.
- Two explicit motor wiring profiles, with disabled optional pins guarded. GPIO4 is not a STOP input in the externally enabled Yashas-map profile.
- Existing exact arc integration, gyro/encoder fusion, directional friction, heading slowdown, exhaustive route planning, radial arrival tests, continuous dwell evidence, calibration variance checks and fault detection retained.

A percentage score from another model is not a validation criterion. Donor code was evaluated by behavior; V4 remains based on the tested combined project. Hardware geometry, signs, electrical connections, controller gains and thresholds still require physical verification.
