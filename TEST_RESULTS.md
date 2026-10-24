# Combined V4 validation

Host checks use g++ C++17, -Wall -Wextra -Werror, AddressSanitizer and UndefinedBehaviorSanitizer. The .ino entry point is compiled separately against the host stubs. Run:

```sh
ASAN_OPTIONS=detect_leaks=0 python3 host_tests/run_tests.py
```

The shipping hardware lock is checked separately. The full regression suite runs with the lock enabled for motion, for both wiring profiles. Each profile runs 20 ideal-feedback route simulations and 10 simulations with quantised encoder counts, gyro noise, mild motor imbalance and isolated sensor-read gaps: 60 simulated routes total.

Coverage includes route optimality, parsing and invalid values, encoder direction, wrap-around, forward/reverse control, continuous dwell, overshoot/realignment, sensor loss, stall/timeout paths, uncertain-pose recovery, button behavior, calibration retries/cancellation, and actuator bounds. V4-specific cases cover preparation without auto-start, start without recalibration, consecutive demo commands, bounded/idle-only PWM tuning, handling during gyro saturation and large wheel increments, rejected resume without pose reset or healthy sensors, retained-fault resume, noise logging, and disabled-pin behavior in both profiles.

These are host shim and synthetic-model tests, not an ESP32 SDK build or hardware tests. Arduino CLI is present, but no board platform is installed and package-index downloads fail because network access to Arduino servers is unavailable. Consequently an actual Arduino-ESP32 compile has NOT been completed here. Verify the sketch in Arduino IDE with Arduino-ESP32 3.x, then test measured geometry/signs, stop behavior, calibration, turning, straight travel, handling/recovery and full routes on the actual robot.
