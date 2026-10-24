# Combined V4 — start here

Open `Round2_Postman/Round2_Postman.ino` in Arduino IDE. Keep Config.h, Core.h and Robot.cpp in that same sketch folder. Do not add another version of Robot.cpp or another setup()/loop() implementation. Use the original ESP32-WROOM target and Arduino-ESP32 3.x.

This is an updated, host-tested firmware project, not a hardware-certified competition release. Read `TEST_RESULTS.md` for the limits of validation.

## 1. Match your actual hardware once

Edit `Round2_Postman/Config.h`:

| WIRING_PROFILE | Left RPWM / LPWM / EN | Right RPWM / LPWM / EN | STOP input |
| --- | --- | --- | --- |
| 0 (default) | 16 / 17 / 27 | 18 / 19 / 23 | GPIO4, active LOW |
| 1 (Yashas PWM map) | 18 / 19 / externally HIGH | 23 / 4 / externally HIGH | Disabled (-1) |

Both profiles assume two independent reversible motor channels. Confirm the actual board, enable connections and driver truth table. Profile 1 cannot disable enables in software; both PWM inputs are driven LOW to stop. With a BTS7960 bridge enabled, this may brake rather than coast. Never connect profile 0's STOP input to a motor PWM signal.

Encoder pins remain 32/33 and 25/26; IMU SDA/SCL are 21/22. Neither profile changes encoder resolution, wheel diameter or motor direction. The 65 mm wheels, 150 mm track and 660 x2 counts per wheel revolution are EXAMPLES: measure and correct them. Counts must correspond to this code's A-CHANGE x2 decoder, not a donor's x1 value. Verify encoder, gyro and motor signs. Confirm the arena bounds and actual origin.

After verifying configuration, set `HARDWARE_CONFIRMED=true`. It is deliberately false in the download. Enter the actual five waypoint coordinates in `POINTS_CM` and set `SOURCE_POINTS_READY=true`, or use the approved Serial WP workflow. Do not leave the example all-zero points enabled.

## 2. Prepare once, start separately

Serial Monitor: **115200 baud**, newline enabled. Before the run, physically put the robot at origin facing +X, keep it still and send:

```text
PREP
```

Wait for `CAL OK` (normally about 5.5 seconds). Preparation never starts the robot. Then send `ZERO` while it is physically at origin facing +X. If waypoints are not configured in the source, enter all five using `WP id x_cm y_cm`; `PLAN` prints the selected order.

Start using **a short BOOT press** or:

```text
START
```

Start checks calibration, configuration, sensor health and stillness, then runs the 5-second countdown. It does NOT recalibrate. Recalibrate after a reboot or when bias/temperature changes warrant it. The robot must be at origin facing +X before START; firmware cannot verify its physical location. Follow the referee's rules on setup and handling.

## 3. Normal pause versus physical handling

For a pause without relocating the robot: short BOOT press or `PAUSE`; short press or `RESUME` continues after a 3-second countdown. Ordinary pause keeps observing motion.

For referee-authorized recovery:

1. Send `HANDLE` **before picking up the robot**. This stops motors, invalidates pose and ignores carried-motion odometry without latching expected gyro-saturation/encoder-motion faults. The run deadline continues.
2. Physically place the robot at its current or a previous target, with a known heading.
3. Send `RECOVER id heading_deg`. Example: `RECOVER 2 0` means **physical source waypoint P2**, facing +X; home is ID 0. You cannot skip a future target.
4. Hands clear; wait for healthy stationary samples (at least 20 good control ticks, normally about 0.2 s), then send `RESUME`. A 3-second countdown precedes movement. Persistent sensor failure, an asserted STOP or an unknown pose still blocks resuming.

Recovery preserves the route and requires a fresh waypoint dwell. RESUME also performs the retained-route fault retry when its checks pass, so a separate RETRY command is no longer necessary. RETRY is retained for diagnostics. If there is no retained route, fix the cause, use CLEAR, then prepare again.

Button equivalent: pause first; while paused, hold >=2 seconds and release to ENTER handling. Reposition the robot. The next >=2-second hold and release resets to the selected target facing +X. After healthy settling, a short press resumes. A 0.6–2 second press cycles the current/previous eligible targets; LED pulses identify the source waypoint (home=6). Long presses act on RELEASE. Do not hold BOOT during reset/power-on.

## 4. Consecutive demonstrations

After preparation, send `ARM` once. Then send a command and wait for COMPLETE before sending the next:

```text
TURN 90
DRIVE 100
```

TURN is an absolute arena heading in degrees; ROTATE is relative. DRIVE is signed centimetres along the current heading. Successful demonstrations remain armed and preserve pose; every new motion still checks health. Commands are not queued: a command received during active motion is rejected. `STOP` disarms. Completing the five-point route also disarms. Faults and timeouts are never reported as successful completion.

## 5. Adjust PWM without re-uploading

While idle and disarmed, for example after STOP:

```text
PWM 85 75
MINPWM 28 32
STATUS
```

PWM sets the drive-controller and turn-controller limits respectively. MINPWM sets their directional friction minimums. Values must be finite and nonnegative, minimums cannot exceed their matching limits, and limits cannot exceed `Config.h`'s hard `PWM_LIMIT` (default 100/255). Steering mixes with the drive output; the final wheel pair is separately capped at that hard limit. Duty cycle is not measured speed or power. Increase settings only after physical tests. Changes are RAM-only; copy successful settings into Config.h to retain them after reboot. STOP cancels any route, so tune before the run.

`GAIND kp ki kd` and `GAINH kp ki kd` remain available under the same idle/disarmed restriction.

## 6. Measure stillness before changing thresholds

No robot recordings were provided, so V4 does NOT claim empirically tuned thresholds. The defaults remain 0.8 mm of absolute travel per wheel per 100 ms window and 2 mm across a continuous hold. Three separately sampled tick movements can exceed the window budget. Signed cancellation would also hide genuine rocking, so it is not used.

After PREP, with motors stopped and the robot on the competition surface:

```text
NOISE 1
```

Record 20–30 seconds of Serial output, then send `NOISE 0`. Lines contain:

```text
NOISE,ms,samples,abs_left_mm,abs_right_mm,peak_gyro_deg_s,bad_imu,quiet_ms
```

Absolute travel is accumulated from each control-tick increment over the report interval (normally 100 ms), not from cancelling signed counts. `samples` reveals incomplete intervals; telemetry may be dropped when TX is full. Record real stops after TURN/DRIVE as well: after COMPLETE, send STOP, then NOISE 1. Compare valid windows and consecutive 2.5-second holds. Reject samples with missing IMU data or dropped intervals. Check both a stationary robot and deliberate slow rolling/rocking before accepting any relaxed threshold. Retain the >=2-second physical stop requirement (firmware requests 2.5 seconds). Logs cannot detect wheel slip or prove absolute position. Share the recordings for evidence-based tuning.
