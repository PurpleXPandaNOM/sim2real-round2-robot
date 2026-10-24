#pragma once
// Original ESP32-WROOM DevKit, Arduino-ESP32 3.x. EXAMPLE values: measure your kit.
namespace cfg {
constexpr bool HARDWARE_CONFIRMED=false;
constexpr int SDA_PIN=21, SCL_PIN=22;
constexpr unsigned char IMU_ADDRESS=0x68;
// x2 decoding: CHANGE on A, direction from A/B equality.
constexpr int ENC_L_A=32, ENC_L_B=33, ENC_R_A=25, ENC_R_B=26;
constexpr int ENC_L_SIGN=1, ENC_R_SIGN=1, GYRO_SIGN=1;
// Choose only after checking wiring. Both profiles require two independent channels.
// 0: GPIO-controlled enables. 1: Yashas PWM map with enables physically tied HIGH.
constexpr int WIRING_PROFILE=0;
static_assert(WIRING_PROFILE==0 || WIRING_PROFILE==1,"Select wiring profile 0 or 1");
constexpr int L_RPWM=WIRING_PROFILE==0?16:18, L_LPWM=WIRING_PROFILE==0?17:19;
constexpr int R_RPWM=WIRING_PROFILE==0?18:23, R_LPWM=WIRING_PROFILE==0?19:4;
constexpr int L_EN=WIRING_PROFILE==0?27:-1, R_EN=WIRING_PROFILE==0?23:-1;
constexpr int MOTOR_L_SIGN=1, MOTOR_R_SIGN=1;
constexpr int STOP_PIN=WIRING_PROFILE==0?4:-1; // Optional normally-open button to ground; input pulled up.
constexpr int CONTROL_PIN=0; // DevKit BOOT button. Never hold during power-on/reset.
constexpr int LED_PIN=2; // -1 disables; confirm LED polarity on actual board.
constexpr float WHEEL_L_DIAMETER_M=0.065f, WHEEL_R_DIAMETER_M=0.065f;
constexpr float TRACK_M=0.150f;
constexpr float TICKS_L_PER_WHEEL_REV=660, TICKS_R_PER_WHEEL_REV=660;
constexpr float GYRO_WEIGHT=0.90f;
constexpr int PWM_HZ=18000, PWM_BITS=8, PWM_LIMIT=100;
// Direct controller outputs are PWM, not m/s. Tune on the floor.
constexpr float DIST_KP=350, DIST_KI=0, DIST_KD=80; // error metres, rate m/s
constexpr float HEADING_KP=55, HEADING_KI=0, HEADING_KD=10; // radians, rad/s
constexpr float DRIVE_PWM_LIMIT=75, TURN_PWM_LIMIT=70, STEER_PWM_LIMIT=35;
constexpr float MIN_DRIVE_PWM=28, MIN_TURN_PWM=32, PWM_SLEW_PER_SEC=300;
constexpr float MOTOR_L_TRIM=1.0f, MOTOR_R_TRIM=1.0f;
// Enabled only after verifying the actual driver truth table and stopping current.
// For two BTS7960 half bridges per wheel: both PWM=0, both EN=HIGH -> low-side brake.
constexpr bool BRAKE_WHEN_SETTLING=false;
constexpr float POS_TOL_M=0.018f, ANG_TOL_RAD=0.025f;
// Aim at centre; accept a close lateral/stalled approach using RADIAL distance.
// Home stays tighter. Dwell exit MUST exceed every accepted arrival radius.
constexpr float ROUTE_ACCEPT_M=0.050f, HOME_ACCEPT_M=0.025f, DWELL_MARGIN_M=0.015f;
constexpr float LOOKAHEAD_M=0.15f;
constexpr unsigned MAX_REALIGNS_PER_LEG=3;
// 100ms motion windows tolerate tick quantisation; total budgets reject creeping.
constexpr float STILL_WINDOW_TRAVEL_M=0.0008f, STILL_TOTAL_TRAVEL_M=0.002f;
constexpr float STILL_TOTAL_YAW_RAD=0.035f;
constexpr unsigned STILL_WINDOW_MS=100, STILL_GAP_MS=40;
constexpr unsigned CAL_SETTLE_MS=1500, CAL_SAMPLE_MS=4000, CAL_TIMEOUT_MS=20000;
constexpr unsigned CAL_MIN_SAMPLES=350;
constexpr float CAL_MAX_TRAVEL_M=0.0005f, CAL_MAX_SD_RADPS=0.015f;
constexpr float STILL_WHEEL_MPS=0.008f, STILL_GYRO_RADPS=0.045f;
constexpr unsigned DWELL_MS=2500, SETTLE_MS=400, TASK_TIMEOUT_MS=90000;
constexpr unsigned RUN_LIMIT_MS=600000, CONTROL_US=10000, START_MS=5000, RESUME_MS=3000;
constexpr unsigned IMU_GRACE_MS=150, HEALTHY_TICKS_TO_RETRY=20;
constexpr unsigned STALL_MS=2000; constexpr int STALL_PWM=40;
// Adjust for actual origin location and chassis footprint inside 240 x 240 cm arena.
constexpr float X_MIN=-2.4f, X_MAX=2.4f, Y_MIN=-2.4f, Y_MAX=2.4f;
// Rulebook says OPEN path, followed by return. Both modes always return home.
constexpr bool OPTIMIZE_RETURN=false;
// OPTIONAL experiment in metres per radian; zero preserves shortest distance.
constexpr float TURN_PENALTY_M_PER_RAD=0.0f;
constexpr bool SOURCE_POINTS_READY=false;
constexpr float POINTS_CM[5][2]={{0,0},{0,0},{0,0},{0,0},{0,0}};
}
