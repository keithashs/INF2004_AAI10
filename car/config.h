#pragma once
#include <stdbool.h>

// ===== Pin map (Cytron Robo Pico, 2-pin drive per motor) =====
// Right motor (M1)
#define M1_IN1          8    // GP8
#define M1_IN2          9    // GP9
#define ENCODER_PIN_M1  16   // GP16 : IR encoder for right motor

// Left motor (M2)
#define M2_IN1          11   // GP11
#define M2_IN2          10   // GP10
#define ENCODER_PIN_M2  6    // GP6  : IR encoder for left motor

//IR Line Sensor (Left IR Sensor) no analog input
#define IR_LEFT_DIGITAL_PIN    7    // GP7 is digital input (sensor D0)
#define IR_LEFT_ADC_PIN       28    // GP28 is ADC0 (sensor A0)
//IR Barcode Sensor (Right IR Sensor)
#define IR_RIGHT_DIGITAL_PIN   27    // GP27 is digital input (sensor D0)
#define IR_RIGHT_ADC_PIN       26    // GP26 is ADC0 (sensor A0)

// Buttons
#define BTN_START       21   // GP21 (START)
#define BTN_STOP        20   // GP20 (STOP)

// I2C port/pins/speed
#define I2C_PORT  i2c1
#define I2C_SDA   2
#define I2C_SCL   3
#define I2C_BAUD  400000

// LSM303DLHC (GY-511) addresses
#define ACC_ADDR  0x19  // accelerometer
#define MAG_ADDR  0x1E  // magnetometer

// LSM303DLHC accelerometer registers (subset)
#define ACC_REG_STATUS_A  0x27
#define ACC_REG_OUT_X_L   0x28
#define ACC_REG_CTRL1_A   0x20
#define ACC_REG_CTRL4_A   0x23
#define ACC_REG_WHOAMI_A  0x0F

// Control / loop timing
#define SYS_CLK_HZ        125000000u
#define PWM_FREQ_HZ       20000u
#define PWM_WRAP          999
#define CONTROL_PERIOD_MS 10     // 100 Hz control loop
#define TELEMETRY_MS      200    // 5 Hz prints
#define IMU_PERIOD_MS     10     // 100 Hz IMU polling

// Encoder parameters (adjust wheel/encoder)
#define TICKS_PER_REV     20.0f   // 20 rising edges per wheel rev (20 slots)
#define WHEEL_DIAMETER_M  0.0635f // 6.35 cm drive wheel (measured)
#define PI_F              3.1415926f
// Wheel Scale Calibration Defaults
#define DEFAULT_SCALE_RIGHT  0.945f
#define DEFAULT_SCALE_LEFT   1.058f

// Speed scaling: 0..100% -> counts per second target
// M1(R) peak ≈ 54 cps (≈21.21 cm/s) M2(L) peak ≈ 50 cps (≈19.63 cm/s) - on air
// M1(R) peak ≈ 47 cps (≈18.46 cm/s) M2(L) peak ≈ 45 cps (≈17.67 cm/s) - on ground
// set to 45 cps on each wheel (≈17.67 cm/s with D=2.5 cm, TPR=20).
#define MAX_CPS           45.0f

// Heading filter (0..1, higher = smoother)
#define HEADING_ALPHA     0.93f   // a touch smoother than before

// --- Defaults for magnetometer calibration (overwritten at runtime) ---
#define MAG_OFF_X   0.0f
#define MAG_OFF_Y   0.0f
#define MAG_OFF_Z   0.0f
#define MAG_SCL_X   1.0f
#define MAG_SCL_Y   1.0f
#define MAG_SCL_Z   1.0f

// ====== Adaptive wheel-scale learning ======
#define SCALE_MIN          0.85f   // lower bound for per-wheel scale
#define SCALE_MAX          1.15f   // upper bound
#define ADAPT_PERIOD_S     0.50f   // update scales every ~0.5 s
// How aggressively to change scale based on persistent encoder diff.
// Effective step ~= ADAPT_GAIN * (diff/base_cps) clamped small each update.
#define ADAPT_GAIN         0.15f   // start small; increase if learning is too slow

// ====== Demo1 Configuration ======
#define DEMO1_RUN_SPEED_PERCENT   20
#define DEMO1_SOFTSTART_SEC       0.6f
#define DEMO1_SETTLE_TIME_MS      4000   // 4s settle before capturing heading

// Calibration timing
#define CAL_MAG_SPIN_DURATION_MS  3000   // 3s magnetometer calibration spin
#define CAL_WHEEL_DRIVE_DURATION_MS 4000 // 4s wheel scale calibration drive
#define CAL_WHEEL_SETTLE_MS       500    // 500ms settle before measurement

// Control loop bias limits (as fraction of base speed)
#define BIAS_TRACK_FRACTION  0.55f   // track PID can use 55% of base speed
#define BIAS_HEAD_FRACTION   0.35f   // heading PID can use 35% of base speed
#define BIAS_TOTAL_FRACTION  0.60f   // combined bias max 60% of base speed
#define BIAS_MIN_CPS         5.0f    // minimum absolute bias limit (cps)
#define BIAS_MIN_HEAD_CPS    3.0f    // minimum heading bias limit (cps)
#define BIAS_MIN_TOTAL_CPS   6.0f    // minimum total bias limit (cps)

// Button debounce
#define BTN_DEBOUNCE_MS      100