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

// Encoder parameters (adjust to your wheel/encoder)
#define TICKS_PER_REV     20.0f   // encoder ticks per wheel revolution
#define WHEEL_DIAMETER_M  0.025f  // 2.5 cm wheel diameter
#define PI_F              3.1415926f

// Speed scaling: 0..100% -> counts per second target
#define MAX_CPS           100.0f  // tune this to your real peak CPS

// Heading filter (0..1, higher = smoother)
#define HEADING_ALPHA     0.90f

// Magnetometer rough offsets/scale (tune later)
#define MAG_OFF_X   0.0f
#define MAG_OFF_Y   0.0f
#define MAG_OFF_Z   0.0f
#define MAG_SCL_X   1.0f
#define MAG_SCL_Y   1.0f
#define MAG_SCL_Z   1.0f
