#pragma once
#include "pico/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// 2-pin (A/B) motor driven by hardware PWM on BOTH pins.
// Sign-magnitude: +duty => A=PWM(mag), B=Low; -duty => A=Low, B=PWM(mag); 0 => brake (both Low)

typedef struct {
    uint     pinA;     // PWM-capable GPIO: MxA
    uint     pinB;     // PWM-capable GPIO: MxB
    uint     sliceA;   // PWM slice for A
    uint     sliceB;   // PWM slice for B
    uint16_t top;      // PWM wrap (TOP)
} Motor;

// Initialize a 2-pin motor with desired PWM frequency (e.g., 20000 for 20 kHz)
void motor_ab_init(Motor *m, uint pinA, uint pinB, float pwm_hz);

// Drive with signed duty in [-1..+1], sign=direction, |duty|=speed
void motor_ab_drive(Motor *m, float signed_duty);

// Stop styles
void motor_ab_brake(Motor *m); // both Low
void motor_ab_coast(Motor *m); // both High

// Convenience for differential pair (LEFT, RIGHT)
void robot_movement(Motor *left, Motor *right, float sL, float sR);

#ifdef __cplusplus
} // extern "C"
#endif
