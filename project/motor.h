#pragma once
#include "pico/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint     pwm_pin;   // EN (PWM)
    uint     in1;       // IN1 / IN3
    uint     in2;       // IN2 / IN4
    uint     slice;     // PWM slice number
    uint16_t top;       // PWM wrap value
} Motor;

// Basic lifecycle
void motor_init   (Motor *m, uint pwm_pin, uint in1, uint in2, float pwm_hz);

// Simple commands
void motor_forward(Motor *m, float duty);  // duty: 0..1
void motor_stop   (Motor *m);
void motor_brake  (Motor *m);
void motor_set_duty(Motor *m, float duty); // duty: 0..1

// signed drive (-1..+1) using DIR pins internally
void motor_drive(Motor *m, float signed_duty);

// convenience for a differential pair (LEFT, RIGHT)
void robot_movement(Motor *left, Motor *right, float sL, float sR);

#ifdef __cplusplus
} // extern "C"
#endif
