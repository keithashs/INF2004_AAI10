#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// ---------------- Pin Map ----------------
#define L_MOTOR_IN1 11
#define L_MOTOR_IN2 10
#define R_MOTOR_IN3 8
#define R_MOTOR_IN4 9
// #define MOTOR_STBY 22  // optional if your driver has STBY

// ---------------- PWM/Speed constants ----------------
#define PWM_MIN_LEFT   80
#define PWM_MIN_RIGHT  80
#define PWM_MID_LEFT   160
#define PWM_MID_RIGHT  160
#define PWM_MAX_LEFT   255
#define PWM_MAX_RIGHT  255
#define PWM_JUMPSTART  120

#define MIN_SPEED                 2.0f
#define MAX_SPEED                 5.0f
#define TURN_SPEED                5.0f
#define JUMPSTART_SPEED_THRESHOLD 0.1f

#define FULL_CIRCLE    360.0f
#define CONTINUOUS_TURN -1.0f

typedef enum { PID_STOP, PID_FWD, PID_REV, PID_LEFT, PID_RIGHT, PID_TURNING, PID_DISABLED } PIDState;

// Init
void motor_init(void);
void motor_pwm_init(void);

// Manual drive
void forward_motor(float pwm_left, float pwm_right);
void reverse_motor(float pwm_left, float pwm_right);
void turn_motor(int direction /*LEFT=0,RIGHT=1*/, float pwm_left, float pwm_right);
void stop_motor(void);

// Friendly wrappers
void disable_pid_control(void);
void forward_motor_manual(float pwm_left, float pwm_right);
void reverse_motor_manual(float pwm_left, float pwm_right);
void turn_motor_manual(int direction, float angle, float pwm_left, float pwm_right);
void stop_motor_manual(void);
void offset_move_motor(int direction /*FORWARDS/BACKWARDS*/, int turn /*LEFT/RIGHT*/, float offset);

// PID interface (used only if you run the PID task)
void enable_pid_control(void);
void forward_motor_pid(float target_speed);
void reverse_motor_pid(float target_speed);
void turn_motor_pid(int direction, float target_speed, float target_turn_angle);
void stop_motor_pid(void);
bool turn_until_angle(float angle);

// PID task (optional)
void pid_task(void *params);
float compute_pid_pwm(float target, float current, float *integral, float *prev_error);

// Optional conditioning
void motor_conditioning(void);

#endif // MOTOR_H
