#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "pico/time.h"

// Movement API
typedef enum {
    MOVE_STOP = 0,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT
} move_t;

void motor_init(void);
void encoder_init(void);

void motor_set_signed(int in1, int in2, int dir, float duty_percent);
void motor_all_stop(void);

// Speed control targets (CPS setpoints) and direction
void motion_command(move_t move, int speed_percent);
void motion_command_with_bias(move_t move, int speed_percent, float left_bias_cps, float right_bias_cps);

// Control loop timer callback (10ms)
bool motor_control_timer_cb(repeating_timer_t *t);

// Telemetry helpers
void get_cps(float* cps_m1, float* cps_m2);
void get_cps_smoothed(float* out_m1, float* out_m2);  // <-- ADDED: declare the smoothed CPS helper
void get_distance_m(float* d_m1, float* d_m2);

// Expose dirs for prints
int get_dir_m1(void);
int get_dir_m2(void);
