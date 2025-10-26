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

// Per-wheel target scaling to cancel systematic drift
void motor_set_wheel_scale(float scale_right, float scale_left);
void motor_get_wheel_scale(float* scale_right, float* scale_left);

// Control loop timer callback (10ms)
bool motor_control_timer_cb(repeating_timer_t *t);

// Telemetry helpers
void get_cps(float* cps_m1, float* cps_m2);
void get_cps_smoothed(float* out_m1, float* out_m2);
void get_distance_m(float* d_m1, float* d_m2);

// Expose dirs for prints
int get_dir_m1(void);
int get_dir_m2(void);

// Clear the 100 ms moving average, IIR filters, and motor PIDs (for clean START)
void motor_reset_speed_filters(void);
void motor_reset_controllers(void);

// Reset encoder distance counters to 0 (so Dist[L/R] prints start at 0.0cm after START)
void motor_reset_distance_counters(void);

// Telemetry print modes
typedef enum {
    TMODE_NONE = 0,  // print nothing (e.g., waiting / settle / soft-start)
    TMODE_CAL  = 1,  // print "CAL ..." lines (no distance; IMU only if available)
    TMODE_RUN  = 2   // print "STAT ..." lines (with distance)
} telemetry_mode_t;

// Set the current telemetry mode
void telemetry_set_mode(telemetry_mode_t mode);

// Helper to print the legend (unchanged)
void print_telemetry_legend(void);

// ===== Additional helpers for demo2 (speed in cm/s, distance in cm) =====

// Get wheel speeds in cm/s (converted from CPS)
void get_speed_cmps(float* left_cmps, float* right_cmps);

// Get distances in cm (converted from meters)
void get_distance_cm(float* left_cm, float* right_cm);

// Average speed in cm/s
float get_average_speed_cmps(void);

// Average distance in cm
float get_average_distance_cm(void);
