#include "pid.h"

float pid_step(PID* pid, float setpoint, float measured, float dt_s) {
    float err = setpoint - measured;
    pid->integ += err * pid->ki * dt_s;

    if (pid->integ > pid->out_max) pid->integ = pid->out_max;
    if (pid->integ < pid->out_min) pid->integ = pid->out_min;

    float deriv = (err - pid->prev_err) / dt_s;
    float out = (pid->kp * err) + pid->integ + (pid->kd * deriv);

    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;

    pid->prev_err = err;
    return out;
}

// === App-level PID instances (owned here so main.c stays clean) ===
// 1) Wheel-balance (encoder-difference) PID: keeps cps_R ~ cps_L (fast)
PID pid_track = {
    .kp = 0.90f, .ki = 1.80f, .kd = 0.00f,
    .integ = 0, .prev_err = 0,
    .out_min = -100.0f, .out_max = +100.0f
};

// 2) IMU heading PID (slow trim)
PID pid_heading = {
    .kp = 0.35f, .ki = 0.04f, .kd = 0.00f,
    .integ = 0, .prev_err = 0,
    .out_min = -100.0f, .out_max = +100.0f
};

void pid_init_defaults(void) {
    pid_reset(&pid_track);
    pid_reset(&pid_heading);
    // (defaults already set in the initializers above)
}

static inline void set_gains(PID* p, float kp, float ki, float kd) {
    p->kp = kp; p->ki = ki; p->kd = kd;
}

void pid_set_gains_track(float kp, float ki, float kd)   { set_gains(&pid_track,   kp, ki, kd); }
void pid_set_gains_heading(float kp, float ki, float kd) { set_gains(&pid_heading, kp, ki, kd); }
