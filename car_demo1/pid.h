#pragma once
typedef struct {
    float kp, ki, kd;
    float integ;
    float prev_err;
    float out_min, out_max;
} PID;

static inline void pid_reset(PID* p) {
    p->integ = 0.0f; p->prev_err = 0.0f;
}

float pid_step(PID* pid, float setpoint, float measured, float dt_s);
