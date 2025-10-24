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

// Library step function (generic PID)
float pid_step(PID* pid, float setpoint, float measured, float dt_s);

// === App-level PIDs (defined in pid.c) ===
extern PID pid_track;    // wheel-balance (enc R-L) PID
extern PID pid_heading;  // heading trim PID

// Initialize app-level PIDs with sane defaults
void pid_init_defaults(void);

// Optional helpers to tweak gains at runtime
void pid_set_gains_track(float kp, float ki, float kd);
void pid_set_gains_heading(float kp, float ki, float kd);
