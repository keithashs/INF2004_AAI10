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
