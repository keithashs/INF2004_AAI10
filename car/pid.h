#ifndef PID_H
#define PID_H
typedef struct {
    float kp, ki, kd;
    float i, prev, out_min, out_max;
} pid_t;

static inline void pid_init(pid_t* p, float kp, float ki, float kd, float out_min, float out_max) {
    p->kp = kp; p->ki = ki; p->kd = kd; p->i = 0; p->prev = 0; p->out_min = out_min; p->out_max = out_max;
}

static inline float pid_step(pid_t* p, float err, float dt) {
    p->i += err * p->ki * dt;
    if (p->i > p->out_max) p->i = p->out_max;
    if (p->i < p->out_min) p->i = p->out_min;
    float d = (err - p->prev) / (dt > 1e-6f ? dt : 1e-6f);
    p->prev = err;
    float out = p->kp * err + p->i + p->kd * d;
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;
    return out;
}
#endif
