#include "motor.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// ---------- internals ----------
static void motor_gpio_init(Motor *m) {
    gpio_init(m->in1); gpio_set_dir(m->in1, GPIO_OUT); gpio_put(m->in1, 0);
    gpio_init(m->in2); gpio_set_dir(m->in2, GPIO_OUT); gpio_put(m->in2, 0);
}

static void motor_pwm_init(Motor *m, float freq_hz) {
    gpio_set_function(m->pwm_pin, GPIO_FUNC_PWM);
    m->slice = pwm_gpio_to_slice_num(m->pwm_pin);

    // 16-bit wrap
    m->top = 65535;
    const float sysclk = 125000000.0f; // Pico default
    float divider = sysclk / (freq_hz * (m->top + 1));
    if (divider < 1.0f)   divider = 1.0f;
    if (divider > 255.0f) divider = 255.0f;

    pwm_set_clkdiv(m->slice, divider);
    pwm_set_wrap(m->slice, m->top);
    pwm_set_gpio_level(m->pwm_pin, 0);
    pwm_set_enabled(m->slice, true);
}

static inline void motor_set_dir_raw(Motor *m, int dir) {
    if (dir > 0) {            // forward: IN1=1, IN2=0
        gpio_put(m->in1, 1);
        gpio_put(m->in2, 0);
    } else if (dir < 0) {     // reverse: IN1=0, IN2=1
        gpio_put(m->in1, 0);
        gpio_put(m->in2, 1);
    } else {                  // coast: IN1=0, IN2=0
        gpio_put(m->in1, 0);
        gpio_put(m->in2, 0);
    }
}

// ---------- public API ----------
void motor_init(Motor *m, uint pwm_pin, uint in1, uint in2, float pwm_hz) {
    m->pwm_pin = pwm_pin;
    m->in1 = in1;
    m->in2 = in2;
    motor_gpio_init(m);
    motor_pwm_init(m, pwm_hz);
}

void motor_set_duty(Motor *m, float duty) {
    if (duty < 0) duty = 0;
    if (duty > 1) duty = 1;
    pwm_set_gpio_level(m->pwm_pin, (uint16_t)((m->top + 1) * duty));
}

void motor_forward(Motor *m, float duty) {
    // avoid shoot-through on direction change
    motor_set_duty(m, 0.0f);
    motor_set_dir_raw(m, +1);
    motor_set_duty(m, duty);
}

void motor_stop(Motor *m) {
    motor_set_duty(m, 0.0f);
    motor_set_dir_raw(m, 0);
}

void motor_brake(Motor *m) {
    // active brake: both legs high on typical H-bridge
    gpio_put(m->in1, 1);
    gpio_put(m->in2, 1);
    motor_set_duty(m, 0.0f);
}

// signed drive (-1..+1), mirrors “Robot_movement” slide semantics
void motor_drive(Motor *m, float signed_duty) {
    // clamp and split sign
    if (signed_duty >  1.0f) signed_duty =  1.0f;
    if (signed_duty < -1.0f) signed_duty = -1.0f;

    int   dir = (signed_duty > 0) - (signed_duty < 0);   // +1, 0, -1
    float mag = signed_duty >= 0 ? signed_duty : -signed_duty;

    // safe direction change
    motor_set_duty(m, 0.0f);
    motor_set_dir_raw(m, dir);
    motor_set_duty(m, mag);
}


// pair helper — this is your “Robot_movement()”
void robot_movement(Motor *left, Motor *right, float sL, float sR) {
    // Notes from slide integrated:
    // - float is signed
    // - level maps 0..1 to full 16-bit PWM via motor_set_duty()
    // - negative values mean reverse
    motor_drive(left,  sL);
    motor_drive(right, sR);
}
