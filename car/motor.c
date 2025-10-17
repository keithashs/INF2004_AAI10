<<<<<<< Updated upstream
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <math.h>
#include "pin.h"

// Helper to setup a PWM pin at ~20kHz
static void setup_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    uint chan  = pwm_gpio_to_channel(pin);
    // 125 MHz / (clkdiv * wrap+1) -> target ~20 kHz
    uint32_t wrap = 6249;    // with clkdiv=1 => 20kHz
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.0f);
    pwm_config_set_wrap(&cfg, wrap);
    pwm_init(slice, &cfg, true);
    pwm_set_chan_level(slice, chan, 0); // start low
}

static inline uint16_t duty_from_float(float x) {
    if (x > 1.0f) x = 1.0f; if (x < 0.0f) x = 0.0f;
    return (uint16_t)lrintf(x * 6249.0f);
}

void motor_init(void) {
    // Both pins per motor are PWM-capable on RP2040; we’ll PWM the “active” pin.
    setup_pwm_pin(M1_IN1);
    setup_pwm_pin(M1_IN2);
    setup_pwm_pin(M2_IN1);
    setup_pwm_pin(M2_IN2);
}

static void set_motor_2pin(uint pin_fwd, uint pin_rev, float v) {
    // Deadband to avoid shoot-through
    const float dead = 0.02f;
    uint slice_f = pwm_gpio_to_slice_num(pin_fwd);
    uint chan_f  = pwm_gpio_to_channel(pin_fwd);
    uint slice_r = pwm_gpio_to_slice_num(pin_rev);
    uint chan_r  = pwm_gpio_to_channel(pin_rev);

    if (v > dead) {
        uint16_t d = duty_from_float(v);
        pwm_set_chan_level(slice_f, chan_f, d);
        pwm_set_chan_level(slice_r, chan_r, 0);
    } else if (v < -dead) {
        uint16_t d = duty_from_float(-v);
        pwm_set_chan_level(slice_f, chan_f, 0);
        pwm_set_chan_level(slice_r, chan_r, d);
    } else {
        // brake (both low)
        pwm_set_chan_level(slice_f, chan_f, 0);
        pwm_set_chan_level(slice_r, chan_r, 0);
    }
}

void motor_set_left(float speed)  { set_motor_2pin(M2_IN1, M2_IN2, speed); }
void motor_set_right(float speed) { set_motor_2pin(M1_IN1, M1_IN2, speed); }

void motor_brake(void) {
    motor_set_left(0.0f);
    motor_set_right(0.0f);
=======
#include "motor.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <math.h>

// Compute and program slice config for a pin
static inline void setup_pwm_pin(uint pin, uint16_t top, float clkdiv) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, top);
    pwm_set_clkdiv(slice, clkdiv);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), 0);
    pwm_set_enabled(slice, true);
}

void motor_ab_init(Motor *m, uint pinA, uint pinB, float pwm_hz) {
    m->pinA  = pinA;
    m->pinB  = pinB;
    m->sliceA = pwm_gpio_to_slice_num(pinA);
    m->sliceB = pwm_gpio_to_slice_num(pinB);

    // Aim for 20 kHz default: TOP=6249, CLKDIV=1 -> 125e6 / (1*(6249+1)) = 20 kHz
    // For arbitrary freq: choose TOP=6249, compute divider accordingly.
    m->top = 6249;
    const float sys = 125000000.0f;
    float clkdiv = sys / (pwm_hz * (m->top + 1));
    if (clkdiv < 1.0f)   clkdiv = 1.0f;
    if (clkdiv > 255.0f) clkdiv = 255.0f;

    setup_pwm_pin(pinA, m->top, clkdiv);
    setup_pwm_pin(pinB, m->top, clkdiv);
}

static inline void set_level_float(uint pin, uint16_t top, float x) {
    // clamp 0..1, then scale to [0..TOP]
    if (x <= 0.f) {
        pwm_set_chan_level(pwm_gpio_to_slice_num(pin),
                           pwm_gpio_to_channel(pin), 0);
        return;
    }
    if (x >= 1.f) {
        pwm_set_chan_level(pwm_gpio_to_slice_num(pin),
                           pwm_gpio_to_channel(pin), top);
        return;
    }
    uint32_t lvl = (uint32_t)lroundf(x * top);
    pwm_set_chan_level(pwm_gpio_to_slice_num(pin),
                       pwm_gpio_to_channel(pin), lvl);
}

void motor_ab_brake(Motor *m) {
    // both Low
    set_level_float(m->pinA, m->top, 0.f);
    set_level_float(m->pinB, m->top, 0.f);
}

void motor_ab_coast(Motor *m) {
    // both High
    set_level_float(m->pinA, m->top, 1.f);
    set_level_float(m->pinB, m->top, 1.f);
}

void motor_ab_drive(Motor *m, float signed_duty) {
    if (signed_duty >  1.0f) signed_duty =  1.0f;
    if (signed_duty < -1.0f) signed_duty = -1.0f;

    int dir   = (signed_duty > 0) - (signed_duty < 0); // +1,0,-1
    float mag = (signed_duty >= 0) ? signed_duty : -signed_duty;

    if (dir > 0) {          // forward: A=PWM, B=Low
        set_level_float(m->pinA, m->top, mag);
        set_level_float(m->pinB, m->top, 0.f);
    } else if (dir < 0) {   // reverse: A=Low,  B=PWM
        set_level_float(m->pinA, m->top, 0.f);
        set_level_float(m->pinB, m->top, mag);
    } else {                // brake
        motor_ab_brake(m);
    }
}

void robot_movement(Motor *left, Motor *right, float sL, float sR) {
    motor_ab_drive(left,  sL);
    motor_ab_drive(right, sR);
>>>>>>> Stashed changes
}
