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
}
