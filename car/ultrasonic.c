#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pin.h"

void ultrasonic_init(void) {
    gpio_init(US_TRIG); gpio_set_dir(US_TRIG, GPIO_OUT); gpio_put(US_TRIG, 0);
    gpio_init(US_ECHO); gpio_set_dir(US_ECHO, GPIO_IN);  // assume external pull-down or floating
}

uint32_t ultrasonic_read_cm(void) {
    // Trigger: 10us pulse
    gpio_put(US_TRIG, 1); sleep_us(10); gpio_put(US_TRIG, 0);

    // Wait for echo rising (timeout ~25ms)
    absolute_time_t to = make_timeout_time_us(25000);
    while (!gpio_get(US_ECHO)) { if (absolute_time_diff_us(get_absolute_time(), to) <= 0) return 0; }

    // Measure high pulse width (timeout ~25ms)
    absolute_time_t start = get_absolute_time();
    while (gpio_get(US_ECHO)) { if (absolute_time_diff_us(get_absolute_time(), to) <= 0) return 0; }
    absolute_time_t end = get_absolute_time();

    uint32_t us = (uint32_t)absolute_time_diff_us(start, end);
    // HC-SR04: distance (cm) = us / 58
    return us / 58U;
}
