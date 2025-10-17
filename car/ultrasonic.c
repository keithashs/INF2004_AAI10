#include "ultrasonic.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

static const uint32_t timeout_us = 26100;   // Timeout for ultrasonic sensor (about 4.5 meters)

static uint32_t getPulse(uint trigPin, uint echoPin) {
    // 10 µs trigger pulse
    gpio_put(trigPin, 0); sleep_us(2);
    gpio_put(trigPin, 1); sleep_us(10);
    gpio_put(trigPin, 0);

    // wait for echo high (timeout)
    absolute_time_t t0 = get_absolute_time();
    while (gpio_get(echoPin) == 0) {
        if (absolute_time_diff_us(t0, get_absolute_time()) > timeout_us) return 0;
        tight_loop_contents();
    }

    // measure high pulse width (timeout)
    absolute_time_t start = get_absolute_time();
    while (gpio_get(echoPin) == 1) {
        if (absolute_time_diff_us(start, get_absolute_time()) > timeout_us) return 0;
        tight_loop_contents();
    }
    absolute_time_t end = get_absolute_time();
    return (uint32_t)absolute_time_diff_us(start, end);
}

void setupUltrasonicPins(uint trigPin, uint echoPin) {
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
    gpio_put(trigPin, 0);
}

uint32_t ultrasonic_get_cm(uint trigPin, uint echoPin) {
    // Integer math: cm ≈ us / 29 / 2
    uint32_t us = getPulse(trigPin, echoPin);
    return us ? (us / 29u / 2u) : 0u;  // integer math
}

uint32_t getInch(uint trigPin, uint echoPin) {
    // Integer math: inch ≈ us / 74 / 2
    uint32_t us = getPulse(trigPin, echoPin);
    return us ? (us / 74u / 2u) : 0u;  // integer math
}
