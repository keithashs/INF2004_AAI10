#include "ultrasonic.h"
#include <stdio.h>

// ============== Timing Constants ==============
#define TRIG_PULSE_US       10      // 10µs trigger pulse
#define SOUND_SPEED_CM_US   0.0343  // Speed of sound: ~343 m/s = 0.0343 cm/µs

// ============== Measurement State ==============
static float last_distance_cm = ULTRASONIC_INVALID_READING;

/**
 * Initialize ultrasonic sensor pins
 */
void ultrasonic_init(void) {
    // Initialize trigger pin as output
    gpio_init(ULTRASONIC_TRIG_PIN);
    gpio_set_dir(ULTRASONIC_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);
    
    // Initialize echo pin as input
    gpio_init(ULTRASONIC_ECHO_PIN);
    gpio_set_dir(ULTRASONIC_ECHO_PIN, GPIO_IN);
    
    printf("[ULTRASONIC] Init OK - TRIG=GPIO%d, ECHO=GPIO%d\n", 
           ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
}

/**
 * Measure distance in cm using single trigger
 * Returns -1.0f on timeout or out of range
 */
float ultrasonic_get_distance_cm(void) {
    // Send 10µs trigger pulse
    gpio_put(ULTRASONIC_TRIG_PIN, 1);
    sleep_us(TRIG_PULSE_US);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);
    
    // Wait for echo pulse to start (LOW → HIGH transition)
    uint32_t timeout = 500000;  // ~500ms timeout
    while (gpio_get(ULTRASONIC_ECHO_PIN) == 0 && timeout--) {
        sleep_us(1);
    }
    
    if (timeout == 0) {
        last_distance_cm = ULTRASONIC_INVALID_READING;
        return ULTRASONIC_INVALID_READING;
    }
    
    // Measure echo pulse duration
    uint32_t pulse_start = time_us_32();
    timeout = ULTRASONIC_TIMEOUT_US;
    
    while (gpio_get(ULTRASONIC_ECHO_PIN) == 1 && timeout--) {
        sleep_us(1);
    }
    
    uint32_t pulse_end = time_us_32();
    
    if (timeout == 0) {
        last_distance_cm = ULTRASONIC_INVALID_READING;
        return ULTRASONIC_INVALID_READING;
    }
    
    // Calculate distance
    // Echo pulse duration represents round trip
    // distance = (pulse_duration / 2) * sound_speed
    uint32_t pulse_duration_us = pulse_end - pulse_start;
    float distance_cm = (pulse_duration_us / 2.0f) * SOUND_SPEED_CM_US;
    
    // Validate range
    if (distance_cm < ULTRASONIC_MIN_DISTANCE_CM || distance_cm > ULTRASONIC_MAX_DISTANCE_CM) {
        last_distance_cm = ULTRASONIC_INVALID_READING;
        return ULTRASONIC_INVALID_READING;
    }
    
    last_distance_cm = distance_cm;
    return distance_cm;
}

/**
 * Get last measurement
 */
float ultrasonic_get_last_distance_cm(void) {
    return last_distance_cm;
}

/**
 * Check if obstacle within threshold
 */
bool ultrasonic_obstacle_detected(float threshold_cm) {
    float distance = ultrasonic_get_distance_cm();
    return (distance > 0.0f && distance <= threshold_cm);
}

/**
 * Non-blocking update (for future async implementation)
 */
bool ultrasonic_update(void) {
    float distance = ultrasonic_get_distance_cm();
    return (distance > 0.0f);
}
