#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// ============== Pin Configuration ==============
// Adjust these to match your wiring
#define ULTRASONIC_TRIG_PIN  4    // Trigger (GPIO output)
#define ULTRASONIC_ECHO_PIN  5    // Echo (GPIO input)

// ============== Timing & Limits ==============
#define ULTRASONIC_MAX_DISTANCE_CM  400.0f
#define ULTRASONIC_MIN_DISTANCE_CM  2.0f
#define ULTRASONIC_TIMEOUT_US       30000    // ~5m max range
#define ULTRASONIC_INVALID_READING   -1.0f

// ============== Function Prototypes ==============

/**
 * Initialize ultrasonic sensor (GPIO pins, interrupts).
 * Call once at startup.
 */
void ultrasonic_init(void);

/**
 * Trigger one measurement and retrieve distance in cm.
 * Returns -1.0f if timeout or out-of-range.
 * Blocking call (~30ms worst case).
 */
float ultrasonic_get_distance_cm(void);

/**
 * Non-blocking version: call this periodically (e.g., every 10ms).
 * Returns true if a new reading is ready, false otherwise.
 * Call get_distance_cm() to retrieve it.
 */
bool ultrasonic_update(void);

/**
 * Get last valid distance measurement (cm).
 * Returns ULTRASONIC_INVALID_READING if no valid measurement yet.
 */
float ultrasonic_get_last_distance_cm(void);

/**
 * Check if an obstacle is within a threshold distance.
 * Useful for quick "obstacle ahead?" check.
 */
bool ultrasonic_obstacle_detected(float threshold_cm);

#endif // ULTRASONIC_H
