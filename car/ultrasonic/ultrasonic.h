#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// ============== Pin Configuration ==============
#define ULTRASONIC_TRIG_PIN  4
#define ULTRASONIC_ECHO_PIN  5

// ============== Timing & Limits ==============
#define ULTRASONIC_MAX_DISTANCE_CM  400.0f
#define ULTRASONIC_MIN_DISTANCE_CM  2.0f
#define ULTRASONIC_TIMEOUT_US       30000
#define ULTRASONIC_INVALID_READING   -1.0f

// ============== Width Scan Parameters (from testDemo3.c) ==============
#define MEASUREMENT_DELAY_MS        200
#define REFINEMENT_DELAY_MS         3000
#define SERVO_SETTLE_DELAY_MS       300
#define INITIAL_STOP_DELAY_MS       1000

#define MIN_DISTANCE_FOR_WIDTH_SCAN 20.0f
#define MAX_REASONABLE_WIDTH_CM     30.0f
#define MAX_EDGE_DISTANCE_CM        40.0f
#define MAX_DISTANCE_DIFF_FACTOR    2.5f

#define MAX_DETECTION_DISTANCE_CM   100.0f
#define EDGE_CONFIRMATION_SAMPLES   10
#define MIN_VALID_SAMPLES           7
#define CONSISTENCY_THRESHOLD_CM    5.0f
#define OUTLIER_THRESHOLD_CM        15.0f

// ============== Scan Parameters ==============
#define SERVO_SCAN_STEP_DEGREE      1.0f
#define SERVO_REFINE_STEP_DEGREE    0.5f

// ============== Function Prototypes ==============

/**
 * Initialize ultrasonic sensor (GPIO pins).
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
 */
bool ultrasonic_update(void);

/**
 * Get last valid distance measurement (cm).
 * Returns ULTRASONIC_INVALID_READING if no valid measurement yet.
 */
float ultrasonic_get_last_distance_cm(void);

/**
 * Check if an obstacle is within a threshold distance.
 */
bool ultrasonic_obstacle_detected(float threshold_cm);

// ============== NEW: Width Scan Functions (from testDemo3.c) ==============

/**
 * Helper function: Calculate width component from adjacent distance and hypotenuse
 */
float ultrasonic_calculate_width_component(float adjacent, float hypotenuse);

/**
 * Take consistent samples with outlier rejection and median filtering
 * Returns true if consistent samples obtained, false otherwise
 */
bool ultrasonic_take_consistent_samples(float *avg_distance_out);

/**
 * Validate edge distance against center distance
 */
bool ultrasonic_validate_edge_distance(float center_distance, float edge_distance, const char* side);

/**
 * Scan left side from 35° to 90° to find obstacle edge
 * Returns width in cm, or 0.0f if no valid edge found
 */
float ultrasonic_scan_left_side(float adjacent);

/**
 * Scan right side from 150° to 90° to find obstacle edge
 * Returns width in cm, or 0.0f if no valid edge found
 */
float ultrasonic_scan_right_side(float adjacent);

/**
 * Run complete width scan sequence
 * Publishes results to MQTT topic if connected
 * This is the main entry point for obstacle width measurement
 */
void ultrasonic_run_width_scan_sequence(float adjacent);

/**
 * Combined width scan and avoidance sequence
 * Wrapper that calls width scan and coordinates with motor/servo
 */
void ultrasonic_run_width_scan_and_avoid(float initial_distance);

// ============== Helper Functions ==============

/**
 * Absolute value for float
 */
static inline float ultrasonic_abs_float(float x) {
    return (x < 0.0f) ? -x : x;
}

/**
 * Comparison function for qsort (median calculation)
 */
int ultrasonic_compare_floats(const void *a, const void *b);

/**
 * Calculate median of float array
 */
float ultrasonic_calculate_median(float *arr, int n);

#endif // ULTRASONIC_H