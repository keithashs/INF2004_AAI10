#include "ultrasonic.h"
#include "servo.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// ============== Timing Constants ==============
#define TRIG_PULSE_US       10
#define SOUND_SPEED_CM_US   0.0343

// ============== Measurement State ==============
static float last_distance_cm = ULTRASONIC_INVALID_READING;

// ============== Servo Angle Definitions (from testDemo3.c) ==============
#define SERVO_CENTER_ANGLE          90.0f
#define SERVO_RIGHT_ANGLE           150.0f
#define SERVO_LEFT_ANGLE            35.0f

/**
 * Initialize ultrasonic sensor pins
 */
void ultrasonic_init(void) {
    gpio_init(ULTRASONIC_TRIG_PIN);
    gpio_set_dir(ULTRASONIC_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);
    
    gpio_init(ULTRASONIC_ECHO_PIN);
    gpio_set_dir(ULTRASONIC_ECHO_PIN, GPIO_IN);
    
    printf("[ULTRASONIC] Init OK - TRIG=GPIO%d, ECHO=GPIO%d\n", 
           ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
}

/**
 * Measure distance in cm using single trigger
 */
float ultrasonic_get_distance_cm(void) {
    gpio_put(ULTRASONIC_TRIG_PIN, 1);
    sleep_us(TRIG_PULSE_US);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);
    
    uint32_t timeout = 500000;
    while (gpio_get(ULTRASONIC_ECHO_PIN) == 0 && timeout--) {
        sleep_us(1);
    }
    
    if (timeout == 0) {
        last_distance_cm = ULTRASONIC_INVALID_READING;
        return ULTRASONIC_INVALID_READING;
    }
    
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
    
    uint32_t pulse_duration_us = pulse_end - pulse_start;
    float distance_cm = (pulse_duration_us / 2.0f) * SOUND_SPEED_CM_US;
    
    if (distance_cm < ULTRASONIC_MIN_DISTANCE_CM || distance_cm > ULTRASONIC_MAX_DISTANCE_CM) {
        last_distance_cm = ULTRASONIC_INVALID_READING;
        return ULTRASONIC_INVALID_READING;
    }
    
    last_distance_cm = distance_cm;
    return distance_cm;
}

float ultrasonic_get_last_distance_cm(void) {
    return last_distance_cm;
}

bool ultrasonic_obstacle_detected(float threshold_cm) {
    float distance = ultrasonic_get_distance_cm();
    return (distance > 0.0f && distance <= threshold_cm);
}

bool ultrasonic_update(void) {
    float distance = ultrasonic_get_distance_cm();
    return (distance > 0.0f);
}

// ============== Width Scan Helper Functions ==============

float ultrasonic_calculate_width_component(float adjacent, float hypotenuse) {
    float h2 = hypotenuse * hypotenuse;
    float a2 = adjacent * adjacent;
    if (h2 <= a2) {
        printf(" [WARNING] Hypotenuse (%.2f) <= adjacent (%.2f), returning 0\n",
               hypotenuse, adjacent);
        return 0.0f;
    }
    return sqrtf(h2 - a2);
}

int ultrasonic_compare_floats(const void *a, const void *b) {
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

float ultrasonic_calculate_median(float *arr, int n) {
    if (n == 0) return 0.0f;
    
    float temp[EDGE_CONFIRMATION_SAMPLES];
    for (int i = 0; i < n; i++) {
        temp[i] = arr[i];
    }
    
    qsort(temp, n, sizeof(float), ultrasonic_compare_floats);
    
    if (n % 2 == 0) {
        return (temp[n/2 - 1] + temp[n/2]) / 2.0f;
    } else {
        return temp[n/2];
    }
}

bool ultrasonic_take_consistent_samples(float *avg_distance_out) {
    float samples[EDGE_CONFIRMATION_SAMPLES];
    int valid = 0;

    for (int i = 0; i < EDGE_CONFIRMATION_SAMPLES; i++) {
        sleep_ms(MEASUREMENT_DELAY_MS);
        float s = ultrasonic_get_distance_cm();
        if (s > 0 && s <= MAX_DETECTION_DISTANCE_CM) {
            samples[valid++] = s;
            printf(" Sample %d: %.2f cm ✓\n", i + 1, s);
        } else {
            printf(" Sample %d: invalid ✗\n", i + 1);
        }
    }

    if (valid < MIN_VALID_SAMPLES) {
        printf(" Only %d/%d valid samples\n", valid, EDGE_CONFIRMATION_SAMPLES);
        return false;
    }

    float median = ultrasonic_calculate_median(samples, valid);
    printf(" Median: %.2f cm\n", median);

    float filtered[EDGE_CONFIRMATION_SAMPLES];
    int filtered_count = 0;
    
    for (int i = 0; i < valid; i++) {
        float diff = ultrasonic_abs_float(samples[i] - median);
        if (diff <= OUTLIER_THRESHOLD_CM) {
            filtered[filtered_count++] = samples[i];
        }
    }

    if (filtered_count < MIN_VALID_SAMPLES - 2) {
        printf(" Only %d samples after filtering\n", filtered_count);
        return false;
    }

    float sum = 0.0f;
    float min_val = 1e9f;
    float max_val = 0.0f;
    
    for (int i = 0; i < filtered_count; i++) {
        sum += filtered[i];
        if (filtered[i] < min_val) min_val = filtered[i];
        if (filtered[i] > max_val) max_val = filtered[i];
    }

    float avg = sum / filtered_count;
    float variance = max_val - min_val;

    if (variance > CONSISTENCY_THRESHOLD_CM) {
        printf(" Variance %.2f cm exceeds threshold\n", variance);
        return false;
    }

    *avg_distance_out = avg;
    printf(" ✓ CONSISTENT: %.2f cm\n", avg);
    return true;
}

bool ultrasonic_validate_edge_distance(float center_distance, float edge_distance, const char* side) {
    float max_expected = center_distance * MAX_DISTANCE_DIFF_FACTOR;
    
    if (edge_distance > MAX_EDGE_DISTANCE_CM) {
        printf(" [VALIDATE-%s] REJECTED: %.2f cm > max allowed (%.2f cm)\n", 
               side, edge_distance, MAX_EDGE_DISTANCE_CM);
        return false;
    }
    
    if (edge_distance > max_expected) {
        printf(" [VALIDATE-%s] REJECTED: %.2f cm > expected max (%.2f cm based on center %.2f cm)\n",
               side, edge_distance, max_expected, center_distance);
        return false;
    }
    
    printf(" [VALIDATE-%s] PASSED: %.2f cm is reasonable\n", side, edge_distance);
    return true;
}

float ultrasonic_scan_left_side(float adjacent) {
    printf("\n[LEFT_SCAN] Starting from 35° scanning toward 90°...\n");
    sleep_ms(2000);

    float current_angle = SERVO_LEFT_ANGLE;
    bool edge_found = false;
    float edge_distance = 0.0f;

    while (current_angle <= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(SERVO_SETTLE_DELAY_MS);
        
        float d = ultrasonic_get_distance_cm();
        if (d > 0 && d <= MAX_DETECTION_DISTANCE_CM) {
            printf("[LEFT_SCAN] %.1f° │ ECHO at %.2f cm, confirming...\n", current_angle, d);
            float avg;
            if (ultrasonic_take_consistent_samples(&avg)) {
                if (ultrasonic_validate_edge_distance(adjacent, avg, "LEFT")) {
                    printf("[LEFT_SCAN] ✓✓ VALID EDGE at %.1f°!\n", current_angle);
                    edge_found = true;
                    edge_distance = avg;
                    break;
                } else {
                    printf("[LEFT_SCAN] ✗ Invalid edge distance, continuing...\n");
                }
            } else {
                printf("[LEFT_SCAN] ✗ Inconsistent, continuing...\n");
            }
        } else {
            printf("[LEFT_SCAN] %.1f° │ No echo\n", current_angle);
        }
        current_angle += SERVO_SCAN_STEP_DEGREE;
    }

    if (!edge_found) {
        printf("[LEFT_SCAN] No valid edge found!\n");
        return 0.0f;
    }

    float left_width = ultrasonic_calculate_width_component(adjacent, edge_distance);
    
    if (left_width > MAX_REASONABLE_WIDTH_CM) {
        printf("[LEFT_SCAN] ✗ REJECTED: Width %.2f cm > max reasonable (%.2f cm)\n",
               left_width, MAX_REASONABLE_WIDTH_CM);
        return 0.0f;
    }
    
    printf("[LEFT_SCAN] Final distance: %.2f cm\n", edge_distance);
    printf("[LEFT_SCAN] LEFT width: %.2f cm ✓\n", left_width);
    return left_width;
}

float ultrasonic_scan_right_side(float adjacent) {
    printf("\n[RIGHT_SCAN] Starting from 150° scanning toward 90°...\n");
    sleep_ms(2000);

    float current_angle = SERVO_RIGHT_ANGLE;
    bool edge_found = false;
    float edge_distance = 0.0f;

    while (current_angle >= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(SERVO_SETTLE_DELAY_MS);
        
        float d = ultrasonic_get_distance_cm();
        if (d > 0 && d <= MAX_DETECTION_DISTANCE_CM) {
            printf("[RIGHT_SCAN] %.1f° │ ECHO at %.2f cm, confirming...\n", current_angle, d);
            float avg;
            if (ultrasonic_take_consistent_samples(&avg)) {
                if (ultrasonic_validate_edge_distance(adjacent, avg, "RIGHT")) {
                    printf("[RIGHT_SCAN] ✓✓ VALID EDGE at %.1f°!\n", current_angle);
                    edge_found = true;
                    edge_distance = avg;
                    break;
                } else {
                    printf("[RIGHT_SCAN] ✗ Invalid edge distance, continuing...\n");
                }
            } else {
                printf("[RIGHT_SCAN] ✗ Inconsistent, continuing...\n");
            }
        } else {
            printf("[RIGHT_SCAN] %.1f° │ No echo\n", current_angle);
        }
        current_angle -= SERVO_SCAN_STEP_DEGREE;
    }

    if (!edge_found) {
        printf("[RIGHT_SCAN] No valid edge found!\n");
        return 0.0f;
    }

    float right_width = ultrasonic_calculate_width_component(adjacent, edge_distance);
    
    if (right_width > MAX_REASONABLE_WIDTH_CM) {
        printf("[RIGHT_SCAN] ✗ REJECTED: Width %.2f cm > max reasonable (%.2f cm)\n",
               right_width, MAX_REASONABLE_WIDTH_CM);
        return 0.0f;
    }
    
    printf("[RIGHT_SCAN] Final distance: %.2f cm\n", edge_distance);
    printf("[RIGHT_SCAN] RIGHT width: %.2f cm ✓\n", right_width);
    return right_width;
}

// Note: MQTT publishing needs to be handled in main.c
// These functions provide the measurement logic only

void ultrasonic_run_width_scan_sequence(float adjacent) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║ *** OBSTACLE DETECTED ***                                         ║\n");
    printf("║ Perpendicular Distance: %.2f cm                                   ║\n", adjacent);
    printf("╚════════════════════════════════════════════════════════════════════╝\n");

    if (adjacent > MIN_DISTANCE_FOR_WIDTH_SCAN) {
        printf("\n[SCAN] Distance %.2f cm > %.2f cm - TOO FAR for reliable width measurement\n",
               adjacent, MIN_DISTANCE_FOR_WIDTH_SCAN);
        printf("[SCAN] Skipping width scan - ultrasonic not reliable at this distance\n");
        
        servo_set_angle(SERVO_CENTER_ANGLE);
        return;
    }

    printf("[SCAN] Distance %.2f cm < %.2f cm - GOOD for width measurement\n",
           adjacent, MIN_DISTANCE_FOR_WIDTH_SCAN);
    printf("Waiting %dms for system to stabilize...\n", INITIAL_STOP_DELAY_MS);
    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(INITIAL_STOP_DELAY_MS);

    float left_w  = ultrasonic_scan_left_side(adjacent);
    sleep_ms(500);
    float right_w = ultrasonic_scan_right_side(adjacent);
    float total_w = left_w + right_w;

    printf("\n[RESULT] LEFT=%.2f cm, RIGHT=%.2f cm → TOTAL WIDTH=%.2f cm\n",
           left_w, right_w, total_w);

    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(300);
    
    printf("\n[SCAN] Obstacle measurement complete\n");
    printf("[SCAN] LEFT: %.2f cm, RIGHT: %.2f cm, TOTAL: %.2f cm\n",
           left_w, right_w, total_w);
}

void ultrasonic_run_width_scan_and_avoid(float initial_distance) {
    // This is a wrapper that coordinates the scan
    // The actual avoidance (turn + line search) is handled in main.c
    ultrasonic_run_width_scan_sequence(initial_distance);
}