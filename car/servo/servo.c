#include "servo.h"
#include <stdio.h>

// ============== PWM Configuration ==============
// SG90 servo uses standard RC PWM: 50Hz, 1-2ms pulse
// 1.0ms = 0°, 1.5ms = 90°, 2.0ms = 180°
#define PWM_FREQUENCY_HZ    50          // Standard RC servo frequency
#define PWM_PERIOD_US       20000       // 1/50Hz = 20ms
#define PWM_MIN_US          1000        // 1ms = 0°
#define PWM_MAX_US          2000        // 2ms = 180°
#define PWM_CENTER_US       1500        // 1.5ms = 90°

static float current_angle = 90.0f;

/**
 * Convert angle (0-180°) to PWM pulse width in microseconds
 * Applies calibration offset for proper alignment
 */
static inline uint16_t angle_to_pwm_us(float angle_deg) {
    // Apply calibration offset to match physical servo alignment
    float calibrated_angle = angle_deg - SERVO_CALIBRATION_OFFSET;
    
    // Clamp to valid range [0°, 180°]
    if (calibrated_angle < 0.0f) calibrated_angle = 0.0f;
    if (calibrated_angle > 180.0f) calibrated_angle = 180.0f;
    
    // Map angle to PWM pulse width: 0° → 1000µs, 180° → 2000µs
    // Formula: pwm_us = 1000 + (angle / 180) * 1000
    uint16_t pwm_us = (uint16_t)(1000.0f + (calibrated_angle / 180.0f) * 1000.0f);
    
    return pwm_us;
}

/**
 * Initialize servo PWM on SERVO_PIN
 */
void servo_init(void) {
    // Set GPIO function to PWM
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    
    // Find which PWM slice and channel this GPIO belongs to
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    uint channel = pwm_gpio_to_channel(SERVO_PIN);
    
    // Configure PWM for 50Hz (20ms period)
    // Clock divider: system clock (125MHz) / divider = target frequency
    // For 50Hz: need period of 20ms = 20000µs
    // With 1µs precision: divider = 125MHz * 20ms / 65536 ≈ 38.15
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f);  // ~1µs per count
    pwm_config_set_wrap(&config, 20000);     // 20ms period
    pwm_init(slice_num, &config, true);      // true = start PWM enabled
    
    // Set initial position to center (90°)
    pwm_set_chan_level(slice_num, channel, angle_to_pwm_us(90.0f));
    
    // Enable the PWM
    pwm_set_enabled(slice_num, true);
    
    printf("[SERVO] Init OK - Pin %d, 50Hz PWM, Center 90°\n", SERVO_PIN);
}

/**
 * Set servo to specific angle
 */
bool servo_set_angle(float angle_deg) {
    // Validate range
    if (angle_deg < SERVO_ANGLE_MIN || angle_deg > SERVO_ANGLE_MAX) {
        printf("[SERVO] Angle %.1f° out of range [%.1f°, %.1f°]\n", 
               (double)angle_deg, (double)SERVO_ANGLE_MIN, (double)SERVO_ANGLE_MAX);
        return false;
    }
    
    current_angle = angle_deg;
    
    // Calculate PWM pulse width with calibration
    uint16_t pwm_us = angle_to_pwm_us(angle_deg);
    
    // Get PWM slice and channel
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    uint channel = pwm_gpio_to_channel(SERVO_PIN);
    
    // Set PWM level
    pwm_set_chan_level(slice_num, channel, pwm_us);
    
    return true;
}

/**
 * Get last set angle
 */
float servo_get_angle(void) {
    return current_angle;
}

/**
 * Move servo to left position
 */
void servo_move_left(uint16_t delay_ms) {
    servo_set_angle(SERVO_ANGLE_LEFT);
    sleep_ms(delay_ms);
}

/**
 * Move servo to center position
 */
void servo_move_center(uint16_t delay_ms) {
    servo_set_angle(SERVO_ANGLE_CENTER);
    sleep_ms(delay_ms);
}

/**
 * Move servo to right position
 */
void servo_move_right(uint16_t delay_ms) {
    servo_set_angle(SERVO_ANGLE_RIGHT);
    sleep_ms(delay_ms);
}
