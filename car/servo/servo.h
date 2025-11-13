#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// ============== Pin Configuration ==============
// SG90 servo typically on GPIO with PWM
#define SERVO_PIN           15     // GPIO pin (pick one with PWM capability)

// ============== Servo Calibration Offset ==============
// Mechanical offset to make servo parallel to car body
// Calibrated: 90° displays as 97° (parallel to car)
#define SERVO_CALIBRATION_OFFSET  10  // degrees

// ============== Servo Scanning Range ==============
// Maximum angles the servo can safely turn
// Center: 90°, Left max: 90+80=170°, Right max: 90-80=10°
#define SERVO_ANGLE_MIN     10.0f   // degrees (right limit)
#define SERVO_ANGLE_MAX     170.0f  // degrees (left limit)

// Convenience angle positions for obstacle scanning
// These are LOGICAL angles; they're internally adjusted by calibration offset
#define SERVO_ANGLE_LEFT    170.0f  // 170° = maximum left
#define SERVO_ANGLE_CENTER  90.0f   // 90° = center (parallel to car)
#define SERVO_ANGLE_RIGHT   10.0f   // 10° = maximum right

// ============== Function Prototypes ==============

/**
 * Initialize servo PWM on SERVO_PIN.
 * Sets up 50Hz PWM frequency (standard for RC servos).
 * Call once at startup.
 */
void servo_init(void);

/**
 * Set servo to a specific angle (0-180 degrees).
 * Non-blocking; PWM takes effect immediately.
 * Returns false if angle out of range, true on success.
 */
bool servo_set_angle(float angle_deg);

/**
 * Get the last set angle (degrees).
 */
float servo_get_angle(void);

/**
 * Convenience: move servo to left position and wait.
 * Blocking call with optional delay_ms.
 */
void servo_move_left(uint16_t delay_ms);

/**
 * Convenience: move servo to center position and wait.
 */
void servo_move_center(uint16_t delay_ms);

/**
 * Convenience: move servo to right position and wait.
 */
void servo_move_right(uint16_t delay_ms);

#endif // SERVO_H
