#ifndef IR_H
#define IR_H

#include "pico/stdlib.h"
#include "hardware/adc.h"

// IR sensor pins (change if needed)
#define IR_LEFT_PIN   26
#define IR_RIGHT_PIN  28

// If IR sensor outputs LOW on black line, set to 1
// If it outputs HIGH on black line, set to 0
#define IR_BLACK_IS_LOW  1

// ======== LINE FOLLOWING SENSOR CONFIGURATION ========
#ifndef LINE_SENSOR_PIN
#define LINE_SENSOR_PIN 28
#endif

// ======== EDGE FOLLOWING PARAMETERS ========
#ifndef TARGET_EDGE_VALUE
#define TARGET_EDGE_VALUE 1800
#endif

#ifndef LINE_FOLLOWING_MODE
#define LINE_FOLLOWING_MODE 0
#endif

#ifndef EDGE_DIRECTION
#define EDGE_DIRECTION 1
#endif

#ifndef BASE_SPEED_CMPS
#define BASE_SPEED_CMPS 2.0f
#endif

// ======== PID STEERING PARAMETERS ========
#ifndef KP_STEER
#define KP_STEER 0.055f
#endif

#ifndef KI_STEER
#define KI_STEER 0.0002f
#endif

#ifndef KD_STEER
#define KD_STEER 0.0010f
#endif

#ifndef MAX_STEER_CORRECTION
#define MAX_STEER_CORRECTION 2.0f
#endif

#ifndef ERROR_DEADBAND
#define ERROR_DEADBAND 30
#endif

#ifndef PWM_SCALE_FACTOR
#define PWM_SCALE_FACTOR 25.0f
#endif

// ======== CORNER HANDLING PARAMETERS ========
#ifndef CORNER_ERROR_THRESHOLD
#define CORNER_ERROR_THRESHOLD 300
#endif

#ifndef CORNER_SPEED_REDUCTION
#define CORNER_SPEED_REDUCTION 8
#endif

#ifndef CORNER_GAIN_MULTIPLIER
#define CORNER_GAIN_MULTIPLIER 1.6f
#endif

#ifndef MIN_CORNER_PWM_DIFF
#define MIN_CORNER_PWM_DIFF 40
#endif

// ======== PID Controller Structure ========
typedef struct {
    float integral;
    float prev_error;
    float kp;
    float ki;
    float kd;
} PIDController;

// Initialize IR sensor GPIOs
void ir_init(void);

// Read digital states (raw 0 or 1)
int ir_read_left_raw(void);
int ir_read_right_raw(void);

// Check whether each sensor detects the black line
int ir_left_is_black(void);
int ir_right_is_black(void);

// ======== LINE FOLLOWING SENSOR FUNCTIONS ========
// Initialize ADC for line sensor
void init_line_adc(void);

// Read ADC value from line sensor
uint16_t read_line_adc(void);

// ======== PID CONTROLLER FUNCTIONS ========
// Initialize PID controller with gains
void pid_init(PIDController *pid, float kp, float ki, float kd);

// Compute PID output given error and time delta
float pid_compute(PIDController *pid, float error, float dt);

// ======== HELPER FUNCTIONS ========
// Absolute value for float
static inline float abs_float(float x) {
    return (x < 0.0f) ? -x : x;
}

// Clamp PWM values to valid ranges
int clamp_pwm_left(int pwm);
int clamp_pwm_right(int pwm);

#endif