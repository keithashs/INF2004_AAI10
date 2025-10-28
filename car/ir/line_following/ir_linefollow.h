#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "config.h"

// ===== Pin Definitions =====
#define IR_LEFT_ADC   IR_LEFT_ADC_PIN        // GP28 (ADC2) for analog line detection

// ===== Detection Thresholds (for 1.7cm thin line) =====
#define IR_BLACK_THRESHOLD  1200    // ADC > 1200 = black line detected
#define IR_WHITE_THRESHOLD  1000    // ADC < 1000 = white surface

// ===== Line Error Structure =====
typedef struct {
    float error;           // PID error (-3.0 to +3.0)
                          // 0.0 = centered, +ve = drift left (turn right), -ve = drift right (turn left)
    bool line_detected;    // True if black line is under sensor
    uint16_t raw_adc;      // Raw ADC value (0-4095)
    float confidence;      // Detection confidence (0.0 to 1.0)
} line_reading_t;

// ===== API Functions =====

// Initialize line following sensor (call once at startup)
void ir_line_init(void);

// Read raw ADC value from left sensor
uint16_t ir_read_left_adc(void);

// Check if sensor detects black line (binary)
bool ir_left_is_black(void);

// Get line following error for PID controller
// This is the main function for control loop
line_reading_t ir_get_line_error(void);

// Reset internal state (useful after turns or line loss recovery)
void ir_line_reset_state(void);

// Run calibration procedure to determine ADC thresholds
void ir_line_calibrate(void);