#pragma once
#include <stdbool.h>
#include "pico/stdlib.h"
#include "config.h"

// ===== Pin Definitions =====
#define IR_LEFT_PIN   IR_LEFT_DIGITAL_PIN    // GP7 for digital (optional)
#define IR_LEFT_ADC   IR_LEFT_ADC_PIN        // GP28 (ADC2) for analog line detection

// ===== Detection Logic =====
#define IR_BLACK_THRESHOLD  2000    // ADC value above = black line (tune for 1.7cm line)
#define IR_WHITE_THRESHOLD  1000    // ADC value below = white surface

// ===== Line Position Encoding =====
typedef enum {
    LINE_LOST = -1,        // Sensor off line completely
    LINE_CENTER = 0,       // Centered on line (ideal)
    LINE_LEFT = 1,         // Drifted left (need to steer right)
    LINE_RIGHT = 2,        // Drifted right (need to steer left)
    LINE_BOTH_OFF = 3      // Lost line or at junction
} line_position_t;

// ===== Line Error Encoding for Thin Line (1.7cm) =====
// For a thin line, we use a single sensor with analog readings
typedef struct {
    float error;           // Line following error (-2.0 to +2.0)
    bool line_detected;    // True if black line is detected
    uint16_t raw_adc;      // Raw ADC value for debugging
} line_reading_t;

// ===== API Functions =====

// Initialize line following sensor (call once at startup)
void ir_line_init(void);

// Read analog value from left sensor
uint16_t ir_read_left_adc(void);

// Check if sensor detects black line
bool ir_left_is_black(void);

// Get line following error for PID controller
// Returns: error value for steering correction
//   0.0 = perfectly centered
//   +ve = drifted left, steer right
//   -ve = drifted right, steer left
line_reading_t ir_get_line_error(void);

// Convert line position to string (for telemetry)
const char* ir_line_position_str(line_position_t pos);