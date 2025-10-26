#pragma once
#include <stdbool.h>
#include "pico/stdlib.h"

// ===== Pin Definitions (from your config) =====
#define IR_LEFT_PIN   0    // GP0 - Left line sensor (digital)
#define IR_RIGHT_PIN  27   // GP27 - Right line sensor (digital)

// ===== Line Detection Logic =====
// Set to 1 if sensor outputs LOW on black line
// Set to 0 if sensor outputs HIGH on black line
#define IR_BLACK_IS_LOW  1

// ===== Line Position Encoding =====
typedef enum {
    LINE_LOST = -1,        // Both sensors off line
    LINE_CENTER = 0,       // Both sensors on line (centered)
    LINE_LEFT = 1,         // Only left sensor on line (robot drifted right)
    LINE_RIGHT = 2,        // Only right sensor on line (robot drifted left)
    LINE_BOTH_OFF = 3      // Both sensors off (might be at intersection or lost)
} line_position_t;

// ===== API Functions =====

// Initialize IR line sensors (call once at startup)
void ir_line_init(void);

// Read raw digital states (0 or 1)
int ir_read_left_raw(void);
int ir_read_right_raw(void);

// Check if each sensor detects black line
bool ir_left_is_black(void);
bool ir_right_is_black(void);

// Get current line position (high-level state)
line_position_t ir_get_line_position(void);

// Convert line position to human-readable string (for telemetry)
const char* ir_line_position_str(line_position_t pos);