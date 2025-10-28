#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

// ===== Barcode Pin (Analog Input) =====
#define BARCODE_ADC_GPIO  26   // GP26 = ADC0 (Right IR sensor)
#define BARCODE_ADC_CH    0

// ===== Thresholds (tune on your track) =====
#define BARCODE_BLACK_THRESHOLD   2000   // ADC > 2000 = black bar
#define BARCODE_WHITE_THRESHOLD   1000   // ADC < 1000 = white space

// ===== Timing Constants =====
#define MIN_BAR_WIDTH_US      500     // Minimum bar width (microseconds)
#define MAX_BAR_WIDTH_US      8000    // Maximum valid width
#define PATTERN_TIMEOUT_US    15000   // Pattern must complete within this time
#define MAX_BARS              30      // Maximum bars to capture

// ===== Barcode Commands =====
typedef enum {
    BARCODE_NONE = 0,
    BARCODE_LEFT,      // Turn left 90°
    BARCODE_RIGHT,     // Turn right 90°
    BARCODE_STOP,      // Stop at checkpoint
    BARCODE_UTURN,     // U-turn 180°
    BARCODE_INVALID    // Detected but not recognized
} barcode_cmd_t;

// ===== Detection Result =====
typedef struct {
    barcode_cmd_t command;
    absolute_time_t detected_time;
    uint8_t confidence;   // 0-100 quality metric
    uint8_t bar_count;    // Number of bars detected
} barcode_result_t;

// ===== API =====

// Initialize ADC for barcode sensor
void barcode_init(void);

// Poll for barcode (call at ~100 Hz in control loop)
// Returns true if new barcode detected
bool barcode_poll(barcode_result_t *result);

// Convert command to string
const char* barcode_cmd_str(barcode_cmd_t cmd);

// Reset detection state (call after executing command)
void barcode_reset(void);

// Get raw ADC value (for debugging)
uint16_t barcode_read_adc(void);