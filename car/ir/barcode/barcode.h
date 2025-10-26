#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

// ===== Barcode Pin (Analog Input) =====
#define BARCODE_ADC_GPIO  26   // GP26 = ADC0 (A0 from sensor)
#define BARCODE_ADC_CH    0

// ===== Barcode Commands (Code-39 subset) =====
typedef enum {
    BARCODE_NONE = 0,
    BARCODE_LEFT,      // Turn left at junction
    BARCODE_RIGHT,     // Turn right at junction
    BARCODE_STOP,      // Stop at checkpoint
    BARCODE_UTURN,     // U-turn (180°)
    BARCODE_INVALID    // Detected but not recognized
} barcode_cmd_t;

// ===== Detection State =====
typedef struct {
    barcode_cmd_t command;
    absolute_time_t detected_time;
    uint16_t confidence;   // 0-100 (quality metric)
} barcode_result_t;

// ===== API =====

// Initialize ADC for barcode sensor (call once)
void barcode_init(void);

// Poll for barcode (non-blocking). Returns true if new barcode detected.
// Result is stored in *result. Call this at ~100 Hz in your control loop.
bool barcode_poll(barcode_result_t *result);

// Convert barcode command to string (for telemetry)
const char* barcode_cmd_str(barcode_cmd_t cmd);

// Reset detection state (call after executing a command)
void barcode_reset(void);