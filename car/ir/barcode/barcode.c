#include "barcode.h"
#include "hardware/adc.h"
#include <math.h>
#include <string.h>

// ===== Thresholds (tune on your track) =====
#define BLACK_THRESHOLD   2700   // ADC value above = black bar
#define WHITE_THRESHOLD   500    // ADC value below = white space

// ===== Timing Constants (Code-39 pattern recognition) =====
#define MIN_BAR_WIDTH_US  800    // Minimum bar/space width (microseconds)
#define MAX_BAR_WIDTH_US  5000   // Maximum valid width
#define PATTERN_TIMEOUT_US 10000 // Pattern must complete within this time

// ===== State Machine =====
typedef enum {
    SCAN_IDLE,
    SCAN_IN_PROGRESS,
    SCAN_COMPLETE
} scan_state_t;

static scan_state_t state = SCAN_IDLE;
static uint64_t last_transition_us = 0;
static uint16_t pattern_buffer[16];  // Store bar/space widths
static uint8_t pattern_index = 0;
static barcode_cmd_t last_command = BARCODE_NONE;

// ===== Pattern Matching (simplified Code-39 subset) =====
// In real Code-39, each character has 9 bars/spaces (5 bars, 4 spaces).
// For this demo, we'll use simplified 3-bar patterns:
// LEFT:  narrow-wide-narrow   (e.g., 1-3-1 ratio)
// RIGHT: narrow-narrow-wide   (1-1-3)
// STOP:  wide-narrow-wide     (3-1-3)

static barcode_cmd_t decode_pattern(uint16_t *widths, uint8_t count) {
    if (count < 3) return BARCODE_INVALID;

    // Normalize to ratios (relative to smallest bar)
    uint16_t min_w = 9999;
    for (uint8_t i = 0; i < count; i++) {
        if (widths[i] < min_w) min_w = widths[i];
    }
    if (min_w < 50) return BARCODE_INVALID; // Too small, noise

    float r0 = (float)widths[0] / min_w;
    float r1 = (float)widths[1] / min_w;
    float r2 = (float)widths[2] / min_w;

    // Simple ratio matching (with tolerance ±0.5)
    if (fabsf(r0 - 1.0f) < 0.5f && fabsf(r1 - 3.0f) < 0.7f && fabsf(r2 - 1.0f) < 0.5f) {
        return BARCODE_LEFT;   // 1-3-1
    }
    if (fabsf(r0 - 1.0f) < 0.5f && fabsf(r1 - 1.0f) < 0.5f && fabsf(r2 - 3.0f) < 0.7f) {
        return BARCODE_RIGHT;  // 1-1-3
    }
    if (fabsf(r0 - 3.0f) < 0.7f && fabsf(r1 - 1.0f) < 0.5f && fabsf(r2 - 3.0f) < 0.7f) {
        return BARCODE_STOP;   // 3-1-3
    }

    return BARCODE_INVALID;
}

// ===== Initialize =====
void barcode_init(void) {
    adc_init();
    adc_gpio_init(BARCODE_ADC_GPIO);
    adc_select_input(BARCODE_ADC_CH);
    barcode_reset();
}

// ===== Poll (call at 100 Hz) =====
bool barcode_poll(barcode_result_t *result) {
    uint16_t adc_val = adc_read();
    uint64_t now_us = time_us_64();
    static bool last_was_black = false;
    bool is_black = (adc_val > BLACK_THRESHOLD);

    // Detect edge transition (black ↔ white)
    if (is_black != last_was_black) {
        uint64_t width_us = now_us - last_transition_us;
        last_transition_us = now_us;

        if (width_us > MIN_BAR_WIDTH_US && width_us < MAX_BAR_WIDTH_US) {
            if (state == SCAN_IDLE) {
                state = SCAN_IN_PROGRESS;
                pattern_index = 0;
            }

            if (state == SCAN_IN_PROGRESS && pattern_index < 16) {
                pattern_buffer[pattern_index++] = (uint16_t)(width_us / 100); // store in 0.1ms units
            }
        }

        last_was_black = is_black;
    }

    // Check for pattern timeout
    if (state == SCAN_IN_PROGRESS && (now_us - last_transition_us > PATTERN_TIMEOUT_US)) {
        // Attempt decode
        barcode_cmd_t cmd = decode_pattern(pattern_buffer, pattern_index);
        if (cmd != BARCODE_INVALID && cmd != BARCODE_NONE) {
            if (result) {
                result->command = cmd;
                result->detected_time = get_absolute_time();
                result->confidence = 80; // placeholder
            }
            last_command = cmd;
            state = SCAN_COMPLETE;
            return true;  // New barcode detected!
        }
        barcode_reset();  // Invalid or incomplete, reset
    }

    return false;  // No new barcode
}

// ===== String Conversion =====
const char* barcode_cmd_str(barcode_cmd_t cmd) {
    switch (cmd) {
        case BARCODE_LEFT:   return "LEFT";
        case BARCODE_RIGHT:  return "RIGHT";
        case BARCODE_STOP:   return "STOP";
        case BARCODE_UTURN:  return "UTURN";
        case BARCODE_INVALID: return "INVALID";
        default:             return "NONE";
    }
}

// ===== Reset =====
void barcode_reset(void) {
    state = SCAN_IDLE;
    pattern_index = 0;
    memset(pattern_buffer, 0, sizeof(pattern_buffer));
}