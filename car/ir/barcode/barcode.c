#include "barcode.h"
#include "hardware/adc.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// ===== State Machine =====
typedef enum {
    SCAN_IDLE,
    SCAN_IN_PROGRESS,
    SCAN_COMPLETE
} scan_state_t;

static scan_state_t state = SCAN_IDLE;
static uint64_t last_transition_us = 0;
static uint16_t bar_widths[MAX_BARS];  // Store widths in 0.1ms units
static uint8_t bar_index = 0;
static barcode_cmd_t last_command = BARCODE_NONE;
static bool last_was_black = false;

// ===== Speed-Adaptive Thresholds =====
static float speed_factor = 1.0f;  // Multiplier for width thresholds

// ===== Code 39 Simplified Patterns (Narrow=1, Wide=2) =====
// Each pattern has 6 bars total (3 black, 3 white alternating)
typedef struct {
    barcode_cmd_t command;
    const char* pattern;    // Pattern string (only '1' and '2')
    uint8_t length;
} barcode_pattern_t;

static const barcode_pattern_t patterns[] = {
    {BARCODE_LEFT,  "121121", 6},   // LEFT:  narrow-wide-narrow-narrow-wide-narrow
    {BARCODE_RIGHT, "112121", 6},   // RIGHT: narrow-narrow-wide-narrow-wide-narrow  
    {BARCODE_STOP,  "212112", 6},   // STOP:  wide-narrow-wide-narrow-narrow-wide
    {BARCODE_UTURN, "121211", 6}    // UTURN: narrow-wide-narrow-wide-narrow-narrow
};

// ===== Initialize =====
void barcode_init(void) {
    adc_init();
    adc_gpio_init(BARCODE_ADC_GPIO);
    barcode_reset();
    printf("[BARCODE] Initialized on GP%d (ADC0)\n", BARCODE_ADC_GPIO);
}

// ===== Read ADC =====
uint16_t barcode_read_adc(void) {
    adc_select_input(BARCODE_ADC_CH);
    return adc_read();
}

// ===== Set speed calibration factor =====
void barcode_set_speed_factor(float factor) {
    speed_factor = factor;
}

// ===== Classify bar width as narrow (1) or wide (2) =====
static char classify_width(uint16_t width, uint16_t min_width) {
    if (min_width == 0) return '1';  // Safety check
    
    float ratio = (float)width / (float)min_width;
    
    // Adaptive threshold based on speed
    // If ratio < 1.6, it's narrow; otherwise wide
    float threshold = 1.6f * speed_factor;
    
    if (ratio < threshold) {
        return '1';  // Narrow
    } else {
        return '2';  // Wide
    }
}

// ===== Pattern Matching =====
static barcode_cmd_t decode_pattern(uint16_t *widths, uint8_t count) {
    if (count < 6) {
        printf("[BARCODE] Too few bars: %d\n", count);
        return BARCODE_INVALID;
    }
    
    // Find minimum width (as reference)
    uint16_t min_w = 9999;
    for (uint8_t i = 0; i < count; i++) {
        if (widths[i] < min_w && widths[i] > 10) {
            min_w = widths[i];
        }
    }
    
    if (min_w < 30) {
        printf("[BARCODE] Min width too small: %d (too fast or noise)\n", min_w);
        return BARCODE_INVALID;
    }
    
    // Build pattern string (only '1' and '2')
    char captured[MAX_BARS + 1] = {0};
    for (uint8_t i = 0; i < count && i < MAX_BARS; i++) {
        captured[i] = classify_width(widths[i], min_w);
    }
    captured[count] = '\0';
    
    printf("[BARCODE] Captured pattern: %s (min_w=%d)\n", captured, min_w);
    
    // Match against known patterns
    for (size_t i = 0; i < sizeof(patterns) / sizeof(patterns[0]); i++) {
        // Compare first 6 characters (core pattern)
        if (count >= patterns[i].length && 
            strncmp(captured, patterns[i].pattern, patterns[i].length) == 0) {
            printf("[BARCODE] ✓ Matched: %s\n", barcode_cmd_str(patterns[i].command));
            return patterns[i].command;
        }
    }
    
    printf("[BARCODE] ✗ No match found\n");
    return BARCODE_INVALID;
}

// ===== Poll (call at 100 Hz) =====
bool barcode_poll(barcode_result_t *result) {
    uint16_t adc_val = barcode_read_adc();
    uint64_t now_us = time_us_64();
    bool is_black = (adc_val > BARCODE_BLACK_THRESHOLD);
    
    // Detect edge transition (black ↔ white)
    if (is_black != last_was_black) {
        uint64_t width_us = now_us - last_transition_us;
        last_transition_us = now_us;
        
        // Filter valid bar widths
        if (width_us > MIN_BAR_WIDTH_US && width_us < MAX_BAR_WIDTH_US) {
            if (state == SCAN_IDLE) {
                state = SCAN_IN_PROGRESS;
                bar_index = 0;
                printf("[BARCODE] Scan started\n");
            }
            
            if (state == SCAN_IN_PROGRESS && bar_index < MAX_BARS) {
                bar_widths[bar_index++] = (uint16_t)(width_us / 100);  // Store in 0.1ms
            }
        }
        
        last_was_black = is_black;
    }
    
    // Check for timeout (pattern complete)
    if (state == SCAN_IN_PROGRESS && (now_us - last_transition_us > PATTERN_TIMEOUT_US)) {
        printf("[BARCODE] Timeout - decoding %d bars\n", bar_index);
        
        barcode_cmd_t cmd = decode_pattern(bar_widths, bar_index);
        
        if (cmd != BARCODE_INVALID && cmd != BARCODE_NONE) {
            if (result) {
                result->command = cmd;
                result->detected_time = get_absolute_time();
                result->confidence = 85;
                result->bar_count = bar_index;
            }
            last_command = cmd;
            state = SCAN_COMPLETE;
            return true;  // ✓ New barcode detected!
        }
        
        // Invalid pattern - reset and try again
        barcode_reset();
    }
    
    return false;
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
    bar_index = 0;
    memset(bar_widths, 0, sizeof(bar_widths));
    last_was_black = false;
}