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
static uint32_t bar_widths[MAX_BARS];  // Store widths in microseconds
static uint8_t bar_colors[MAX_BARS];   // 1 = black, 0 = white
static uint8_t bar_index = 0;
static barcode_cmd_t last_command = BARCODE_NONE;
static bool last_was_black = false;
static float speed_factor = 1.0f;

// ===== Code 39 Pattern Database =====
// Each pattern: 9 elements representing bar widths
// Format: "BWBWBWBWB" where B=Black, W=White
// 1=narrow, 2=wide (based on Code 39 standard)

typedef struct {
    char character;
    const char pattern[10];  // 9 bars + null terminator
    barcode_cmd_t command;
} code39_entry_t;

// Simplified Code 39 patterns (5 black + 4 white bars, 3 wide total)
// Pattern format: each digit is bar width (1=narrow, 2=wide)
static const code39_entry_t code39_table[] = {
    // Commands we care about
    {'L', "211121112", BARCODE_LEFT},   // LEFT command
    {'R', "121121211", BARCODE_RIGHT},  // RIGHT command  
    {'S', "221111121", BARCODE_STOP},   // STOP command
    {'U', "121211121", BARCODE_UTURN},  // UTURN command
    
    // Standard Code 39 characters (for reference)
    {'0', "121121211", BARCODE_NONE},
    {'1', "221111121", BARCODE_NONE},
    {'2', "121211121", BARCODE_NONE},
    {'3', "221211111", BARCODE_NONE},
    {'4', "121121211", BARCODE_NONE},
    {'5', "221121111", BARCODE_NONE},
    {'6', "121221111", BARCODE_NONE},
    {'7', "121111221", BARCODE_NONE},
    {'8', "221111121", BARCODE_NONE},
    {'9', "121211121", BARCODE_NONE},
    {'A', "221121111", BARCODE_NONE},
    // Add more as needed...
};

#define CODE39_TABLE_SIZE (sizeof(code39_table) / sizeof(code39_entry_t))

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
static char classify_bar_width(uint32_t width_us, uint32_t narrow_ref) {
    if (narrow_ref == 0) return '1';
    
    float ratio = (float)width_us / (float)narrow_ref;
    
    // Code 39 standard: wide bars are 2-3x narrow bars
    // Use 1.6 as threshold (adjusted for sensor noise)
    if (ratio < 1.6f) {
        return '1';  // Narrow
    } else {
        return '2';  // Wide
    }
}

// ===== Find narrowest bar width (reference) =====
static uint32_t find_narrowest_bar(uint32_t *widths, uint8_t count) {
    uint32_t min_width = 999999;
    
    for (uint8_t i = 0; i < count; i++) {
        if (widths[i] > MIN_BAR_WIDTH_US && widths[i] < min_width) {
            min_width = widths[i];
        }
    }
    
    return (min_width == 999999) ? 0 : min_width;
}

// ===== Pattern Matching =====
static barcode_cmd_t decode_pattern(uint32_t *widths, uint8_t *colors, uint8_t count) {
    if (count < 9) {
        printf("[BARCODE] Too few bars: %d (need at least 9)\n", count);
        return BARCODE_INVALID;
    }
    
    // Find narrowest bar as reference
    uint32_t narrow_ref = find_narrowest_bar(widths, count);
    
    if (narrow_ref < MIN_BAR_WIDTH_US) {
        printf("[BARCODE] Reference too small: %lu us (noise or too fast)\n", narrow_ref);
        return BARCODE_INVALID;
    }
    
    printf("[BARCODE] Narrow reference: %lu us\n", narrow_ref);
    
    // Try to decode starting from each position (handle leading/trailing noise)
    for (uint8_t start = 0; start <= count - 9; start++) {
        // Build pattern string from 9 consecutive bars
        char captured[10] = {0};
        
        for (uint8_t i = 0; i < 9; i++) {
            captured[i] = classify_bar_width(widths[start + i], narrow_ref);
        }
        captured[9] = '\0';
        
        printf("[BARCODE] Pattern at offset %d: %s\n", start, captured);
        
        // Match against known patterns
        for (size_t i = 0; i < CODE39_TABLE_SIZE; i++) {
            if (strcmp(captured, code39_table[i].pattern) == 0) {
                printf("[BARCODE] ✓ Matched: '%c' -> %s\n", 
                       code39_table[i].character,
                       barcode_cmd_str(code39_table[i].command));
                return code39_table[i].command;
            }
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
        
        // Filter noise and valid bar widths
        if (width_us > NOISE_FILTER_US && width_us < MAX_BAR_WIDTH_US) {
            if (state == SCAN_IDLE && is_black) {
                // Start scanning on first black bar
                state = SCAN_IN_PROGRESS;
                bar_index = 0;
                printf("[BARCODE] Scan started\n");
            }
            
            if (state == SCAN_IN_PROGRESS && bar_index < MAX_BARS) {
                bar_widths[bar_index] = (uint32_t)width_us;
                bar_colors[bar_index] = last_was_black ? 1 : 0;
                bar_index++;
            }
        }
        
        last_was_black = is_black;
    }
    
    // Check for timeout (pattern complete)
    if (state == SCAN_IN_PROGRESS && (now_us - last_transition_us > PATTERN_TIMEOUT_US)) {
        printf("[BARCODE] Timeout - decoding %d bars\n", bar_index);
        
        // Print captured widths for debugging
        printf("[BARCODE] Widths (us): ");
        for (uint8_t i = 0; i < bar_index && i < 15; i++) {
            printf("%lu(%c) ", bar_widths[i], bar_colors[i] ? 'B' : 'W');
        }
        printf("\n");
        
        barcode_cmd_t cmd = decode_pattern(bar_widths, bar_colors, bar_index);
        
        if (cmd != BARCODE_INVALID && cmd != BARCODE_NONE) {
            if (result) {
                result->command = cmd;
                result->detected_time = get_absolute_time();
                result->confidence = 85;
                result->bar_count = bar_index;
                result->decoded_char = '?';  // Could enhance to return actual char
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
        case BARCODE_LEFT:    return "LEFT";
        case BARCODE_RIGHT:   return "RIGHT";
        case BARCODE_STOP:    return "STOP";
        case BARCODE_UTURN:   return "UTURN";
        case BARCODE_INVALID: return "INVALID";
        default:              return "NONE";
    }
}

// ===== Reset =====
void barcode_reset(void) {
    state = SCAN_IDLE;
    bar_index = 0;
    memset(bar_widths, 0, sizeof(bar_widths));
    memset(bar_colors, 0, sizeof(bar_colors));
    last_was_black = false;
    last_transition_us = time_us_64();
}