#include "ir_linefollow.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <math.h>

// ===== Low-pass filter for ADC readings =====
#define ADC_FILTER_ALPHA 0.85f // lower = more smoothing and responsive
static float adc_filtered = 0.0f;
static bool filter_initialized = false;

// ===== Line position history for edge detection =====
#define HISTORY_SIZE 5
static uint16_t adc_history[HISTORY_SIZE] = {0};
static int history_index = 0;

// ===== Initialize ADC for line sensor =====
void ir_line_init(void) {
    adc_init();
    adc_gpio_init(IR_LEFT_ADC);
    
    // Initialize filter with first reading
    adc_select_input(2);  // ADC2 = GP28
    uint16_t initial = adc_read();
    adc_filtered = (float)initial;
    filter_initialized = true;
    
    // Initialize history
    for (int i = 0; i < HISTORY_SIZE; i++) {
        adc_history[i] = initial;
    }
    
    printf("[LINE] Initialized on GP%d (ADC2), initial reading: %u\n", IR_LEFT_ADC, initial);
}

// ===== Read raw ADC value =====
uint16_t ir_read_left_adc(void) {
    adc_select_input(2);  // ADC2 = GP28
    uint16_t raw = adc_read();
    
    // Apply low-pass filter to reduce noise
    if (filter_initialized) {
        adc_filtered = ADC_FILTER_ALPHA * adc_filtered + (1.0f - ADC_FILTER_ALPHA) * (float)raw;
    } else {
        adc_filtered = (float)raw;
        filter_initialized = true;
    }
    
    // Update history
    adc_history[history_index] = raw;
    history_index = (history_index + 1) % HISTORY_SIZE;
    
    return (uint16_t)adc_filtered;
}

// ===== Binary black detection =====
bool ir_left_is_black(void) {
    return (ir_read_left_adc() >= IR_BLACK_THRESHOLD);
}

// ===== Get line following error with analog positioning =====
line_reading_t ir_get_line_error(void) {
    line_reading_t reading;
    reading.raw_adc = ir_read_left_adc();
    
    // Detect if line is present
    reading.line_detected = (reading.raw_adc > IR_BLACK_THRESHOLD);
    
    if (!reading.line_detected) {
        // Line completely lost - maximum correction
        // Assume drifted left (most common), turn right aggressively
        reading.error = +3.0f;
        reading.confidence = 0.0f;
        return reading;
    }
    
    // ===== Analog positioning within black line =====
    // For 1.7cm thin line, we map ADC intensity to position error
    // Higher ADC = more centered, Lower ADC = edge of line
    
    // Define ADC ranges for different positions
    // These values are tuned for your specific sensor and line
    const uint16_t CENTERED_MIN = 2500;   // Very centered
    const uint16_t GOOD_MIN = 2300;       // Slightly off-center
    const uint16_t EDGE_MIN = 1200;       // Near edge
    // Below 2000 = barely on line
    
    if (reading.raw_adc >= CENTERED_MIN) {
        // Well-centered on line
        reading.error = 0.0f;
        reading.confidence = 1.0f;
    } 
    else if (reading.raw_adc >= GOOD_MIN) {
        // Slightly off-center (left edge visible)
        // Linear interpolation between 0.0 and 0.8
        float ratio = (float)(CENTERED_MIN - reading.raw_adc) / (float)(CENTERED_MIN - GOOD_MIN);
        reading.error = 0.8f * ratio;
        reading.confidence = 0.9f;
    } 
    else if (reading.raw_adc >= EDGE_MIN) {
        // Near edge of line - moderate correction
        float ratio = (float)(GOOD_MIN - reading.raw_adc) / (float)(GOOD_MIN - EDGE_MIN);
        reading.error = 0.8f + 1.2f * ratio;  // 0.8 to 2.0
        reading.confidence = 0.7f;
    } 
    else {
        // Barely on line - strong correction needed
        float ratio = (float)(EDGE_MIN - reading.raw_adc) / (float)(EDGE_MIN - IR_BLACK_THRESHOLD);
        reading.error = 2.0f + 1.0f * ratio;  // 2.0 to 3.0
        reading.confidence = 0.5f;
    }
    
    return reading;
}

// ===== Reset internal state =====
void ir_line_reset_state(void) {
    filter_initialized = false;
    history_index = 0;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        adc_history[i] = 0;
    }
}