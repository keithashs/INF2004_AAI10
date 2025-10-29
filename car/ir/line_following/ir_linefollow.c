#include "ir_linefollow.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <math.h>

// ===== Low-pass filter for ADC readings =====
#define ADC_FILTER_ALPHA 0.85f // Reduced for faster response to drift
static float adc_filtered = 0.0f;
static bool filter_initialized = false;

// ===== Line position history for edge detection =====
#define HISTORY_SIZE 5
static uint16_t adc_history[HISTORY_SIZE] = {0};
static int history_index = 0;

// ===== Calibration values (CRITICAL: measure these on your track!) =====
// Run calibration mode to determine these values
static uint16_t cal_centered_adc = 2000;  // ADC when perfectly centered on line
static uint16_t cal_good_adc = 1650;      // ADC when slightly off-center
static uint16_t cal_edge_adc = 1200;      // ADC when at edge of line (still seeing black)
static uint16_t cal_barely_on_adc = 800; // ADC when barely touching line

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
    printf("[LINE] Calibration values: centered=%u, good=%u, edge=%u, barely=%u\n",
           cal_centered_adc, cal_good_adc, cal_edge_adc, cal_barely_on_adc);
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

// ===== IMPROVED: Get line following error with smoother analog positioning =====
line_reading_t ir_get_line_error(void) {
    line_reading_t reading;
    reading.raw_adc = ir_read_left_adc();
    
    // Detect if line is present
    reading.line_detected = (reading.raw_adc >= IR_BLACK_THRESHOLD);
    
    if (!reading.line_detected) {
        // Line completely lost - maximum correction to recover
        // Positive error = drifted left, need to turn right
        reading.error = +3.0f;
        reading.confidence = 0.0f;
        return reading;
    }
    
    // ===== Smoother analog positioning with non-linear mapping =====
    // Map ADC intensity to position error using calibrated thresholds
    
    if (reading.raw_adc >= cal_centered_adc) {
        // ZONE 1: Well-centered on line (ADC >= 2000)
        // Small or zero error - car is tracking well
        reading.error = 0.0f;
        reading.confidence = 0.5f;
    } 
    else if (reading.raw_adc >= cal_good_adc) {
        // ZONE 2: Slightly off-center (1650 ≤ ADC < 2000)
        // Small correction needed - linear ramp 2000→1650 maps to error 0.0→0.5
        float ratio = (float)(cal_centered_adc - reading.raw_adc) / 
                      (float)(cal_centered_adc - cal_good_adc);
        reading.error = 0.5f * ratio;  // 0.0 to 0.5
        reading.confidence = 0.95f;
    } 
    else if (reading.raw_adc >= cal_edge_adc) {
        // ZONE 3: Near edge of line (1200 ≤ ADC < 1650)
        // Moderate correction - linear ramp 1650→1200 maps to error 0.5→1.3
        float ratio = (float)(cal_good_adc - reading.raw_adc) / 
                      (float)(cal_good_adc - cal_edge_adc);
        reading.error = 0.5f + (0.8f * ratio);
        reading.confidence = 0.80f;
    } 
    else if (reading.raw_adc >= cal_barely_on_adc) {
        // ZONE 4: Barely on line (800 ≤ ADC < 1200)
        // Strong correction needed - linear ramp 1200→800 maps to error 1.3→2.2
        float ratio = (float)(cal_edge_adc - reading.raw_adc) / 
                      (float)(cal_edge_adc - cal_barely_on_adc);
        reading.error = 1.3f + (0.9f * ratio);
        reading.confidence = 0.60f;
    } 
    else {
        // ZONE 5: Just barely detecting line (ADC < 1000)
        // Maximum correction - approaching line loss 1000→threshold maps to error 2.2→3.0
        float ratio = (float)(cal_barely_on_adc - reading.raw_adc) / 
                      (float)(cal_barely_on_adc - IR_BLACK_THRESHOLD);
        reading.error = 2.2f + (0.8f * ratio);  // 2.2 to 3.0
        reading.confidence = 0.40f;
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