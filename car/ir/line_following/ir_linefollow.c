#include "ir_linefollow.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <math.h>

// ===== Low-pass filter for ADC readings =====
#define ADC_FILTER_ALPHA 0.75f // Reduced for faster response to drift
static float adc_filtered = 0.0f;
static bool filter_initialized = false;

// ===== Line position history for edge detection =====
#define HISTORY_SIZE 5
static uint16_t adc_history[HISTORY_SIZE] = {0};
static int history_index = 0;

// ===== Calibration values (CRITICAL: measure these on your track!) =====
// Run calibration mode to determine these values
static uint16_t cal_centered_adc = 2800;  // ADC when perfectly centered on line
static uint16_t cal_good_adc = 2300;      // ADC when slightly off-center
static uint16_t cal_edge_adc = 1800;      // ADC when at edge of line (still seeing black)
static uint16_t cal_barely_on_adc = 1200; // ADC when barely touching line

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
    reading.line_detected = (reading.raw_adc > IR_BLACK_THRESHOLD);
    
    if (!reading.line_detected) {
        // Line completely lost - maximum correction to recover
        // Positive error = drifted left, need to turn right
        reading.error = +3.0f;
        reading.confidence = 0.0f;
        return reading;
    }
    
    // ===== IMPROVED: Smoother analog positioning with non-linear mapping =====
    // Map ADC intensity to position error using calibrated thresholds
    
    if (reading.raw_adc >= cal_centered_adc) {
        // ZONE 1: Well-centered on line (ADC >= 2800)
        // Small or zero error - car is tracking well
        reading.error = 0.0f;
        reading.confidence = 1.0f;
    } 
    else if (reading.raw_adc >= cal_good_adc) {
        // ZONE 2: Slightly off-center (2300 <= ADC < 2800)
        // Small correction needed - linear ramp from 0 to 0.5
        float ratio = (float)(cal_centered_adc - reading.raw_adc) / 
                      (float)(cal_centered_adc - cal_good_adc);
        reading.error = 0.5f * ratio;  // 0.0 to 0.5
        reading.confidence = 0.95f;
    } 
    else if (reading.raw_adc >= cal_edge_adc) {
        // ZONE 3: Near edge of line (1800 <= ADC < 2300)
        // Moderate correction - linear ramp from 0.5 to 1.3
        float ratio = (float)(cal_good_adc - reading.raw_adc) / 
                      (float)(cal_good_adc - cal_edge_adc);
        reading.error = 0.5f + 0.8f * ratio;  // 0.5 to 1.3
        reading.confidence = 0.80f;
    } 
    else if (reading.raw_adc >= cal_barely_on_adc) {
        // ZONE 4: Barely on line (1200 <= ADC < 1800)
        // Strong correction needed - linear ramp from 1.3 to 2.2
        float ratio = (float)(cal_edge_adc - reading.raw_adc) / 
                      (float)(cal_edge_adc - cal_barely_on_adc);
        reading.error = 1.3f + 0.9f * ratio;  // 1.3 to 2.2
        reading.confidence = 0.60f;
    } 
    else {
        // ZONE 5: Just barely detecting line (ADC < 1200)
        // Maximum correction - approaching line loss
        float ratio = (float)(cal_barely_on_adc - reading.raw_adc) / 
                      (float)(cal_barely_on_adc - IR_BLACK_THRESHOLD);
        reading.error = 2.2f + 0.8f * ratio;  // 2.2 to 3.0
        reading.confidence = 0.40f;
    }
    
    return reading;
}

// ===== NEW: Calibration helper function =====
void ir_line_calibrate(void) {
    printf("\n╔════════════════════════════════════════════════╗\n");
    printf("║   LINE SENSOR CALIBRATION PROCEDURE           ║\n");
    printf("╚════════════════════════════════════════════════╝\n\n");
    
    printf("This will help you determine the correct ADC thresholds.\n");
    printf("Place the car in each position and press a key:\n\n");
    
    // Position 1: Centered
    printf("1. Place car CENTERED on line, press ENTER...\n");
    getchar();
    sleep_ms(500);
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += ir_read_left_adc();
        sleep_ms(50);
    }
    cal_centered_adc = sum / 10;
    printf("   → Centered ADC: %u\n\n", cal_centered_adc);
    
    // Position 2: Slightly off (good tracking)
    printf("2. Move car SLIGHTLY RIGHT (still mostly on line), press ENTER...\n");
    getchar();
    sleep_ms(500);
    sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += ir_read_left_adc();
        sleep_ms(50);
    }
    cal_good_adc = sum / 10;
    printf("   → Slightly off ADC: %u\n\n", cal_good_adc);
    
    // Position 3: Edge of line
    printf("3. Move car to EDGE of line (left sensor barely on black), press ENTER...\n");
    getchar();
    sleep_ms(500);
    sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += ir_read_left_adc();
        sleep_ms(50);
    }
    cal_edge_adc = sum / 10;
    printf("   → Edge ADC: %u\n\n", cal_edge_adc);
    
    // Position 4: Barely on line
    printf("4. Move car FURTHER RIGHT (barely touching line), press ENTER...\n");
    getchar();
    sleep_ms(500);
    sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += ir_read_left_adc();
        sleep_ms(50);
    }
    cal_barely_on_adc = sum / 10;
    printf("   → Barely on ADC: %u\n\n", cal_barely_on_adc);
    
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║   CALIBRATION COMPLETE!                        ║\n");
    printf("╚════════════════════════════════════════════════╝\n\n");
    printf("Copy these values to your code:\n");
    printf("static uint16_t cal_centered_adc = %u;\n", cal_centered_adc);
    printf("static uint16_t cal_good_adc = %u;\n", cal_good_adc);
    printf("static uint16_t cal_edge_adc = %u;\n", cal_edge_adc);
    printf("static uint16_t cal_barely_on_adc = %u;\n\n", cal_barely_on_adc);
}

// ===== Reset internal state =====
void ir_line_reset_state(void) {
    filter_initialized = false;
    history_index = 0;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        adc_history[i] = 0;
    }
}