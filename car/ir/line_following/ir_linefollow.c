#include "ir_linefollow.h"
#include "hardware/adc.h"
#include <stdio.h>

// ===== Initialize ADC for line sensor =====
void ir_line_init(void) {
    adc_init();
    adc_gpio_init(IR_LEFT_ADC);
    printf("[LINE] Initialized on GP%d (ADC2)\n", IR_LEFT_ADC);
}

// ===== Read raw ADC value =====
uint16_t ir_read_left_adc(void) {
    adc_select_input(2);  // ADC2 = GP28
    return adc_read();
}

// ===== Binary black detection =====
bool ir_left_is_black(void) {
    return (ir_read_left_adc() > IR_BLACK_THRESHOLD);
}

// ===== Get line following error with analog positioning =====
line_reading_t ir_get_line_error(void) {
    line_reading_t reading;
    reading.raw_adc = ir_read_left_adc();
    
    // Detect if line is present
    reading.line_detected = (reading.raw_adc > IR_BLACK_THRESHOLD);
    
    if (!reading.line_detected) {
        // Line lost - major correction needed
        reading.error = +2.5f;  // Assume drifted left, turn right aggressively
        return reading;
    }
    
    // ===== Analog positioning within black line =====
    // Map ADC value to position error
    // Higher ADC = more centered, Lower ADC = edge of line
    
    if (reading.raw_adc > 2800) {
        // Well-centered on line
        reading.error = 0.0f;
    } else if (reading.raw_adc > 2400) {
        // Slightly off-center (left edge)
        reading.error = +0.5f;  // Gentle correction right
    } else if (reading.raw_adc > 2000) {
        // Near edge of line
        reading.error = +1.2f;  // Moderate correction right
    } else {
        // Barely on line
        reading.error = +2.0f;  // Strong correction right
    }
    
    return reading;
}

// ===== String conversion for telemetry =====
const char* ir_line_position_str(line_position_t pos) {
    switch (pos) {
        case LINE_CENTER:   return "CENTER";
        case LINE_LEFT:     return "LEFT";
        case LINE_RIGHT:    return "RIGHT";
        case LINE_BOTH_OFF: return "LOST";
        case LINE_LOST:     return "LOST";
        default:            return "UNKNOWN";
    }
}