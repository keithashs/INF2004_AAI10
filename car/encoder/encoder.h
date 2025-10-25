#pragma once
#include <stdint.h>
#include <stdbool.h>

// Initialize both wheel encoders and IRQ handlers
void encoder_init(void);

// Reset total counts and the window sampler
void encoder_reset_all(void);

// Return the number of ticks counted for each wheel since the last call,
// and advance the internal "window" for sampling (e.g., every 10 ms).
void encoder_sample_window(uint16_t *win_m1, uint16_t *win_m2);

// Read total tick counters (monotonic since last reset)
uint32_t encoder_total_m1(void);
uint32_t encoder_total_m2(void);
