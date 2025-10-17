#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float ax, ay, az;     // g
    float mx, my, mz;     // uT (relative units)
    float roll_deg;
    float pitch_deg;
    float heading_deg;        // raw (tilt-compensated)
    float heading_deg_filt;   // low-pass filtered
    bool  ok;
} imu_state_t;

bool imu_init(void);
bool imu_read(imu_state_t* out);         // reads acc+mag, computes R/P/heading and filter
void imu_reset_heading_filter(float init_heading_deg);
