#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float ax, ay, az;     // g
    float mx, my, mz;     // raw mag units
    float roll_deg;
    float pitch_deg;
    float heading_deg;        // raw (tilt-compensated)
    float heading_deg_filt;   // low-pass filtered
    bool  ok;
} imu_state_t;

bool imu_init(void);
bool imu_read(imu_state_t* out);
void imu_reset_heading_filter(float init_heading_deg);

// Mag calibration helpers
void imu_cal_begin(void);
void imu_cal_end(void);

// Heading Supervisor & Bias Calculation

// Reset the heading supervisor state (call on START or after calibration)
void imu_reset_heading_supervisor(void);

// Get IMU heading bias with supervisor gating
// Returns weighted bias in CPS units, automatically handles:
// - Heading error calculation
// - Tilt and rate health checks
// - Smooth weight ramping (0..1)
// - PID calculation with deadband
float imu_get_heading_bias(float target_heading_deg, float dt);

// Get current supervisor health weight (0..1) for telemetry
float imu_get_supervisor_weight(void);

// --- Telemetry shared with motor loop ---
extern volatile imu_state_t g_imu_last;
extern volatile float       g_heading_err_deg;
extern volatile float       g_bias_cps;
extern volatile bool        g_imu_ok;
extern volatile float       g_head_weight;