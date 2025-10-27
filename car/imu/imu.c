// imu.c - Enhanced with heading supervisor
#include "imu.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include "config.h"
#include "pid/pid.h"

static float heading_filt = 0.0f;
static bool  filt_init = false;

// Runtime mag offsets/scales
static float mag_off_x = MAG_OFF_X, mag_off_y = MAG_OFF_Y, mag_off_z = MAG_OFF_Z;
static float mag_scl_x = MAG_SCL_X, mag_scl_y = MAG_SCL_Y, mag_scl_z = MAG_SCL_Z;

// Calibration state
static bool  cal_active = false;
static float raw_min_x, raw_min_y, raw_min_z;
static float raw_max_x, raw_max_y, raw_max_z;

// ===== NEW: Heading Supervisor State =====
typedef struct {
    float last_heading_deg;
    absolute_time_t last_t;
    float head_weight;          // 0..1 multiplier on IMU bias
    bool  initialized;
} head_sup_t;

static head_sup_t HS = {0};

// ===== Telemetry Cache =====
volatile imu_state_t g_imu_last = {0};
volatile float       g_heading_err_deg = 0.0f;
volatile float       g_bias_cps        = 0.0f;
volatile bool        g_imu_ok          = false;
volatile float       g_head_weight     = 0.0f;

// I2C helpers
static int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
}

static int i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t* dst, size_t n) {
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    return i2c_read_blocking(I2C_PORT, addr, dst, n, false);
}

static inline bool read_with_retry(uint8_t addr, uint8_t reg, uint8_t* dst, size_t n) {
    if (i2c_read_regs(addr, reg, dst, n) >= 0) return true;
    sleep_us(200);
    return i2c_read_regs(addr, reg, dst, n) >= 0;
}

// Math helpers
static inline float rad2deg(float r) { return r * (180.0f / PI_F); }

static inline float deg_wrap_360(float d) {
    while (d < 0.0f) d += 360.0f;
    while (d >= 360.0f) d -= 360.0f;
    return d;
}

static inline float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static inline float lpf(float prev, float x, float alpha) {
    return alpha * prev + (1.0f - alpha) * x;
}

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// API Implementation
bool imu_init(void) {
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // ACC: 100 Hz, all axes on
    if (i2c_write_reg(ACC_ADDR, ACC_REG_CTRL1_A, 0x57) < 0) return false;
    if (i2c_write_reg(ACC_ADDR, ACC_REG_CTRL4_A, 0x00) < 0) return false;

    // MAG: 30Hz, ±1.3 Gauss, continuous
    if (i2c_write_reg(MAG_ADDR, 0x00, 0x14) < 0) return false;
    if (i2c_write_reg(MAG_ADDR, 0x01, 0x20) < 0) return false;
    if (i2c_write_reg(MAG_ADDR, 0x02, 0x00) < 0) return false;

    filt_init = false;
    return true;
}

bool imu_read(imu_state_t* out) {
    if (!out) return false;

    // ACC read
    uint8_t acc_raw[6];
    if (!read_with_retry(ACC_ADDR, ACC_REG_OUT_X_L | 0x80, acc_raw, 6)) { 
        out->ok = false; 
        return false; 
    }
    
    int16_t ax = (int16_t)((acc_raw[1] << 8) | acc_raw[0]);
    int16_t ay = (int16_t)((acc_raw[3] << 8) | acc_raw[2]);
    int16_t az = (int16_t)((acc_raw[5] << 8) | acc_raw[4]);

    out->ax = ax / 16384.0f;
    out->ay = ay / 16384.0f;
    out->az = az / 16384.0f;

    // MAG read
    uint8_t mag_raw[6];
    if (!read_with_retry(MAG_ADDR, 0x03, mag_raw, 6)) { 
        out->ok = false; 
        return false; 
    }
    
    int16_t mx_raw = (int16_t)((mag_raw[0] << 8) | mag_raw[1]);
    int16_t mz_raw = (int16_t)((mag_raw[2] << 8) | mag_raw[3]);
    int16_t my_raw = (int16_t)((mag_raw[4] << 8) | mag_raw[5]);

    // Track RAW min/max during calibration
    if (cal_active) {
        if (mx_raw < raw_min_x) raw_min_x = mx_raw;
        if (my_raw < raw_min_y) raw_min_y = my_raw;
        if (mz_raw < raw_min_z) raw_min_z = mz_raw;
        if (mx_raw > raw_max_x) raw_max_x = mx_raw;
        if (my_raw > raw_max_y) raw_max_y = my_raw;
        if (mz_raw > raw_max_z) raw_max_z = mz_raw;
    }

    // Apply runtime offset/scale
    float mx = (mx_raw - mag_off_x) * mag_scl_x;
    float my = (my_raw - mag_off_y) * mag_scl_y;
    float mz = (mz_raw - mag_off_z) * mag_scl_z;

    out->mx = mx; 
    out->my = my; 
    out->mz = mz;

    // Orientation from acc
    float roll  = atan2f(out->ay, out->az);
    float pitch = atan2f(-out->ax, sqrtf(out->ay * out->ay + out->az * out->az));

    out->roll_deg  = rad2deg(roll);
    out->pitch_deg = rad2deg(pitch);

    // Tilt-compensated heading
    float mx_c = mx * cosf(pitch) + mz * sinf(pitch);
    float my_c = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);
    float heading = rad2deg(atan2f(my_c, mx_c));
    heading = deg_wrap_360(heading);

    out->heading_deg = heading;

    if (!filt_init) { 
        heading_filt = heading; 
        filt_init = true; 
    }
    
    heading_filt = lpf(heading_filt, heading, HEADING_ALPHA);
    out->heading_deg_filt = heading_filt;

    out->ok = true;
    return true;
}

void imu_reset_heading_filter(float init_heading_deg) {
    heading_filt = init_heading_deg;
    filt_init = true;
}

void imu_cal_begin(void) {
    cal_active = true;
    raw_min_x = raw_min_y = raw_min_z =  1e9f;
    raw_max_x = raw_max_y = raw_max_z = -1e9f;
}

void imu_cal_end(void) {
    if (!cal_active) return;
    cal_active = false;

    // Compute offsets/scales from RAW min/max
    float off_x = 0.5f * (raw_max_x + raw_min_x);
    float off_y = 0.5f * (raw_max_y + raw_min_y);
    float off_z = 0.5f * (raw_max_z + raw_min_z);

    float r_x = 0.5f * (raw_max_x - raw_min_x);
    float r_y = 0.5f * (raw_max_y - raw_min_y);
    float r_z = 0.5f * (raw_max_z - raw_min_z);

    const float R_MIN = 50.0f;
    if (r_x < R_MIN) r_x = R_MIN;
    if (r_y < R_MIN) r_y = R_MIN;
    if (r_z < R_MIN) r_z = R_MIN;

    float r_avg = (r_x + r_y + r_z) / 3.0f;

    mag_off_x = off_x; 
    mag_off_y = off_y; 
    mag_off_z = off_z;
    mag_scl_x = r_avg / r_x;
    mag_scl_y = r_avg / r_y;
    mag_scl_z = r_avg / r_z;
}

// ===== NEW: Heading Supervisor Functions =====

void imu_reset_heading_supervisor(void) {
    HS.initialized = false;
    HS.head_weight = 0.0f;
    HS.last_heading_deg = 0.0f;
    g_head_weight = 0.0f;
}

float imu_get_heading_bias(float target_heading_deg, float dt) {
    imu_state_t s;
    bool imu_ok = imu_read(&s) && s.ok;

    if (!imu_ok) {
        // IMU not available: ramp weight down
        float a_dn = clampf(dt / 0.1f, 0.0f, 1.0f);
        HS.head_weight = HS.head_weight + (0.0f - HS.head_weight) * a_dn;
        HS.head_weight = clampf(HS.head_weight, 0.0f, 1.0f);
        g_head_weight  = HS.head_weight;
        g_imu_ok = false;
        return 0.0f;
    }

    // Compute heading error
    float err_deg = wrap180(s.heading_deg_filt - target_heading_deg);

    // Deadband
    if (fabsf(err_deg) < 2.0f) err_deg = 0.0f;

    // Health checks
    bool tilt_ok = (fabsf(s.roll_deg) <= 10.0f) && (fabsf(s.pitch_deg) <= 10.0f);

    bool rate_ok = true;
    if (!HS.initialized) {
        HS.last_heading_deg = s.heading_deg_filt;
        HS.last_t = get_absolute_time();
        HS.head_weight = 0.0f;
        HS.initialized = true;
    } else {
        absolute_time_t now = get_absolute_time();
        float dt_s = absolute_time_diff_us(HS.last_t, now) / 1e6f;
        if (dt_s > 0.0005f) {
            float d = wrap180(s.heading_deg_filt - HS.last_heading_deg);
            float rate = fabsf(d) / dt_s;
            rate_ok = (rate <= 30.0f);
            HS.last_heading_deg = s.heading_deg_filt;
            HS.last_t = now;
        }
    }

    bool healthy = tilt_ok && rate_ok;

    // Smooth weight adjustment
    float tau_up = 0.35f;
    float tau_dn = 0.10f;
    float a_up = clampf(dt / tau_up, 0.0f, 1.0f);
    float a_dn = clampf(dt / tau_dn, 0.0f, 1.0f);

    if (healthy) {
        HS.head_weight = HS.head_weight + (1.0f - HS.head_weight) * a_up;
    } else {
        HS.head_weight = HS.head_weight + (0.0f - HS.head_weight) * a_dn;
    }

    HS.head_weight = clampf(HS.head_weight, 0.0f, 1.0f);
    g_head_weight  = HS.head_weight;

    // IMU PID with weight
    extern PID pid_heading;
    float raw_bias_head = pid_step(&pid_heading, 0.0f, -err_deg, dt);
    float bias_head = raw_bias_head * HS.head_weight;

    // Update telemetry cache
    g_imu_last = s;
    g_heading_err_deg = err_deg;
    g_bias_cps = bias_head;
    g_imu_ok = true;

    return bias_head;
}

float imu_get_supervisor_weight(void) {
    return g_head_weight;
}