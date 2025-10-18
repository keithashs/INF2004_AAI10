// imu.c
#include "imu.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include "config.h"

static float heading_filt = 0.0f;
static bool  filt_init = false;

// runtime mag offsets/scales (initialized from config.h)
static float mag_off_x = MAG_OFF_X, mag_off_y = MAG_OFF_Y, mag_off_z = MAG_OFF_Z;
static float mag_scl_x = MAG_SCL_X, mag_scl_y = MAG_SCL_Y, mag_scl_z = MAG_SCL_Z;

// min/max capture for on-the-fly calibration
static bool  cal_active = false;
static float mag_min_x, mag_min_y, mag_min_z;
static float mag_max_x, mag_max_y, mag_max_z;

// RAW min/max used to compute offsets/scales (the correct data source)
static float raw_min_x, raw_min_y, raw_min_z;
static float raw_max_x, raw_max_y, raw_max_z;

static int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
}

static int i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t* dst, size_t n) {
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    return i2c_read_blocking(I2C_PORT, addr, dst, n, false);
}

// --- retry wrapper ---
static inline bool read_with_retry(uint8_t addr, uint8_t reg, uint8_t* dst, size_t n) {
    if (i2c_read_regs(addr, reg, dst, n) >= 0) return true;
    sleep_us(200); // brief pause
    return i2c_read_regs(addr, reg, dst, n) >= 0;
}

bool imu_init(void) {
    // I2C pins
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // ACC: 100 Hz, all axes on
    if (i2c_write_reg(ACC_ADDR, ACC_REG_CTRL1_A, 0x57) < 0) return false; // ODR=100Hz, all axes enable
    if (i2c_write_reg(ACC_ADDR, ACC_REG_CTRL4_A, 0x00) < 0) return false; // ±2g, normal mode

    // MAG: CRA=30Hz, CRB=gain=±1.3 Gauss, MR=continuous
    if (i2c_write_reg(MAG_ADDR, 0x00, 0x14) < 0) return false; // CRA_REG_M
    if (i2c_write_reg(MAG_ADDR, 0x01, 0x20) < 0) return false; // CRB_REG_M
    if (i2c_write_reg(MAG_ADDR, 0x02, 0x00) < 0) return false; // MR_REG_M

    filt_init = false;
    return true;
}

// Helpers
static inline float rad2deg(float r) { return r * (180.0f / PI_F); }
static inline float deg_wrap_360(float d) {
    while (d < 0.0f) d += 360.0f;
    while (d >= 360.0f) d -= 360.0f;
    return d;
}
static inline float lpf(float prev, float x, float alpha) {
    return alpha * prev + (1.0f - alpha) * x;
}

bool imu_read(imu_state_t* out) {
    if (!out) return false;

    // -------- ACC read (6 bytes, auto-increment starting at 0x28) --------
    uint8_t acc_raw[6];
    if (!read_with_retry(ACC_ADDR, ACC_REG_OUT_X_L | 0x80, acc_raw, 6)) { out->ok=false; return false; }
    int16_t ax = (int16_t)((acc_raw[1] << 8) | acc_raw[0]);
    int16_t ay = (int16_t)((acc_raw[3] << 8) | acc_raw[2]);
    int16_t az = (int16_t)((acc_raw[5] << 8) | acc_raw[4]);

    // LSM303DLHC acc approx scale at ±2g
    out->ax = ax / 16384.0f;
    out->ay = ay / 16384.0f;
    out->az = az / 16384.0f;

    // -------- MAG read (X,Z,Y order, 6 bytes starting at 0x03) --------
    uint8_t mag_raw[6];
    if (!read_with_retry(MAG_ADDR, 0x03, mag_raw, 6)) { out->ok=false; return false; }
    int16_t mx_raw = (int16_t)((mag_raw[0] << 8) | mag_raw[1]);
    int16_t mz_raw = (int16_t)((mag_raw[2] << 8) | mag_raw[3]);
    int16_t my_raw = (int16_t)((mag_raw[4] << 8) | mag_raw[5]);

    // track RAW min/max during calibration (before corrections)
    if (cal_active) {
        if (mx_raw < raw_min_x) raw_min_x = mx_raw;
        if (my_raw < raw_min_y) raw_min_y = my_raw;
        if (mz_raw < raw_min_z) raw_min_z = mz_raw;
        if (mx_raw > raw_max_x) raw_max_x = mx_raw;
        if (my_raw > raw_max_y) raw_max_y = my_raw;
        if (mz_raw > raw_max_z) raw_max_z = mz_raw;
    }

    // Apply runtime offset/scale (updated by calibration)
    float mx = (mx_raw - mag_off_x) * mag_scl_x;
    float my = (my_raw - mag_off_y) * mag_scl_y;
    float mz = (mz_raw - mag_off_z) * mag_scl_z;

    out->mx = mx; out->my = my; out->mz = mz;

    // (optional) keep corrected min/max (useful if you later want to log/show coverage)
    if (cal_active) {
        if (mx < mag_min_x) mag_min_x = mx;
        if (my < mag_min_y) mag_min_y = my;
        if (mz < mag_min_z) mag_min_z = mz;
        if (mx > mag_max_x) mag_max_x = mx;
        if (my > mag_max_y) mag_max_y = my;
        if (mz > mag_max_z) mag_max_z = mz;
    }

    // -------- Orientation from acc --------
    float roll  = atan2f(out->ay, out->az);
    float pitch = atan2f(-out->ax, sqrtf(out->ay*out->ay + out->az*out->az));

    out->roll_deg  = rad2deg(roll);
    out->pitch_deg = rad2deg(pitch);

    // -------- Tilt-compensated heading from mag + acc --------
    float mx_c = mx * cosf(pitch) + mz * sinf(pitch);
    float my_c = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);
    float heading = rad2deg(atan2f(my_c, mx_c));
    heading = deg_wrap_360(heading);

    out->heading_deg = heading;

    if (!filt_init) { heading_filt = heading; filt_init = true; }
    heading_filt = lpf(heading_filt, heading, HEADING_ALPHA);
    out->heading_deg_filt = heading_filt;

    out->ok = true;
    return true;
}

void imu_reset_heading_filter(float init_heading_deg) {
    heading_filt = init_heading_deg;
    filt_init = true;
}

// ---- Simple mag calibration (min/max hard-iron + soft scale) ----
void imu_cal_begin(void) {
    cal_active = true;
    mag_min_x = mag_min_y = mag_min_z =  1e9f;
    mag_max_x = mag_max_y = mag_max_z = -1e9f;

    // RAW min/max are used for actual calibration math
    raw_min_x = raw_min_y = raw_min_z =  1e9f;
    raw_max_x = raw_max_y = raw_max_z = -1e9f;
}

void imu_cal_feed(const imu_state_t* s) {
    (void)s; // min/max updated inside imu_read while cal_active
}

void imu_cal_end(void) {
    if (!cal_active) return;
    cal_active = false;

    // compute offsets/scales from RAW min/max
    float off_x = 0.5f * (raw_max_x + raw_min_x);
    float off_y = 0.5f * (raw_max_y + raw_min_y);
    float off_z = 0.5f * (raw_max_z + raw_min_z);

    float r_x = 0.5f * (raw_max_x - raw_min_x);
    float r_y = 0.5f * (raw_max_y - raw_min_y);
    float r_z = 0.5f * (raw_max_z - raw_min_z);

    // clamp tiny radii to avoid massive scales if coverage was poor
    const float R_MIN = 50.0f; // tune to your sensor's raw counts
    if (r_x < R_MIN) r_x = R_MIN;
    if (r_y < R_MIN) r_y = R_MIN;
    if (r_z < R_MIN) r_z = R_MIN;

    float r_avg = (r_x + r_y + r_z) / 3.0f;

    mag_off_x = off_x; mag_off_y = off_y; mag_off_z = off_z;
    mag_scl_x = r_avg / r_x;
    mag_scl_y = r_avg / r_y;
    mag_scl_z = r_avg / r_z;
}
