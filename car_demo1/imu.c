#include "imu.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include "config.h"

static float heading_filt = 0.0f;
static bool  filt_init = false;

static int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
}

static int i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t* dst, size_t n) {
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    return i2c_read_blocking(I2C_PORT, addr, dst, n, false);
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
    if (i2c_read_regs(ACC_ADDR, ACC_REG_OUT_X_L | 0x80, acc_raw, 6) < 0) { out->ok=false; return false; }
    int16_t ax = (int16_t)((acc_raw[1] << 8) | acc_raw[0]);
    int16_t ay = (int16_t)((acc_raw[3] << 8) | acc_raw[2]);
    int16_t az = (int16_t)((acc_raw[5] << 8) | acc_raw[4]);

    // LSM303DLHC acc 12-bit (left-justified in 16-bit); scale ~ 1 mg/LSB at ±2g (approx)
    out->ax = ax / 16384.0f;
    out->ay = ay / 16384.0f;
    out->az = az / 16384.0f;

    // -------- MAG read (X,Z,Y order, 6 bytes starting at 0x03) --------
    uint8_t mag_raw[6];
    if (i2c_read_regs(MAG_ADDR, 0x03, mag_raw, 6) < 0) { out->ok=false; return false; }
    int16_t mx_raw = (int16_t)((mag_raw[0] << 8) | mag_raw[1]);
    int16_t mz_raw = (int16_t)((mag_raw[2] << 8) | mag_raw[3]);
    int16_t my_raw = (int16_t)((mag_raw[4] << 8) | mag_raw[5]);

    // Simple offset/scale correction (calibrate later)
    float mx = (mx_raw - MAG_OFF_X) * MAG_SCL_X;
    float my = (my_raw - MAG_OFF_Y) * MAG_SCL_Y;
    float mz = (mz_raw - MAG_OFF_Z) * MAG_SCL_Z;

    out->mx = mx; out->my = my; out->mz = mz;

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
