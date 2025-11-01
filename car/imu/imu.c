#include "imu.h"
#include <stdio.h>
#include <math.h>

// ===== ACC registers =====
#define ACC_REG_CTRL1_A    0x20
#define ACC_REG_CTRL4_A    0x23
#define ACC_REG_OUT_X_L_A  0x28  // auto-inc with bit7=1

// ===== MAG registers =====
#define MAG_REG_CRA_M      0x00
#define MAG_REG_CRB_M      0x01
#define MAG_REG_MR_M       0x02
#define MAG_REG_OUT_X_H_M  0x03  // order: X_H,X_L, Z_H,Z_L, Y_H,Y_L

static int wr_then_rd(i2c_inst_t *i2c, uint8_t addr,
                      const uint8_t *w, size_t wn, uint8_t *r, size_t rn) {
    int wr = i2c_write_blocking(i2c, addr, w, wn, rn > 0);
    if (wr != (int)wn) return PICO_ERROR_GENERIC;
    if (!rn) return wr;
    int rr = i2c_read_blocking(i2c, addr, r, rn, false);
    return (rr == (int)rn) ? rr : PICO_ERROR_GENERIC;
}

static bool acc_init(i2c_inst_t *i2c) {
    uint8_t w1[2] = { ACC_REG_CTRL1_A, 0x57 }; // 100 Hz, all axes ON
    uint8_t w4[2] = { ACC_REG_CTRL4_A, 0x88 }; // BDU=1, HR=1, ±2g
    if (i2c_write_blocking(i2c, IMU_ACC_ADDR, w1, 2, false) != 2) return false;
    if (i2c_write_blocking(i2c, IMU_ACC_ADDR, w4, 2, false) != 2) return false;
    return true;
}

static bool mag_init(i2c_inst_t *i2c) {
    uint8_t cra[2] = { MAG_REG_CRA_M, 0x10 }; // 15 Hz
    uint8_t crb[2] = { MAG_REG_CRB_M, 0x20 }; // gain 1.3 Gauss
    uint8_t mr [2] = { MAG_REG_MR_M,  0x00 }; // continuous
    if (i2c_write_blocking(i2c, IMU_MAG_ADDR, cra, 2, false) != 2) return false;
    if (i2c_write_blocking(i2c, IMU_MAG_ADDR, crb, 2, false) != 2) return false;
    if (i2c_write_blocking(i2c, IMU_MAG_ADDR, mr,  2, false) != 2) return false;
    return true;
}

bool imu_init(imu_t *imu) {
    if (!imu) return false;

    i2c_init(imu->i2c, imu->i2c_baud);
    gpio_set_function(imu->pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(imu->pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(imu->pin_sda);
    gpio_pull_up(imu->pin_scl);

    imu->inited_acc = acc_init(imu->i2c);
    imu->inited_mag = mag_init(imu->i2c);
    imu->mx_off = imu->my_off = imu->mz_off = 0.f;
    imu->heading_deg = 0.f;

    return imu->inited_acc && imu->inited_mag;
}

// void imu_calibrate_mag(imu_t *imu) {
//     if (!imu || !imu->inited_mag) return;
    
//     printf("[IMU] Magnetometer calibration - rotate car 360 degrees slowly...\n");
//     sleep_ms(2000);
    
//     float mx_min = 1e9f, mx_max = -1e9f;
//     float my_min = 1e9f, my_max = -1e9f;
//     float mz_min = 1e9f, mz_max = -1e9f;
    
//     const int samples = 200;
//     for (int i = 0; i < samples; i++) {
//         float mx, my, mz;
//         if (imu_read_mag_raw(imu, &mx, &my, &mz)) {
//             if (mx < mx_min) mx_min = mx;
//             if (mx > mx_max) mx_max = mx;
//             if (my < my_min) my_min = my;
//             if (my > my_max) my_max = my;
//             if (mz < mz_min) mz_min = mz;
//             if (mz > mz_max) mz_max = mz;
            
//             if (i % 20 == 0) {
//                 printf("[IMU] Cal %d/%d: mx[%.0f,%.0f] my[%.0f,%.0f] mz[%.0f,%.0f]\n",
//                        i, samples, mx_min, mx_max, my_min, my_max, mz_min, mz_max);
//             }
//         }
//         sleep_ms(50);
//     }
    
//     // Hard-iron offset = midpoint of min/max
//     imu->mx_off = (mx_max + mx_min) / 2.0f;
//     imu->my_off = (my_max + my_min) / 2.0f;
//     imu->mz_off = (mz_max + mz_min) / 2.0f;
    
//     printf("[IMU] Calibration complete!\n");
//     printf("[IMU] Offsets: mx=%.1f, my=%.1f, mz=%.1f\n", 
//            imu->mx_off, imu->my_off, imu->mz_off);
// }

bool imu_read_accel_g(imu_t *imu, float *ax, float *ay, float *az) {
    if (!imu || !imu->inited_acc) return false;
    uint8_t reg = ACC_REG_OUT_X_L_A | 0x80; // auto-inc
    uint8_t b[6];
    if (wr_then_rd(imu->i2c, IMU_ACC_ADDR, &reg, 1, b, 6) < 0) return false;

    int16_t x16 = (int16_t)((b[1] << 8) | b[0]);
    int16_t y16 = (int16_t)((b[3] << 8) | b[2]);
    int16_t z16 = (int16_t)((b[5] << 8) | b[4]);
    int16_t x12 = x16 >> 4, y12 = y16 >> 4, z12 = z16 >> 4;

    if (ax) *ax = x12 * 0.001f;
    if (ay) *ay = y12 * 0.001f;
    if (az) *az = z12 * 0.001f;
    return true;
}

bool imu_read_mag_raw(imu_t *imu, float *mx, float *my, float *mz) {
    if (!imu || !imu->inited_mag) return false;
    uint8_t reg = MAG_REG_OUT_X_H_M;
    uint8_t b[6];
    if (wr_then_rd(imu->i2c, IMU_MAG_ADDR, &reg, 1, b, 6) < 0) return false;

    int16_t x = (int16_t)((b[0] << 8) | b[1]);
    int16_t z = (int16_t)((b[2] << 8) | b[3]);
    int16_t y = (int16_t)((b[4] << 8) | b[5]);

    if (mx) *mx = (float)x;
    if (my) *my = (float)y;
    if (mz) *mz = (float)z;
    return true;
}

float imu_compute_heading_deg(imu_t *imu, float ax, float ay, float az,
                              float mx, float my, float mz) {
    float na = sqrtf(ax*ax + ay*ay + az*az);
    if (na < 1e-6f) return imu ? imu->heading_deg : 0.f;
    ax /= na; ay /= na; az /= na;

    if (imu) { mx -= imu->mx_off; my -= imu->my_off; mz -= imu->mz_off; }

    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    float mx2 = mx * cosf(pitch) + mz * sinf(pitch);
    float my2 = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

    // Calculate heading using corrected atan2 arguments
    float hdg = atan2f(mx2, my2) * 180.0f / (float)M_PI;

    // INVERT heading to compensate for IMU mounting orientation
    // This makes clockwise physical rotation increase heading
    hdg = -hdg;

    // Normalize to 0-360 range
    while (hdg < 0.0f) hdg += 360.0f;
    while (hdg >= 360.0f) hdg -= 360.0f;
    
    if (imu) imu->heading_deg = hdg;
    return hdg;
}

float imu_update_and_get_heading(imu_t *imu) {
    float ax, ay, az, mx, my, mz;
    if (!imu_read_accel_g(imu, &ax, &ay, &az)) return imu_get_heading(imu);
    if (!imu_read_mag_raw(imu, &mx, &my, &mz)) return imu_get_heading(imu);
    return imu_compute_heading_deg(imu, ax, ay, az, mx, my, mz);
}
