#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ===== Board wiring defaults (edit if needed) =====
#define IMU_I2C_BAUD   115200   // you can set 400000 if your wiring is short/stable
#define IMU_SDA_PIN    2
#define IMU_SCL_PIN    3

// ===== LSM303DLHC I2C addresses =====
#define IMU_ACC_ADDR   0x19
#define IMU_MAG_ADDR   0x1E

typedef struct {
    // configuration
    i2c_inst_t *i2c;     // i2c0 or i2c1
    uint32_t    i2c_baud;
    uint8_t     pin_sda;
    uint8_t     pin_scl;

    // state
    bool        inited_acc;
    bool        inited_mag;

    // simple hard-iron offsets (counts)
    float       mx_off, my_off, mz_off;

    // last computed heading (deg 0..360)
    float       heading_deg;
} imu_t;

bool  imu_init(imu_t *imu);
bool  imu_read_accel_g(imu_t *imu, float *ax, float *ay, float *az);
bool  imu_read_mag_raw(imu_t *imu, float *mx, float *my, float *mz);
float imu_compute_heading_deg(imu_t *imu, float ax, float ay, float az,
                              float mx, float my, float mz);
float imu_update_and_get_heading(imu_t *imu);

// helpers
static inline void imu_set_mag_offsets(imu_t *imu, float mx_off, float my_off, float mz_off) {
    if (!imu) return; 
    imu->mx_off = mx_off; 
    imu->my_off = my_off; 
    imu->mz_off = mz_off;
}
static inline float imu_get_heading(imu_t *imu) { return imu ? imu->heading_deg : 0.f; }

#endif // IMU_H
