#ifndef IMU_LSM303_H
#define IMU_LSM303_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float ax, ay, az;      // g
    float mx, my, mz;      // uT (scaled approx)
    float heading_deg;     // raw computed heading (0..360)
    float heading_f_deg;   // filtered heading
    bool  mag_is_qmc;      // true if QMC5883L detected
    bool  ok_acc, ok_mag;
} imu_t;

void imu_init(imu_t* imu);
void imu_read(imu_t* imu);  // update ax.., mx.., heading

#endif
