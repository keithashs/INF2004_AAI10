#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pin.h"
#include <math.h>
#include <string.h>

static inline void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
}
static inline bool i2c_read_n(uint8_t addr, uint8_t reg, uint8_t* out, size_t n) {
    if (i2c_write_blocking(I2C_PORT, addr, &reg, 1, true) < 0) return false;
    return i2c_read_blocking(I2C_PORT, addr, out, n, false) >= 0;
}

static bool acc_init(void) {
    // LSM303DLHC accel: 0x19
    // CTRL1_A: 0b 0 1 1 1 0 1 1 1 => 50Hz, all axes on (0x57)
    i2c_write_reg(ACC_ADDR, 0x20, 0x57);
    // CTRL4_A: BDU=1, +/-2g, HR=1 => 0x88
    i2c_write_reg(ACC_ADDR, 0x23, 0x88);
    uint8_t who=0;
    i2c_read_n(ACC_ADDR, 0x0F, &who, 1); // not all clones support this; ignore result
    return true;
}

static bool mag_init_qmc(void) {
    // QMC5883L basic init
    // Set/reset period
    i2c_write_reg(ADDR_QMC5883L, 0x0B, 0x01);
    // Control: OSR=512(0), RNG=2000(01), ODR=50Hz(101), MODE=Continuous(01) => 0b 00 01 101 01 = 0x1D
    i2c_write_reg(ADDR_QMC5883L, 0x09, 0x1D);
    return true;
}
static bool mag_init_hmc(void) {
    // HMC5883L: 8-sample avg, 15Hz, normal
    i2c_write_reg(ADDR_HMC5883L, 0x00, 0x70);
    // Gain = 1.3 Ga (default)
    i2c_write_reg(ADDR_HMC5883L, 0x01, 0x20);
    // Continuous measurement
    i2c_write_reg(ADDR_HMC5883L, 0x02, 0x00);
    return true;
}

static bool mag_try_detect_qmc(void) {
    uint8_t id[3] = {0};
    // Many QMC clones don’t have ID regs; read data status as a cheap probe
    return i2c_read_n(ADDR_QMC5883L, 0x06, id, 1);
}

void imu_init(imu_t* imu) {
    memset(imu, 0, sizeof(*imu));
    // I2C block + pins
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    acc_init();
    if (mag_try_detect_qmc()) { imu->mag_is_qmc = true; mag_init_qmc(); }
    else { imu->mag_is_qmc = false; mag_init_hmc(); }
    imu->ok_acc = imu->ok_mag = true;
}

void imu_read(imu_t* imu) {
    // ACC (HR: 12-bit left-justified in OUT_X_L_A..)
    uint8_t buf[6];
    if (i2c_read_n(ACC_ADDR, 0x28 | 0x80, buf, 6)) {
        int16_t rx = (int16_t)(buf[1] << 8 | buf[0]);
        int16_t ry = (int16_t)(buf[3] << 8 | buf[2]);
        int16_t rz = (int16_t)(buf[5] << 8 | buf[4]);
        // Scale: at +/-2g, 1 mg/LSB in HR mode approx
        imu->ax = rx / 16384.0f * 2.0f;  // ~g (rough scaling)
        imu->ay = ry / 16384.0f * 2.0f;
        imu->az = rz / 16384.0f * 2.0f;
        imu->ok_acc = true;
    } else imu->ok_acc = false;

    // MAG
    if (imu->mag_is_qmc) {
        // QMC5883L data registers 0x00..0x05 (LSB first)
        if (i2c_read_n(ADDR_QMC5883L, 0x00, buf, 6)) {
            int16_t mx = (int16_t)(buf[1] << 8 | buf[0]);
            int16_t my = (int16_t)(buf[3] << 8 | buf[2]);
            int16_t mz = (int16_t)(buf[5] << 8 | buf[4]);
            imu->mx = mx * 0.06f; imu->my = my * 0.06f; imu->mz = mz * 0.06f; // ~uT
            imu->ok_mag = true;
        } else imu->ok_mag = false;
    } else {
        // HMC5883L: data registers 0x03..0x08 (MSB first), X Z Y
        if (i2c_read_n(ADDR_HMC5883L, 0x03, buf, 6)) {
            int16_t x = (int16_t)(buf[0] << 8 | buf[1]);
            int16_t z = (int16_t)(buf[2] << 8 | buf[3]);
            int16_t y = (int16_t)(buf[4] << 8 | buf[5]);
            imu->mx = x * 0.92f; imu->my = y * 0.92f; imu->mz = z * 0.92f; // ~mG->uT-ish
            imu->ok_mag = true;
        } else imu->ok_mag = false;
    }

    // Heading from mag (simple 2D)
    float heading = atan2f(imu->my, imu->mx) * 180.0f / (float)M_PI;
    if (heading < 0) heading += 360.0f;
    imu->heading_deg = heading;
}
