#ifndef PIN_H
#define PIN_H

// ===== Pin map (Cytron Robo Pico, 2-pin drive per motor) =====
// Right motor (M1)
#define M1_IN1   8    // GP8
#define M1_IN2   9    // GP9
// Left motor (M2)
#define M2_IN1  11    // GP11
#define M2_IN2  10    // GP10

// IR Line Sensor / Barcode (kept for structure)
#define IR_DIGITAL_GPIO    0
#define IR_ADC_GPIO        26

// I2C port/pins/speed
#define I2C_PORT  i2c1
#define I2C_SDA   2
#define I2C_SCL   3
#define I2C_BAUD  400000

// LSM303DLHC Accelerometer (ACC)
#define ACC_ADDR          0x19

// LSM303DLHC mag-compatible
#define ADDR_HMC5883L     0x1E
#define ADDR_QMC5883L     0x0D

// Ultrasonic
#define US_TRIG  4
#define US_ECHO  5

// Buttons (active-low: use onboard pull-ups)
#define BTN_START 21
#define BTN_STOP  20

#endif
