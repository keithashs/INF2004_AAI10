# 🤖 Intelligent Autonomous Line-Following Robot (AAI10)

## 🌟 Overview
This project delivers an **intelligent autonomous robotic car** designed and developed as part of the *INF2004 Embedded Systems* course.  
The system is powered by a **Raspberry Pi Pico (RP2040)** microcontroller and demonstrates advanced embedded control through real-time sensor integration, PID motor regulation, and MQTT-based telemetry.

The robot autonomously follows a track line, decodes barcodes for navigation commands, detects and avoids obstacles, and streams live data to a remote dashboard — all while maintaining robust, real-time performance.

---

## ✨ Key Features
- 🧭 **Line Following with PID Control** — Tracks the line using an IR sensor array and maintains high precision (±5 mm error) through closed-loop speed and steering correction.  
- 📦 **Barcode Command Execution** — Decodes Code-39 patterns (LEFT / RIGHT / U-TURN / STOP) with ≥ 98 % accuracy at up to 0.4 m/s.  
- ⚙️ **DC Motor Control via PWM** — Dual-motor H-bridge drive with PID-regulated speed and direction for smooth and stable motion.  
- 🛰️ **IMU-Stabilized Turning** — Utilizes gyroscope and accelerometer data for precise 90° / 180° turns and stable reacquisition of the line.  
- 🛑 **Obstacle Detection and Avoidance** — Ultrasonic sensor on a scanning servo detects obstacles, measures width, and executes bypass maneuvers.  
- 📡 **MQTT Telemetry and Dashboard Integration** — Publishes speed, distance, heading, and state data in real time with QoS 1 reliability.  
- ⚡ **Real-Time Feedback** — 100 Hz control loop and < 20 ms latency for smooth, responsive performance.  

---

## 🛠️ Components Used
| Component | Function |
|------------|-----------|
| 💻 **Raspberry Pi Pico** | Central control unit (PWM, ADC, I²C, GPIO). |
| 🔦 **IR Line & Barcode Sensors** | Detects track and reads navigation barcodes. |
| 🚘 **DC Motors + H-Bridge Driver** | Provides differential drive motion. |
| ⚙️ **Wheel Encoders** | Measures wheel speed and distance. |
| 📡 **Ultrasonic Sensor + Servo** | Detects and scans obstacles. |
| 🧭 **IMU (MPU-9250)** | Provides orientation, gyro, and accelerometer data. |
| 🌐 **Wi-Fi Module + MQTT Broker** | Handles telemetry communication. |
| 🔋 **Power Supply (5 V DC)** | Powers sensors, actuators, and controller. |

---

## 🏗️ System Architecture
### Core Modules
- **Line Sensor Subsystem** – Computes lateral error and decodes barcodes.  
- **Motor Control Subsystem** – Executes PID loop for wheel velocity and direction.  
- **Obstacle Detection Subsystem** – Scans via servo-mounted ultrasonic sensor and manages STOP/AVOID behaviors.  
- **IMU Subsystem** – Enhances turn accuracy and heading correction.  
- **Telemetry Subsystem** – Publishes data to MQTT / Dashboard in real time.  
- **Calibration & Diagnostics** – Guides sensor calibration and stores parameters in non-volatile memory.

### Control Loop
- **Sense → Process → Actuate** at **100 Hz**  
- Latency < 20 ms (median), ensuring stable real-time motion control.

---

## 🚀 How to Run
1. **Hardware Setup**
   - Connect IR sensors, ultrasonic + servo, IMU, encoders, and motors to the Pico according to pin assignments.
   - Ensure stable 5 V power and H-bridge driver connections.
   - Configure Wi-Fi credentials and MQTT broker details.

2. **Flash the Code**
   - Open the project in the **Raspberry Pi Pico SDK** or **Thonny (MicroPython)**.  
   - Compile and flash the firmware to the Pico.

3. **Run the System**
   - Press the **START** button to initialize sensors and enter *FOLLOW mode* (≤ 3 s startup).  
   - Observe telemetry on the dashboard (speed, heading, line error, obstacles).

4. **Calibrate & Test**
   - Use the **Calibration Mode** to fine-tune IR thresholds, encoder ticks, and IMU bias.  
   - Verify smooth line tracking, barcode detection, and obstacle avoidance.

5. **Debug & Monitor**
   - Monitor MQTT logs or LED status indicators for system states: POWER, READY, FOLLOW, CALIB, ERROR.

---

## 🧩 Software Modules
- **motor.c / motor.h** — PWM-based motor control with watchdog safety.  
- **enc.c / enc.h** — Wheel encoder tick counting and velocity estimation.  
- **line.c / line.h** — Line tracking and barcode decoding.  
- **servo.c / servo.h** — Servo positioning for ultrasonic scanning.  
- **imu.c / imu.h** — Sensor fusion and heading computation.  
- **mqtt.c / mqtt.h** — Network connectivity and telemetry.  
- **cfg.c / cfg.h** — Configuration storage (Flash / EEPROM).  
- **ui.c / ui.h** — Buttons and LED user interface.  
- **logx.c / logx.h** — Diagnostic event logging.

---

## 📈 Performance Highlights
- Line-tracking error ≤ ±5 mm  
- Speed control steady-state error ≤ 5 %  
- Turning accuracy ± 5° (90° turns)  
- Obstacle detection range 5–200 cm  
- Telemetry ≥ 10 Hz with QoS 1  
- Mean system uptime ≥ 99 % over 15 min runs  

---

## 🤝 Contributions
Contributions and improvements are welcome!  
You can help by:
- Enhancing PID tuning and sensor fusion algorithms.  
- Extending obstacle avoidance with SLAM or vision.  
- Integrating AI-based decision modules or route planning.  

---

## 📜 License
This project is licensed under the **MIT License**.  
Feel free to use, modify, and distribute for educational or research purposes.

---

## 🧠 Team AAI10
| Member | Student ID | Role |
|---------|-------------|------|
| Cheong Wai Hong Jared | 2401641 | Systems Integration & Control Design |
| Chan Jing Chun | 2402867 | Sensor Interface & Signal Processing |
| Hing Zheng Wen | 2401599 | Motor Driver and PID Control |
| Wong Liang Jin | 2400598 | Networking & Telemetry (MQTT) |
| Lee Xu Xiang Keith | 2400845 | System Testing & Documentation |

https://github.com/jaywmz/Embedded-Systems-ROBOCAR.git
Seniors repo:

https://github.com/balavigneshsureshkumar/RobotCar.git
Pico board i using:
https://sg.cytron.io/p-robo-pico-simplifying-robotics-with-raspberry-pi-pico

You may refer to my seniors git repo above from and by following their file directory and coding style like having different file for different sensor module. 
all the coding c programming and spilt to main.c, motor, IMD Accelerometer and magnetometer. 
now i just want to do this demo 1 only first without wifi and dashboard then name it car_demo1 folder and all the code in this folder. show the full modified code here in the chat
Demo 1: Basic Motion & Sensing Integration
 Purpose: Validate motor control with PID, IMU data 
acquisition & filtering, and basic telemetry.
 Setup:
 • Place the robot on a simple straight track with no barcodes or obstacles or lines.
 • Enable PID control for motor speed & heading.
 • IMU sensor provides acceleration, orientation, and filtered heading to ensure straight motion.
 • Telemetry (speed, distance, heading, raw vs filtered IMU data) print on the serial monitor first Later on the work on the published via MQTT to the dashboard.
 Success Criteria:
 • Robot maintains a straight path with minimal drift (thanks to IMU corrections).
 • Live speed & heading data available on serial monitor first later then do MQTT dashboard.
 • Filtering visibly improves IMU data stability (less noise).


All my connection pin below:
// ===== Pin map (Cytron Robo Pico, 2-pin drive per motor) =====
// Right motor (M1)
#define M1_IN1   8    // GP8
#define M1_IN2   9    // GP9
#define ENCODER_PIN_M1 16 // GP16 : IR encoder signal input right motor M1 for Telemetry the speed and dist
// Left motor (M2)
#define M2_IN1  11    // GP11
#define M2_IN2  10    // GP10
#define ENCODER_PIN_M2 6 // GP6 : IR encoder signal input left motor M2 for Telemetry the speed and dist
//IR Line Sensor 
#define IR_DIGITAL_GPIO    0    // GP0 is digital input (sensor D0)
//IR Barcode Sensor
#define IR_ADC_GPIO        26    // GP26 is ADC0 (sensor A0)
// I2C port/pins/speed
#define I2C_PORT  i2c1             // Use I2C1 block (matches GP2/GP3)
#define I2C_SDA   2                // SDA on GPIO 2
#define I2C_SCL   3                // SCL on GPIO 3
#define I2C_BAUD  400000           // 400 kHz Fast-mode (works well here)

// I2C port/pins/speed
#define I2C_PORT  i2c1             // Use I2C1 block (matches GP2/GP3)
#define I2C_SDA   2                // SDA on GPIO 2
#define I2C_SCL   3                // SCL on GPIO 3
#define I2C_BAUD  400000           // 400 kHz Fast-mode (works well here)

// LSM303DLHC Accelerometer (ACC)
// GY-511 (LSM303) has separate I2C addresses for accel (0x19) and mag (0x1E).
#define ACC_ADDR          0x19     // 7-bit I2C address of the ACC block

// LSM303DLHC accelerometer register map (subset used here)
#define ACC_REG_STATUS_A  0x27     // STATUS_REG_A (data-ready flags)
#define ACC_REG_OUT_X_L   0x28     // OUT_X_L_A (first data register)
#define ACC_REG_CTRL1_A   0x20     // CTRL_REG1_A (ODR, axis enables)
#define ACC_REG_CTRL4_A   0x23     // CTRL_REG4_A (BDU, scale, HR mode)
#define ACC_REG_WHOAMI_A  0x0F     // WHO_AM_I_A (optional identification)

// Ultrasonic (change if needed)
#define US_TRIG  4
#define US_ECHO  5

// Buttons (active-low: use onboard pull-ups)
#define BTN_START 21   // press to enable movement
#define BTN_STOP  20   // press to force stop



basic motor moving
// robo_pico_dc_pwm.c
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <math.h>

// ===== User settings =====
#define PWM_FREQ      20000            // 20 kHz (quiet for small DC motors)
// Choose TOP & CLKDIV so Fpwm = 125e6 / (CLKDIV * (TOP+1)) ~= PWM_FREQ.
// With CLKDIV=1 and TOP=6249, Fpwm = 125e6 / (1 * 6250) = 20 kHz.
#define PWM_TOP       6249
#define PWM_CLKDIV    1.0f

// Motor pins (Robo Pico)
#define PWM_M1A  8
#define PWM_M1B  9
#define PWM_M2A  10
#define PWM_M2B  11

// Buttons (from your slide)
#define EXECUTE_BTN   20
#define EXECUTE_BTN2  21

// ---------- PWM helpers ----------
static inline void setup_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);

    // Configure slice once. (If both pins share the slice, this runs twice—OK.)
    pwm_set_wrap(slice, PWM_TOP);
    pwm_set_clkdiv(slice, PWM_CLKDIV);
    pwm_set_enabled(slice, true);
}

static inline void set_level_u16(uint gpio, uint16_t level_u16) {
    // Scale 16-bit input to [0..PWM_TOP]
    uint32_t level = ((uint32_t)level_u16 * (PWM_TOP + 1)) >> 16;
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpio),
                       pwm_gpio_to_channel(gpio),
                       level);
}

static inline void set_level_float(uint gpio, float level_0_to_1) {
    if (level_0_to_1 <= 0.f) {
        pwm_set_chan_level(pwm_gpio_to_slice_num(gpio),
                           pwm_gpio_to_channel(gpio), 0);
    } else if (level_0_to_1 >= 1.f) {
        pwm_set_chan_level(pwm_gpio_to_slice_num(gpio),
                           pwm_gpio_to_channel(gpio), PWM_TOP);
    } else {
        uint32_t lvl = (uint32_t)lroundf(level_0_to_1 * (PWM_TOP));
        pwm_set_chan_level(pwm_gpio_to_slice_num(gpio),
                           pwm_gpio_to_channel(gpio), lvl);
    }
}

// ---------- Motor control (sign-magnitude) ----------
// speed in [-1.0 .. +1.0]. + = forward, - = reverse.
// For brake set both low; for coast set both high (optional function below).
static inline void drive_pair_signmag(uint pinA, uint pinB, float speed) {
    if (speed > 0.f) {
        set_level_float(pinA, fminf(speed, 1.f));  // PWM on A
        set_level_float(pinB, 0.f);                // Low on B
    } else if (speed < 0.f) {
        set_level_float(pinA, 0.f);                // Low on A
        set_level_float(pinB, fminf(-speed, 1.f)); // PWM on B
    } else {
        // Brake: both Low
        set_level_float(pinA, 0.f);
        set_level_float(pinB, 0.f);
    }
}

static inline void coast_pair(uint pinA, uint pinB) {
    set_level_float(pinA, 1.f); // High
    set_level_float(pinB, 1.f); // High
}

// Robot-style API from your slide
void robot_movement(float sL, float sR) {
    // sL/sR ∈ [-1.0 .. +1.0]
    drive_pair_signmag(PWM_M1A, PWM_M1B, sL); // Left motor on M1
    drive_pair_signmag(PWM_M2A, PWM_M2B, sR); // Right motor on M2
}

// ---------- Demo program ----------
int main() {
    stdio_init_all();

    // Setup PWM pins
    setup_pwm(PWM_M1A);
    setup_pwm(PWM_M1B);
    setup_pwm(PWM_M2A);
    setup_pwm(PWM_M2B);

    // Setup buttons
    gpio_init(EXECUTE_BTN);
    gpio_set_dir(EXECUTE_BTN, GPIO_IN);
    gpio_pull_up(EXECUTE_BTN);

    gpio_init(EXECUTE_BTN2);
    gpio_set_dir(EXECUTE_BTN2, GPIO_IN);
    gpio_pull_up(EXECUTE_BTN2);

    printf("Robo Pico DC motor demo ready. Press GP20 or GP21 to start.\n");

    while (true) {
        // Wait for either button press (active-low w/ pull-ups)
        while (gpio_get(EXECUTE_BTN) && gpio_get(EXECUTE_BTN2)) {
            tight_loop_contents();
        }
        printf("Button pressed! Starting movement sequence...\n");

        // Ensure we start from brake
        robot_movement(0.f, 0.f);
        sleep_ms(300);

        // ---- Forward ramp ----
        for (int i = 0; i <= 10; ++i) {
            float s = i / 10.0f;            // 0.0 -> 1.0
            robot_movement(s, s);
            sleep_ms(250);
        }
        sleep_ms(1000);

        // ---- Backward ----
        robot_movement(-0.5f, -0.5f);
        sleep_ms(2000);

        // ---- Turn Left (right wheel faster) ----
        robot_movement(0.1f, 0.5f);
        sleep_ms(1500);

        // ---- Turn Right (left wheel faster) ----
        robot_movement(0.5f, 0.1f);
        sleep_ms(1500);

        // ---- Brake stop ----
        robot_movement(0.f, 0.f);
        printf("-- Sequence complete. Press a button to run again --\n");

        // Debounce: wait for release
        while (!gpio_get(EXECUTE_BTN) || !gpio_get(EXECUTE_BTN2)) {
            sleep_ms(10);
        }
        sleep_ms(150);
    }
    // not reached
}


serial monitor output: 
STAT M1[cps=   0.0 tgt=  20.0 duty=  10.2% dir= 1]  M2[cps=   0.0 tgt=  20.0 duty=  10.2% dir= 1]  Dist[L=  0.000 R=  0.000]  IMU[roll=   5.7 pitch=   4.1 head= 159.4 filt= 159.5 err=   0.0 bias=   0.0 ]
STAT M1[cps=   0.0 tgt=  20.0 duty=  17.7% dir= 1]  M2[cps=   0.0 tgt=  20.0 duty=  18.7% dir= 1]  Dist[L=  0.000 R=  0.000]  IMU[roll=   2.6 pitch=   3.4 head= 159.4 filt= 159.3 err=   0.0 bias=   0.0 ]
STAT M1[cps=   0.0 tgt=  20.0 duty=  25.6% dir= 1]  M2[cps=   0.0 tgt=  20.0 duty=  26.8% dir= 1]  Dist[L=  0.000 R=  0.000]  IMU[roll=   3.4 pitch=   4.1 head= 159.5 filt= 159.4 err=   0.0 bias=   0.0 ]
STAT M1[cps=   0.0 tgt=  20.0 duty=  33.6% dir= 1]  M2[cps=   0.0 tgt=  20.0 duty=  34.8% dir= 1]  Dist[L=  0.000 R=  0.000]  IMU[roll=   4.2 pitch=   3.4 head= 158.4 filt= 158.7 err=   0.0 bias=   0.0 ]
STAT M1[cps=   0.0 tgt=  28.0 duty=  44.4% dir= 1]  M2[cps=  10.0 tgt=  12.0 duty=  36.7% dir= 1]  Dist[L=  0.004 R=  0.000]  IMU[roll=  10.4 pitch=  11.0 head= 163.7 filt= 161.1 err=   1.7 bias=  -8.0 ]
STAT M1[cps=  20.0 tgt=  20.7 duty=  44.6% dir= 1]  M2[cps=  20.0 tgt=  19.3 duty=  39.6% dir= 1]  Dist[L=  0.016 R=  0.012]  IMU[roll=   7.2 pitch=   5.3 head= 164.3 filt= 163.4 err=   4.0 bias=  -0.7 ]
STAT M1[cps=  20.0 tgt=  17.1 duty=  44.9% dir= 1]  M2[cps=  20.0 tgt=  22.9 duty=  43.0% dir= 1]  Dist[L=  0.027 R=  0.024]  IMU[roll=   4.4 pitch=  -0.5 head= 164.4 filt= 164.5 err=   5.2 bias=   2.9 ]
STAT M1[cps=  20.0 tgt=  17.1 duty=  45.6% dir= 1]  M2[cps=  20.0 tgt=  22.9 duty=  44.4% dir= 1]  Dist[L=  0.039 R=  0.035]  IMU[roll=   0.0 pitch=  -6.1 head= 165.0 filt= 164.8 err=   5.4 bias=   2.9 ]
STAT M1[cps=  20.0 tgt=  23.0 duty=  48.4% dir= 1]  M2[cps=  10.0 tgt=  17.0 duty=  45.0% dir= 1]  Dist[L=  0.051 R=  0.051]  IMU[roll=   6.5 pitch=   6.0 head= 166.7 filt= 166.3 err=   6.9 bias=  -3.0 ]
STAT M1[cps=  20.0 tgt=  17.1 duty=  46.0% dir= 1]  M2[cps=  20.0 tgt=  22.9 duty=  47.3% dir= 1]  Dist[L=  0.067 R=  0.063]  IMU[roll=   5.1 pitch=   2.6 head= 168.2 filt= 167.7 err=   8.3 bias=   2.9 ]
STAT M1[cps=  20.0 tgt=  17.5 duty=  46.2% dir= 1]  M2[cps=  20.0 tgt=  22.5 duty=  49.1% dir= 1]  Dist[L=  0.082 R=  0.079]  IMU[roll=   5.6 pitch=   9.1 head= 170.1 filt= 169.3 err=   9.9 bias=   2.5 ]
STAT M1[cps=  20.0 tgt=  18.4 duty=  47.4% dir= 1]  M2[cps=  20.0 tgt=  21.6 duty=  49.3% dir= 1]  Dist[L=  0.094 R=  0.090]  IMU[roll=   7.5 pitch= -13.9 head= 169.7 filt= 171.1 err=  11.7 bias=   1.6 ]
STAT M1[cps=  20.0 tgt=  17.5 duty=  47.1% dir= 1]  M2[cps=  20.0 tgt=  22.5 duty=  50.0% dir= 1]  Dist[L=  0.110 R=  0.106]  IMU[roll=   0.0 pitch=  13.7 head= 172.9 filt= 172.7 err=  13.4 bias=   2.5 ]
STAT M1[cps=  20.0 tgt=  19.7 duty=  48.3% dir= 1]  M2[cps=  20.0 tgt=  20.3 duty=  49.7% dir= 1]  Dist[L=  0.126 R=  0.122]  IMU[roll=   4.3 pitch=  -6.3 head= 174.9 filt= 174.6 err=  15.2 bias=   0.3 ]
STAT M1[cps=  20.0 tgt=  19.4 duty=  47.8% dir= 1]  M2[cps=  20.0 tgt=  20.6 duty=  50.0% dir= 1]  Dist[L=  0.145 R=  0.134]  IMU[roll=   0.7 pitch=  14.4 head= 176.5 filt= 176.9 err=  17.5 bias=   0.6 ]
STAT M1[cps=  20.0 tgt=  19.5 duty=  47.9% dir= 1]  M2[cps=  20.0 tgt=  20.5 duty=  49.8% dir= 1]  Dist[L=  0.161 R=  0.149]  IMU[roll=   5.0 pitch=  16.5 head= 178.2 filt= 176.6 err=  17.2 bias=   0.5 ]
STAT M1[cps=  20.0 tgt=  22.4 duty=  49.1% dir= 1]  M2[cps=  20.0 tgt=  17.6 duty=  49.1% dir= 1]  Dist[L=  0.177 R=  0.165]  IMU[roll=   0.3 pitch=  14.9 head= 175.9 filt= 176.4 err=  17.0 bias=  -2.4 ]
STAT M1[cps=  20.0 tgt=  19.1 duty=  48.0% dir= 1]  M2[cps=  20.0 tgt=  20.9 duty=  50.3% dir= 1]  Dist[L=  0.192 R=  0.181]  IMU[roll=   5.8 pitch=   1.2 head= 175.9 filt= 176.1 err=  16.7 bias=   0.9 ]
STAT M1[cps=  20.0 tgt=  22.0 duty=  49.6% dir= 1]  M2[cps=  20.0 tgt=  18.0 duty=  48.9% dir= 1]  Dist[L=  0.208 R=  0.196]  IMU[roll=  15.4 pitch=  -1.0 head= 176.0 filt= 174.4 err=  15.0 bias=  -2.0 ]
STAT M1[cps=  20.0 tgt=  21.4 duty=  48.7% dir= 1]  M2[cps=  20.0 tgt=  18.6 duty=  49.6% dir= 1]  Dist[L=  0.224 R=  0.212]  IMU[roll=   5.1 pitch=  -5.8 head= 171.4 filt= 172.1 err=  12.7 bias=  -1.4 ]
STAT M1[cps=  20.0 tgt=  21.1 duty=  49.9% dir= 1]  M2[cps=  20.0 tgt=  18.9 duty=  48.9% dir= 1]  Dist[L=  0.240 R=  0.228]  IMU[roll=  -4.9 pitch=  -1.0 head= 167.7 filt= 169.8 err=  10.4 bias=  -1.1 ]
STAT M1[cps=  10.0 tgt=  19.0 duty=  51.0% dir= 1]  M2[cps=  20.0 tgt=  21.0 duty=  49.7% dir= 1]  Dist[L=  0.255 R=  0.240]  IMU[roll=   9.9 pitch=  -5.1 head= 168.7 filt= 167.2 err=   7.9 bias=   1.0 ]
STAT M1[cps=  20.0 tgt=  22.2 duty=  51.5% dir= 1]  M2[cps=  20.0 tgt=  17.8 duty=  48.0% dir= 1]  Dist[L=  0.271 R=  0.255]  IMU[roll=  -3.3 pitch=  12.2 head= 161.6 filt= 164.9 err=   5.6 bias=  -2.2 ]
STAT M1[cps=  20.0 tgt=  21.9 duty=  51.2% dir= 1]  M2[cps=  20.0 tgt=  18.1 duty=  48.3% dir= 1]  Dist[L=  0.287 R=  0.275]  IMU[roll=   5.3 pitch=  10.8 head= 162.0 filt= 161.8 err=   2.4 bias=  -1.9 ]
STAT M1[cps=  20.0 tgt=  23.3 duty=  52.6% dir= 1]  M2[cps=  20.0 tgt=  16.7 duty=  46.8% dir= 1]  Dist[L=  0.302 R=  0.291]  IMU[roll=  -3.5 pitch=  -8.9 head= 159.5 filt= 159.4 err=   0.0 bias=  -3.3 ]
STAT M1[cps=  20.0 tgt=  23.2 duty=  53.3% dir= 1]  M2[cps=  10.0 tgt=  16.8 duty=  47.2% dir= 1]  Dist[L=  0.314 R=  0.306]  IMU[roll=   7.9 pitch=  -7.6 head= 156.9 filt= 157.3 err=  -2.1 bias=  -3.2 ]
STAT M1[cps=  20.0 tgt=  21.7 duty=  53.0% dir= 1]  M2[cps=  20.0 tgt=  18.3 duty=  46.8% dir= 1]  Dist[L=  0.330 R=  0.322]  IMU[roll=  -1.1 pitch=  -3.4 head= 153.7 filt= 154.5 err=  -4.9 bias=  -1.7 ]
STAT M1[cps=  20.0 tgt=  22.1 duty=  54.2% dir= 1]  M2[cps=  20.0 tgt=  17.9 duty=  46.1% dir= 1]  Dist[L=  0.346 R=  0.338]  IMU[roll=  15.3 pitch=  -0.9 head= 152.5 filt= 152.3 err=  -7.1 bias=  -2.1 ]
STAT M1[cps=  20.0 tgt=  22.4 duty=  53.7% dir= 1]  M2[cps=  10.0 tgt=  17.6 duty=  47.5% dir= 1]  Dist[L=  0.357 R=  0.353]  IMU[roll=  12.3 pitch=  18.0 head= 149.5 filt= 148.7 err= -10.7 bias=  -2.4 ]
STAT M1[cps=  20.0 tgt=  21.5 duty=  52.9% dir= 1]  M2[cps=  20.0 tgt=  18.5 duty=  46.5% dir= 1]  Dist[L=  0.373 R=  0.373]  IMU[roll=  10.7 pitch=   4.5 head= 146.0 filt= 145.9 err= -13.5 bias=  -1.5 ]
STAT M1[cps=  20.0 tgt=  24.1 duty=  54.5% dir= 1]  M2[cps=  20.0 tgt=  15.9 duty=  44.9% dir= 1]  Dist[L=  0.389 R=  0.389]  IMU[roll=   9.5 pitch=   7.2 head= 142.5 filt= 142.5 err= -16.9 bias=  -4.1 ]
STAT M1[cps=  20.0 tgt=  19.7 duty=  53.6% dir= 1]  M2[cps=  20.0 tgt=  20.3 duty=  46.9% dir= 1]  Dist[L=  0.404 R=  0.404]  IMU[roll=   2.6 pitch=   0.9 head= 139.6 filt= 139.5 err= -19.9 bias=   0.3 ]
STAT M1[cps=  30.0 tgt=  23.6 duty=  54.1% dir= 1]  M2[cps=  10.0 tgt=  16.4 duty=  46.0% dir= 1]  Dist[L=  0.416 R=  0.424]  IMU[roll=   5.4 pitch=  11.5 head= 138.0 filt= 137.5 err= -21.9 bias=  -3.6 ]
STAT M1[cps=  20.0 tgt=  21.6 duty=  54.5% dir= 1]  M2[cps=  20.0 tgt=  18.4 duty=  45.4% dir= 1]  Dist[L=  0.432 R=  0.440]  IMU[roll=   7.0 pitch=  -7.7 head= 133.3 filt= 135.1 err= -24.3 bias=  -1.6 ]
STAT M1[cps=  20.0 tgt=  20.0 duty=  54.1% dir= 1]  M2[cps=  20.0 tgt=  20.0 duty=  46.1% dir= 1]  Dist[L=  0.448 R=  0.456]  IMU[roll=   9.0 pitch=  -3.8 head= 132.8 filt= 134.4 err= -25.0 bias=  -0.0 ]
STAT M1[cps=   0.0 tgt=  22.4 duty=  65.8% dir= 1]  M2[cps=   0.0 tgt=  17.6 duty=  54.6% dir= 1]  Dist[L=  0.448 R=  0.456]  IMU[roll=  -7.3 pitch= -17.9 head= 132.1 filt= 134.3 err= -25.0 bias=  -2.4 ]
my car still slighlty drift to the right the to the left as the car drive futher away? how can i calibrate it so that the car move in straight line? can use IMU sensor provides acceleration, orientation, and 
filtered heading to ensure straight motion? Show full modify code if any


How to use it (quick steps)

Flash this build. Place the car on a clear, magnetically clean floor (avoid rebar).

Press START: it will spin ~3 s to calibrate mag, then roll forward ~1.5 s to self-trim wheel scales, then begin the straight run.

If you move to a different area with magnetic differences, press STOP then START again to re-calibrate.