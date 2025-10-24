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


serial monitor output: 
=== Car Demo 1: Cascaded Heading (Enc diff + IMU trim + Cal + Supervisor + Adaptive scale) ===
IMU ready.
BOOT: waiting 10s before auto calibration...
CAL: magnetometer min/max... spinning 3s
CAL M1[speed=  0.00cm/s tgt=  9.82cm/s duty=  11.8% dir= 1]  M2[speed=  0.00cm/s tgt=  9.82cm/s duty=  11.8% dir=-1]
CAL M1[speed=  0.00cm/s tgt=  9.82cm/s duty=  16.8% dir= 1]  M2[speed=  0.00cm/s tgt=  9.82cm/s duty=  16.8% dir=-1]
CAL M1[speed=  0.00cm/s tgt=  9.82cm/s duty=  26.8% dir= 1]  M2[speed=  0.00cm/s tgt=  9.82cm/s duty=  26.8% dir=-1]
CAL M1[speed=  0.00cm/s tgt=  9.82cm/s duty=  36.8% dir= 1]  M2[speed=  0.00cm/s tgt=  9.82cm/s duty=  36.8% dir=-1]
CAL M1[speed=  0.00cm/s tgt=  9.82cm/s duty=  46.8% dir= 1]  M2[speed=  0.00cm/s tgt=  9.82cm/s duty=  46.8% dir=-1]
CAL M1[speed=  0.00cm/s tgt=  9.82cm/s duty=  54.2% dir= 1]  M2[speed=  3.93cm/s tgt=  9.82cm/s duty=  51.9% dir=-1]
CAL M1[speed=  7.85cm/s tgt=  9.82cm/s duty=  55.1% dir= 1]  M2[speed=  7.85cm/s tgt=  9.82cm/s duty=  54.6% dir=-1]
CAL M1[speed=  7.85cm/s tgt=  9.82cm/s duty=  56.2% dir= 1]  M2[speed=  3.93cm/s tgt=  9.82cm/s duty=  59.0% dir=-1]
CAL M1[speed=  7.85cm/s tgt=  9.82cm/s duty=  56.2% dir= 1]  M2[speed=  7.85cm/s tgt=  9.82cm/s duty=  60.5% dir=-1]
CAL M1[speed=  7.85cm/s tgt=  9.82cm/s duty=  56.7% dir= 1]  M2[speed=  7.85cm/s tgt=  9.82cm/s duty=  62.5% dir=-1]
CAL M1[speed= 11.78cm/s tgt=  9.82cm/s duty=  55.4% dir= 1]  M2[speed=  7.85cm/s tgt=  9.82cm/s duty=  64.1% dir=-1]
CAL M1[speed= 11.78cm/s tgt=  9.82cm/s duty=  55.4% dir= 1]  M2[speed= 11.78cm/s tgt=  9.82cm/s duty=  64.3% dir=-1]
CAL M1[speed= 11.78cm/s tgt=  9.82cm/s duty=  56.4% dir= 1]  M2[speed=  7.85cm/s tgt=  9.82cm/s duty=  66.2% dir=-1]
CAL M1[speed= 11.78cm/s tgt=  9.82cm/s duty=  56.3% dir= 1]  M2[speed= 11.78cm/s tgt=  9.82cm/s duty=  65.9% dir=-1]
CAL M1[speed= 11.78cm/s tgt=  9.82cm/s duty=  56.2% dir= 1]  M2[speed=  7.85cm/s tgt=  9.82cm/s duty=  68.1% dir=-1]
CAL: done. heading_ref=204.6 deg
TRIM: auto wheel scale... driving 4.0s
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  55.8% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  66.4% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  55.1% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  70.2% dir= 1]
CAL M1[speed= 11.78cm/s tgt=  7.85cm/s duty=  52.1% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  66.7% dir= 1]
CAL M1[speed= 11.78cm/s tgt=  7.85cm/s duty=  50.9% dir= 1]  M2[speed= 11.78cm/s tgt=  7.85cm/s duty=  63.1% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  50.9% dir= 1]  M2[speed= 11.78cm/s tgt=  7.85cm/s duty=  60.8% dir= 1]
CAL M1[speed= 11.78cm/s tgt=  7.85cm/s duty=  48.8% dir= 1]  M2[speed= 11.78cm/s tgt=  7.85cm/s duty=  58.9% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  49.1% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  58.6% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  48.6% dir= 1]  M2[speed= 11.78cm/s tgt=  7.85cm/s duty=  56.6% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  48.2% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  56.9% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  47.9% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  56.4% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  47.9% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  55.4% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  47.5% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  54.9% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  47.5% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  54.8% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  47.1% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  54.2% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  47.1% dir= 1]  M2[speed= 11.78cm/s tgt=  7.85cm/s duty=  52.9% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  46.7% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  53.9% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  46.7% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  53.9% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  46.7% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  53.9% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  46.3% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  53.5% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  46.3% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  53.9% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  46.3% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  53.5% dir= 1]
CAL M1[speed=  3.93cm/s tgt=  7.85cm/s duty=  47.3% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  53.9% dir= 1]
CAL M1[speed=  7.85cm/s tgt=  7.85cm/s duty=  45.9% dir= 1]  M2[speed=  7.85cm/s tgt=  7.85cm/s duty=  54.3% dir= 1]
TRIM: scales R=1.012 L=0.988
BOOT: auto calibration complete. Press START when ready.

LEGEND:
STAT M1[speed cm/s  tgt cm/s  duty %  dir]  M2[speed cm/s  tgt cm/s  duty %  dir]  Dist[L cm R cm]  IMU[err deg roll deg pitch deg head deg filt deg bias cps w]
Notes: bias>0 speeds LEFT up and RIGHT down; w is IMU trust 0..1.

START: settle 4s, capture heading_ref, then soft-start. Scale[R=1.012 L=0.988]
START -> 20%, heading_ref=217.1 deg
STAT M1[speed=  0.00cm/s tgt=  0.40cm/s duty=   0.3% dir= 1]  M2[speed=  0.00cm/s tgt=  0.39cm/s duty=   0.3% dir= 1]  Dist[L=    0.0cm R=    0.0cm]   IMU[err=   0.0 roll=   1.8 pitch=   4.8 head= 217.2 filt= 217.1 bias=   0.0 w=0.10]
STAT M1[speed=  0.00cm/s tgt=  2.39cm/s duty=   3.1% dir= 1]  M2[speed=  0.00cm/s tgt=  2.33cm/s duty=   3.0% dir= 1]  Dist[L=    0.0cm R=    0.0cm]   IMU[err=   0.0 roll=   2.3 pitch=   4.6 head= 217.4 filt= 217.2 bias=   0.0 w=0.32]
STAT M1[speed=  0.00cm/s tgt=  4.37cm/s duty=   7.9% dir= 1]  M2[speed=  0.00cm/s tgt=  4.27cm/s duty=   7.7% dir= 1]  Dist[L=    0.0cm R=    0.0cm]   IMU[err=   0.0 roll=   1.8 pitch=   4.8 head= 217.5 filt= 217.3 bias=   0.0 w=0.49]
STAT M1[speed=  0.00cm/s tgt=  6.36cm/s duty=  14.7% dir= 1]  M2[speed=  0.00cm/s tgt=  6.21cm/s duty=  14.4% dir= 1]  Dist[L=    0.0cm R=    0.0cm]   IMU[err=   0.0 roll=   2.8 pitch=   4.8 head= 217.5 filt= 217.3 bias=   0.0 w=0.62]
STAT M1[speed=  0.00cm/s tgt=  7.95cm/s duty=  23.3% dir= 1]  M2[speed=  0.00cm/s tgt=  7.76cm/s duty=  22.7% dir= 1]  Dist[L=    0.0cm R=    0.0cm]   IMU[err=   0.0 roll=   2.0 pitch=   4.8 head= 217.6 filt= 217.4 bias=   0.0 w=0.71]
STAT M1[speed=  0.00cm/s tgt=  7.95cm/s duty=  31.4% dir= 1]  M2[speed=  0.00cm/s tgt=  7.76cm/s duty=  30.6% dir= 1]  Dist[L=    0.0cm R=    0.0cm]   IMU[err=   0.0 roll=   1.8 pitch=   4.9 head= 217.8 filt= 217.6 bias=   0.0 w=0.79]
STAT M1[speed=  3.93cm/s tgt=  4.59cm/s duty=  30.0% dir= 1]  M2[speed=  3.93cm/s tgt= 11.04cm/s duty=  41.9% dir= 1]  Dist[L=    0.4cm R=    0.8cm]   IMU[err=   0.0 roll=   7.1 pitch=   4.6 head= 212.9 filt= 215.8 bias=   8.5 w=0.18]
STAT M1[speed=  3.93cm/s tgt=  7.50cm/s duty=  35.0% dir= 1]  M2[speed=  3.93cm/s tgt=  8.20cm/s duty=  43.1% dir= 1]  Dist[L=    1.2cm R=    1.6cm]   IMU[err=  -2.4 roll=   0.5 pitch=  -4.0 head= 215.4 filt= 214.7 bias=   1.1 w=0.31]
STAT M1[speed=  7.85cm/s tgt=  7.22cm/s duty=  35.6% dir= 1]  M2[speed=  7.85cm/s tgt=  8.48cm/s duty=  44.7% dir= 1]  Dist[L=    2.4cm R=    2.7cm]   IMU[err=   0.0 roll=   5.6 pitch=  -4.6 head= 216.1 filt= 216.5 bias=   1.8 w=0.26]
STAT M1[speed=  7.85cm/s tgt=  8.79cm/s duty=  39.6% dir= 1]  M2[speed=  7.85cm/s tgt=  6.93cm/s duty=  44.6% dir= 1]  Dist[L=    3.5cm R=    3.9cm]   IMU[err=   0.0 roll=  -0.4 pitch=  17.2 head= 222.2 filt= 219.0 bias=  -2.2 w=0.26]
STAT M1[speed=  3.93cm/s tgt=  9.78cm/s duty=  44.0% dir= 1]  M2[speed=  3.93cm/s tgt=  5.97cm/s duty=  46.6% dir= 1]  Dist[L=    4.7cm R=    4.7cm]   IMU[err=   4.5 roll=   5.3 pitch=  -2.0 head= 222.0 filt= 221.5 bias=  -4.6 w=0.24]
STAT M1[speed=  7.85cm/s tgt=  7.83cm/s duty=  41.5% dir= 1]  M2[speed=  7.85cm/s tgt=  7.87cm/s duty=  49.3% dir= 1]  Dist[L=    6.3cm R=    6.3cm]   IMU[err=   7.3 roll=   4.0 pitch=  -5.1 head= 223.0 filt= 224.3 bias=   0.3 w=0.20]
STAT M1[speed=  7.85cm/s tgt=  7.81cm/s duty=  43.2% dir= 1]  M2[speed=  7.85cm/s tgt=  7.90cm/s duty=  48.7% dir= 1]  Dist[L=    7.5cm R=    7.5cm]   IMU[err=  10.9 roll=  -7.3 pitch=  16.4 head= 230.7 filt= 227.9 bias=   0.3 w=0.14]
STAT M1[speed=  7.85cm/s tgt=  6.97cm/s duty=  43.5% dir= 1]  M2[speed=  7.85cm/s tgt=  8.71cm/s duty=  50.5% dir= 1]  Dist[L=    9.0cm R=    9.0cm]   IMU[err=  12.8 roll=   2.9 pitch=  17.3 head= 242.6 filt= 229.9 bias=   2.5 w=0.14]
STAT M1[speed=  7.85cm/s tgt=  7.61cm/s duty=  44.4% dir= 1]  M2[speed=  7.85cm/s tgt=  8.09cm/s duty=  50.9% dir= 1]  Dist[L=   10.6cm R=   10.6cm]   IMU[err=  17.0 roll=  -3.8 pitch=   2.0 head= 225.2 filt= 234.0 bias=   0.9 w=0.06]
STAT M1[speed=  7.85cm/s tgt=  8.09cm/s duty=  45.6% dir= 1]  M2[speed=  3.93cm/s tgt=  7.62cm/s duty=  51.2% dir= 1]  Dist[L=   11.8cm R=   11.8cm]   IMU[err=  17.4 roll=   7.3 pitch=  15.6 head= 253.9 filt= 234.4 bias=  -0.3 w=0.04]
STAT M1[speed=  7.85cm/s tgt=  8.47cm/s duty=  46.3% dir= 1]  M2[speed=  7.85cm/s tgt=  7.25cm/s duty=  50.0% dir= 1]  Dist[L=   13.4cm R=   13.4cm]   IMU[err=  19.6 roll=   4.5 pitch=   5.1 head= 240.4 filt= 236.7 bias=  -1.3 w=0.02]
STAT M1[speed=  7.85cm/s tgt=  8.04cm/s duty=  46.7% dir= 1]  M2[speed=  7.85cm/s tgt=  7.67cm/s duty=  50.3% dir= 1]  Dist[L=   14.9cm R=   14.9cm]   IMU[err=  20.4 roll=   4.2 pitch= -17.7 head= 216.4 filt= 237.5 bias=  -0.2 w=0.04]
STAT M1[speed=  7.85cm/s tgt=  7.53cm/s duty=  46.6% dir= 1]  M2[speed=  7.85cm/s tgt=  8.16cm/s duty=  50.7% dir= 1]  Dist[L=   16.5cm R=   16.5cm]   IMU[err=  21.4 roll= -10.1 pitch=   7.1 head= 225.0 filt= 238.4 bias=   1.1 w=0.01]
STAT M1[speed=  7.85cm/s tgt=  7.94cm/s duty=  46.6% dir= 1]  M2[speed=  7.85cm/s tgt=  7.76cm/s duty=  50.8% dir= 1]  Dist[L=   18.1cm R=   18.1cm]   IMU[err=  26.1 roll= -13.8 pitch=   3.7 head= 201.7 filt= 243.2 bias=   0.1 w=0.01]
STAT M1[speed=  7.85cm/s tgt=  8.20cm/s duty=  47.6% dir= 1]  M2[speed=  7.85cm/s tgt=  7.51cm/s duty=  50.2% dir= 1]  Dist[L=   19.6cm R=   19.6cm]   IMU[err=  27.8 roll=   9.3 pitch=  -2.4 head= 239.1 filt= 244.9 bias=  -0.6 w=0.02]
STAT M1[speed=  7.85cm/s tgt=  8.67cm/s duty=  47.5% dir= 1]  M2[speed=  7.85cm/s tgt=  7.06cm/s duty=  50.9% dir= 1]  Dist[L=   21.2cm R=   21.2cm]   IMU[err=  30.2 roll=  -2.4 pitch=  -4.6 head= 218.0 filt= 247.2 bias=  -1.7 w=0.03]
STAT M1[speed=  7.85cm/s tgt=  8.80cm/s duty=  47.9% dir= 1]  M2[speed=  7.85cm/s tgt=  6.94cm/s duty=  50.3% dir= 1]  Dist[L=   22.8cm R=   22.8cm]   IMU[err=  39.1 roll=   9.6 pitch=  17.9 head= 288.8 filt= 256.2 bias=  -2.0 w=0.02]
STAT M1[speed=  7.85cm/s tgt=  7.98cm/s duty=  47.2% dir= 1]  M2[speed=  7.85cm/s tgt=  7.73cm/s duty=  51.0% dir= 1]  Dist[L=   24.3cm R=   24.3cm]   IMU[err=  31.7 roll=   1.7 pitch= -15.8 head= 210.1 filt= 248.7 bias=   0.0 w=0.01]
STAT M1[speed=  7.85cm/s tgt=  7.99cm/s duty=  47.3% dir= 1]  M2[speed=  7.85cm/s tgt=  7.72cm/s duty=  50.9% dir= 1]  Dist[L=   25.9cm R=   25.9cm]   IMU[err=  43.1 roll=   2.2 pitch=  13.0 head= 287.4 filt= 260.1 bias=   0.0 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  8.75cm/s duty=  48.2% dir= 1]  M2[speed=  7.85cm/s tgt=  6.99cm/s duty=  50.0% dir= 1]  Dist[L=   27.5cm R=   27.5cm]   IMU[err=  44.7 roll=   0.4 pitch=   3.4 head= 247.4 filt= 261.7 bias=  -1.9 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  7.64cm/s duty=  46.9% dir= 1]  M2[speed=  7.85cm/s tgt=  8.06cm/s duty=  51.7% dir= 1]  Dist[L=   29.1cm R=   29.1cm]   IMU[err=  40.4 roll=   3.3 pitch=   3.1 head= 250.0 filt= 257.5 bias=   0.9 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  8.90cm/s duty=  47.6% dir= 1]  M2[speed=  7.85cm/s tgt=  6.84cm/s duty=  49.9% dir= 1]  Dist[L=   30.6cm R=   30.6cm]   IMU[err=  45.3 roll=  -3.1 pitch=  -8.5 head= 204.9 filt= 262.3 bias=  -2.3 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  8.13cm/s duty=  47.5% dir= 1]  M2[speed=  7.85cm/s tgt=  7.58cm/s duty=  50.7% dir= 1]  Dist[L=   32.2cm R=   32.2cm]   IMU[err=  35.8 roll=   6.0 pitch=   0.3 head= 245.5 filt= 252.8 bias=  -0.4 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  7.76cm/s duty=  46.9% dir= 1]  M2[speed=  7.85cm/s tgt=  7.94cm/s duty=  51.6% dir= 1]  Dist[L=   33.8cm R=   33.8cm]   IMU[err=  20.5 roll=  -5.7 pitch=  -2.2 head= 204.0 filt= 237.6 bias=   0.6 w=0.01]
STAT M1[speed=  7.85cm/s tgt=  7.86cm/s duty=  47.0% dir= 1]  M2[speed=  7.85cm/s tgt=  7.84cm/s duty=  51.3% dir= 1]  Dist[L=   35.3cm R=   35.3cm]   IMU[err=  50.7 roll=   1.0 pitch=  18.7 head= 317.9 filt= 267.8 bias=   0.3 w=0.03]
STAT M1[speed=  7.85cm/s tgt=  8.85cm/s duty=  48.4% dir= 1]  M2[speed=  7.85cm/s tgt=  6.89cm/s duty=  50.6% dir= 1]  Dist[L=   36.9cm R=   36.9cm]   IMU[err=  46.8 roll=   7.6 pitch=  23.4 head= 315.2 filt= 263.9 bias=  -2.1 w=0.01]
STAT M1[speed=  7.85cm/s tgt=  7.83cm/s duty=  47.5% dir= 1]  M2[speed=  7.85cm/s tgt=  7.88cm/s duty=  51.5% dir= 1]  Dist[L=   38.5cm R=   38.5cm]   IMU[err=  56.0 roll=   1.3 pitch=   6.5 head= 270.9 filt= 273.0 bias=   0.4 w=0.02]
STAT M1[speed=  7.85cm/s tgt=  6.62cm/s duty=  45.4% dir= 1]  M2[speed=  7.85cm/s tgt=  9.05cm/s duty=  52.9% dir= 1]  Dist[L=   40.1cm R=   40.1cm]   IMU[err=  42.3 roll=   9.4 pitch=   6.1 head= 270.0 filt= 259.3 bias=   3.4 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  5.62cm/s duty=  44.3% dir= 1]  M2[speed=  7.85cm/s tgt= 10.01cm/s duty=  54.1% dir= 1]  Dist[L=   41.6cm R=   41.6cm]   IMU[err=  28.4 roll=   8.0 pitch=  19.1 head= 308.8 filt= 245.4 bias=   5.9 w=0.01]
STAT M1[speed=  7.85cm/s tgt=  6.96cm/s duty=  46.0% dir= 1]  M2[speed=  7.85cm/s tgt=  8.72cm/s duty=  51.8% dir= 1]  Dist[L=   43.2cm R=   43.2cm]   IMU[err=  34.6 roll=   6.5 pitch=  11.2 head= 290.4 filt= 251.7 bias=   2.6 w=0.01]
STAT M1[speed=  7.85cm/s tgt=  8.78cm/s duty=  48.5% dir= 1]  M2[speed=  7.85cm/s tgt=  6.96cm/s duty=  50.5% dir= 1]  Dist[L=   44.8cm R=   44.8cm]   IMU[err=  45.4 roll=   2.5 pitch=  10.2 head= 294.3 filt= 262.4 bias=  -2.0 w=0.02]
STAT M1[speed=  7.85cm/s tgt=  6.84cm/s duty=  45.9% dir= 1]  M2[speed=  7.85cm/s tgt=  8.83cm/s duty=  52.5% dir= 1]  Dist[L=   46.3cm R=   46.3cm]   IMU[err=  55.3 roll=   4.3 pitch=  12.6 head= 301.5 filt= 272.3 bias=   2.9 w=0.01]
STAT M1[speed=  7.85cm/s tgt=  7.83cm/s duty=  46.9% dir= 1]  M2[speed=  7.85cm/s tgt=  7.87cm/s duty=  51.7% dir= 1]  Dist[L=   47.9cm R=   47.9cm]   IMU[err=  36.3 roll=   4.5 pitch=   4.0 head= 266.3 filt= 253.4 bias=   0.4 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  8.02cm/s duty=  47.8% dir= 1]  M2[speed=  7.85cm/s tgt=  7.69cm/s duty=  51.1% dir= 1]  Dist[L=   49.5cm R=   49.5cm]   IMU[err=  53.7 roll=   2.5 pitch=  14.7 head= 317.6 filt= 270.8 bias=  -0.1 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  8.15cm/s duty=  48.0% dir= 1]  M2[speed=  7.85cm/s tgt=  7.56cm/s duty=  51.0% dir= 1]  Dist[L=   51.1cm R=   51.1cm]   IMU[err=  32.1 roll=   1.3 pitch=   4.5 head= 273.3 filt= 249.2 bias=  -0.4 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  8.83cm/s duty=  49.1% dir= 1]  M2[speed=  3.93cm/s tgt=  6.90cm/s duty=  51.9% dir= 1]  Dist[L=   52.2cm R=   52.6cm]   IMU[err=  42.2 roll=   5.8 pitch=  12.2 head= 306.2 filt= 259.2 bias=  -2.1 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  7.84cm/s duty=  47.5% dir= 1]  M2[speed=  7.85cm/s tgt=  7.86cm/s duty=  51.8% dir= 1]  Dist[L=   53.8cm R=   54.2cm]   IMU[err=   0.0 roll=   6.1 pitch= -11.3 head= 216.4 filt= 218.4 bias=   0.4 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  7.65cm/s duty=  46.5% dir= 1]  M2[speed=  7.85cm/s tgt=  8.05cm/s duty=  52.4% dir= 1]  Dist[L=   55.4cm R=   55.8cm]   IMU[err=   6.2 roll=  -3.8 pitch=  11.0 head= 353.1 filt= 223.2 bias=   0.8 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  7.68cm/s duty=  46.3% dir= 1]  M2[speed=  7.85cm/s tgt=  8.02cm/s duty=  52.5% dir= 1]  Dist[L=   56.9cm R=   57.3cm]   IMU[err=  22.0 roll=   2.8 pitch=  13.2 head= 323.7 filt= 239.0 bias=   0.7 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  7.68cm/s duty=  46.2% dir= 1]  M2[speed=  7.85cm/s tgt=  8.02cm/s duty=  52.7% dir= 1]  Dist[L=   58.5cm R=   58.9cm]   IMU[err=  14.4 roll=  -2.3 pitch=  12.8 head= 349.8 filt= 231.4 bias=   0.7 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  7.60cm/s duty=  45.9% dir= 1]  M2[speed=  7.85cm/s tgt=  8.10cm/s duty=  52.9% dir= 1]  Dist[L=   60.1cm R=   60.5cm]   IMU[err=  37.1 roll=  -3.2 pitch=  -1.2 head= 184.8 filt= 254.2 bias=   0.9 w=0.01]
STAT M1[speed=  7.85cm/s tgt=  7.46cm/s duty=  45.2% dir= 1]  M2[speed=  7.85cm/s tgt=  8.24cm/s duty=  54.0% dir= 1]  Dist[L=   61.7cm R=   62.0cm]   IMU[err=  41.9 roll=   6.7 pitch=  11.0 head= 306.2 filt= 258.9 bias=   1.3 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  8.10cm/s duty=  44.7% dir= 1]  M2[speed=  7.85cm/s tgt=  7.61cm/s duty=  54.5% dir= 1]  Dist[L=   63.2cm R=   63.6cm]   IMU[err=  29.2 roll=  -1.1 pitch=  -0.2 head= 204.9 filt= 246.2 bias=  -0.3 w=0.01]
STAT M1[speed=  7.85cm/s tgt=  5.70cm/s duty=  42.6% dir= 1]  M2[speed=  7.85cm/s tgt=  9.96cm/s duty=  57.0% dir= 1]  Dist[L=   64.8cm R=   65.2cm]   IMU[err=  74.5 roll=   9.3 pitch=  10.8 head= 300.1 filt= 291.6 bias=   5.7 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  7.53cm/s duty=  43.3% dir= 1]  M2[speed=  7.85cm/s tgt=  8.17cm/s duty=  56.3% dir= 1]  Dist[L=   66.4cm R=   66.8cm]   IMU[err=   4.4 roll=   0.6 pitch= -18.4 head= 188.2 filt= 221.5 bias=   1.1 w=0.00]
STAT M1[speed=  3.93cm/s tgt=  8.12cm/s duty=  46.0% dir= 1]  M2[speed=  7.85cm/s tgt=  7.59cm/s duty=  54.8% dir= 1]  Dist[L=   67.9cm R=   67.9cm]   IMU[err=  19.8 roll=  -7.0 pitch=  -1.0 head= 137.1 filt= 236.9 bias=  -0.4 w=0.01]
STAT M1[speed=  7.85cm/s tgt=  7.98cm/s duty=  45.6% dir= 1]  M2[speed=  7.85cm/s tgt=  7.73cm/s duty=  54.0% dir= 1]  Dist[L=   69.5cm R=   69.5cm]   IMU[err=  -2.1 roll=   1.3 pitch=   3.3 head= 263.8 filt= 214.9 bias=  -0.0 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  8.09cm/s duty=  45.1% dir= 1]  M2[speed=  7.85cm/s tgt=  7.62cm/s duty=  53.9% dir= 1]  Dist[L=   71.1cm R=   71.5cm]   IMU[err=  18.3 roll=  -5.4 pitch=  -3.4 head= 155.7 filt= 235.3 bias=  -0.3 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  7.11cm/s duty=  43.3% dir= 1]  M2[speed= 11.78cm/s tgt=  8.57cm/s duty=  52.7% dir= 1]  Dist[L=   73.0cm R=   73.0cm]   IMU[err= -23.1 roll=  21.5 pitch= -13.1 head= 232.5 filt= 194.0 bias=   2.2 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  7.14cm/s duty=  43.8% dir= 1]  M2[speed=  7.85cm/s tgt=  8.54cm/s duty=  54.0% dir= 1]  Dist[L=   74.6cm R=   74.6cm]   IMU[err=  -4.9 roll=  -2.2 pitch=   2.4 head= 162.0 filt= 212.1 bias=   2.1 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  7.44cm/s duty=  43.4% dir= 1]  M2[speed=  3.93cm/s tgt=  8.25cm/s duty=  56.1% dir= 1]  Dist[L=   75.8cm R=   76.2cm]   IMU[err=  17.2 roll=  -2.9 pitch=  34.4 head=   2.3 filt= 234.3 bias=   1.3 w=0.01]
STAT M1[speed=  7.85cm/s tgt=  6.75cm/s duty=  41.4% dir= 1]  M2[speed=  7.85cm/s tgt=  8.92cm/s duty=  57.0% dir= 1]  Dist[L=   77.4cm R=   77.8cm]   IMU[err= -13.3 roll=  10.9 pitch= -13.2 head= 213.4 filt= 203.8 bias=   3.1 w=0.00]
STAT M1[speed=  7.85cm/s tgt=  5.02cm/s duty=  36.6% dir= 1]  M2[speed=  7.85cm/s tgt= 10.60cm/s duty=  60.6% dir= 1]  Dist[L=   78.9cm R=   79.7cm]   IMU[err=  13.0 roll=  10.7 pitch=  -2.4 head= 237.5 filt= 230.0 bias=   7.4 w=0.00]
STOP -> stopped.


FF M1: cps ≈ 0.567*duty + -8.778
FF M2: cps ≈ 0.533*duty + -10.222
Done FF calibration.
SETTLE 2s...
FAST: tgt=6.00 cm/s | M1=2.16 cm/s duty=43.5  M2=9.02 cm/s duty=46.7
FAST: tgt=6.00 cm/s | M1=4.56 cm/s duty=42.4  M2=12.88 cm/s duty=28.9
1s CPS: exp=15.28  M1=16.00  M2=15.00  |  Dist: M1=49.1 cm  M2=51.8 cm  (cm/tick=0.3927)
STEP: target = 7.85 cm/s (expected CPS=19.99)
Wheel D=25.0 mm  TPR=20.0  circumference=7.8540 cm
Anti-stiction: min duty 20.0% for 800 ms; PID(Kp=0.30,Ki=1.60,Kd=0.004), I clamp=±30%
Per-wheel target scale: M1=1.000  M2=0.960
Feedforward calibration...


FF M1: cps ≈ 0.567*duty + -7.889
FF M2: cps ≈ 0.733*duty + -21.111
Done FF calibration.
SETTLE 2s...
FAST: tgt=0.00 cm/s | M1=0.00 cm/s duty=13.9  M2=0.00 cm/s duty=28.8
FAST: tgt=0.00 cm/s | M1=0.00 cm/s duty=13.9  M2=0.00 cm/s duty=28.8
1s CPS: exp=0.00  M1=0.00  M2=0.00  |  Dist: M1=0.0 cm  M2=0.0 cm  (cm/tick=0.3927)
FAST: tgt=0.00 cm/s | M1=0.00 cm/s duty=13.9  M2=0.00 cm/s duty=28.8
FAST: tgt=0.00 cm/s | M1=0.00 cm/s duty=13.9  M2=0.00 cm/s duty=28.8
1s CPS: exp=0.00  M1=0.00  M2=0.00  |  Dist: M1=0.0 cm  M2=0.0 cm  (cm/tick=0.3927)
STEP: target = 6.00 cm/s (expected CPS=15.28)
FAST: tgt=6.00 cm/s | M1=0.00 cm/s duty=51.8  M2=0.00 cm/s duty=59.3
FAST: tgt=6.00 cm/s | M1=3.41 cm/s duty=45.9  M2=9.93 cm/s duty=48.5
1s CPS: exp=15.28  M1=17.00  M2=19.00  |  Dist: M1=6.7 cm  M2=7.5 cm  (cm/tick=0.3927)
FAST: tgt=6.00 cm/s | M1=4.82 cm/s duty=41.8  M2=9.84 cm/s duty=44.2
FAST: tgt=6.00 cm/s | M1=6.58 cm/s duty=39.3  M2=13.35 cm/s duty=23.6
1s CPS: exp=15.28  M1=17.00  M2=17.00  |  Dist: M1=13.4 cm  M2=14.1 cm  (cm/tick=0.3927)
FAST: tgt=6.00 cm/s | M1=2.25 cm/s duty=39.7  M2=6.52 cm/s duty=41.2
FAST: tgt=6.00 cm/s | M1=13.35 cm/s duty=17.8  M2=4.56 cm/s duty=40.6
1s CPS: exp=15.28  M1=16.00  M2=16.00  |  Dist: M1=19.6 cm  M2=20.4 cm  (cm/tick=0.3927)
FAST: tgt=6.00 cm/s | M1=13.35 cm/s duty=17.2  M2=9.01 cm/s duty=38.5
FAST: tgt=6.00 cm/s | M1=4.30 cm/s duty=37.5  M2=0.69 cm/s duty=44.2
1s CPS: exp=15.28  M1=14.00  M2=12.00  |  Dist: M1=25.1 cm  M2=25.1 cm  (cm/tick=0.3927)
FAST: tgt=6.00 cm/s | M1=1.47 cm/s duty=40.8  M2=3.19 cm/s duty=44.2
FAST: tgt=6.00 cm/s | M1=1.56 cm/s duty=41.8  M2=3.08 cm/s duty=44.7
1s CPS: exp=15.28  M1=14.00  M2=14.00  |  Dist: M1=30.6 cm  M2=30.6 cm  (cm/tick=0.3927)
FAST: tgt=6.00 cm/s | M1=12.88 cm/s duty=21.4  M2=1.51 cm/s duty=45.6
FAST: tgt=6.00 cm/s | M1=6.29 cm/s duty=41.1  M2=8.78 cm/s duty=42.9
1s CPS: exp=15.28  M1=15.00  M2=15.00  |  Dist: M1=36.5 cm  M2=36.5 cm  (cm/tick=0.3927)
FAST: tgt=6.00 cm/s | M1=2.23 cm/s duty=43.1  M2=6.29 cm/s duty=44.0
FAST: tgt=6.00 cm/s | M1=4.81 cm/s duty=41.9  M2=6.28 cm/s duty=44.5
1s CPS: exp=15.28  M1=15.00  M2=13.00  |  Dist: M1=42.4 cm  M2=41.6 cm  (cm/tick=0.3927)
FAST: tgt=6.00 cm/s | M1=4.40 cm/s duty=43.0  M2=1.47 cm/s duty=47.8
FAST: tgt=6.00 cm/s | M1=3.37 cm/s duty=42.7  M2=14.14 cm/s duty=25.0
1s CPS: exp=15.28  M1=17.00  M2=17.00  |  Dist: M1=49.1 cm  M2=48.3 cm  (cm/tick=0.3927)
STEP: target = 7.85 cm/s (expected CPS=19.99)
FAST: tgt=7.85 cm/s | M1=2.36 cm/s duty=59.2  M2=3.21 cm/s duty=62.9
FAST: tgt=7.85 cm/s | M1=5.32 cm/s duty=50.8  M2=5.00 cm/s duty=55.6
1s CPS: exp=19.99  M1=22.00  M2=20.00  |  Dist: M1=57.7 cm  M2=56.2 cm  (cm/tick=0.3927)
FAST: tgt=7.85 cm/s | M1=9.91 cm/s duty=49.7  M2=7.02 cm/s duty=57.7
FAST: tgt=7.85 cm/s | M1=9.91 cm/s duty=49.7  M2=14.33 cm/s duty=36.6
1s CPS: exp=19.99  M1=20.00  M2=21.00  |  Dist: M1=65.6 cm  M2=64.4 cm  (cm/tick=0.3927)
FAST: tgt=7.85 cm/s | M1=9.91 cm/s duty=49.7  M2=9.92 cm/s duty=54.1
FAST: tgt=7.85 cm/s | M1=14.16 cm/s duty=31.5  M2=14.16 cm/s duty=35.3
1s CPS: exp=19.99  M1=19.00  M2=19.00  |  Dist: M1=73.0 cm  M2=71.9 cm  (cm/tick=0.3927)
FAST: tgt=7.85 cm/s | M1=14.16 cm/s duty=31.5  M2=3.40 cm/s duty=55.9
FAST: tgt=7.85 cm/s | M1=14.30 cm/s duty=31.5  M2=4.86 cm/s duty=54.9
1s CPS: exp=19.99  M1=21.00  M2=20.00  |  Dist: M1=81.3 cm  M2=79.7 cm  (cm/tick=0.3927)
FAST: tgt=7.85 cm/s | M1=9.93 cm/s duty=49.6  M2=6.94 cm/s duty=53.8
FAST: tgt=7.85 cm/s | M1=9.91 cm/s duty=49.6  M2=9.90 cm/s duty=52.5
1s CPS: exp=19.99  M1=20.00  M2=19.00  |  Dist: M1=89.1 cm  M2=87.2 cm  (cm/tick=0.3927)
FAST: tgt=7.85 cm/s | M1=9.91 cm/s duty=49.6  M2=14.16 cm/s duty=33.7
FAST: tgt=7.85 cm/s | M1=9.91 cm/s duty=49.6  M2=4.85 cm/s duty=53.9
1s CPS: exp=19.99  M1=20.00  M2=20.00  |  Dist: M1=97.0 cm  M2=95.0 cm  (cm/tick=0.3927)
FAST: tgt=7.85 cm/s | M1=9.91 cm/s duty=49.6  M2=9.91 cm/s duty=52.1
FAST: tgt=7.85 cm/s | M1=9.91 cm/s duty=49.6  M2=3.40 cm/s duty=54.6
1s CPS: exp=19.99  M1=20.00  M2=19.00  |  Dist: M1=104.9 cm  M2=102.5 cm  (cm/tick=0.3927)
FAST: tgt=7.85 cm/s | M1=9.91 cm/s duty=49.6  M2=6.94 cm/s duty=53.1
FAST: tgt=7.85 cm/s | M1=7.44 cm/s duty=50.1  M2=3.23 cm/s duty=54.9
1s CPS: exp=19.99  M1=20.00  M2=19.00  |  Dist: M1=112.7 cm  M2=110.0 cm  (cm/tick=0.3927)

my car still slightly drift to the right the to the left as the car drive further away? how can i calibrate it so that the car move in straight line? can use IMU sensor provides acceleration, orientation, and 
filtered heading to ensure straight motion? Show full modify code if any

Can u make my code overall more modular, readability and easier to debugged my code, make it easier to understand for the whole logic flow between each module and sensor.
Reduce any redundant, remove any unwanted function and variable if any. Show full code and highlight the modify part of the code


Can u explain detail to me the flow between each line of the code with the serial monitor output with example for me, u may also modify the serial output for me to have a better understanding in order to calibrate and tune the code to prevent the any slightly drift to either to right or left side as the car drive further away? 



Show exactly full modified code here if for the part that is not modified type it out in exactly same format for me to copy and paste directly again.

heading_ref=312.3 deg (initial_heading_deg) The heading captured at “START.” The direction your car should go straight toward.
err: If you drift right (err positive), the system steers left. If you drift left (err negative), it steers right.
err = +10°.
The controller then commands the right wheel to slow down and left wheel to speed up (turning left to fix it).
err = -10°.
The controller does the opposite: speeds up right wheel, slows down left wheel.
What the Deadband Does (±2°): That means if your heading only drifts a little (within ±2°), ignore it — it’s too small to matter and could cause oscillation (jittery steering).

head is Current heading (°) Compass reading (0–360) (wraps around 360)
filt=213.4 A low-pass filtered version of heading for smoothness. Like averaging your steering over the last second to avoid jitter.
base_cps is target counts per second for each wheel at your chosen speed (e.g., 20 %).
The IMU and encoder PIDs output a bias in cps (bias_head, bias_track), which represent how much faster or slower one wheel should spin to steer back.
Meaning: 
Right wheel gets +total_bias
Left wheel gets −total_bias

w = HS.head_weight: Determined by IMU health (tilt, rate, etc.) in the supervisor block:
If the IMU is stable and level → w rises toward 1.
If it tilts or rotates too fast → w decays toward 0.
It’s smoothed with time constants (tau_up, tau_dn) so the transition isn’t abrupt.

if w = 1.0 → use 100% of the IMU heading correction.
if w = 0.54 → use 54% of the IMU heading correction (still very much used).
if w = 0.02 → use 2% (tiny, but not zero).
if w = 0.00 → no IMU heading correction (only encoder-balance PID acts).
Think of w as a “volume knob” for how loud the IMU’s voice is in steering.
Think of w as a “volume knob” for how loud the IMU’s voice is in steering.
This protects against magnetic spikes or tilt errors during motion.

Interaction Between Encoders and IMU
Encoder PID (pid_track) corrects short-term left/right imbalance (mechanical).
IMU PID (pid_heading) corrects long-term directional drift (compass).
Both biases combine into a total correction limited by lim_total (≈ 60 % of base speed).
So:
IMU gives absolute orientation,
Encoders give relative speed balance,
Together → smooth, straight travel with adaptive self-correction.