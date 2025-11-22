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


https://mosquitto.org/download/
download this:
mosquitto-2.0.22-install-windows x64.exe

 Error: Only one usage of each socket address (protocol/network address/port) is normally permitted.
admin cmd
netstat -ano | find "1883"
tasklist | find "mosquitto"
tasklist | findstr mosquitto
taskkill /PID 25576 /F

if nothing happen means mosquitto not running proceed with next task

normal cmd
cd "C:\Program Files\mosquitto"
mosquitto.exe -c mosquitto.conf -v

another normal cmd
cd "C:\Program Files\mosquitto"
mosquitto_sub -h localhost -p 1883 -t "pico/demo1/telemetry" -v

mosquitto.conf
listener 1883 0.0.0.0
allow_anonymous true
socket_domain ipv4
connection_messages true


window firewall defender 
- turn window firewall defender off for private and public network

mosquitto -v
mosquitto_sub -h localhost -t test
mosquitto_pub -h localhost -t test -m "hello"
http://127.0.0.1:1880
http://127.0.0.1:1880/ui/
node-red


// --- Wi-Fi & MQTT ---
#define WIFI_SSID                 "Keithiphone"
#define WIFI_PASS                 "testong1"
// #define WIFI_SSID                 "Jared"
// #define WIFI_PASS                 "1teddygodie"
// #define WIFI_SSID                 "Oppo"
// #define WIFI_PASS                 "happy1234"
#define WIFI_CONNECT_TIMEOUT_MS   20000

#define BROKER_IP_STR             "172.20.10.3"
// #define BROKER_IP_STR             "10.22.173.48"
// #define BROKER_IP_STR             "10.86.216.48"