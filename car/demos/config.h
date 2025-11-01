#ifndef CONFIG_H
#define CONFIG_H

// ================= Robot Geometry =================
#define TRACK_WIDTH_M        0.115f   // wheel center-to-center (meters)

// ================= Target Speed ===================
#define V_TARGET_MPS         0.20f
#define V_TARGET_CMPS        (V_TARGET_MPS * 100.0f)

// ================= IMU Outer PID ==================
#define KP_HEADING           2.0f
#define KI_HEADING           0.05f
#define KD_HEADING           0.10f
#define HEADING_DEADBAND_DEG 1.5f
#define HEADING_RATE_SCALE   0.02f      // deg/s -> rad/s scaling
#define DEG2RAD              (3.14159265358979323846f / 180.0f)
#define HDG_EMA_ALPHA        0.08f
#define HEADING_OFFSET_DEG   0.0f       // add constant offset if sensor is skewed

// ================= DRIFT CALIBRATION ==============
// CRITICAL: Adjust this to fix drafting without IMU
// Negative value = compensate for right drift (increase left PWM, decrease right PWM)
// Positive value = compensate for left drift (decrease left PWM, increase right PWM)

// Positive s_trim increases left power | Positive s_trim decreases right power

#define BATTERY_POWER_TRIM        (+4.0f)   // for power bank (estimate) 
#define USB_POWER_TRIM            (+8.0f)   // Calibrated for PC USB

#define GLOBAL_S_TRIM_OFFSET  USB_POWER_TRIM
#define K_LINE_RAD_S          2.0f

// ================= Speed Inner PID ================
#define SPID_KP              2.5f
#define SPID_KI              0.0f
#define SPID_KD              0.3f
#define SPID_OUT_MIN         10.0f
#define SPID_OUT_MAX         100.0f
#define SPID_IWIND_CLAMP     50.0f     // integral clamp for inner PIDs

// ================= Straightness PI ===============
// Increase KP for slower correction, but decrease KP for faster response may cause oscillation
// Increase KI for eliminating steady-state drift
#define STRAIGHT_KP          0.50f      // Increased for slower correction
#define STRAIGHT_KI          0.10f     // Increased for better drift elimination
#define STRAIGHT_I_CLAMP     30.0f

// ================= PWM Base & Slew ===============
#ifndef PWM_MAX_LEFT
#define PWM_MAX_LEFT   255
#endif
#ifndef PWM_MAX_RIGHT
#define PWM_MAX_RIGHT  255
#endif
#ifndef PWM_MIN_LEFT
#define PWM_MIN_LEFT   80
#endif
#ifndef PWM_MIN_RIGHT
#define PWM_MIN_RIGHT  80
#endif
#define BASE_PWM_L     (PWM_MIN_LEFT)
#define BASE_PWM_R     (PWM_MIN_RIGHT)
#define MAX_PWM_STEP   20

// ================= Control Loop Timing ===========
#define LOOP_DT_MS     10
#define DT_S           ((float)LOOP_DT_MS / 1000.0f)

#endif // CONFIG_H