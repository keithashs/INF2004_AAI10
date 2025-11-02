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

// Positive s_trim increases left power | Positive s_trim decreases right power
// Positive Left faster, Right slower | Negative Left slower, Right faster
#define BATTERY_POWER_TRIM        (+0.0f)   // for power bank (estimate) 
#define USB_POWER_TRIM            (+2.0f)   // Calibrated for PC USB

#define GLOBAL_S_TRIM_OFFSET  USB_POWER_TRIM
#define K_LINE_RAD_S          2.0f

// ================= Speed Inner PID ================
#define SPID_KP              2.0f
#define SPID_KI              0.3f
#define SPID_KD              0.0f
#define SPID_OUT_MIN         -50.0f  // Allow correction range
#define SPID_OUT_MAX         +50.0f  // Allow boost range
#define SPID_IWIND_CLAMP     166.7f  // ← OUT_MAX/KI = 50/0.3 = 166.7

// ================= Straightness PI ===============
// Increase KP for slower correction, but decrease KP for faster response may cause oscillation
// Increase KI for eliminating steady-state drift
#define STRAIGHT_KP          0.00f      // Increased for slower correction
#define STRAIGHT_KI          0.00f     // Increased for better drift elimination
#define STRAIGHT_I_CLAMP     30.0f

// ================= PWM Base & Slew ===============
#ifndef PWM_MIN_LEFT
#define PWM_MIN_LEFT   140
#endif
#ifndef PWM_MIN_RIGHT
#define PWM_MIN_RIGHT  135
#endif
#ifndef PWM_MAX_LEFT
#define PWM_MAX_LEFT   255
#endif
#ifndef PWM_MAX_RIGHT
#define PWM_MAX_RIGHT  255
#endif

#ifndef PWM_MID_LEFT
#define PWM_MID_LEFT   170
#endif
#ifndef PWM_MID_RIGHT
#define PWM_MID_RIGHT  160
#endif

#ifndef PWM_JUMPSTART
#define PWM_JUMPSTART  120
#endif

#ifndef MIN_SPEED
#define MIN_SPEED                 2.0f
#endif
#ifndef MAX_SPEED
#define MAX_SPEED                 5.0f
#endif
#ifndef TURN_SPEED
#define TURN_SPEED                5.0f
#endif
#ifndef JUMPSTART_SPEED_THRESHOLD
#define JUMPSTART_SPEED_THRESHOLD 0.1f
#endif

#define BASE_PWM_L     (PWM_MIN_LEFT)
#define BASE_PWM_R     (PWM_MIN_RIGHT)
#define MAX_PWM_STEP   20

// ================= Turning Constants =============
#define FULL_CIRCLE    360.0f
#define CONTINUOUS_TURN -1.0f

// ================= Control Loop Timing ===========
#define LOOP_DT_MS     10
#define DT_S           ((float)LOOP_DT_MS / 1000.0f)

#endif // CONFIG_H