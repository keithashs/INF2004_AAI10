#ifndef CONFIG_H
#define CONFIG_H

// ================= Robot Geometry =================
#define TRACK_WIDTH_M        0.115f   // wheel center-to-center (meters)

// ================= Target Speed ===================
#define V_TARGET_MPS         0.20f
#define V_TARGET_CMPS        (V_TARGET_MPS * 100.0f)

// ================= IMU Outer PID ==================
#define KP_HEADING           0.30f
#define KI_HEADING           0.00f
#define KD_HEADING           0.18f
#define HEADING_DEADBAND_DEG 2.0f
#define HEADING_RATE_SCALE   0.02f      // deg/s -> rad/s scaling
#define DEG2RAD              (3.14159265358979323846f / 180.0f)
#define HDG_EMA_ALPHA        0.20f
#define HEADING_OFFSET_DEG   0.0f       // add constant offset if sensor is skewed

#define GLOBAL_S_TRIM_OFFSET  (+6.0f) // negative draft left, positive draft right (5 to 10)
#define K_LINE_RAD_S          2.0f

// ================= Speed Inner PID ================
#define SPID_KP              6.0f
#define SPID_KI              0.8f
#define SPID_KD              0.0f
#define SPID_OUT_MIN         0.0f
#define SPID_OUT_MAX         255.0f
#define SPID_IWIND_CLAMP     300.0f     // integral clamp for inner PIDs

// ================= Straightness PI ===============
#define STRAIGHT_KP          1.2f
#define STRAIGHT_KI          0.30f
#define STRAIGHT_I_CLAMP     50.0f

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
#define MAX_PWM_STEP   6

// ================= Control Loop Timing ===========
#define LOOP_DT_MS     10
#define DT_S           ((float)LOOP_DT_MS / 1000.0f)

#endif // CONFIG_H