#ifndef CONFIG_H
#define CONFIG_H

// ================= Robot Geometry =================
#define TRACK_WIDTH_M        0.115f   // wheel center-to-center (meters)

// ================= Target Speed ===================
#define V_TARGET_MPS         0.20f
#define V_TARGET_CMPS        (V_TARGET_MPS * 100.0f)

// ================= IMU Outer PID ==================
#ifndef KP_HEADING
#define KP_HEADING           0.30f
#endif
#ifndef KI_HEADING
#define KI_HEADING           0.00f      // keep 0 during inner-loop ID
#endif
#ifndef KD_HEADING
#define KD_HEADING           0.18f
#endif
#define HEADING_DEADBAND_DEG 2.0f
#define HEADING_RATE_SCALE   0.02f      // deg/s -> rad/s scaling (lower = less aggressive)
#define DEG2RAD              (3.14159265358979323846f / 180.0f)
#define HDG_EMA_ALPHA        0.20f
#define HEADING_OFFSET_DEG   20.0f       // add constant offset if sensor is skewed

// ================= DRIFT CALIBRATION ==============
#ifndef WHEEL_TRIM_LEFT
#define WHEEL_TRIM_LEFT      0   // start at 0; tweak after encoder & FF tuning
#endif
#ifndef WHEEL_TRIM_RIGHT
#define WHEEL_TRIM_RIGHT     -5   // start at 0; tweak after encoder & FF tuning
#endif

// ================= Feed-Forward Control ===========
// Simple linear model per wheel: PWM ≈ kS + kV * v_cmd (v in cm/s)
// Start with these; refine from logs (steady-state PWM vs. measured v)
#ifndef KS_L
#define KS_L 70.0f
#endif
#ifndef KV_L
#define KV_L 3.0f
#endif
#ifndef KS_R
#define KS_R 70.0f
#endif
#ifndef KV_R
#define KV_R 3.0f
#endif

// Gate to disable FF quickly if needed
#ifndef FF_ENABLE
#define FF_ENABLE 1
#endif

// ================= Encoder Scaling ================
// Temporary per-wheel scale (1.0 = no correction). Use 1m test to compute these.
#ifndef ENC_SCALE_L
#define ENC_SCALE_L 1.0f
#endif
#ifndef ENC_SCALE_R
#define ENC_SCALE_R 1.0f
#endif

// ================= Speed Inner PID ================
// Allow PID to command both up and down around FF (signed)
#define SPID_KP              6.0f
#define SPID_KI              0.50f  // slightly lower than before; FF carries most of the load
#define SPID_KD              0.0f
#define SPID_OUT_MIN         (-255.0f)
#define SPID_OUT_MAX         ((float)PWM_MAX_RIGHT)
#define SPID_IWIND_CLAMP     300.0f

// ================= Straightness PI ===============
// Increase KP for slower correction, but decrease KP for faster response may cause oscillation
// Increase KI for eliminating steady-state drift
#define STRAIGHT_KP          1.2f      // Increased for slower correction
#define STRAIGHT_KI          0.30f     // Increased for better drift elimination
#define STRAIGHT_I_CLAMP     50.0f

// ================= PWM Base & Slew ===============
#ifndef PWM_MIN_LEFT
#define PWM_MIN_LEFT   80  // Battery VIN(3.6V–6V)
#endif
#ifndef PWM_MIN_RIGHT
#define PWM_MIN_RIGHT  80  // Battery VIN(3.6V–6V)
#endif
#ifndef PWM_MAX_LEFT
#define PWM_MAX_LEFT   255
#endif
#ifndef PWM_MAX_RIGHT
#define PWM_MAX_RIGHT  255
#endif

#ifndef PWM_MID_LEFT
#define PWM_MID_LEFT   160
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

// --- No base PWM; FF takes its place ---
#define BASE_PWM_L           0
#define BASE_PWM_R           0
#define MAX_PWM_STEP         6

// ================= Turning Constants =============
#define FULL_CIRCLE    360.0f
#define CONTINUOUS_TURN -1.0f

// ================= Control Loop Timing ===========
#define LOOP_DT_MS     10
#define DT_S           ((float)LOOP_DT_MS / 1000.0f)

#endif // CONFIG_H