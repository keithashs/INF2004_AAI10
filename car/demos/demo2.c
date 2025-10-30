// demo2.c - Line Following + Barcode Navigation + IMU Heading Correction
// Purpose: Complete perception-decision-action loop for autonomous navigation
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "pid.h"
#include "ir_linefollow.h"
#include "barcode.h"

// ============================================================================
// CONFIGURATION PARAMETERS
// ============================================================================

#define LINE_FOLLOW_SPEED_PCT  10       // Base speed (matches demo1 20%)
#define TURN_SPEED_PCT         10       // Speed during turns (slightly lower)
#define TURN_ANGLE_90          90.0f    // LEFT/RIGHT turn angle
#define TURN_ANGLE_180         180.0f   // U-TURN angle
#define TURN_TOLERANCE_DEG     5.0f     // Turn completion threshold (±5°)
#define TURN_OVERSHOOT_COMP    3.0f     // Overshoot compensation (degrees)

#define LOOP_DT_MS             10       // 100 Hz control loop (same as demo1)
#define DT_S                   ((float)LOOP_DT_MS / 1000.0f)

// ============================================================================
// LINE FOLLOWING PID GAINS (optimized for 1.7cm thin line)
// ============================================================================
// These are SEPARATE from motor PID - this controls steering bias
#define LINE_KP                0.50f    //gentler proportional response
#define LINE_KI                0.15f    // Reduced from 0.18f - slower integral buildup
#define LINE_KD                0.08f    // Reduced from 0.25f - less aggressive damping
#define LINE_BIAS_MAX_CPS      12.0f    // Reduced from 18.0f - limit max correction

// IMU heading correction gains (keep gentler for stability)
#define IMU_HEADING_KP         0.30f
#define IMU_HEADING_KI         0.03f
#define IMU_HEADING_KD         0.00f
#define IMU_BIAS_MAX_CPS       4.0f

// Line loss recovery
#define LINE_LOST_COUNT_MAX    30      // 0.3s of no line = lost (100Hz * 0.3s)
#define LINE_RECOVERY_TIMEOUT  500      // 5.0s max recovery attempt
#define RECOVERY_SPEED_PCT     10       // Slow speed during recovery

// Barcode detection safety
#define BARCODE_MIN_INTERVAL_MS  3000   // 3 seconds between barcode actions

// ============================================================================
// STATE MACHINE
// ============================================================================
typedef enum {
    STATE_IDLE,              // Waiting for START button
    STATE_LINE_FOLLOW,       // Normal line following mode
    STATE_BARCODE_DETECTED,  // Barcode just detected, processing command
    STATE_EXECUTING_TURN,    // Performing commanded turn
    STATE_REACQUIRE_LINE,    // Finding line after turn
    STATE_STOPPED,           // Emergency stop or completion
    STATE_LINE_LOST          // Line lost, attempting recovery
} robot_state_t;

// ============================================================================
// GLOBAL STATE
// ============================================================================
static robot_state_t g_state = STATE_IDLE;
static barcode_cmd_t g_pending_command = BARCODE_NONE;
static float g_target_heading_deg = 0.0f;
static float g_initial_heading_deg = 0.0f;
static uint32_t g_line_lost_count = 0;
static uint32_t g_recovery_count = 0;
static absolute_time_t g_last_barcode_time;
static bool g_first_run = true;

// NEW: Directional memory for smarter recovery
static float g_last_error_direction = 0.0f;  // Positive = was drifting left
static bool g_have_error_memory = false;

// Run control
static volatile bool running = false;
static volatile bool g_override_motion = false;

// Timers
static repeating_timer_t control_timer_motor;
static repeating_timer_t control_timer_main;

// ============================================================================
// LINE FOLLOWING PID (separate from motor PID)
// ============================================================================
static PID pid_line_follow = {
    .kp = LINE_KP,
    .ki = LINE_KI,
    .kd = LINE_KD,
    .integ = 0,
    .prev_err = 0,
    .out_min = -LINE_BIAS_MAX_CPS,
    .out_max = LINE_BIAS_MAX_CPS
};

// ============================================================================
// TELEMETRY CACHE (for 5Hz printing)
// ============================================================================
static struct {
    line_reading_t line_reading;
    barcode_cmd_t last_barcode;
    float heading_deg;
    float heading_error_deg;
    float speed_cmps;
    float distance_cm;
    float line_bias_cps;
    float imu_bias_cps;
    float encoder_diff_cps;
    robot_state_t state;
    uint32_t loop_count;
} g_telem = {0};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static inline float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static inline bool btn_pressed(uint gpio) {
    return gpio_get(gpio) == 0;
}

// ============================================================================
// LINE FOLLOWING CONTROLLER (PID on analog error)
// ============================================================================
static float line_follow_pid_controller(line_reading_t *reading, float dt) {
    float err = reading->error;
    
    // Store error direction for recovery (only when confident)
    if (reading->line_detected && reading->confidence > 0.7f) {
        g_last_error_direction = err;
        g_have_error_memory = true;
    }
    
    // Apply PID with confidence-based anti-windup
    float bias = pid_step(&pid_line_follow, 0.0f, -err, dt);
    
    // Scale output based on confidence - less confident = gentler correction
    if (reading->confidence < 0.8f) {
        bias *= (0.6f + 0.4f * reading->confidence);  // Scale: 0.6 to 1.0
        
        // Reduce integral windup when uncertain
        if (reading->confidence < 0.5f) {
            pid_line_follow.integ *= 0.95f;  // Decay integral term
        }
    }
    
    g_telem.line_bias_cps = bias;
    return bias;
}

// ============================================================================
// IMU HEADING CORRECTION (Long-term drift prevention)
// ============================================================================
static float imu_heading_correction(float current_heading, float dt) {
    // Reuse demo1's IMU heading bias calculation with supervisor
    float bias = imu_get_heading_bias(g_initial_heading_deg, dt);
    
    // Store for telemetry
    g_telem.imu_bias_cps = bias;
    g_telem.heading_error_deg = g_heading_err_deg;
    
    return bias;
}

// ============================================================================
// TURN EXECUTION (With IMU feedback)
// ============================================================================
static bool execute_turn(float current_heading, float dt) {
    static float last_hdg = 0.0f;
    static int stall_count = 0;
    static int settle_count = 0;

    float delta = wrap180(g_target_heading_deg - current_heading);
    
    // Check if we're within tolerance
    if (fabsf(delta) < TURN_TOLERANCE_DEG) {
        settle_count++;
        
        // Require stable heading for 300ms (30 samples @ 100Hz)
        if (settle_count > 30) {
            motion_command(MOVE_STOP, 0);
            stall_count = 0;
            settle_count = 0;
            printf("[DEMO2] Turn complete: target=%.1f° current=%.1f° delta=%.1f°\n",
                   g_target_heading_deg, current_heading, delta);
            return true;
        }
    } else {
        settle_count = 0;
    }

    // Proportional turn rate with speed limits
    float turn_rate_cps = clampf(delta * 0.7f, -20.0f, 20.0f);
    
    // Apply differential turn (one wheel forward, one backward)
    if (delta > 0) {
        // Need to turn left (CCW)
        motion_command_with_bias(MOVE_FORWARD, TURN_SPEED_PCT, 
                                -fabsf(turn_rate_cps), fabsf(turn_rate_cps));
    } else {
        // Need to turn right (CW)
        motion_command_with_bias(MOVE_FORWARD, TURN_SPEED_PCT, 
                                fabsf(turn_rate_cps), -fabsf(turn_rate_cps));
    }

    // Stall detection (heading not changing)
    if (fabsf(current_heading - last_hdg) < 0.3f) {
        stall_count++;
        if (stall_count > 150) { // 1.5 seconds
            printf("[DEMO2] Turn stalled, forcing completion\n");
            stall_count = 0;
            settle_count = 0;
            return true;
        }
    } else {
        stall_count = 0;
    }
    last_hdg = current_heading;

    return false;
}

// ============================================================================
// LINE REACQUISITION (with directional memory & speed ramping)
// ============================================================================
static bool reacquire_line(line_reading_t *reading) {
    static uint32_t reacq_count = 0;
    static bool sweep_right = true;
    static uint32_t speed_ramp = 0;
    
    // Check if line found with HIGH confidence
    if (reading->line_detected && reading->confidence > 0.8f) {
        reacq_count = 0;
        sweep_right = true;
        speed_ramp = 0;
        printf("[DEMO2] Line reacquired (ADC=%u, conf=%.2f)\n", 
               reading->raw_adc, reading->confidence);
        return true;
    }
    
    reacq_count++;
    
    // Use error memory to determine initial sweep direction
    if (reacq_count == 1 && g_have_error_memory) {
        // If we were drifting left (positive error), search right first
        sweep_right = (g_last_error_direction > 0.5f);
        printf("[REACQUIRE] Using error memory: drift=%.2f, searching %s\n",
               g_last_error_direction, sweep_right ? "RIGHT" : "LEFT");
    }
    
    // Speed ramping: start slow, increase gradually
    speed_ramp = (reacq_count / 10) + 10;  // 10% to 20% over 100 iterations
    if (speed_ramp > RECOVERY_SPEED_PCT) speed_ramp = RECOVERY_SPEED_PCT;
    
    // Alternate sweep direction every 0.5s (50 iterations)
    if (reacq_count % 50 == 0) {
        sweep_right = !sweep_right;
        printf("[REACQUIRE] Changing direction to %s (count=%lu)\n",
               sweep_right ? "RIGHT" : "LEFT", reacq_count);
    }
    
    if (sweep_right) {
        motion_command(MOVE_RIGHT, speed_ramp);
    } else {
        motion_command(MOVE_LEFT, speed_ramp);
    }
    
    // Timeout after 3 seconds (300 iterations @ 100Hz)
    if (reacq_count > 300) {
        printf("[REACQUIRE] Timeout after %lu iterations\n", reacq_count);
        reacq_count = 0;
        sweep_right = true;
        speed_ramp = 0;
        return false;
    }
    
    return false;
}

// ============================================================================
// LINE LOSS RECOVERY (with directional memory)
// ============================================================================
static bool attempt_line_recovery(line_reading_t *reading) {
    // static bool recovery_phase = 0;  // 0=reverse, 1=sweep
    
    g_recovery_count++;
    
    // Check if line reacquired with good confidence
    if (reading->line_detected && reading->confidence > 0.7f) {
        g_recovery_count = 0;
        // recovery_phase = 0;
        printf("[RECOVERY] Line reacquired (ADC=%u)\n", reading->raw_adc);
        return true;
    }
    
    // Phase 0: Reverse slowly (first 1 second = 100 iterations)
    if (g_recovery_count < 100) {
        if (g_recovery_count == 1) {
            printf("[RECOVERY] Phase 1: Reversing...\n");
        }
        motion_command(MOVE_BACKWARD, RECOVERY_SPEED_PCT);
    }
    // Phase 1: Sweep based on last error direction
    else {
        if (g_recovery_count == 100) {
            printf("[RECOVERY] Phase 2: Sweeping based on last error=%.2f\n", 
                   g_last_error_direction);
            // recovery_phase = 1;
        }
        
        // Use error memory for intelligent sweep
        bool sweep_right = g_have_error_memory ? (g_last_error_direction > 0) : true;
        
        // Alternate every 0.5s (50 iterations)
        if ((g_recovery_count / 50) % 2 == 0) {
            if (sweep_right) {
                motion_command(MOVE_RIGHT, RECOVERY_SPEED_PCT);
            } else {
                motion_command(MOVE_LEFT, RECOVERY_SPEED_PCT);
            }
        } else {
            if (sweep_right) {
                motion_command(MOVE_LEFT, RECOVERY_SPEED_PCT);
            } else {
                motion_command(MOVE_RIGHT, RECOVERY_SPEED_PCT);
            }
        }
    }
    
    // Timeout after 3 seconds
    if (g_recovery_count > LINE_RECOVERY_TIMEOUT) {
        printf("[RECOVERY] Failed after %lu iterations - line not found\n", g_recovery_count);
        g_recovery_count = 0;
        // recovery_phase = 0;
        return false;
    }
    
    return false;
}

// ============================================================================
// MAIN CONTROL LOOP CALLBACK (100 Hz) - Reuses demo1 architecture
// ============================================================================
static bool main_control_cb(repeating_timer_t* t) {
    volatile bool* p_run = (volatile bool*)t->user_data;
    const float dt = DT_S;

    // Stop motors if not running and not overridden
    if (!p_run || !*p_run) {
        if (!g_override_motion) {
            motion_command(MOVE_STOP, 0);
        }
        g_first_run = true;
        return true;
    }

    g_telem.loop_count++;

    // ===== 1) READ ALL SENSORS =====
    
    // Line sensor (left IR)
    line_reading_t line = ir_get_line_error();
    g_telem.line_reading = line;
    
    // Barcode sensor (right IR) - non-blocking poll
    barcode_result_t barcode_res = {0};
    bool barcode_detected = barcode_poll(&barcode_res);
    
    // IMU data
    imu_state_t imu_data;
    bool imu_ok = imu_read(&imu_data) && imu_data.ok;
    float current_heading = imu_ok ? imu_data.heading_deg_filt : g_target_heading_deg;
    g_telem.heading_deg = current_heading;

    // Encoder data (for telemetry)
    float cps_r, cps_l;
    get_cps_smoothed(&cps_r, &cps_l);
    g_telem.encoder_diff_cps = cps_r - cps_l;
    g_telem.speed_cmps = get_average_speed_cmps();
    g_telem.distance_cm = get_average_distance_cm();
    g_telem.state = g_state;

    // ===== 2) STATE MACHINE =====
    
    switch (g_state) {
        // --------------------------------------------------------------------
        case STATE_LINE_FOLLOW: {
            // Check for barcode detection
            uint64_t time_since_last = absolute_time_diff_us(g_last_barcode_time, get_absolute_time());
            
            if (barcode_detected && 
                barcode_res.command != BARCODE_NONE &&
                time_since_last > (BARCODE_MIN_INTERVAL_MS * 1000)) {
                
                g_pending_command = barcode_res.command;
                g_telem.last_barcode = barcode_res.command;
                g_state = STATE_BARCODE_DETECTED;
                g_last_barcode_time = get_absolute_time();
                
                printf("[BARCODE] Detected: %s (bars=%u, conf=%u%%)\n", 
                       barcode_cmd_str(g_pending_command),
                       barcode_res.bar_count,
                       barcode_res.confidence);
                
                barcode_reset();
                break;
            }

            // Check for line loss
            if (!line.line_detected) {
                g_line_lost_count++;
                if (g_line_lost_count > LINE_LOST_COUNT_MAX) {
                    g_state = STATE_LINE_LOST;
                    motion_command(MOVE_STOP, 0);
                    printf("[WARN] Line LOST (count=%lu)\n", g_line_lost_count);
                    break;
                }
            } else {
                g_line_lost_count = 0;
            }

            // === NORMAL LINE FOLLOWING WITH CASCADED PID ===
            
            // Get current smoothed wheel speeds
            float base_pct = (float)LINE_FOLLOW_SPEED_PCT;
            float base_cps = (MAX_CPS * base_pct) / 100.0f;
            
            // (A) Line following PID (primary steering)
            float line_bias = line_follow_pid_controller(&line, dt);
            
            // (B) IMU heading correction (slow drift prevention) only when line confident
            float imu_bias = 0.0f;
            if (line.confidence > 0.7f) {  // Only apply IMU when line tracking is good
                imu_bias = imu_heading_correction(current_heading, dt);
            }
            
            // (C) Encoder balance PID (from demo1 architecture)
            float diff_meas = cps_r - cps_l;
            float encoder_bias = pid_step(&pid_track, 0.0f, -diff_meas, dt);
            
            // Limits scale with speed and line confidence
            float confidence_factor = clampf(line.confidence, 0.5f, 1.0f);
            
            float lim_line = fmaxf(BIAS_MIN_CPS, 0.45f * base_cps * confidence_factor);
            float lim_imu = fmaxf(BIAS_MIN_HEAD_CPS, 0.20f * base_cps);  // Reduced IMU
            float lim_enc = fmaxf(BIAS_MIN_CPS, 0.30f * base_cps);
            
            line_bias = clampf(line_bias, -lim_line, lim_line);
            imu_bias = clampf(imu_bias, -lim_imu, lim_imu);
            encoder_bias = clampf(encoder_bias, -lim_enc, lim_enc);
            
            // Dynamic weighting based on confidence
            float w_line = 0.70f * confidence_factor;                   // Trust line more when confident
            float w_enc = 0.25f * (1.5f - 0.5f * confidence_factor);  // Trust encoders less when confident
            float w_imu = 0.05f;                                        // Minimal IMU influence
            
            // Normalize weights
            float w_total = w_line + w_enc + w_imu;
            w_line /= w_total;
            w_enc /= w_total;
            w_imu /= w_total;
            
            float total_bias = line_bias * w_line +
                              encoder_bias * w_enc +
                              imu_bias * w_imu;
            
            // Final safety clamp with reduced limit
            float lim_total = fmaxf(BIAS_MIN_TOTAL_CPS, 0.50f * base_cps);  // Reduced from 0.60
            total_bias = clampf(total_bias, -lim_total, lim_total);
            
            // Update adaptive wheel scaling (from demo1)
            motor_update_adaptive_scale(diff_meas, dt, base_cps);
            
            // Command motion with smoother bias application
            // Apply total bias symmetrically: positive = left faster, right slower
            motion_command_with_bias(MOVE_FORWARD, LINE_FOLLOW_SPEED_PCT,
                                    total_bias,   // Left wheel adjustment
                                    -total_bias); // Right wheel adjustment (opposite)
            
            break;
        }
        
        // --------------------------------------------------------------------
        case STATE_BARCODE_DETECTED: {
            // Process barcode command
            switch (g_pending_command) {
                case BARCODE_LEFT:
                    // Apply overshoot compensation
                    g_target_heading_deg = wrap180(current_heading - TURN_ANGLE_90 + TURN_OVERSHOOT_COMP);
                    g_state = STATE_EXECUTING_TURN;
                    printf("[TURN] LEFT 90° → target=%.1f°\n", g_target_heading_deg);
                    break;

                case BARCODE_RIGHT:
                    g_target_heading_deg = wrap180(current_heading + TURN_ANGLE_90 - TURN_OVERSHOOT_COMP);
                    g_state = STATE_EXECUTING_TURN;
                    printf("[TURN] RIGHT 90° → target=%.1f°\n", g_target_heading_deg);
                    break;

                case BARCODE_UTURN:
                    g_target_heading_deg = wrap180(current_heading + TURN_ANGLE_180);
                    g_state = STATE_EXECUTING_TURN;
                    printf("[TURN] U-TURN 180° → target=%.1f°\n", g_target_heading_deg);
                    break;

                case BARCODE_STOP:
                    motion_command(MOVE_STOP, 0);
                    g_state = STATE_STOPPED;
                    printf("[STOP] Halted by barcode command\n");
                    break;

                default:
                    g_state = STATE_LINE_FOLLOW;
                    break;
            }
            
            g_pending_command = BARCODE_NONE;
            break;
        }
        
        // --------------------------------------------------------------------
        case STATE_EXECUTING_TURN: {
            bool turn_done = execute_turn(current_heading, dt);
            
            if (turn_done) {
                printf("[TURN] Complete (heading=%.1f°)\n", current_heading);
                motion_command(MOVE_STOP, 0);
                sleep_ms(300); // Brief settle
                
                // Update reference heading
                g_initial_heading_deg = g_target_heading_deg;
                
                // Reset all PIDs
                pid_reset(&pid_line_follow);
                pid_reset(&pid_track);
                imu_reset_heading_filter(g_target_heading_deg);
                imu_reset_heading_supervisor();
                
                g_state = STATE_REACQUIRE_LINE;
                printf("[REACQUIRE] Searching for line...\n");
            }
            break;
        }
        
        // --------------------------------------------------------------------
        case STATE_REACQUIRE_LINE: {
            bool line_found = reacquire_line(&line);
            
            if (line_found) {
                g_state = STATE_LINE_FOLLOW;
                g_line_lost_count = 0;
                printf("[RESUME] Line following resumed\n");
            } else if (g_recovery_count > 300) { // 3 second timeout
                g_state = STATE_LINE_LOST;
                printf("[REACQUIRE] Timeout → LINE_LOST\n");
            }
            break;
        }
        
        // --------------------------------------------------------------------
        case STATE_LINE_LOST: {
            bool recovered = attempt_line_recovery(&line);
            
            if (recovered) {
                g_state = STATE_LINE_FOLLOW;
                g_line_lost_count = 0;
                g_recovery_count = 0;
            } else if (g_recovery_count >= LINE_RECOVERY_TIMEOUT) {
                g_state = STATE_STOPPED;
                motion_command(MOVE_STOP, 0);
                printf("[ERROR] Recovery failed - stopping\n");
            }
            break;
        }
        
        // --------------------------------------------------------------------
        case STATE_STOPPED: {
            motion_command(MOVE_STOP, 0);
            
            // Allow manual restart with START button
            if (btn_pressed(BTN_START)) {
                sleep_ms(200);
                pid_reset(&pid_line_follow);
                pid_reset(&pid_track);
                imu_reset_heading_supervisor();
                g_state = STATE_LINE_FOLLOW;
                g_line_lost_count = 0;
                g_recovery_count = 0;
                g_have_error_memory = false;
                printf("[RESTART] Resuming from STOPPED\n");
            }
            break;
        }
        
        default:
            g_state = STATE_LINE_FOLLOW;
            break;
    }

    return true;
}

// ============================================================================
// TELEMETRY PRINT (5 Hz) - Enhanced version
// ============================================================================
static void print_telemetry(void) {
    const char* state_str[] = {
        "IDLE", "LINE_FOLLOW", "BARCODE_DET", "TURN", "REACQUIRE", "STOPPED", "LINE_LOST"
    };
    
    printf("[DEMO2] State=%s Line[err=%.2f ADC=%u det=%d conf=%.2f] "
           "Bias[line=%.1f imu=%.1f enc_diff=%.1f] "
           "IMU[hdg=%.1f° err=%.1f° w=%.2f] "
           "Speed=%.1fcm/s Dist=%.1fcm LastBC=%s MemErr=%.2f\n",
           state_str[g_telem.state],
           g_telem.line_reading.error,
           g_telem.line_reading.raw_adc,
           g_telem.line_reading.line_detected,
           g_telem.line_reading.confidence,
           g_telem.line_bias_cps,
           g_telem.imu_bias_cps,
           g_telem.encoder_diff_cps,
           g_telem.heading_deg,
           g_telem.heading_error_deg,
           (double)g_head_weight,
           g_telem.speed_cmps,
           g_telem.distance_cm,
           barcode_cmd_str(g_telem.last_barcode),
           g_last_error_direction);
}

// ============================================================================
// CALIBRATION ROUTINES (from demo1)
// ============================================================================

static void do_mag_calibration(void) {
    printf("CAL: magnetometer min/max... spinning 3s\n");
    telemetry_set_mode(TMODE_CAL);

    g_override_motion = true;
    imu_cal_begin();

    absolute_time_t t0 = get_absolute_time();
    motion_command(MOVE_LEFT, 25);
    
    while (absolute_time_diff_us(t0, get_absolute_time()) < 3000000) {
        imu_state_t s;
        if (imu_read(&s) && s.ok) { }
        tight_loop_contents();
    }
    
    motion_command(MOVE_STOP, 0);
    imu_cal_end();
    g_override_motion = false;

    imu_state_t s;
    if (imu_read(&s) && s.ok) {
        imu_reset_heading_filter(s.heading_deg_filt);
        g_initial_heading_deg = s.heading_deg_filt;
        g_target_heading_deg = s.heading_deg_filt;
        imu_reset_heading_supervisor();
    }

    printf("CAL: done. heading_ref=%.1f deg\n", g_initial_heading_deg);
    telemetry_set_mode(TMODE_NONE);
}

static void do_auto_wheel_scale(void) {
    printf("TRIM: auto wheel scale... driving 4.0s\n");
    telemetry_set_mode(TMODE_CAL);

    g_override_motion = true;
    motion_command(MOVE_FORWARD, 20);
    sleep_ms(500);

    float sum_r = 0, sum_l = 0;
    int n = 0;
    absolute_time_t t0 = get_absolute_time();
    
    while (absolute_time_diff_us(t0, get_absolute_time()) < 4000000) {
        float r, l;
        get_cps_smoothed(&r, &l);
        sum_r += r;
        sum_l += l;
        n++;
        sleep_ms(10);
    }
    
    motion_command(MOVE_STOP, 0);

    if (n > 0 && sum_l > 1.0f && sum_r > 1.0f) {
        float r_avg = sum_r / (float)n;
        float l_avg = sum_l / (float)n;
        motor_calibrate_wheel_scale(r_avg, l_avg);
        
        float sr, sl;
        motor_get_wheel_scale(&sr, &sl);
        printf("TRIM: scales R=%.3f L=%.3f\n", sr, sl);
    } else {
        motor_set_wheel_scale(DEFAULT_SCALE_RIGHT, DEFAULT_SCALE_LEFT);
        printf("TRIM: skipped (low cps)\n");
    }

    g_override_motion = false;
    telemetry_set_mode(TMODE_NONE);
}

static void do_boot_auto_cal(void) {
    // do_mag_calibration();
    do_auto_wheel_scale();
    motion_command(MOVE_STOP, 0);
    printf("BOOT: auto calibration complete.\n");
}

static bool prompt_boot_cal(void) {
    printf("BOOT: run auto-calibration before START? [y/N]\n");
    printf("Press START = yes, STOP = no, or type y/n. Auto-skip in 10s...\n");
    
    absolute_time_t t0 = get_absolute_time();
    while (absolute_time_diff_us(t0, get_absolute_time()) < 10000000) {
        int ch = getchar_timeout_us(0);
        if (ch == 'y' || ch == 'Y') { printf(" -> yes\n"); return true; }
        if (ch == 'n' || ch == 'N') { printf(" -> no\n"); return false; }

        if (btn_pressed(BTN_START)) {
            sleep_ms(100);
            if (btn_pressed(BTN_START)) {
                printf(" -> yes (START)\n");
                return true;
            }
        }
        if (btn_pressed(BTN_STOP)) {
            sleep_ms(100);
            if (btn_pressed(BTN_STOP)) {
                printf(" -> no (STOP)\n");
                return false;
            }
        }

        tight_loop_contents();
    }
    
    printf("-> timeout; skipping auto-calibration.\n");
    return false;
}

// ============================================================================
// BUTTON INITIALIZATION
// ============================================================================
static void buttons_init(void) {
    gpio_init(BTN_START);
    gpio_set_dir(BTN_START, GPIO_IN);
    gpio_pull_up(BTN_START);
    
    gpio_init(BTN_STOP);
    gpio_set_dir(BTN_STOP, GPIO_IN);
    gpio_pull_up(BTN_STOP);
}

// ============================================================================
// SYSTEM INITIALIZATION
// ============================================================================
static void system_init(void) {
    stdio_init_all();
    sleep_ms(10000); // Allow USB enumeration
    
    printf("\n╔═══════════════════════════════════════════════════════════╗\n");
    printf("║   DEMO 2: Line Following + Barcode + PID + IMU (Full)    ║\n");
    printf("║   Built on Demo1 Control Architecture                    ║\n");
    printf("║   Optimized for THIN LINE (1.7cm) - Single Left Sensor   ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n\n");

    buttons_init();

    // Initialize IMU
    if (!imu_init()) {
        printf("[ERROR] IMU init FAILED - check I2C wiring!\n");
        while (1) sleep_ms(1000);
    }
    printf("[OK] IMU initialized (GP%d=SDA, GP%d=SCL)\n", I2C_SDA, I2C_SCL);

    // Initialize motors
    motor_init();
    printf("[OK] Motors initialized (M1: GP%d,GP%d,GP%d | M2: GP%d,GP%d,GP%d)\n",
           M1_IN1, M1_IN2, ENCODER_PIN_M1, M2_IN1, M2_IN2, ENCODER_PIN_M2);

    // Initialize line sensor
    ir_line_init();
    printf("[OK] Line sensor initialized (GP%d=ADC2)\n", IR_LEFT_ADC);

    // Initialize barcode scanner
    barcode_init();
    printf("[OK] Barcode scanner initialized (GP%d=ADC0)\n", BARCODE_ADC_GPIO);

    // Initialize PIDs
    pid_init_defaults();
    pid_reset(&pid_line_follow);
    printf("[OK] PID controllers initialized (track + heading + line)\n");

    // Start control timers (reuse demo1 architecture)
    add_repeating_timer_ms(-CONTROL_PERIOD_MS, motor_control_timer_cb, NULL, &control_timer_motor);
    add_repeating_timer_ms(-CONTROL_PERIOD_MS, main_control_cb, (void*)&running, &control_timer_main);
    printf("[OK] Control timers started (100 Hz)\n");

    // Capture initial heading
    printf("[INIT] Capturing initial heading...\n");
    sleep_ms(1000);
    imu_state_t imu;
    if (imu_read(&imu) && imu.ok) {
        g_initial_heading_deg = imu.heading_deg_filt;
        g_target_heading_deg = g_initial_heading_deg;
        imu_reset_heading_filter(g_initial_heading_deg);
        imu_reset_heading_supervisor();
        printf("[OK] Initial heading: %.1f°\n", g_initial_heading_deg);
    } else {
        printf("[WARN] IMU not ready, using 0.0°\n");
        g_initial_heading_deg = 0.0f;
        g_target_heading_deg = 0.0f;
    }
    
    g_last_barcode_time = get_absolute_time();
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================
int main(void) {
    system_init();

    // Boot calibration prompt
    if (prompt_boot_cal()) {
        do_boot_auto_cal();
    } else {
        printf("BOOT: auto-calibration skipped.\n");
    }

    printf("═══════════════════════════════════════════════════════════\n");
    printf("  Press START (GP21) to begin autonomous navigation...\n");
    printf("═══════════════════════════════════════════════════════════\n\n");
    
    // Wait for START button
    while (btn_pressed(BTN_START) != true) {
        tight_loop_contents();
    }
    sleep_ms(200);
    
    printf("[START] Robot activated!\n\n");
    
    // Reset everything for clean start
    motor_reset_distance_counters();
    motor_reset_controllers();
    motor_reset_speed_filters();
    motor_reset_adaptive_scale();
    pid_reset(&pid_line_follow);
    pid_reset(&pid_track);
    imu_reset_heading_supervisor();
    ir_line_reset_state();
    barcode_reset();
    
    g_state = STATE_LINE_FOLLOW;
    g_first_run = true;
    g_have_error_memory = false;
    running = true;
    
    print_telemetry_legend();

    absolute_time_t t_next = make_timeout_time_ms(LOOP_DT_MS);
    uint32_t telem_div = 0;

    // ========== MAIN LOOP (Background for telemetry and emergency stop) ==========
    while (true) {
        // Telemetry printing (5 Hz)
        if ((telem_div++ % (1000 / LOOP_DT_MS / 5)) == 0) {
            print_telemetry();
        }

        // Emergency stop button
        if (btn_pressed(BTN_STOP)) {
            motion_command(MOVE_STOP, 0);
            running = false;
            g_state = STATE_STOPPED;
            printf("\n[EMERGENCY] STOP button pressed\n");
            sleep_ms(500);
            
            // Wait for START to resume
            while (!btn_pressed(BTN_START)) {
                tight_loop_contents();
            }
            sleep_ms(200);
            
            printf("[RESTART] Resuming...\n");
            running = true;
            g_state = STATE_LINE_FOLLOW;
            g_first_run = true;
        }

        // Loop timing
        sleep_until(t_next);
        t_next = delayed_by_ms(t_next, LOOP_DT_MS);
    }

    return 0;
}