// demo2.c - Line Following + Barcode Navigation + IMU Turns + Telemetry
// Purpose: Full perception-decision-action loop for Demo 2

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
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

// Base speeds (percentage 0-100)
#define LINE_FOLLOW_SPEED_PCT  25       // Speed while following line
#define TURN_SPEED_PCT         20       // Speed during barcode turns
#define RECOVERY_SPEED_PCT     15       // Slower speed during line recovery

// Turn angles (degrees)
#define TURN_ANGLE_90          90.0f    // LEFT/RIGHT turn
#define TURN_ANGLE_180         180.0f   // U-TURN
#define TURN_TOLERANCE_DEG     3.0f     // Turn completion threshold

// Control loop timing
#define LOOP_DT_MS             10       // 100 Hz control loop
#define DT_S                   ((float)LOOP_DT_MS / 1000.0f)

// ============================================================================
// LINE FOLLOWING PID GAINS (for thin 1.7cm line)
// ============================================================================
// Strategy: LEFT sensor is PRIMARY (must stay on black)
//           RIGHT sensor validates we haven't drifted too far
#define LINE_KP                0.60f    // Increased for faster response
#define LINE_KI                0.08f    // Moderate integral for steady-state
#define LINE_KD                0.08f    // Damping to prevent oscillation
#define LINE_BIAS_MAX_CPS      10.0f    // Max steering correction (cps)

// IMU heading correction gains (additional layer)
#define IMU_HEADING_KP         0.25f    // Gentle heading correction
#define IMU_HEADING_KI         0.02f    // Very slow integral
#define IMU_HEADING_KD         0.00f    // No derivative on heading
#define IMU_BIAS_MAX_CPS       6.0f     // Max IMU steering correction

// Line loss detection
#define LINE_LOST_COUNT_MAX    50       // 0.5s of both sensors off = lost

// ============================================================================
// STATE MACHINE DEFINITIONS
// ============================================================================
typedef enum {
    STATE_IDLE,              // Waiting for START button
    STATE_LINE_FOLLOW,       // Active line following mode
    STATE_BARCODE_DETECTED,  // Barcode detected, preparing turn
    STATE_EXECUTING_TURN,    // Turning based on barcode command
    STATE_STOPPED,           // STOP command or emergency stop (can restart)
    STATE_LINE_LOST,         // Lost line, attempting recovery
    STATE_LINE_RECOVERY      // Actively recovering line position
} robot_state_t;

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================
static robot_state_t g_state = STATE_IDLE;
static barcode_cmd_t g_pending_command = BARCODE_NONE;
static float g_target_heading_deg = 0.0f;
static float g_turn_start_heading_deg = 0.0f;
static float g_initial_heading_deg = 0.0f;  // Captured on START

// ============================================================================
// TELEMETRY CACHE (for cleaner prints)
// ============================================================================
static struct {
    line_position_t line_pos;
    barcode_cmd_t last_barcode;
    float heading_deg;
    float heading_error_deg;
    float speed_cmps;
    float distance_cm;
    float line_bias_cps;
    float imu_bias_cps;
    robot_state_t state;
    bool left_sensor_black;
    bool right_sensor_black;
} g_telem = {0};

// ============================================================================
// LINE FOLLOWING PID STATE
// ============================================================================
static float g_line_integ = 0.0f;
static float g_line_prev_err = 0.0f;

// ============================================================================
// IMU HEADING PID STATE (for long-term drift correction)
// ============================================================================
static float g_imu_integ = 0.0f;
static float g_imu_prev_err = 0.0f;

// ============================================================================
// LINE RECOVERY STATE
// ============================================================================
static uint32_t g_line_lost_count = 0;
static absolute_time_t g_recovery_start_time;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Wrap angle to ±180°
static inline float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// Clamp value between min and max
static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// ============================================================================
// LINE FOLLOWING CONTROLLER (for thin 1.7cm line)
// ============================================================================
// Strategy:
// - LEFT sensor MUST detect black (primary control)
// - RIGHT sensor validates position (secondary)
// - Returns steering bias in CPS (positive = steer right, negative = steer left)
//
// Error encoding for THIN LINE:
//   LEFT=BLACK, RIGHT=OFF  → err = 0.0   (PERFECT - on line)
//   LEFT=OFF,   RIGHT=OFF  → err = +1.5  (drifted LEFT - steer right aggressively)
//   LEFT=BLACK, RIGHT=BLACK → err = -0.5  (drifted RIGHT - steer left gently)
//   LEFT=OFF,   RIGHT=BLACK → err = +2.0  (way off - major correction needed)
// ============================================================================
static float line_follow_pid(bool left_black, bool right_black, float dt) {
    float err = 0.0f;
    
    // **PRIMARY STRATEGY: LEFT sensor must be on black**
    if (left_black && !right_black) {
        // IDEAL CASE: Left on line, right off line
        err = 0.0f;  // Perfect tracking
    } 
    else if (!left_black && !right_black) {
        // BOTH OFF: Drifted left off the line
        err = +1.5f;  // Steer RIGHT strongly
    }
    else if (left_black && right_black) {
        // BOTH ON: Drifted right (line is under both sensors)
        err = -0.5f;  // Steer LEFT gently to center on left sensor
    }
    else if (!left_black && right_black) {
        // LEFT OFF, RIGHT ON: Significantly drifted left
        err = +2.0f;  // Steer RIGHT very strongly
    }

    // PID calculation
    g_line_integ += err * dt;
    g_line_integ = clampf(g_line_integ, -10.0f, 10.0f);  // Anti-windup
    
    float deriv = (err - g_line_prev_err) / dt;
    g_line_prev_err = err;

    float bias = LINE_KP * err + LINE_KI * g_line_integ + LINE_KD * deriv;
    
    // Store for telemetry
    g_telem.line_bias_cps = clampf(bias, -LINE_BIAS_MAX_CPS, LINE_BIAS_MAX_CPS);
    
    return g_telem.line_bias_cps;
}

// ============================================================================
// IMU HEADING CORRECTION (slow drift compensation)
// ============================================================================
// Provides gentle long-term heading correction to complement encoder-based
// line following. Only active when IMU data is valid.
// ============================================================================
static float imu_heading_pid(float current_heading, float dt) {
    imu_state_t imu;
    if (!imu_read(&imu) || !imu.ok) {
        // IMU not available - no correction
        g_telem.imu_bias_cps = 0.0f;
        return 0.0f;
    }

    // Calculate heading error relative to initial heading
    float err_deg = wrap180(current_heading - g_initial_heading_deg);
    g_telem.heading_error_deg = err_deg;
    
    // Deadband to avoid micro-corrections
    if (fabsf(err_deg) < 2.0f) {
        err_deg = 0.0f;
    }

    // PID calculation
    g_imu_integ += err_deg * dt;
    g_imu_integ = clampf(g_imu_integ, -20.0f, 20.0f);  // Anti-windup
    
    float deriv = (err_deg - g_imu_prev_err) / dt;
    g_imu_prev_err = err_deg;

    float bias = IMU_HEADING_KP * err_deg + IMU_HEADING_KI * g_imu_integ + IMU_HEADING_KD * deriv;
    
    // Store for telemetry
    g_telem.imu_bias_cps = clampf(bias, -IMU_BIAS_MAX_CPS, IMU_BIAS_MAX_CPS);
    
    return g_telem.imu_bias_cps;
}

// ============================================================================
// TURN EXECUTION (IMU-based accurate turning)
// ============================================================================
// Returns true when turn is complete
// ============================================================================
static bool execute_turn(float current_heading, float dt) {
    static float last_hdg = 0.0f;
    static int stall_count = 0;

    float delta = wrap180(g_target_heading_deg - current_heading);
    
    // Check if turn complete
    if (fabsf(delta) < TURN_TOLERANCE_DEG) {
        motion_command(MOVE_STOP, 0);
        stall_count = 0;
        return true;  // Turn complete
    }

    // Proportional turn rate (smoother than bang-bang)
    float turn_rate_cps = clampf(delta * 0.6f, -18.0f, 18.0f);
    
    // Differential drive: positive delta = turn right
    if (delta > 0) {
        // Turn right: left wheel forward faster, right wheel slower
        motion_command_with_bias(MOVE_FORWARD, TURN_SPEED_PCT, 
                                -turn_rate_cps, turn_rate_cps);
    } else {
        // Turn left: right wheel forward faster, left wheel slower
        motion_command_with_bias(MOVE_FORWARD, TURN_SPEED_PCT, 
                                turn_rate_cps, -turn_rate_cps);
    }

    // Stall detection (not moving for 1 second)
    if (fabsf(current_heading - last_hdg) < 0.5f) {
        stall_count++;
        if (stall_count > 100) {  // 1 second at 100 Hz
            printf("[DEMO2] Turn stalled at %.1f deg, forcing completion\n", 
                   (double)current_heading);
            stall_count = 0;
            return true;
        }
    } else {
        stall_count = 0;
    }
    last_hdg = current_heading;

    return false;  // Turn in progress
}

// ============================================================================
// LINE RECOVERY LOGIC
// ============================================================================
// Attempts to recover when both sensors lose the line
// ============================================================================
static void attempt_line_recovery(void) {
    // Simple strategy: slow reverse for 300ms, then check
    static bool reversing = false;
    
    if (!reversing) {
        printf("[DEMO2] Line lost - attempting recovery (reverse)\n");
        motion_command(MOVE_BACKWARD, RECOVERY_SPEED_PCT);
        g_recovery_start_time = get_absolute_time();
        reversing = true;
    }
    
    // Check if we've been reversing for 300ms
    if (absolute_time_diff_us(g_recovery_start_time, get_absolute_time()) > 300000) {
        motion_command(MOVE_STOP, 0);
        reversing = false;
        
        // Check if line reacquired
        line_position_t pos = ir_get_line_position();
        bool left_black = ir_left_is_black();
        
        if (left_black || pos != LINE_BOTH_OFF) {
            g_state = STATE_LINE_FOLLOW;
            g_line_lost_count = 0;
            printf("[DEMO2] Line reacquired!\n");
        } else {
            printf("[DEMO2] Recovery failed - still lost\n");
            g_state = STATE_STOPPED;
        }
    }
}

// ============================================================================
// TELEMETRY PRINT (5 Hz for readability)
// ============================================================================
static void print_telemetry(void) {
    printf("[DEMO2] State=%d Sensors[L=%d R=%d] LineErr=%.2f IMU_Err=%.1f° "
           "Speed=%.1fcm/s Dist=%.1fcm LineBias=%.1f IMUBias=%.1f\n",
           g_telem.state,
           g_telem.left_sensor_black,
           g_telem.right_sensor_black,
           (double)g_line_prev_err,
           (double)g_telem.heading_error_deg,
           (double)g_telem.speed_cmps,
           (double)g_telem.distance_cm,
           (double)g_telem.line_bias_cps,
           (double)g_telem.imu_bias_cps);
}

// ============================================================================
// TELEMETRY LEGEND (printed once at start)
// ============================================================================
static void print_legend(void) {
    printf("\n=== DEMO 2 TELEMETRY LEGEND ===\n");
    printf("State: 0=IDLE, 1=LINE_FOLLOW, 2=BARCODE_DETECTED, 3=EXECUTING_TURN, 4=STOPPED, 5=LINE_LOST\n");
    printf("Sensors: L=Left_IR(primary), R=Right_IR(validation) - 1=BLACK, 0=WHITE\n");
    printf("LineErr: Line following error (0=perfect, +ve=steer right, -ve=steer left)\n");
    printf("IMU_Err: Heading error in degrees\n");
    printf("LineBias: Steering correction from line PID (cps)\n");
    printf("IMUBias: Steering correction from IMU heading PID (cps)\n");
    printf("===================================\n\n");
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================
int main(void) {
    // ========== INITIALIZATION ==========
    stdio_init_all();
    sleep_ms(10000);
    printf("\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║   DEMO 2: Line Following + Barcode + IMU                  ║\n");
    printf("║   Optimized for THIN LINE (1.7cm) - Left Sensor Primary  ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");

    // Initialize all subsystems
    if (!imu_init()) {
        printf("[ERROR] IMU init FAILED - check I2C wiring (GP2=SDA, GP3=SCL)\n");
        while (1) sleep_ms(1000);
    }
    printf("[OK] IMU initialized\n");

    motor_init();
    printf("[OK] Motors initialized\n");

    ir_line_init();
    printf("[OK] Line sensors initialized (Left=GP%d, Right=GP%d)\n", 
           IR_LEFT_DIGITAL_PIN, IR_RIGHT_DIGITAL_PIN);

    barcode_init();
    printf("[OK] Barcode scanner initialized (GP%d)\n", IR_RIGHT_ADC_GPIO);

    pid_init_defaults();
    printf("[OK] PID controllers initialized\n");

    // Start motor control timer (100 Hz)
    static repeating_timer_t motor_control_timer;
    add_repeating_timer_ms(-CONTROL_PERIOD_MS, motor_control_timer_cb, NULL, &motor_control_timer);
    printf("[OK] Motor control timer started (100 Hz)\n");

    // Capture initial heading after IMU settles
    printf("[INIT] Capturing initial heading...\n");
    sleep_ms(1000);
    imu_state_t imu;
    if (imu_read(&imu) && imu.ok) {
        g_initial_heading_deg = imu.heading_deg_filt;
        g_target_heading_deg = g_initial_heading_deg;
        printf("[OK] Initial heading: %.1f°\n", (double)g_initial_heading_deg);
    } else {
        printf("[WARN] IMU not ready, using 0.0° as initial heading\n");
        g_initial_heading_deg = 0.0f;
        g_target_heading_deg = 0.0f;
    }

    print_legend();

    // ========== WAIT FOR START ==========
    printf("═══════════════════════════════════════════════════════════\n");
    printf("  Press START (GP21) to begin line following...\n");
    printf("═══════════════════════════════════════════════════════════\n\n");
    
    while (gpio_get(BTN_START) != 0) {
        tight_loop_contents();
    }
    sleep_ms(200);  // Debounce
    
    printf("[START] Robot activated! Beginning line follow mode.\n\n");
    motor_reset_distance_counters();
    g_state = STATE_LINE_FOLLOW;

    absolute_time_t t_next = make_timeout_time_ms(LOOP_DT_MS);
    uint32_t telem_div = 0;

    // ========== MAIN CONTROL LOOP (100 Hz) ==========
    while (true) {
        // ===== 1) READ ALL SENSORS =====
        bool left_black = ir_left_is_black();
        bool right_black = ir_right_is_black();
        line_position_t line_pos = ir_get_line_position();
        
        barcode_result_t barcode_res = {0};
        bool barcode_detected = barcode_poll(&barcode_res);
        
        imu_state_t imu_data;
        bool imu_ok = imu_read(&imu_data) && imu_data.ok;
        float current_heading = imu_ok ? imu_data.heading_deg_filt : g_target_heading_deg;

        float avg_speed = get_average_speed_cmps();
        float avg_dist = get_average_distance_cm();

        // Update telemetry cache
        g_telem.line_pos = line_pos;
        g_telem.heading_deg = current_heading;
        g_telem.speed_cmps = avg_speed;
        g_telem.distance_cm = avg_dist;
        g_telem.state = g_state;
        g_telem.left_sensor_black = left_black;
        g_telem.right_sensor_black = right_black;

        // ===== 2) STATE MACHINE =====
        switch (g_state) {
            case STATE_LINE_FOLLOW: {
                // ---- Check for barcode first ----
                if (barcode_detected && barcode_res.command != BARCODE_NONE) {
                    g_pending_command = barcode_res.command;
                    g_telem.last_barcode = barcode_res.command;
                    g_state = STATE_BARCODE_DETECTED;
                    printf("[BARCODE] Detected: %s\n", barcode_cmd_str(g_pending_command));
                    barcode_reset();
                    break;
                }

                // ---- Check for line loss ----
                if (!left_black && !right_black) {
                    g_line_lost_count++;
                    if (g_line_lost_count > LINE_LOST_COUNT_MAX) {
                        g_state = STATE_LINE_LOST;
                        motion_command(MOVE_STOP, 0);
                        printf("[WARN] Line LOST (both sensors off for >0.5s)\n");
                        break;
                    }
                } else {
                    g_line_lost_count = 0;  // Reset when line found
                }

                // ---- Normal line following with combined PID ----
                float line_bias = line_follow_pid(left_black, right_black, DT_S);
                float imu_bias = imu_heading_pid(current_heading, DT_S);
                
                // Combine both corrections (line following is primary, IMU is secondary)
                float total_bias = line_bias + (imu_bias * 0.3f);  // Scale IMU contribution
                total_bias = clampf(total_bias, -LINE_BIAS_MAX_CPS, LINE_BIAS_MAX_CPS);
                
                // Apply differential drive with bias
                // Positive bias = steer right (left wheel faster, right wheel slower)
                motion_command_with_bias(MOVE_FORWARD, LINE_FOLLOW_SPEED_PCT, 
                                        total_bias, -total_bias);
                break;
            }

            case STATE_BARCODE_DETECTED: {
                // Execute barcode command
                switch (g_pending_command) {
                    case BARCODE_LEFT:
                        g_turn_start_heading_deg = current_heading;
                        g_target_heading_deg = wrap180(current_heading - TURN_ANGLE_90);
                        g_state = STATE_EXECUTING_TURN;
                        printf("[TURN] Executing LEFT turn to %.1f°\n", 
                               (double)g_target_heading_deg);
                        break;

                    case BARCODE_RIGHT:
                        g_turn_start_heading_deg = current_heading;
                        g_target_heading_deg = wrap180(current_heading + TURN_ANGLE_90);
                        g_state = STATE_EXECUTING_TURN;
                        printf("[TURN] Executing RIGHT turn to %.1f°\n", 
                               (double)g_target_heading_deg);
                        break;

                    case BARCODE_UTURN:
                        g_turn_start_heading_deg = current_heading;
                        g_target_heading_deg = wrap180(current_heading + TURN_ANGLE_180);
                        g_state = STATE_EXECUTING_TURN;
                        printf("[TURN] Executing U-TURN to %.1f°\n", 
                               (double)g_target_heading_deg);
                        break;

                    case BARCODE_STOP:
                        motion_command(MOVE_STOP, 0);
                        g_state = STATE_STOPPED;
                        printf("[STOP] STOP command executed - press START to resume\n");
                        break;

                    default:
                        printf("[WARN] Invalid barcode command, resuming line follow\n");
                        g_state = STATE_LINE_FOLLOW;
                        break;
                }
                g_pending_command = BARCODE_NONE;
                break;
            }

            case STATE_EXECUTING_TURN: {
                bool turn_done = execute_turn(current_heading, DT_S);
                if (turn_done) {
                    printf("[TURN] Turn complete (final heading: %.1f°)\n", 
                           (double)current_heading);
                    motion_command(MOVE_STOP, 0);
                    sleep_ms(300);  // Brief pause to settle
                    
                    // Update reference heading for line following
                    g_initial_heading_deg = g_target_heading_deg;
                    
                    // Reset line PID integrators
                    g_line_integ = 0.0f;
                    g_line_prev_err = 0.0f;
                    g_imu_integ = 0.0f;
                    g_imu_prev_err = 0.0f;
                    
                    g_state = STATE_LINE_FOLLOW;
                    printf("[RESUME] Resuming line follow mode\n");
                }
                break;
            }

            case STATE_STOPPED: {
                motion_command(MOVE_STOP, 0);
                
                // **IMPROVED: Check for START button to resume (not infinite loop)**
                if (gpio_get(BTN_START) == 0) {
                    sleep_ms(200);  // Debounce
                    
                    // Reset PIDs
                    g_line_integ = 0.0f;
                    g_line_prev_err = 0.0f;
                    g_imu_integ = 0.0f;
                    g_imu_prev_err = 0.0f;
                    
                    g_state = STATE_LINE_FOLLOW;
                    printf("[RESTART] Resuming from STOP state\n");
                }
                break;
            }

            case STATE_LINE_LOST: {
                attempt_line_recovery();
                break;
            }

            default:
                g_state = STATE_LINE_FOLLOW;
                break;
        }

        // ===== 3) TELEMETRY OUTPUT (5 Hz for readability) =====
        if ((telem_div++ % (1000 / LOOP_DT_MS / 5)) == 0) {
            print_telemetry();
        }

        // ===== 4) EMERGENCY STOP (STOP button = halt, not restart) =====
        if (gpio_get(BTN_STOP) == 0) {
            motion_command(MOVE_STOP, 0);
            g_state = STATE_STOPPED;
            printf("\n[EMERGENCY] STOP button pressed - press START to resume\n");
            sleep_ms(500);  // Debounce
        }

        // ===== 5) LOOP TIMING (maintain 100 Hz) =====
        sleep_until(t_next);
        t_next = delayed_by_ms(t_next, LOOP_DT_MS);
    }

    return 0;
}


