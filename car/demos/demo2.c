// demo2.c - Line Following + Barcode Navigation + IMU Heading Correction
// Purpose: Full perception-decision-action loop for Demo 2 using existing libraries

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
#define LINE_KP                0.60f
#define LINE_KI                0.08f
#define LINE_KD                0.08f
#define LINE_BIAS_MAX_CPS      10.0f

// IMU heading correction gains
#define IMU_HEADING_KP         0.25f
#define IMU_HEADING_KI         0.02f
#define IMU_HEADING_KD         0.00f
#define IMU_BIAS_MAX_CPS       6.0f

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
static float g_initial_heading_deg = 0.0f;

// ============================================================================
// TELEMETRY CACHE
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
// LINE FOLLOWING PID STATE (separate from motor PID)
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
// IMU HEADING PID STATE (for demo2 long-term drift)
// ============================================================================
static PID pid_imu_heading = {
    .kp = IMU_HEADING_KP,
    .ki = IMU_HEADING_KI,
    .kd = IMU_HEADING_KD,
    .integ = 0,
    .prev_err = 0,
    .out_min = -IMU_BIAS_MAX_CPS,
    .out_max = IMU_BIAS_MAX_CPS
};

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


// ============================================================================
// LINE FOLLOWING CONTROLLER (using library PID)
// ============================================================================
static float line_follow_pid_controller(bool left_black, bool right_black, float dt) {
    float err = 0.0f;
    
    // Error encoding for THIN LINE (LEFT sensor is PRIMARY)
    if (left_black && !right_black) {
        err = 0.0f;  // PERFECT
    } 
    else if (!left_black && !right_black) {
        err = +1.5f;  // Drifted LEFT - steer RIGHT
    }
    else if (left_black && right_black) {
        err = -0.5f;  // Drifted RIGHT - steer LEFT gently
    }
    else if (!left_black && right_black) {
        err = +2.0f;  // Way off - major correction
    }

    // Use library PID function
    float bias = pid_step(&pid_line_follow, 0.0f, -err, dt);
    g_telem.line_bias_cps = bias;
    
    return bias;
}

// ============================================================================
// IMU HEADING CORRECTION (using library)
// ============================================================================
static float imu_heading_correction(float current_heading, float dt) {
    imu_state_t imu;
    if (!imu_read(&imu) || !imu.ok) {
        g_telem.imu_bias_cps = 0.0f;
        return 0.0f;
    }

    float err_deg = wrap180(current_heading - g_initial_heading_deg);
    g_telem.heading_error_deg = err_deg;
    
    // Deadband
    if (fabsf(err_deg) < 2.0f) {
        err_deg = 0.0f;
    }

    // Use library PID function
    float bias = pid_step(&pid_imu_heading, 0.0f, -err_deg, dt);
    g_telem.imu_bias_cps = bias;
    
    return bias;
}

// ============================================================================
// TURN EXECUTION (using motor library)
// ============================================================================
static bool execute_turn(float current_heading, float dt) {
    static float last_hdg = 0.0f;
    static int stall_count = 0;

    float delta = wrap180(g_target_heading_deg - current_heading);
    
    // Check if turn complete
    if (fabsf(delta) < TURN_TOLERANCE_DEG) {
        motion_command(MOVE_STOP, 0);
        stall_count = 0;
        return true;
    }

    // Proportional turn rate
    float turn_rate_cps = clampf(delta * 0.6f, -18.0f, 18.0f);
    
    // Use motor library differential drive
    if (delta > 0) {
        motion_command_with_bias(MOVE_FORWARD, TURN_SPEED_PCT, 
                                -turn_rate_cps, turn_rate_cps);
    } else {
        motion_command_with_bias(MOVE_FORWARD, TURN_SPEED_PCT, 
                                turn_rate_cps, -turn_rate_cps);
    }

    // Stall detection
    if (fabsf(current_heading - last_hdg) < 0.5f) {
        stall_count++;
        if (stall_count > 100) {
            printf("[DEMO2] Turn stalled at %.1f deg, forcing completion\n", 
                   (double)current_heading);
            stall_count = 0;
            return true;
        }
    } else {
        stall_count = 0;
    }
    last_hdg = current_heading;

    return false;
}

// ============================================================================
// LINE RECOVERY LOGIC
// ============================================================================
static void attempt_line_recovery(void) {
    static bool reversing = false;
    
    if (!reversing) {
        printf("[DEMO2] Line lost - attempting recovery (reverse)\n");
        motion_command(MOVE_BACKWARD, RECOVERY_SPEED_PCT);
        g_recovery_start_time = get_absolute_time();
        reversing = true;
    }
    
    if (absolute_time_diff_us(g_recovery_start_time, get_absolute_time()) > 300000) {
        motion_command(MOVE_STOP, 0);
        reversing = false;
        
        // Check if line reacquired
        bool left_black = ir_left_is_black();
        line_position_t pos = ir_get_line_position();
        
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
// TELEMETRY PRINT (5 Hz)
// ============================================================================
static void print_telemetry(void) {
    printf("[DEMO2] State=%d Sensors[L=%d R=%d] LineErr=%.2f IMU_Err=%.1f° "
           "Speed=%.1fcm/s Dist=%.1fcm LineBias=%.1f IMUBias=%.1f\n",
           g_telem.state,
           g_telem.left_sensor_black,
           g_telem.right_sensor_black,
           (double)pid_line_follow.prev_err,
           (double)g_telem.heading_error_deg,
           (double)g_telem.speed_cmps,
           (double)g_telem.distance_cm,
           (double)g_telem.line_bias_cps,
           (double)g_telem.imu_bias_cps);
}

// ============================================================================
// TELEMETRY LEGEND
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
// SYSTEM INITIALIZATION
// ============================================================================
static void system_init(void) {
    stdio_init_all();
    sleep_ms(10000);
    printf("\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║   DEMO 2: Line Following + Barcode + IMU (Refactored)     ║\n");
    printf("║   Optimized for THIN LINE (1.7cm) - Left Sensor Primary   ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");

    // Initialize subsystems using libraries
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
    printf("[OK] Barcode scanner initialized (GP%d)\n", IR_RIGHT_ADC_PIN);

    // Initialize demo2-specific PIDs (separate from motor PIDs)
    pid_reset(&pid_line_follow);
    pid_reset(&pid_imu_heading);
    printf("[OK] Demo2 PID controllers initialized\n");

    // Start motor control timer (uses library function)
    static repeating_timer_t motor_control_timer;
    add_repeating_timer_ms(-CONTROL_PERIOD_MS, motor_control_timer_cb, NULL, &motor_control_timer);
    printf("[OK] Motor control timer started (100 Hz)\n");

    // Capture initial heading
    printf("[INIT] Capturing initial heading...\n");
    sleep_ms(1000);
    imu_state_t imu;
    if (imu_read(&imu) && imu.ok) {
        g_initial_heading_deg = imu.heading_deg_filt;
        g_target_heading_deg = g_initial_heading_deg;
        imu_reset_heading_filter(g_initial_heading_deg);
        printf("[OK] Initial heading: %.1f°\n", (double)g_initial_heading_deg);
    } else {
        printf("[WARN] IMU not ready, using 0.0° as initial heading\n");
        g_initial_heading_deg = 0.0f;
        g_target_heading_deg = 0.0f;
    }

    print_legend();
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================
int main(void) {
    system_init();

    // Wait for START button
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
        // ===== 1) READ ALL SENSORS (using libraries) =====
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
                // Check for barcode
                if (barcode_detected && barcode_res.command != BARCODE_NONE) {
                    g_pending_command = barcode_res.command;
                    g_telem.last_barcode = barcode_res.command;
                    g_state = STATE_BARCODE_DETECTED;
                    printf("[BARCODE] Detected: %s\n", barcode_cmd_str(g_pending_command));
                    barcode_reset();
                    break;
                }

                // Check for line loss
                if (!left_black && !right_black) {
                    g_line_lost_count++;
                    if (g_line_lost_count > LINE_LOST_COUNT_MAX) {
                        g_state = STATE_LINE_LOST;
                        motion_command(MOVE_STOP, 0);
                        printf("[WARN] Line LOST (both sensors off for >0.5s)\n");
                        break;
                    }
                } else {
                    g_line_lost_count = 0;
                }

                // Normal line following with combined PID (using libraries)
                float line_bias = line_follow_pid_controller(left_black, right_black, DT_S);
                float imu_bias = imu_heading_correction(current_heading, DT_S);
                
                // Combine corrections (line following is primary)
                float total_bias = line_bias + (imu_bias * 0.3f);
                total_bias = clampf(total_bias, -LINE_BIAS_MAX_CPS, LINE_BIAS_MAX_CPS);
                
                // Apply differential drive using motor library
                motion_command_with_bias(MOVE_FORWARD, LINE_FOLLOW_SPEED_PCT, 
                                        total_bias, -total_bias);
                break;
            }

            case STATE_BARCODE_DETECTED: {
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
                    sleep_ms(300);
                    
                    // Update reference heading
                    g_initial_heading_deg = g_target_heading_deg;
                    
                    // Reset PIDs using library function
                    pid_reset(&pid_line_follow);
                    pid_reset(&pid_imu_heading);
                    
                    g_state = STATE_LINE_FOLLOW;
                    printf("[RESUME] Resuming line follow mode\n");
                }
                break;
            }

            case STATE_STOPPED: {
                motion_command(MOVE_STOP, 0);
                
                // Check for START button to resume
                if (gpio_get(BTN_START) == 0) {
                    sleep_ms(200);
                    
                    pid_reset(&pid_line_follow);
                    pid_reset(&pid_imu_heading);
                    
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

        // ===== 3) TELEMETRY OUTPUT (5 Hz) =====
        if ((telem_div++ % (1000 / LOOP_DT_MS / 5)) == 0) {
            print_telemetry();
        }

        // ===== 4) EMERGENCY STOP =====
        if (gpio_get(BTN_STOP) == 0) {
            motion_command(MOVE_STOP, 0);
            g_state = STATE_STOPPED;
            printf("\n[EMERGENCY] STOP button pressed - press START to resume\n");
            sleep_ms(500);
        }

        // ===== 5) LOOP TIMING (maintain 100 Hz) =====
        sleep_until(t_next);
        t_next = delayed_by_ms(t_next, LOOP_DT_MS);
    }

    return 0;
}