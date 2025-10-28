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

#define LINE_FOLLOW_SPEED_PCT  25       // Base speed while following line
#define TURN_SPEED_PCT         20       // Speed during turns
#define TURN_ANGLE_90          90.0f    // LEFT/RIGHT turn angle
#define TURN_ANGLE_180         180.0f   // U-TURN angle
#define TURN_TOLERANCE_DEG     5.0f     // Turn completion threshold

#define LOOP_DT_MS             10       // 100 Hz control loop
#define DT_S                   ((float)LOOP_DT_MS / 1000.0f)

// ============================================================================
// LINE FOLLOWING PID GAINS (for 1.7cm thin line with single sensor)
// ============================================================================
#define LINE_KP                0.75f    // Aggressive for tight line
#define LINE_KI                0.10f
#define LINE_KD                0.12f
#define LINE_BIAS_MAX_CPS      12.0f    // Max steering correction

// IMU heading correction (same as demo1)
#define IMU_HEADING_KP         0.25f
#define IMU_HEADING_KI         0.02f
#define IMU_HEADING_KD         0.00f
#define IMU_BIAS_MAX_CPS       6.0f

// Line loss detection
#define LINE_LOST_COUNT_MAX    100      // 1.0s of no line = lost

// ============================================================================
// STATE MACHINE
// ============================================================================
typedef enum {
    STATE_IDLE,
    STATE_LINE_FOLLOW,
    STATE_BARCODE_DETECTED,
    STATE_EXECUTING_TURN,
    STATE_STOPPED,
    STATE_LINE_LOST
} robot_state_t;

// ============================================================================
// GLOBAL STATE
// ============================================================================
static robot_state_t g_state = STATE_IDLE;
static barcode_cmd_t g_pending_command = BARCODE_NONE;
static float g_target_heading_deg = 0.0f;
static float g_initial_heading_deg = 0.0f;
static uint32_t g_line_lost_count = 0;

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
// IMU HEADING PID (for long-term drift correction)
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
// TELEMETRY CACHE
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
    robot_state_t state;
} g_telem = {0};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static inline float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}


// ============================================================================
// LINE FOLLOWING CONTROLLER
// ============================================================================
static float line_follow_pid_controller(line_reading_t *reading, float dt) {
    // Use analog error from line sensor
    float err = reading->error;
    
    // Apply PID
    float bias = pid_step(&pid_line_follow, 0.0f, -err, dt);
    g_telem.line_bias_cps = bias;
    
    return bias;
}

// ============================================================================
// IMU HEADING CORRECTION
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
    if (fabsf(err_deg) < 2.0f) err_deg = 0.0f;

    float bias = pid_step(&pid_imu_heading, 0.0f, -err_deg, dt);
    g_telem.imu_bias_cps = bias;
    
    return bias;
}

// ============================================================================
// TURN EXECUTION
// ============================================================================
static bool execute_turn(float current_heading, float dt) {
    static float last_hdg = 0.0f;
    static int stall_count = 0;

    float delta = wrap180(g_target_heading_deg - current_heading);
    
    if (fabsf(delta) < TURN_TOLERANCE_DEG) {
        motion_command(MOVE_STOP, 0);
        stall_count = 0;
        return true;
    }

    // Proportional turn rate
    float turn_rate_cps = clampf(delta * 0.6f, -18.0f, 18.0f);
    
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
            printf("[DEMO2] Turn stalled, forcing completion\n");
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
// TELEMETRY PRINT (5 Hz)
// ============================================================================
static void print_telemetry(void) {
    printf("[DEMO2] State=%d LineErr=%.2f ADC=%u IMU_Err=%.1f° "
           "Speed=%.1fcm/s Dist=%.1fcm LineBias=%.1f IMUBias=%.1f\n",
           g_telem.state,
           g_telem.line_reading.error,
           g_telem.line_reading.raw_adc,
           g_telem.heading_error_deg,
           g_telem.speed_cmps,
           g_telem.distance_cm,
           g_telem.line_bias_cps,
           g_telem.imu_bias_cps);
}

// ============================================================================
// SYSTEM INITIALIZATION
// ============================================================================
static void system_init(void) {
    stdio_init_all();
    sleep_ms(10000);
    printf("\n╔═══════════════════════════════════════════════════════════╗\n");
    printf("║   DEMO 2: Line Following + Barcode + IMU (Refactored)    ║\n");
    printf("║   Optimized for THIN LINE (1.7cm) - Single Left Sensor   ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n\n");

    if (!imu_init()) {
        printf("[ERROR] IMU init FAILED\n");
        while (1) sleep_ms(1000);
    }
    printf("[OK] IMU initialized\n");

    motor_init();
    printf("[OK] Motors initialized\n");

    ir_line_init();
    printf("[OK] Line sensor initialized (GP%d)\n", IR_LEFT_ADC);

    barcode_init();
    printf("[OK] Barcode scanner initialized (GP%d)\n", BARCODE_ADC_GPIO);

    pid_reset(&pid_line_follow);
    pid_reset(&pid_imu_heading);
    printf("[OK] PID controllers initialized\n");

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
        printf("[OK] Initial heading: %.1f°\n", g_initial_heading_deg);
    } else {
        printf("[WARN] IMU not ready, using 0.0°\n");
        g_initial_heading_deg = 0.0f;
        g_target_heading_deg = 0.0f;
    }
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================
int main(void) {
    system_init();

    printf("═══════════════════════════════════════════════════════════\n");
    printf("  Press START (GP21) to begin line following...\n");
    printf("═══════════════════════════════════════════════════════════\n\n");
    
    while (gpio_get(BTN_START) != 0) {
        tight_loop_contents();
    }
    sleep_ms(200);
    
    printf("[START] Robot activated!\n\n");
    motor_reset_distance_counters();
    g_state = STATE_LINE_FOLLOW;

    absolute_time_t t_next = make_timeout_time_ms(LOOP_DT_MS);
    uint32_t telem_div = 0;

    // ========== MAIN CONTROL LOOP (100 Hz) ==========
    while (true) {
        // ===== 1) READ SENSORS =====
        line_reading_t line = ir_get_line_error();
        
        barcode_result_t barcode_res = {0};
        bool barcode_detected = barcode_poll(&barcode_res);
        
        imu_state_t imu_data;
        bool imu_ok = imu_read(&imu_data) && imu_data.ok;
        float current_heading = imu_ok ? imu_data.heading_deg_filt : g_target_heading_deg;

        float avg_speed = get_average_speed_cmps();
        float avg_dist = get_average_distance_cm();

        // Update telemetry
        g_telem.line_reading = line;
        g_telem.heading_deg = current_heading;
        g_telem.speed_cmps = avg_speed;
        g_telem.distance_cm = avg_dist;
        g_telem.state = g_state;

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
                if (!line.line_detected) {
                    g_line_lost_count++;
                    if (g_line_lost_count > LINE_LOST_COUNT_MAX) {
                        g_state = STATE_LINE_LOST;
                        motion_command(MOVE_STOP, 0);
                        printf("[WARN] Line LOST\n");
                        break;
                    }
                } else {
                    g_line_lost_count = 0;
                }

                // Normal line following with combined PID
                float line_bias = line_follow_pid_controller(&line, DT_S);
                float imu_bias = imu_heading_correction(current_heading, DT_S);
                
                // Combine (line following is primary)
                float total_bias = line_bias + (imu_bias * 0.25f);
                total_bias = clampf(total_bias, -LINE_BIAS_MAX_CPS, LINE_BIAS_MAX_CPS);
                
                motion_command_with_bias(MOVE_FORWARD, LINE_FOLLOW_SPEED_PCT, 
                                        total_bias, -total_bias);
                break;
            }

            case STATE_BARCODE_DETECTED: {
                switch (g_pending_command) {
                    case BARCODE_LEFT:
                        g_target_heading_deg = wrap180(current_heading - TURN_ANGLE_90);
                        g_state = STATE_EXECUTING_TURN;
                        printf("[TURN] LEFT to %.1f°\n", g_target_heading_deg);
                        break;

                    case BARCODE_RIGHT:
                        g_target_heading_deg = wrap180(current_heading + TURN_ANGLE_90);
                        g_state = STATE_EXECUTING_TURN;
                        printf("[TURN] RIGHT to %.1f°\n", g_target_heading_deg);
                        break;

                    case BARCODE_UTURN:
                        g_target_heading_deg = wrap180(current_heading + TURN_ANGLE_180);
                        g_state = STATE_EXECUTING_TURN;
                        printf("[TURN] U-TURN to %.1f°\n", g_target_heading_deg);
                        break;

                    case BARCODE_STOP:
                        motion_command(MOVE_STOP, 0);
                        g_state = STATE_STOPPED;
                        printf("[STOP] Halted\n");
                        break;

                    default:
                        g_state = STATE_LINE_FOLLOW;
                        break;
                }
                g_pending_command = BARCODE_NONE;
                break;
            }

            case STATE_EXECUTING_TURN: {
                bool turn_done = execute_turn(current_heading, DT_S);
                if (turn_done) {
                    printf("[TURN] Complete (%.1f°)\n", current_heading);
                    motion_command(MOVE_STOP, 0);
                    sleep_ms(300);
                    
                    g_initial_heading_deg = g_target_heading_deg;
                    pid_reset(&pid_line_follow);
                    pid_reset(&pid_imu_heading);
                    
                    g_state = STATE_LINE_FOLLOW;
                    printf("[RESUME] Line follow\n");
                }
                break;
            }

            case STATE_STOPPED: {
                motion_command(MOVE_STOP, 0);
                
                if (gpio_get(BTN_START) == 0) {
                    sleep_ms(200);
                    pid_reset(&pid_line_follow);
                    pid_reset(&pid_imu_heading);
                    g_state = STATE_LINE_FOLLOW;
                    printf("[RESTART] Resuming\n");
                }
                break;
            }

            case STATE_LINE_LOST: {
                // Attempt recovery (reverse slowly)
                motion_command(MOVE_BACKWARD, 15);
                sleep_ms(500);
                motion_command(MOVE_STOP, 0);
                
                // Check if line reacquired
                line_reading_t check = ir_get_line_error();
                if (check.line_detected) {
                    g_state = STATE_LINE_FOLLOW;
                    g_line_lost_count = 0;
                    printf("[RECOVERY] Line reacquired\n");
                } else {
                    g_state = STATE_STOPPED;
                    printf("[RECOVERY] Failed - stopping\n");
                }
                break;
            }

            default:
                g_state = STATE_LINE_FOLLOW;
                break;
        }

        // ===== 3) TELEMETRY (5 Hz) =====
        if ((telem_div++ % (1000 / LOOP_DT_MS / 5)) == 0) {
            print_telemetry();
        }

        // ===== 4) EMERGENCY STOP =====
        if (gpio_get(BTN_STOP) == 0) {
            motion_command(MOVE_STOP, 0);
            g_state = STATE_STOPPED;
            printf("\n[EMERGENCY] STOP pressed\n");
            sleep_ms(500);
        }

        // ===== 5) LOOP TIMING =====
        sleep_until(t_next);
        t_next = delayed_by_ms(t_next, LOOP_DT_MS);
    }

    return 0;
}