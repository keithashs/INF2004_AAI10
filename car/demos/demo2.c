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

// ===== Demo 2 Parameters =====
#define LINE_FOLLOW_SPEED_PCT  25       // Base speed while following line
#define TURN_SPEED_PCT         20       // Speed during barcode turns
#define TURN_ANGLE_90          90.0f    // Degrees for LEFT/RIGHT
#define TURN_ANGLE_180         180.0f   // Degrees for UTURN
#define TURN_TOLERANCE_DEG     3.0f     // Turn completion threshold

#define LOOP_DT_MS             10       // 100 Hz control loop
#define DT_S                   ((float)LOOP_DT_MS / 1000.0f)

// ===== Line Following PID (steering correction) =====
#define LINE_KP                0.40f
#define LINE_KI                0.05f
#define LINE_KD                0.10f
#define LINE_BIAS_MAX_CPS      8.0f     // Max steering correction (cps)

// ===== State Machine =====
typedef enum {
    STATE_IDLE,
    STATE_LINE_FOLLOW,
    STATE_BARCODE_DETECTED,
    STATE_EXECUTING_TURN,
    STATE_STOPPED,
    STATE_LINE_LOST
} robot_state_t;

static robot_state_t g_state = STATE_IDLE;
static barcode_cmd_t g_pending_command = BARCODE_NONE;
static float g_target_heading_deg = 0.0f;
static float g_turn_start_heading_deg = 0.0f;

// ===== Telemetry Cache =====
static struct {
    line_position_t line_pos;
    barcode_cmd_t last_barcode;
    float heading_deg;
    float speed_cmps;
    float distance_cm;
    robot_state_t state;
} g_telem = {0};

// ===== Helpers =====
static inline float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// ===== Line Following Controller =====
// Uses P-PID to convert line position error → steering bias
static float g_line_integ = 0.0f;
static float g_line_prev_err = 0.0f;

static float line_follow_pid(line_position_t pos, float dt) {
    float err = 0.0f;
    
    switch (pos) {
        case LINE_CENTER:   err = 0.0f;  break;
        case LINE_LEFT:     err = -1.0f; break;  // Robot drifted right, steer left (negative bias)
        case LINE_RIGHT:    err = +1.0f; break;  // Robot drifted left, steer right (positive bias)
        case LINE_BOTH_OFF: err = 0.0f;  break;  // Might be at junction, hold course
        default:            err = 0.0f;  break;
    }

    g_line_integ += err * dt;
    g_line_integ = clampf(g_line_integ, -10.0f, 10.0f);  // Anti-windup
    
    float deriv = (err - g_line_prev_err) / dt;
    g_line_prev_err = err;

    float bias = LINE_KP * err + LINE_KI * g_line_integ + LINE_KD * deriv;
    return clampf(bias, -LINE_BIAS_MAX_CPS, LINE_BIAS_MAX_CPS);
}

// ===== Turn Execution (IMU-based) =====
static bool execute_turn(float angle_deg, float current_heading, float dt) {
    static float last_hdg = 0.0f;
    static int stall_count = 0;

    float delta = wrap180(g_target_heading_deg - current_heading);
    
    // Check if turn complete
    if (fabsf(delta) < TURN_TOLERANCE_DEG) {
        motion_command(MOVE_STOP, 0);
        return true;  // Turn complete
    }

    // Simple proportional turn rate
    float turn_rate_cps = clampf(delta * 0.5f, -15.0f, 15.0f);
    
    // Differential drive: right wheel faster for right turn, left wheel faster for left turn
    if (delta > 0) {
        // Turn right: left wheel forward, right wheel slower/reverse
        motion_command_with_bias(MOVE_FORWARD, TURN_SPEED_PCT, turn_rate_cps, -turn_rate_cps);
    } else {
        // Turn left: right wheel forward, left wheel slower/reverse
        motion_command_with_bias(MOVE_FORWARD, TURN_SPEED_PCT, -turn_rate_cps, turn_rate_cps);
    }

    // Stall detection
    if (fabsf(current_heading - last_hdg) < 0.5f) {
        stall_count++;
        if (stall_count > 100) {  // 1 second of no progress
            printf("[DEMO2] Turn stalled, forcing completion\n");
            return true;
        }
    } else {
        stall_count = 0;
    }
    last_hdg = current_heading;

    return false;  // Turn in progress
}

// ===== Telemetry Print (5 Hz) =====
static void print_telemetry(void) {
    printf("[DEMO2] State=%d LinePos=%s Barcode=%s Heading=%.1f Speed=%.1fcm/s Dist=%.1fcm\n",
           g_telem.state,
           ir_line_position_str(g_telem.line_pos),
           barcode_cmd_str(g_telem.last_barcode),
           (double)g_telem.heading_deg,
           (double)g_telem.speed_cmps,
           (double)g_telem.distance_cm);
}

// ===== Main =====
int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("\n=== Car Demo 2: Line Following + Barcode + IMU Turns ===\n");

    // Initialize modules
    if (!imu_init()) {
        printf("[DEMO2] IMU init FAILED\n");
        while (1) sleep_ms(1000);
    }
    printf("[DEMO2] IMU OK\n");

    motor_init();
    printf("[DEMO2] Motors OK\n");

    ir_line_init();
    printf("[DEMO2] Line sensors OK\n");

    barcode_init();
    printf("[DEMO2] Barcode scanner OK\n");

    pid_init_defaults();

    // Capture initial heading
    sleep_ms(500);
    imu_state_t imu;
    if (imu_read(&imu) && imu.ok) {
        g_target_heading_deg = imu.heading_deg_filt;
        printf("[DEMO2] Initial heading: %.1f deg\n", (double)g_target_heading_deg);
    }

    // Wait for button press
    printf("[DEMO2] Press START (GP21) to begin...\n");
    while (gpio_get(BTN_START) != 0) {
        tight_loop_contents();
    }
    sleep_ms(200);  // Debounce
    printf("[DEMO2] Starting line follow!\n");

    motor_reset_distance_counters();
    g_state = STATE_LINE_FOLLOW;

    absolute_time_t t_next = make_timeout_time_ms(LOOP_DT_MS);
    uint32_t telem_div = 0;

    // ===== Main Loop =====
    while (true) {
        // === 1) Read Sensors ===
        line_position_t line_pos = ir_get_line_position();
        barcode_result_t barcode_res = {0};
        bool barcode_detected = barcode_poll(&barcode_res);
        
        imu_state_t imu_data;
        bool imu_ok = imu_read(&imu_data) && imu_data.ok;
        float current_heading = imu_ok ? imu_data.heading_deg_filt : g_target_heading_deg;

        float avg_speed = get_average_speed();  // cm/s
        float avg_dist = get_average_distance();  // cm/s

        // Update telemetry cache
        g_telem.line_pos = line_pos;
        g_telem.heading_deg = current_heading;
        g_telem.speed_cmps = avg_speed;
        g_telem.distance_cm = avg_dist;
        g_telem.state = g_state;

        // === 2) State Machine ===
        switch (g_state) {
            case STATE_LINE_FOLLOW:
                // Check for barcode first
                if (barcode_detected && barcode_res.command != BARCODE_NONE) {
                    g_pending_command = barcode_res.command;
                    g_telem.last_barcode = barcode_res.command;
                    g_state = STATE_BARCODE_DETECTED;
                    printf("[DEMO2] Barcode detected: %s\n", barcode_cmd_str(g_pending_command));
                    barcode_reset();
                    break;
                }

                // Check for line loss
                {
                    static uint32_t lost_count = 0;
                    
                    if (line_pos == LINE_BOTH_OFF) {
                        lost_count++;
                        if (lost_count > 50) {  // 0.5s of being lost
                            g_state = STATE_LINE_LOST;
                            motion_command(MOVE_STOP, 0);
                            printf("[DEMO2] Line LOST\n");
                            break;
                        }
                    } else {
                        lost_count = 0;  // Reset when line is found
                    }
                }

                // Normal line following with PID steering
                {
                    float line_bias = line_follow_pid(line_pos, DT_S);
                    // line_bias > 0 means steer right (left wheel faster)
                    motion_command_with_bias(MOVE_FORWARD, LINE_FOLLOW_SPEED_PCT, 
                                            line_bias, -line_bias);
                }
                break;


            case STATE_BARCODE_DETECTED:
                // Execute command
                switch (g_pending_command) {
                    case BARCODE_LEFT:
                        g_turn_start_heading_deg = current_heading;
                        g_target_heading_deg = wrap180(current_heading - TURN_ANGLE_90);
                        g_state = STATE_EXECUTING_TURN;
                        printf("[DEMO2] Executing LEFT turn to %.1f deg\n", (double)g_target_heading_deg);
                        break;

                    case BARCODE_RIGHT:
                        g_turn_start_heading_deg = current_heading;
                        g_target_heading_deg = wrap180(current_heading + TURN_ANGLE_90);
                        g_state = STATE_EXECUTING_TURN;
                        printf("[DEMO2] Executing RIGHT turn to %.1f deg\n", (double)g_target_heading_deg);
                        break;

                    case BARCODE_UTURN:
                        g_turn_start_heading_deg = current_heading;
                        g_target_heading_deg = wrap180(current_heading + TURN_ANGLE_180);
                        g_state = STATE_EXECUTING_TURN;
                        printf("[DEMO2] Executing U-TURN to %.1f deg\n", (double)g_target_heading_deg);
                        break;

                    case BARCODE_STOP:
                        motion_command(MOVE_STOP, 0);
                        g_state = STATE_STOPPED;
                        printf("[DEMO2] STOP command executed\n");
                        break;

                    default:
                        g_state = STATE_LINE_FOLLOW;  // Resume if invalid
                        break;
                }
                g_pending_command = BARCODE_NONE;
                break;

            case STATE_EXECUTING_TURN:
                {
                    bool turn_done = execute_turn(
                        wrap180(g_target_heading_deg - g_turn_start_heading_deg),
                        current_heading,
                        DT_S
                    );
                    if (turn_done) {
                        printf("[DEMO2] Turn complete, resuming line follow\n");
                        motion_command(MOVE_STOP, 0);
                        sleep_ms(200);  // Brief pause to settle
                        g_state = STATE_LINE_FOLLOW;
                        g_line_integ = 0.0f;  // Reset line PID
                        g_line_prev_err = 0.0f;
                    }
                }
                break;

            case STATE_STOPPED:
                motion_command(MOVE_STOP, 0);
                // Check if START button pressed to resume
                if (gpio_get(BTN_START) == 0) {
                    sleep_ms(200);
                    g_state = STATE_LINE_FOLLOW;
                    printf("[DEMO2] Resuming from STOP\n");
                }
                break;

            case STATE_LINE_LOST:
                // Recovery strategy: slow reverse, try to reacquire
                motion_command(MOVE_BACKWARD, 10);
                sleep_ms(300);
                motion_command(MOVE_STOP, 0);
                if (ir_get_line_position() != LINE_BOTH_OFF) {
                    g_state = STATE_LINE_FOLLOW;
                    printf("[DEMO2] Line reacquired\n");
                }
                break;

            default:
                g_state = STATE_LINE_FOLLOW;
                break;
        }

        // === 3) Telemetry (5 Hz) ===
        if ((telem_div++ % (1000 / LOOP_DT_MS / 5)) == 0) {
            print_telemetry();
        }

        // === 4) Emergency stop ===
        if (gpio_get(BTN_STOP) == 0) {
            motion_command(MOVE_STOP, 0);
            printf("[DEMO2] EMERGENCY STOP\n");
            while (1) sleep_ms(1000);
        }

        // === 5) Loop timing ===
        sleep_until(t_next);
        t_next = delayed_by_ms(t_next, LOOP_DT_MS);
    }

    return 0;
}