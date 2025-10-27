// demo1.c - version using existing libraries
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include <stdio.h>

#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "pid.h"
#include "wifi_connect.h"

// ============================================================================
// STATE MANAGEMENT
// ============================================================================
static volatile bool running = false;
static volatile bool g_override_motion = false;
static float initial_heading_deg = 0.0f;
static repeating_timer_t control_timer_motor;
static repeating_timer_t control_timer_imu;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

static inline bool btn_pressed(uint gpio) {
    return gpio_get(gpio) == 0;
}

static void buttons_init(void) {
    gpio_init(BTN_START); 
    gpio_set_dir(BTN_START, GPIO_IN); 
    gpio_pull_up(BTN_START);
    
    gpio_init(BTN_STOP);  
    gpio_set_dir(BTN_STOP, GPIO_IN);  
    gpio_pull_up(BTN_STOP);
}

static bool wait_for_button_with_debounce(uint gpio, uint32_t debounce_ms) {
    if (btn_pressed(gpio)) {
        sleep_ms(debounce_ms);
        if (btn_pressed(gpio)) {
            while (btn_pressed(gpio)) tight_loop_contents();
            return true;
        }
    }
    return false;
}

// ============================================================================
// CONTROL LOOP CALLBACK (100 Hz)
// ============================================================================
static bool control_cb(repeating_timer_t* t) {
    volatile bool* p_run = (volatile bool*)t->user_data;
    const float dt = CONTROL_PERIOD_MS / 1000.0f;
    static absolute_time_t run_t0;
    static bool first_run = true;

    // Initialize run start time on first run
    if (first_run && p_run && *p_run) {
        run_t0 = get_absolute_time();
        first_run = false;
    }

    // Stop motors if not running and not overridden
    if (!p_run || !*p_run) {
        if (!g_override_motion) {
            motion_command(MOVE_STOP, 0);
        }
        first_run = true;
        return true;
    }

    // Get current smoothed wheel speeds using encoder library
    float cps_r, cps_l;
    get_cps_smoothed(&cps_r, &cps_l);

    // (A) Encoder-balance PID: use existing pid_lib
    float diff_meas = cps_r - cps_l;
    float bias_track = pid_step(&pid_track, 0.0f, -diff_meas, dt);

    // (B) IMU heading correction with supervisor gating (uses imu_lib API)
    float bias_head = imu_get_heading_bias(initial_heading_deg, dt);

    // Soft-start speed ramp
    float base_pct = (float)DEMO1_RUN_SPEED_PERCENT;
    float pct_eff = base_pct;
    if (DEMO1_SOFTSTART_SEC > 0.0f) {
        float tsoft = absolute_time_diff_us(run_t0, get_absolute_time()) / 1e6f;
        if (tsoft < DEMO1_SOFTSTART_SEC) {
            pct_eff = base_pct * (tsoft / DEMO1_SOFTSTART_SEC);
        }
    }

    float base_cps = (MAX_CPS * pct_eff) / 100.0f;

    // Apply dynamic limits to bias corrections (from config.h)
    float lim_track = fmaxf(BIAS_MIN_CPS, BIAS_TRACK_FRACTION * base_cps);
    float lim_head  = fmaxf(BIAS_MIN_HEAD_CPS, BIAS_HEAD_FRACTION * base_cps);
    
    bias_track = clampf(bias_track, -lim_track, lim_track);
    bias_head  = clampf(bias_head, -lim_head, lim_head);

    // Combine biases with total safety limit
    float total_bias = bias_track + bias_head;
    float lim_total = fmaxf(BIAS_MIN_TOTAL_CPS, BIAS_TOTAL_FRACTION * base_cps);
    total_bias = clampf(total_bias, -lim_total, lim_total);

    // Update adaptive wheel scaling (uses motor_lib API)
    motor_update_adaptive_scale(diff_meas, dt, base_cps);

    // Command motion with combined bias (uses motor_lib API)
    motion_command_with_bias(MOVE_FORWARD, (int)pct_eff, +total_bias, -total_bias);

    return true;
}

// ============================================================================
// CALIBRATION ROUTINES
// ============================================================================

static void do_mag_calibration(void) {
    printf("CAL: magnetometer min/max... spinning %dms\n", CAL_MAG_SPIN_DURATION_MS);
    telemetry_set_mode(TMODE_CAL);

    g_override_motion = true;
    imu_cal_begin();

    absolute_time_t t0 = get_absolute_time();
    motion_command(MOVE_LEFT, 25);
    
    while (absolute_time_diff_us(t0, get_absolute_time()) < (CAL_MAG_SPIN_DURATION_MS * 1000)) {
        imu_state_t s;
        if (imu_read(&s) && s.ok) { /* keep filter updated */ }
        tight_loop_contents();
    }
    
    motion_command(MOVE_STOP, 0);
    imu_cal_end();
    g_override_motion = false;

    // Seed heading filter with calibrated value
    imu_state_t s;
    if (imu_read(&s) && s.ok) {
        imu_reset_heading_filter(s.heading_deg_filt);
        initial_heading_deg = s.heading_deg_filt;
        imu_reset_heading_supervisor();
    }

    printf("CAL: done. heading_ref=%.1f deg\n", initial_heading_deg);
    telemetry_set_mode(TMODE_NONE);
}

static void do_auto_wheel_scale(void) {
    printf("TRIM: auto wheel scale... driving %dms\n", CAL_WHEEL_DRIVE_DURATION_MS); 
    telemetry_set_mode(TMODE_CAL);

    g_override_motion = true;

    // Brief settle
    motion_command(MOVE_FORWARD, 20);
    sleep_ms(CAL_WHEEL_SETTLE_MS);

    // Measure window
    float sum_r = 0, sum_l = 0;
    int n = 0;
    absolute_time_t t0 = get_absolute_time();
    
    while (absolute_time_diff_us(t0, get_absolute_time()) < (CAL_WHEEL_DRIVE_DURATION_MS * 1000)) {
        float r, l;
        get_cps_smoothed(&r, &l);
        sum_r += r;
        sum_l += l;
        n++;
        sleep_ms(CONTROL_PERIOD_MS);
    }
    
    motion_command(MOVE_STOP, 0);

    // Use motor_lib calibration function
    if (n > 0 && sum_l > 1.0f && sum_r > 1.0f) {
        float r_avg = sum_r / (float)n;
        float l_avg = sum_l / (float)n;
        motor_calibrate_wheel_scale(r_avg, l_avg);
        
        float sr, sl;
        motor_get_wheel_scale(&sr, &sl);
        printf("TRIM: scales R=%.3f L=%.3f\n", sr, sl);
    } else {
        motor_set_wheel_scale(1.0f, 1.0f);
        printf("TRIM: skipped (low cps)\n");
    }

    g_override_motion = false;
    telemetry_set_mode(TMODE_NONE);
}

static void do_boot_auto_cal(void) {
    do_mag_calibration();
    do_auto_wheel_scale();
    motion_command(MOVE_STOP, 0);
    printf("BOOT: auto calibration complete.\n");
}

// ============================================================================
// BOOT CALIBRATION PROMPT
// ============================================================================
static bool prompt_boot_cal(void) {
    printf("BOOT: run auto-calibration before START? [y/N]\n");
    printf("Press START = yes, STOP = no, or type y/n. Auto-skip in 10s...\n");
    
    absolute_time_t t0 = get_absolute_time();
    while (absolute_time_diff_us(t0, get_absolute_time()) < 10000000) {
        int ch = getchar_timeout_us(0);
        if (ch == 'y' || ch == 'Y') { 
            printf(" -> yes\n"); 
            return true; 
        }
        if (ch == 'n' || ch == 'N') { 
            printf(" -> no\n");  
            return false; 
        }

        if (wait_for_button_with_debounce(BTN_START, BTN_DEBOUNCE_MS)) {
            printf(" -> yes (START)\n");
            return true;
        }
        if (wait_for_button_with_debounce(BTN_STOP, BTN_DEBOUNCE_MS)) {
            printf(" -> no (STOP)\n");
            return false;
        }

        tight_loop_contents();
    }
    
    printf("-> timeout; skipping auto-calibration.\n");
    return false;
}

// ============================================================================
// SYSTEM INITIALIZATION
// ============================================================================
static void system_init(void) {
    stdio_init_all();
    sleep_ms(10000);
    printf("\n=== Car Demo 1: Cascaded Heading Control ===\n");

    buttons_init();

    if (!imu_init()) {
        printf("IMU init failed! Check I2C wiring (GP2 SDA, GP3 SCL).\n");
    } else {
        printf("IMU ready.\n");
    }

    motor_init();
    pid_init_defaults();

    // Start control timers
    add_repeating_timer_ms(-CONTROL_PERIOD_MS, motor_control_timer_cb, NULL, &control_timer_motor);
    add_repeating_timer_ms(-CONTROL_PERIOD_MS, control_cb, (void*)&running, &control_timer_imu);

    // Optional Wi-Fi connection
    bool wifi_ok = wifi_try_connect_once(20000);
    if (wifi_ok) {
        printf("[BOOT] Wi-Fi OK. IP: %s\n", wifi_ip_str());
    } else {
        printf("[BOOT] Wi-Fi not available. Sensors and motors will continue.\n");
    }
}

// ============================================================================
// RUN STATE MANAGEMENT
// ============================================================================
static void start_run_sequence(void) {
    print_telemetry_legend();

    float sr, sl;
    motor_get_wheel_scale(&sr, &sl);
    printf("START: settle %dms, capture heading_ref, then soft-start. Scale[R=%.3f L=%.3f]\n", 
           DEMO1_SETTLE_TIME_MS, sr, sl);

    // Settle period
    telemetry_set_mode(TMODE_NONE);
    g_override_motion = true;
    motion_command(MOVE_STOP, 0);
    
    absolute_time_t t0 = get_absolute_time();
    imu_state_t s;
    while (absolute_time_diff_us(t0, get_absolute_time()) < (DEMO1_SETTLE_TIME_MS * 1000)) {
        if (imu_read(&s) && s.ok) { /* keep filter updated */ }
        tight_loop_contents();
    }
    
    // Capture initial heading
    if (imu_read(&s) && s.ok) {
        initial_heading_deg = s.heading_deg_filt;
        imu_reset_heading_filter(initial_heading_deg);
    } else {
        initial_heading_deg = 0.0f;
    }

    // Reset all controllers and filters
    pid_reset(&pid_track);
    pid_reset(&pid_heading);
    motor_reset_controllers();
    motor_reset_speed_filters();
    motor_reset_distance_counters();
    motor_reset_adaptive_scale();
    imu_reset_heading_supervisor();

    g_override_motion = false;
    motion_command(MOVE_FORWARD, 0);
    running = true;

    printf("START -> %d%%, heading_ref=%.1f deg\n", DEMO1_RUN_SPEED_PERCENT, initial_heading_deg);
    telemetry_set_mode(TMODE_RUN);
}

static void stop_run_sequence(void) {
    running = false;
    telemetry_set_mode(TMODE_NONE);
    motion_command(MOVE_STOP, 0);
    motor_all_stop();
    printf("STOP -> stopped.\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
int main() {
    system_init();

    // Boot calibration prompt
    if (prompt_boot_cal()) {
        do_boot_auto_cal();
    } else {
        printf("BOOT: auto-calibration skipped.\n");
    }

    printf("Press START when ready.\n");

    // Main event loop
    while (true) {
        if (!running) {
            if (wait_for_button_with_debounce(BTN_START, BTN_DEBOUNCE_MS)) {
                start_run_sequence();
            } else {
                if (!g_override_motion) {
                    motor_all_stop();
                }
            }
        } else {
            if (wait_for_button_with_debounce(BTN_STOP, 30)) {
                stop_run_sequence();
            }
        }
        tight_loop_contents();
    }
}