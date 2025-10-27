// demo1.c - Main control logic with calibration and adaptive wheel scaling
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include <stdio.h>

#include "config.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"
#include "wifi_connect.h"

// CONFIGURATION
#define RUN_SPEED_PERCENT   20
#define SOFTSTART_SEC       0.6f

// STATE MANAGEMENT
static volatile bool running = false;
static volatile bool g_override_motion = false;
static float initial_heading_deg = 0.0f;
static repeating_timer_t control_timer_motor;
static repeating_timer_t control_timer_imu;

// FORWARD DECLARATIONS
static bool control_cb(repeating_timer_t* t);
static void do_boot_auto_cal(void);
static bool prompt_boot_cal(void);
static void buttons_init(void);
static inline bool btn_pressed(uint gpio);

// BUTTON HELPERS
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

// MAIN CONTROL CALLBACK (100 Hz)
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

    // Get current smoothed wheel speeds
    float cps_r, cps_l;
    get_cps_smoothed(&cps_r, &cps_l);

    // Calculate encoder balance error (right - left)
    float diff_meas = cps_r - cps_l;
    float bias_track = pid_step(&pid_track, 0.0f, -diff_meas, dt);

    // Get IMU heading correction with supervisor gating
    float bias_head = imu_get_heading_bias(initial_heading_deg, dt);

    // Soft-start speed ramp
    float base_pct = (float)RUN_SPEED_PERCENT;
    float pct_eff = base_pct;
    if (SOFTSTART_SEC > 0.0f) {
        float tsoft = absolute_time_diff_us(run_t0, get_absolute_time()) / 1e6f;
        if (tsoft < SOFTSTART_SEC) {
            pct_eff = base_pct * (tsoft / SOFTSTART_SEC);
        }
    }

    float base_cps = (MAX_CPS * pct_eff) / 100.0f;

    // Apply dynamic limits to bias corrections
    float lim_track = fmaxf(5.0f, 0.55f * base_cps);
    float lim_head  = fmaxf(3.0f, 0.35f * base_cps);
    
    bias_track = clampf(bias_track, -lim_track, lim_track);
    bias_head  = clampf(bias_head, -lim_head, lim_head);

    // Combine biases with total safety limit
    float total_bias = bias_track + bias_head;
    float lim_total = fmaxf(6.0f, 0.60f * base_cps);
    total_bias = clampf(total_bias, -lim_total, lim_total);

    // Update adaptive wheel scaling
    motor_update_adaptive_scale(diff_meas, dt, base_cps);

    // Command motion with combined bias
    motion_command_with_bias(MOVE_FORWARD, (int)pct_eff, +total_bias, -total_bias);

    return true;
}

// CALIBRATION ROUTINES
static void do_mag_calibration(void) {
    printf("CAL: magnetometer min/max... spinning 3s\n");
    telemetry_set_mode(TMODE_CAL);

    g_override_motion = true;
    imu_cal_begin();

    absolute_time_t t0 = get_absolute_time();
    motion_command(MOVE_LEFT, 25);
    
    while (absolute_time_diff_us(t0, get_absolute_time()) < 3000000) {
        imu_state_t s;
        if (imu_read(&s) && s.ok) { /* keep filter updated */ }
        tight_loop_contents();
    }
    
    motion_command(MOVE_STOP, 0);
    imu_cal_end();
    g_override_motion = false;

    // Seed heading filter
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
    printf("TRIM: auto wheel scale... driving 4.0s\n");
    telemetry_set_mode(TMODE_CAL);

    g_override_motion = true;
    motion_command(MOVE_FORWARD, 20);
    sleep_ms(500);

    // Measure window (4.0 s)
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

// MAIN FUNCTION
int main() {
    stdio_init_all();
    sleep_ms(10000);
    printf("\n=== Car Demo 1: Cascaded Heading Control ===\n");

    // Initialize subsystems
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

    // Boot calibration prompt
    if (prompt_boot_cal()) {
        do_boot_auto_cal();
    } else {
        printf("BOOT: auto-calibration skipped.\n");
    }

    printf("Press START when ready.\n");

    // Main loop
    while (true) {
        if (!running) {
            if (btn_pressed(BTN_START)) {
                sleep_ms(100);
                if (btn_pressed(BTN_START)) {
                    print_telemetry_legend();

                    float sr, sl;
                    motor_get_wheel_scale(&sr, &sl);
                    printf("START: settle 4s, capture heading_ref, then soft-start. Scale[R=%.3f L=%.3f]\n", sr, sl);

                    // Settle period (4s)
                    telemetry_set_mode(TMODE_NONE);
                    g_override_motion = true;
                    motion_command(MOVE_STOP, 0);
                    
                    absolute_time_t t0 = get_absolute_time();
                    imu_state_t s;
                    while (absolute_time_diff_us(t0, get_absolute_time()) < 4000000) {
                        if (imu_read(&s) && s.ok) { /* keep filter updated */ }
                        tight_loop_contents();
                    }
                    
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

                    printf("START -> %d%%, heading_ref=%.1f deg\n", RUN_SPEED_PERCENT, initial_heading_deg);
                    telemetry_set_mode(TMODE_RUN);

                    while (btn_pressed(BTN_START)) tight_loop_contents();
                }
            } else {
                if (!g_override_motion) motor_all_stop();
            }
        } else {
            if (btn_pressed(BTN_STOP)) {
                sleep_ms(30);
                if (btn_pressed(BTN_STOP)) {
                    running = false;
                    telemetry_set_mode(TMODE_NONE);
                    motion_command(MOVE_STOP, 0);
                    motor_all_stop();
                    printf("STOP -> stopped.\n");
                    while (btn_pressed(BTN_STOP)) tight_loop_contents();
                }
            }
        }
        tight_loop_contents();
    }
}