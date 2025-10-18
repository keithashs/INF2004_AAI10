#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include <stdio.h>
#include <math.h>

#include "config.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"

// ================== PIDs ==================
// 1) Wheel-balance (encoder-difference) PID: keeps cps_R ~ cps_L (fast)
static PID pid_track = {
    .kp = 0.90f, .ki = 1.8f, .kd = 0.00f,
    .integ = 0, .prev_err = 0,
    .out_min = -100.0f, .out_max = +100.0f
};

// 2) IMU heading PID (slow trim) -- softened
static PID pid_heading = {
    .kp = 0.25f, .ki = 0.04f, .kd = 0.00f,
    .integ = 0, .prev_err = 0,
    .out_min = -100.0f, .out_max = +100.0f
};

// Timers
static repeating_timer_t control_timer_motor;   // 10 ms: motor PID & telemetry (in motor.c)
static repeating_timer_t control_timer_imu;     // 10 ms: encoder-balance + IMU trim (this file)

// Run state
static volatile bool running = false;

// Override so control_cb doesn't STOP motors during calibration/trim
static volatile bool g_override_motion = false;

// Demo settings
static float initial_heading_deg = 0.0f;
static int   run_speed_percent   = 20;

// ---- Telemetry cache ----
volatile imu_state_t g_imu_last = {0};
volatile float       g_heading_err_deg = 0.0f;
volatile float       g_bias_cps        = 0.0f;
volatile bool        g_imu_ok          = false;

// ---- Heading supervisor (gate IMU influence) ----
typedef struct {
    float last_heading_deg;
    absolute_time_t last_t;
    float head_weight;          // 0..1 multiplier on IMU bias
    bool  initialized;
} head_sup_t;

static head_sup_t HS = {0};

static inline bool btn_pressed(uint gpio) {
    return gpio_get(gpio) == 0;
}

static void buttons_init(void) {
    gpio_init(BTN_START); gpio_set_dir(BTN_START, GPIO_IN); gpio_pull_up(BTN_START);
    gpio_init(BTN_STOP);  gpio_set_dir(BTN_STOP,  GPIO_IN);  gpio_pull_up(BTN_STOP);
}

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

// ----------------- Fast control @ 100 Hz -----------------
// Does BOTH: (A) wheel-balance PID from encoders, (B) IMU trim gated by health
static bool control_cb(repeating_timer_t* t) {
    volatile bool* p_run = (volatile bool*)t->user_data;

    // If not running and not in an override phase, keep motors stopped.
    if (!p_run || !*p_run) {
        if (!g_override_motion) {
            motion_command(MOVE_STOP, 0);
        }
        return true;
    }

    // Current smoothed cps from motor.c
    float cps_r, cps_l;
    get_cps_smoothed(&cps_r, &cps_l);

    // (A) Encoder-balance error: right minus left (we want 0)
    float diff_meas = cps_r - cps_l;
    const float dt = CONTROL_PERIOD_MS / 1000.0f;

    // positive bias -> LEFT faster, RIGHT slower
    float bias_track = pid_step(&pid_track, 0.0f, -diff_meas, dt);

    // (B) IMU heading small trim, gated by supervisor
    float bias_head = 0.0f;
    float head_weight = HS.head_weight; // default from previous step

    imu_state_t s;
    bool imu_ok = imu_read(&s) && s.ok;

    if (imu_ok) {
        // compute heading error
        float err_deg = wrap180(s.heading_deg_filt - initial_heading_deg);

        // deadband
        if (fabsf(err_deg) < 0.8f) err_deg = 0.0f;

        // --- heading supervisor: check sanity ---
        // 1) tilt within bounds
        bool tilt_ok = (fabsf(s.roll_deg) <= 15.0f) && (fabsf(s.pitch_deg) <= 15.0f);

        // 2) heading rate limit (deg/s)
        float rate_ok = true;
        if (!HS.initialized) {
            HS.last_heading_deg = s.heading_deg_filt;
            HS.last_t = get_absolute_time();
            HS.head_weight = 0.0f; // ramp in
            HS.initialized = true;
        } else {
            absolute_time_t now = get_absolute_time();
            float dt_s = absolute_time_diff_us(HS.last_t, now) / 1e6f;
            if (dt_s > 0.0005f) {
                float d = wrap180(s.heading_deg_filt - HS.last_heading_deg);
                float rate = fabsf(d) / dt_s; // deg/s
                // If heading is "whipping" faster than 45°/s, don't trust it
                rate_ok = (rate <= 45.0f);
                HS.last_heading_deg = s.heading_deg_filt;
                HS.last_t = now;
            }
        }

        bool healthy = tilt_ok && rate_ok;

        // Smoothly move head_weight toward 1 when healthy, toward 0 when not
        float tau_up = 0.5f;   // seconds to trust IMU after it's healthy
        float tau_dn = 0.15f;  // seconds to drop trust when unhealthy
        float a_up = clampf(dt / tau_up, 0.0f, 1.0f);
        float a_dn = clampf(dt / tau_dn, 0.0f, 1.0f);

        if (healthy) head_weight = head_weight + (1.0f - head_weight) * a_up;
        else         head_weight = head_weight + (0.0f - head_weight) * a_dn;

        head_weight = clampf(head_weight, 0.0f, 1.0f);
        HS.head_weight = head_weight;

        // IMU PID with weight
        float raw_bias_head = pid_step(&pid_heading, 0.0f, -err_deg, dt);
        bias_head = raw_bias_head * head_weight;

        // Telemetry cache
        g_imu_last = s;
        g_heading_err_deg = err_deg;
        g_imu_ok = true;
    } else {
        // IMU not available: ramp weight down
        float a_dn = clampf(dt / 0.1f, 0.0f, 1.0f);
        head_weight = HS.head_weight + (0.0f - HS.head_weight) * a_dn;
        HS.head_weight = clampf(head_weight, 0.0f, 1.0f);
        g_imu_ok = false;
    }

    // Total bias (CPS) = fast track + gated heading trim
    float base_cps = (MAX_CPS * (float)run_speed_percent) / 100.0f;

    float lim_track = fmaxf(5.0f, 0.55f * base_cps);
    float lim_head  = fmaxf(2.0f, 0.25f * base_cps); // tighter IMU clamp

    if (bias_track > +lim_track) bias_track = +lim_track;
    if (bias_track < -lim_track) bias_track = -lim_track;

    if (bias_head > +lim_head)  bias_head  = +lim_head;
    if (bias_head < -lim_head)  bias_head  = -lim_head;

    float total_bias = bias_track + bias_head;

    // Final absolute clamp (safety)
    float lim_total  = fmaxf(6.0f, 0.60f * base_cps);
    if (total_bias > +lim_total) total_bias = +lim_total;
    if (total_bias < -lim_total) total_bias = -lim_total;

    motion_command_with_bias(MOVE_FORWARD, run_speed_percent, +total_bias, -total_bias);

    // Telemetry cache
    g_bias_cps = total_bias;

    return true;
}

// ======= Calibration / Trim helpers =======

// Spin in place for ~3 s to collect mag min/max
static void do_mag_calibration(void) {
    printf("CAL: magnetometer min/max... spinning 3s\n");
    g_override_motion = true;
    imu_cal_begin();

    absolute_time_t t0 = get_absolute_time();

    // spin left
    motion_command(MOVE_LEFT, 25);
    while (absolute_time_diff_us(t0, get_absolute_time()) < 3000000) {
        imu_state_t s;
        if (imu_read(&s) && s.ok) { imu_cal_feed(&s); }
        tight_loop_contents();
    }
    motion_command(MOVE_STOP, 0);

    imu_cal_end();
    g_override_motion = false;

    // Seed heading filter with the current filtered value
    imu_state_t s;
    if (imu_read(&s) && s.ok) {
        imu_reset_heading_filter(s.heading_deg_filt);
        initial_heading_deg = s.heading_deg_filt;
        HS.initialized = false; // re-init supervisor with new seed
        HS.head_weight = 0.0f;
    }
    printf("CAL: done. heading_ref=%.1f deg\n", initial_heading_deg);
}

// Drive straight gently for ~1.5 s to measure cps ratio and set scale
static void do_auto_wheel_scale(void) {
    printf("TRIM: auto wheel scale... driving 1.5s\n");
    g_override_motion = true;

    // brief settle
    motion_command(MOVE_FORWARD, 20);
    sleep_ms(500);

    // measure window
    float sum_r = 0, sum_l = 0; int n = 0;
    absolute_time_t t0 = get_absolute_time();
    while (absolute_time_diff_us(t0, get_absolute_time()) < 1500000) {
        float r, l; get_cps_smoothed(&r, &l);
        sum_r += r; sum_l += l; n++;
        sleep_ms(10);
    }
    motion_command(MOVE_STOP, 0);

    if (n > 0 && sum_l > 1.0f && sum_r > 1.0f) {
        float r_avg = sum_r / (float)n;
        float l_avg = sum_l / (float)n;

        // ratio k = r/l ; symmetric scale: R*=1/sqrt(k), L*=sqrt(k)
        float k  = r_avg / l_avg;
        float sr = 1.0f / sqrtf(k);
        float sl = sqrtf(k);

        // Limit scales to sane bounds (±15% to allow stronger correction)
        if (sr < 0.85f) { sr = 0.85f; }
        if (sr > 1.15f) { sr = 1.15f; }
        if (sl < 0.85f) { sl = 0.85f; }
        if (sl > 1.15f) { sl = 1.15f; }

        motor_set_wheel_scale(sr, sl);
        printf("TRIM: scales R=%.3f L=%.3f\n", sr, sl);
    } else {
        motor_set_wheel_scale(1.0f, 1.0f);
        printf("TRIM: skipped (low cps)\n");
    }

    g_override_motion = false;
}

// ======= Boot-time auto calibration sequence =======
static void do_boot_auto_cal(void) {
    printf("BOOT: waiting 5s before auto calibration...\n");
    sleep_ms(5000);

    // 3 s spin to calibrate mag + seed heading
    do_mag_calibration();

    // 1.5 s wheel auto-scale
    do_auto_wheel_scale();

    // stop motors and settle
    motion_command(MOVE_STOP, 0);
    printf("BOOT: auto calibration complete. Press START when ready.\n");
}

int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("\n=== Car Demo 1: Cascaded Heading (Enc diff + IMU trim + Cal + Supervisor) ===\n");

    buttons_init();

    if (!imu_init()) printf("IMU init failed! Check I2C wiring (GP2 SDA, GP3 SCL) and power.\n");
    else             printf("IMU ready.\n");

    motor_init();

    // Start timers first so smoothing/telemetry run during routines
    add_repeating_timer_ms(-CONTROL_PERIOD_MS, motor_control_timer_cb, NULL, &control_timer_motor);
    add_repeating_timer_ms(-CONTROL_PERIOD_MS, control_cb, (void*)&running, &control_timer_imu);

    // ---- BOOT auto calibration sequence ----
    do_boot_auto_cal();

    while (true) {
        if (!running) {
            if (btn_pressed(BTN_START)) {
                sleep_ms(30);
                if (btn_pressed(BTN_START)) {
                    printf("START: settle 4s, capture heading_ref, then go.\n");

                    // Keep completely still for 4s, take fresh filtered heading
                    g_override_motion = true;
                    motion_command(MOVE_STOP, 0);
                    absolute_time_t t0 = get_absolute_time();
                    imu_state_t s;
                    while (absolute_time_diff_us(t0, get_absolute_time()) < 4000000) {
                        if (imu_read(&s) && s.ok) { /* keep filter updated */ }
                        tight_loop_contents();
                    }

                    // Seed ref from current filtered heading; reset the filter so "moving IMU heading" matches start
                    if (imu_read(&s) && s.ok) {
                        initial_heading_deg = s.heading_deg_filt;
                        imu_reset_heading_filter(initial_heading_deg);
                    } else {
                        initial_heading_deg = 0.0f;
                    }

                    // Reset PIDs, supervisor and start motion
                    pid_reset(&pid_track);
                    pid_reset(&pid_heading);
                    HS.initialized = false;
                    HS.head_weight = 0.0f;

                    g_override_motion = false;
                    motion_command(MOVE_FORWARD, run_speed_percent);
                    running = true;
                    printf("START -> %d%%, heading_ref=%.1f deg\n", run_speed_percent, initial_heading_deg);
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
