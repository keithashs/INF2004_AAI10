// car_demo1/main/main.c
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include <stdio.h>
#include <math.h>

#include "config.h"
#include "motor/motor.h"
#include "imu/imu.h"
#include "pid/pid.h"

/* ===== WiFi + MQTT (lwIP background arch) ===== */
#include "mqtt_client.h"

/*
 * Provide safe defaults when the build system does not pass these
 * as compile definitions. The dashboard example passes them for the
 * background `wifi_mqtt` target, but the `car_demo1` app may be
 * built separately and therefore must either receive them via
 * target_compile_definitions or use these fallbacks.
 */
#ifndef WIFI_SSID
#define WIFI_SSID "UNKNOWN_SSID"
#warning "WIFI_SSID not defined; using default 'UNKNOWN_SSID'"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#warning "WIFI_PASSWORD not defined; using empty password"
#endif

#ifndef MQTT_BROKER_IP
#define MQTT_BROKER_IP "127.0.0.1"
#warning "MQTT_BROKER_IP not defined; using 127.0.0.1"
#endif

#ifndef MQTT_BROKER_PORT
#define MQTT_BROKER_PORT 1883
#warning "MQTT_BROKER_PORT not defined; using 1883"
#endif

// Timers
static repeating_timer_t control_timer_motor;   // 10 ms: motor PID & telemetry (in motor.c)
static repeating_timer_t control_timer_imu;     // 10 ms: encoder-balance + IMU trim (this file)
static repeating_timer_t telemetry_timer;       // 200 ms: MQTT publish + poll

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
volatile float       g_head_weight     = 0.0f;

// ---- Heading supervisor (gate IMU influence) ----
typedef struct {
    float last_heading_deg;
    absolute_time_t last_t;
    float head_weight;          // 0..1 multiplier on IMU bias
    bool  initialized;
} head_sup_t;

static head_sup_t HS = {0};

// Soft-start bookkeeping
static absolute_time_t run_t0;
// Slightly shorter soft-start so we overcome stiction a bit quicker
static const float SOFTSTART_SEC = 0.6f;

// ===== Adaptive scale state =====
static float diff_lp = 0.0f;         // low-pass of encoder cps difference
static float adapt_accum = 0.0f;     // period accumulator for scale updates

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

extern PID pid_track;
extern PID pid_heading;
// ----------------- Fast control @ 100 Hz -----------------
// Does BOTH: (A) wheel-balance PID from encoders, (B) IMU trim gated by health,
// and (C) slow adaptive per-wheel scaling to cancel residual bias.
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

        // slightly larger deadband
        if (fabsf(err_deg) < 2.0f) err_deg = 0.0f;

        // --- stricter supervisor thresholds ---
        bool tilt_ok = (fabsf(s.roll_deg) <= 10.0f) && (fabsf(s.pitch_deg) <= 10.0f);

        // 2) heading rate limit (deg/s)
        float rate_ok = true;
        if (!HS.initialized) {
            HS.last_heading_deg = s.heading_deg_filt;
            HS.last_t = get_absolute_time();
            HS.head_weight = 0.0f;
            HS.initialized = true;
        } else {
            absolute_time_t now = get_absolute_time();
            float dt_s = absolute_time_diff_us(HS.last_t, now) / 1e6f;
            if (dt_s > 0.0005f) {
                float d = wrap180(s.heading_deg_filt - HS.last_heading_deg);
                float rate = fabsf(d) / dt_s; // deg/s
                rate_ok = (rate <= 30.0f);
                HS.last_heading_deg = s.heading_deg_filt;
                HS.last_t = now;
            }
        }

        bool healthy = tilt_ok && rate_ok;

        // Smoothly move head_weight toward 1 when healthy, toward 0 when not
        float tau_up = 0.5f;   // slower ramp-in
        float tau_dn = 0.10f;  // faster drop when unhealthy
        float a_up = clampf(dt / tau_up, 0.0f, 1.0f);
        float a_dn = clampf(dt / tau_dn, 0.0f, 1.0f);

        if (healthy) head_weight = head_weight + (1.0f - head_weight) * a_up;
        else         head_weight = head_weight + (0.0f - head_weight) * a_dn;

        head_weight = clampf(head_weight, 0.0f, 1.0f);
        HS.head_weight = head_weight;
        g_head_weight  = head_weight;

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
        g_head_weight  = HS.head_weight;
        g_imu_ok = false;
    }

    // Soft-start speed ramp over first ~0.8 s
    float base_pct = (float)run_speed_percent;
    float pct_eff = base_pct;
    if (SOFTSTART_SEC > 0.0f) {
        float tsoft = absolute_time_diff_us(run_t0, get_absolute_time()) / 1e6f;
        if (tsoft < SOFTSTART_SEC) pct_eff = base_pct * (tsoft / SOFTSTART_SEC);
    }

    float base_cps = (MAX_CPS * pct_eff) / 100.0f;

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

    // (C) ADAPTIVE WHEEL SCALE (very slow)
    // Low-pass the encoder difference (favor long-term bias, not noise).
    diff_lp = 0.98f * diff_lp + 0.02f * diff_meas;
    adapt_accum += dt;

    if (adapt_accum >= ADAPT_PERIOD_S && base_cps > 5.0f) {
        float sr, sl;
        motor_get_wheel_scale(&sr, &sl);

        // If right is persistently faster (diff_lp > 0), reduce right scale a bit.
        float rel = diff_lp / (base_cps + 1e-6f);             // dimensionless, ~[-1..+1]
        float dscale = clampf(-ADAPT_GAIN * rel, -0.01f, 0.01f); // tiny step each update
        sr += dscale;
        sl -= dscale;

        // Keep within safety bounds
        sr = clampf(sr, SCALE_MIN, SCALE_MAX);
        sl = clampf(sl, SCALE_MIN, SCALE_MAX);
        motor_set_wheel_scale(sr, sl);

        // (Suppressed noisy background prints during run per request)
        printf("TRIM: adapt scales R=%.3f L=%.3f (diff_lp=%.2f cps)\n", sr, sl, diff_lp);

        adapt_accum = 0.0f;
    }

    // Command motion with final bias
    motion_command_with_bias(MOVE_FORWARD, (int)pct_eff, +total_bias, -total_bias);

    // Telemetry cache
    g_bias_cps = total_bias;

    return true;
}

/* ================= MQTT topics ================= */
static const char *TOP_TELEM = "robot/telemetry";  // JSON status payloads
static const char *TOP_EVENT = "robot/event";      // text events (START/STOP/etc)

/* ================= MQTT publishing ================= */
static void telemetry_publish_once(void) {
    if (!mqtt_is_ready()) return;

    float cps_r, cps_l; get_cps_smoothed(&cps_r, &cps_l);
    float dR_m, dL_m;   get_distance_m(&dR_m, &dL_m);
    int dirR = get_dir_m1(), dirL = get_dir_m2();

    // Convert cps to cm/s using wheel circumference
    const float circ_m = PI_F * WHEEL_DIAMETER_M;
    float m1_cmps = (cps_r / TICKS_PER_REV) * circ_m * 100.0f;
    float m2_cmps = (cps_l / TICKS_PER_REV) * circ_m * 100.0f;

    float sr, sl; motor_get_wheel_scale(&sr, &sl);

    uint32_t ts = to_ms_since_boot(get_absolute_time());
    mqtt_publish_printf(TOP_TELEM, 0, false,
        "{"
          "\"ts\":%u,"
          "\"m1_cmps\":%.2f,\"m2_cmps\":%.2f,"
          "\"dirR\":%d,\"dirL\":%d,"
          "\"dist_R_cm\":%.1f,\"dist_L_cm\":%.1f,"
          "\"heading\":%.1f,\"heading_f\":%.1f,"
          "\"imu_ok\":%d,"
          "\"bias_cps\":%.2f,"
          "\"w\":%.2f,"
          "\"scale_R\":%.3f,\"scale_L\":%.3f"
        "}",
        (unsigned)ts,
        m1_cmps, m2_cmps,
        dirR, dirL,
        dR_m*100.0f, dL_m*100.0f,
        g_imu_last.heading_deg, g_imu_last.heading_deg_filt,
        g_imu_ok ? 1 : 0,
        g_bias_cps,
        g_head_weight,
        sr, sl
    );
}

/* 200 ms periodic: keep MQTT alive + publish a snapshot */
static bool telemetry_cb(repeating_timer_t* t) {
    (void)t;
    mqtt_poll();                 // drive reconnect pacing
    telemetry_publish_once();   // publish if connected
    return true;
}

// ======= Calibration / Trim helpers =======

// Spin in place for ~3 s to collect mag min/max
static void do_mag_calibration(void) {
    // Print right when we enter this state (as requested)
    printf("CAL: magnetometer min/max... spinning 3s\n");
    telemetry_set_mode(TMODE_CAL);

    g_override_motion = true;
    imu_cal_begin();

    absolute_time_t t0 = get_absolute_time();

    // spin left
    motion_command(MOVE_LEFT, 25);
    while (absolute_time_diff_us(t0, get_absolute_time()) < 3000000) {
        imu_state_t s;
        if (imu_read(&s) && s.ok) { /* keep filter updated while spinning */ }
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
    // Stop CAL prints after this stage
    telemetry_set_mode(TMODE_NONE);
}

// Drive straight gently for ~4.0 s to measure cps ratio and set scale
static void do_auto_wheel_scale(void) {
    printf("TRIM: auto wheel scale... driving 4.0s\n"); 
    telemetry_set_mode(TMODE_CAL);

    g_override_motion = true;

    // brief settle
    motion_command(MOVE_FORWARD, 20); // same base cps for both sides
    sleep_ms(500);

    // measure window (4.0 s)
    float sum_r = 0, sum_l = 0; int n = 0;
    absolute_time_t t0 = get_absolute_time();
    while (absolute_time_diff_us(t0, get_absolute_time()) < 4000000) {  // (4,000,000 us) = 4s
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
        if (sr < SCALE_MIN) { sr = SCALE_MIN; }
        if (sr > SCALE_MAX) { sr = SCALE_MAX; }
        if (sl < SCALE_MIN) { sl = SCALE_MIN; }
        if (sl > SCALE_MAX) { sl = SCALE_MAX; }

        motor_set_wheel_scale(sr, sl);
        printf("TRIM: scales R=%.3f L=%.3f\n", sr, sl);
        
    } else {
        motor_set_wheel_scale(1.0f, 1.0f);
        printf("TRIM: skipped (low cps)\n");
    }

    g_override_motion = false;
    telemetry_set_mode(TMODE_NONE);
}

// ======= Boot-time auto calibration sequence =======
static void do_boot_auto_cal(void) {
    // 3 s spin to calibrate mag + seed heading
    do_mag_calibration();

    // 4.0 s wheel auto-scale
    do_auto_wheel_scale();

    // stop motors and settle (silent)
    motion_command(MOVE_STOP, 0);
    printf("BOOT: auto calibration complete. Press START when ready.\n");
}

/* ===================== App task (moves your old main() here) ===================== */

/* ===================== MAIN ===================== */
int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    printf("\n=== Car Demo 1: Cascaded Heading + MQTT Telemetry ===\n");

    buttons_init();

    if (!imu_init()) {
        printf("IMU init failed! Check I2C wiring (GP2 SDA, GP3 SCL) and power.\n");
    } else {
        printf("IMU ready.\n");
    }

    motor_init();
    pid_init_defaults();
    encoder_init();

    /* Timers: 100 Hz control (motor.c + this file) */
    add_repeating_timer_ms(-CONTROL_PERIOD_MS, motor_control_timer_cb, NULL, &control_timer_motor);
    add_repeating_timer_ms(-CONTROL_PERIOD_MS, control_cb, (void*)&running, &control_timer_imu);

    /* Boot auto-calibration (mag + wheel scale) */
    printf("BOOT: waiting 3s before auto calibration...\n");
    sleep_ms(3000);
    do_boot_auto_cal();

    /* ---- Wi-Fi + MQTT bring-up (non-blocking) ---- */
    /* Log which connection parameters are being used (do not print password) */
    printf("[wifi] starting + MQTT... SSID='%s' broker=%s:%d\n", WIFI_SSID, MQTT_BROKER_IP, MQTT_BROKER_PORT);
    mqtt_start(WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER_IP, MQTT_BROKER_PORT);

    /* 5 Hz telemetry & background MQTT polling */
    add_repeating_timer_ms(-200, telemetry_cb, NULL, &telemetry_timer);

    /* ---- Button loop ---- */
    for (;;) {
        if (!running) {
            if (btn_pressed(BTN_START)) {
                sleep_ms(100);
                if (btn_pressed(BTN_START)) {
                    print_telemetry_legend();

                    float sr, sl; motor_get_wheel_scale(&sr, &sl);
                    printf("START: settle 4s, capture heading_ref, then soft-start. Scale[R=%.3f L=%.3f]\n", sr, sl);

                    if (mqtt_is_ready()) {
                        mqtt_publish_printf(TOP_EVENT, 0, false, "START: settle-and-capture");
                    }

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

                    pid_reset(&pid_track);
                    pid_reset(&pid_heading);
                    motor_reset_controllers();
                    motor_reset_speed_filters();
                    motor_reset_distance_counters();

                    HS.initialized = false;
                    HS.head_weight = 0.0f;
                    g_head_weight  = 0.0f;

                    diff_lp = 0.0f;
                    adapt_accum = 0.0f;

                    g_override_motion = false;
                    run_t0 = get_absolute_time();
                    motion_command(MOVE_FORWARD, 0);
                    running = true;

                    printf("START -> %d%%, heading_ref=%.1f deg\n", run_speed_percent, initial_heading_deg);
                    telemetry_set_mode(TMODE_RUN);
                    if (mqtt_is_ready()) {
                        mqtt_publish_printf(TOP_EVENT, 0, false, "START -> %d%% (heading_ref=%.1f)",
                                      run_speed_percent, initial_heading_deg);
                    }

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
                    if (mqtt_is_ready()) mqtt_publish_printf(TOP_EVENT, 0, false, "STOP");
                    while (btn_pressed(BTN_STOP)) tight_loop_contents();
                }
            }
        }

        // keep MQTT alive even if no timers fire (belt & suspenders)
        mqtt_poll();
        tight_loop_contents();
    }
}