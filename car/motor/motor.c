#include "motor.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include <stdio.h>
#include <math.h>
#include "pid/pid.h"
#include "config.h"
#include "encoder/encoder.h"
#include "imu/imu.h"            // for telemetry cache + g_head_weight

// --- Targets & state ---
static volatile float tgt_cps_m1 = 0.0f, tgt_cps_m2 = 0.0f;
static volatile int dir_m1 = 0, dir_m2 = 0;
static volatile float duty_m1 = 0.0f, duty_m2 = 0.0f;
static volatile uint64_t last_telemetry_ms = 0;

// Odometry/reporting correction (does NOT affect control or MAX_CPS)
static const float ODOM_CORR = 1.067f;   // ≈ 150 / (55.8 * (0.063/0.025))

// -------- CPS smoothing: 100 ms moving average --------
#define CPS_AVG_TAPS 10
static uint16_t win_hist_m1[CPS_AVG_TAPS] = {0};
static uint16_t win_hist_m2[CPS_AVG_TAPS] = {0};
static uint32_t win_sum_m1 = 0, win_sum_m2 = 0;
static int      win_idx = 0;

// Extra low-pass after the 100ms average
static float cps_m1_f = 0.0f, cps_m2_f = 0.0f;
static const float CPS_ALPHA = 0.70f;

// PID controllers (duty output in %)
static PID pid_m1 = { .kp=0.25f, .ki=2.0f, .kd=0.002f, .integ=0, .prev_err=0, .out_min=0.0f, .out_max=100.0f };
static PID pid_m2 = { .kp=0.25f, .ki=2.0f, .kd=0.002f, .integ=0, .prev_err=0, .out_min=0.0f, .out_max=100.0f };

// per-wheel scale (compensate steady bias)
static volatile float scale_right = 1.0f;
static volatile float scale_left  = 1.0f;

// Telemetry mode
static telemetry_mode_t tmode = TMODE_NONE;

// ------------- Helpers -------------
static inline uint16_t duty_from_percent(float pct) {
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (uint16_t)((pct * PWM_WRAP) / 100.0f);
}

static void setup_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_config cfg = pwm_get_default_config();
    float clkdiv = (float)SYS_CLK_HZ / ((float)PWM_FREQ_HZ * (float)(PWM_WRAP + 1));
    pwm_config_set_clkdiv(&cfg, clkdiv);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(slice, &cfg, true);
    uint channel = pwm_gpio_to_channel(gpio);
    pwm_set_chan_level(slice, channel, 0);
}

void motor_set_signed(int in1, int in2, int dir, float duty_percent) {
    uint slice1 = pwm_gpio_to_slice_num(in1);
    uint chan1  = pwm_gpio_to_channel(in1);
    uint slice2 = pwm_gpio_to_slice_num(in2);
    uint chan2  = pwm_gpio_to_channel(in2);
    uint16_t d = duty_from_percent(duty_percent);

    if (dir > 0) { pwm_set_chan_level(slice1, chan1, d); pwm_set_chan_level(slice2, chan2, 0); }
    else if (dir < 0) { pwm_set_chan_level(slice1, chan1, 0); pwm_set_chan_level(slice2, chan2, d); }
    else { pwm_set_chan_level(slice1, chan1, 0); pwm_set_chan_level(slice2, chan2, 0); }
}

void motor_all_stop(void) {
    motor_set_signed(M1_IN1, M1_IN2, 0, 0);
    motor_set_signed(M2_IN1, M2_IN2, 0, 0);
}

static inline float pct_to_cps(int percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return (MAX_CPS * (float)percent) / 100.0f;
}

void motor_set_wheel_scale(float sr, float sl) { scale_right = sr; scale_left = sl; }
void motor_get_wheel_scale(float* sr, float* sl) { if (sr) *sr = scale_right; if (sl) *sl = scale_left; }

void motion_command(move_t move, int speed_percent) {
    motion_command_with_bias(move, speed_percent, 0.0f, 0.0f);
}

void motion_command_with_bias(move_t move, int speed_percent, float left_bias_cps, float right_bias_cps) {
    float cps = pct_to_cps(speed_percent);
    switch (move) {
        case MOVE_FORWARD:
            dir_m1 = +1; dir_m2 = +1;
            // Apply per-wheel scaling to cancel systematic drift
            tgt_cps_m1 = (cps + right_bias_cps) * scale_right; // right motor
            tgt_cps_m2 = (cps + left_bias_cps)  * scale_left;  // left motor
            break;
        case MOVE_BACKWARD:
            dir_m1 = -1; dir_m2 = -1;
            tgt_cps_m1 = (cps + right_bias_cps) * scale_right;
            tgt_cps_m2 = (cps + left_bias_cps)  * scale_left;
            break;
        case MOVE_LEFT:
            dir_m1 = +1; dir_m2 = -1; tgt_cps_m1 = cps; tgt_cps_m2 = cps; break;
        case MOVE_RIGHT:
            dir_m1 = -1; dir_m2 = +1; tgt_cps_m1 = cps; tgt_cps_m2 = cps; break;
        case MOVE_STOP:
        default:
            dir_m1 = 0;  dir_m2 = 0;  tgt_cps_m1 = 0; tgt_cps_m2 = 0; break;
    }
}

void motor_init(void) {
    setup_pwm(M1_IN1); setup_pwm(M1_IN2);
    setup_pwm(M2_IN1); setup_pwm(M2_IN2);

    // init encoders
    encoder_init();

    pid_reset(&pid_m1); pid_reset(&pid_m2);

    for (int i = 0; i < CPS_AVG_TAPS; ++i) { win_hist_m1[i] = 0; win_hist_m2[i] = 0; }
    win_sum_m1 = win_sum_m2 = 0; win_idx = 0;
    cps_m1_f = cps_m2_f = 0.0f;
    scale_right = 1.0f; scale_left = 1.0f;

    // "prime" the window sampler so first delta is zero
    uint16_t dump1, dump2;
    encoder_sample_window(&dump1, &dump2);
}

// Called on START — reset only smoothing, not distance totals
void motor_reset_speed_filters(void) {
    for (int i = 0; i < CPS_AVG_TAPS; ++i) { win_hist_m1[i] = 0; win_hist_m2[i] = 0; }
    win_sum_m1 = win_sum_m2 = 0; win_idx = 0;
    cps_m1_f = cps_m2_f = 0.0f;

    // Reset the encoder window baseline (do NOT zero totals here)
    uint16_t dump1, dump2;
    encoder_sample_window(&dump1, &dump2);
}

void motor_reset_controllers(void) {
    pid_reset(&pid_m1); pid_reset(&pid_m2);
}

// Reset odometry distance (totals) to zero
void motor_reset_distance_counters(void) {
    encoder_reset_all();          // zeros both totals and window baseline
    // also clear smoothing so speed readouts start clean
    motor_reset_speed_filters();
}

// Telemetry mode setter
void telemetry_set_mode(telemetry_mode_t mode) {
    tmode = mode;
}

bool motor_control_timer_cb(repeating_timer_t *t) {
    const float dt = CONTROL_PERIOD_MS / 1000.0f;

    // Get encoder ticks since last window sample
    uint16_t w1 = 0, w2 = 0;
    encoder_sample_window(&w1, &w2);

    // 100 ms moving average of windows
    win_sum_m1 -= win_hist_m1[win_idx];
    win_sum_m2 -= win_hist_m2[win_idx];
    win_hist_m1[win_idx] = w1;
    win_hist_m2[win_idx] = w2;
    win_sum_m1 += win_hist_m1[win_idx];
    win_sum_m2 += win_hist_m2[win_idx];
    win_idx = (win_idx + 1) % CPS_AVG_TAPS;

    float cps_m1_raw = (float)win_sum_m1 / (CPS_AVG_TAPS * dt);
    float cps_m2_raw = (float)win_sum_m2 / (CPS_AVG_TAPS * dt);

    // Extra IIR smoothing
    cps_m1_f = CPS_ALPHA * cps_m1_f + (1.0f - CPS_ALPHA) * cps_m1_raw;
    cps_m2_f = CPS_ALPHA * cps_m2_f + (1.0f - CPS_ALPHA) * cps_m2_raw;

    float out1 = pid_step(&pid_m1, tgt_cps_m1, cps_m1_f, dt);
    float out2 = pid_step(&pid_m2, tgt_cps_m2, cps_m2_f, dt);
    duty_m1 = out1; duty_m2 = out2;

    motor_set_signed(M1_IN1, M1_IN2, dir_m1, duty_m1);
    motor_set_signed(M2_IN1, M2_IN2, dir_m2, duty_m2);

    // ---- Telemetry (5 Hz) ----
    uint64_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms - last_telemetry_ms >= TELEMETRY_MS) {
        last_telemetry_ms = now_ms;

        // Distances (meters) from totals
        uint32_t tot_m1 = encoder_total_m1();
        uint32_t tot_m2 = encoder_total_m2();
        float revs_m1 = (float)tot_m1 / TICKS_PER_REV;
        float revs_m2 = (float)tot_m2 / TICKS_PER_REV;
        float circ_m  = PI_F * WHEEL_DIAMETER_M;
        float dist_m1 = (revs_m1 * circ_m) * ODOM_CORR;  // right motor distance
        float dist_m2 = (revs_m2 * circ_m) * ODOM_CORR;  // left motor distance

        // Convert CPS to speed (cm/s). speed = cps/TICKS_PER_REV * circumference(m) * 100
        float m1_cmps      = (cps_m1_raw / TICKS_PER_REV) * circ_m * 100.0f * ODOM_CORR;
        float m2_cmps      = (cps_m2_raw / TICKS_PER_REV) * circ_m * 100.0f * ODOM_CORR;
        float tgt_m1_cmps  = (tgt_cps_m1  / TICKS_PER_REV) * circ_m * 100.0f * ODOM_CORR;
        float tgt_m2_cmps  = (tgt_cps_m2  / TICKS_PER_REV) * circ_m * 100.0f * ODOM_CORR;

        imu_state_t s = g_imu_last;
        float err  = g_heading_err_deg;
        float bias = g_bias_cps;
        bool  iok  = g_imu_ok;

        if (tmode == TMODE_RUN) {
            // === RUN prints (your requested format, no Scale[] here) ===
            printf("STAT "
                   "M1[speed=%6.2fcm/s tgt=%6.2fcm/s duty=%6.1f%% dir=%2d]  "
                   "M2[speed=%6.2fcm/s tgt=%6.2fcm/s duty=%6.1f%% dir=%2d]  "
                   "Dist[L=%7.1fcm R=%7.1fcm]   "
                   "IMU[err=%6.1f roll=%6.1f pitch=%6.1f head=%6.1f filt=%6.1f bias=%6.1f w=%4.2f%s]\n",
                   m1_cmps, tgt_m1_cmps, duty_m1, dir_m1,
                   m2_cmps, tgt_m2_cmps, duty_m2, dir_m2,
                   dist_m2 * 100.0f, dist_m1 * 100.0f,
                   iok ? err : 0.0f,
                   iok ? s.roll_deg  : 0.0f,
                   iok ? s.pitch_deg : 0.0f,
                   iok ? s.heading_deg      : 0.0f,
                   iok ? s.heading_deg_filt : 0.0f,
                   iok ? bias : 0.0f,
                   (double)g_head_weight,
                   iok ? "" : " IMU=NA");
        } else if (tmode == TMODE_CAL) {
            // === CAL prints: no Dist[], IMU only if available ===
            if (iok) {
                printf("CAL "
                       "M1[speed=%6.2fcm/s tgt=%6.2fcm/s duty=%6.1f%% dir=%2d]  "
                       "M2[speed=%6.2fcm/s tgt=%6.2fcm/s duty=%6.1f%% dir=%2d]  "
                       "IMU[roll=%6.1f pitch=%6.1f head=%6.1f filt=%6.1f err=%6.1f bias=%6.1f w=%4.2f]\n",
                       m1_cmps, tgt_m1_cmps, duty_m1, dir_m1,
                       m2_cmps, tgt_m2_cmps, duty_m2, dir_m2,
                       s.roll_deg, s.pitch_deg, s.heading_deg, s.heading_deg_filt, err, bias, (double)g_head_weight);
            } else {
                printf("CAL "
                       "M1[speed=%6.2fcm/s tgt=%6.2fcm/s duty=%6.1f%% dir=%2d]  "
                       "M2[speed=%6.2fcm/s tgt=%6.2fcm/s duty=%6.1f%% dir=%2d]\n",
                       m1_cmps, tgt_m1_cmps, duty_m1, dir_m1,
                       m2_cmps, tgt_m2_cmps, duty_m2, dir_m2);
            }
        } else {
            // TMODE_NONE -> no telemetry
        }
    }
    return true;
}

// ----- Helpers -----
void get_cps(float* out_cps_m1, float* out_cps_m2) {
    // Note: this consumes a fresh window; callers should be aware of timing.
    const float dt = CONTROL_PERIOD_MS / 1000.0f;
    uint16_t w1 = 0, w2 = 0;
    encoder_sample_window(&w1, &w2);
    if (out_cps_m1) *out_cps_m1 = (float)w1 / dt;
    if (out_cps_m2) *out_cps_m2 = (float)w2 / dt;
}

void get_cps_smoothed(float* out_m1, float* out_m2) {
    if (out_m1) *out_m1 = cps_m1_f;
    if (out_m2) *out_m2 = cps_m2_f;
}

void get_distance_m(float* d_m1, float* d_m2) {
    float circ_m  = PI_F * WHEEL_DIAMETER_M;
    uint32_t tot_m1 = encoder_total_m1();
    uint32_t tot_m2 = encoder_total_m2();
    if (d_m1) *d_m1 = ((float)tot_m1 / TICKS_PER_REV) * circ_m;
    if (d_m2) *d_m2 = ((float)tot_m2 / TICKS_PER_REV) * circ_m;
}

int get_dir_m1(void) { return dir_m1; }
int get_dir_m2(void) { return dir_m2; }

void print_telemetry_legend(void) {
    printf("\nLEGEND:\n");
    printf("STAT M1[speed cm/s  tgt cm/s  duty %%  dir]  "
           "M2[speed cm/s  tgt cm/s  duty %%  dir]  "
           "Dist[L cm R cm]  "
           "IMU[err deg roll deg pitch deg head deg filt deg bias cps w]\n");
    printf("Notes: bias>0 speeds LEFT up and RIGHT down; w is IMU trust 0..1.\n\n");
}

// Helper FUNCTIONS FOR DEMO2 SUPPORT

// Get wheel speeds in cm/s (converted from smoothed CPS)
void get_speed_cmps(float* left_cmps, float* right_cmps) {
    float circ_m = PI_F * WHEEL_DIAMETER_M;
    float cps_r, cps_l;
    get_cps_smoothed(&cps_r, &cps_l);
    
    // Convert: cps / TICKS_PER_REV * circumference(m) * 100 * ODOM_CORR
    if (right_cmps) *right_cmps = (cps_r / TICKS_PER_REV) * circ_m * 100.0f * ODOM_CORR;
    if (left_cmps)  *left_cmps  = (cps_l / TICKS_PER_REV) * circ_m * 100.0f * ODOM_CORR;
}

// Get distances in cm (converted from meters)
void get_distance_cm(float* left_cm, float* right_cm) {
    float d_r_m, d_l_m;
    get_distance_m(&d_r_m, &d_l_m);
    
    if (right_cm) *right_cm = d_r_m * 100.0f * ODOM_CORR;
    if (left_cm)  *left_cm  = d_l_m * 100.0f * ODOM_CORR;
}

// Average speed in cm/s
float get_average_speed_cmps(void) {
    float left, right;
    get_speed_cmps(&left, &right);
    return (left + right) / 2.0f;
}

// Average distance in cm
float get_average_distance_cm(void) {
    float left, right;
    get_distance_cm(&left, &right);
    return (left + right) / 2.0f;
}