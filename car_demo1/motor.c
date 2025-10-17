#include "motor.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include <stdio.h>
#include <math.h>
#include "pid.h"
#include "config.h"
#include "imu.h"   // for telemetry cache (g_imu_last, etc.)

static volatile uint32_t enc_count_m1 = 0, enc_count_m2 = 0;
static volatile uint32_t enc_win_m1 = 0,  enc_win_m2 = 0;

static volatile float tgt_cps_m1 = 0.0f, tgt_cps_m2 = 0.0f;
static volatile int dir_m1 = 0, dir_m2 = 0;
static volatile float duty_m1 = 0.0f, duty_m2 = 0.0f;
static volatile uint64_t last_telemetry_ms = 0;

// -------- CPS smoothing: 100 ms moving average of 10 x 10ms windows --------
#define CPS_AVG_TAPS 10
static uint16_t win_hist_m1[CPS_AVG_TAPS] = {0};
static uint16_t win_hist_m2[CPS_AVG_TAPS] = {0};
static uint32_t win_sum_m1 = 0, win_sum_m2 = 0;
static int      win_idx = 0;

// Extra low-pass after the 100ms average (reduces residual jitter)
static float cps_m1_f = 0.0f, cps_m2_f = 0.0f;
static const float CPS_ALPHA = 0.80f;   // stronger smoothing

// Per-wheel velocity PIDs (tuned for smooth duty output)
static PID pid_m1 = { .kp=0.25f, .ki=2.0f, .kd=0.002f, .integ=0, .prev_err=0, .out_min=0.0f, .out_max=100.0f };
static PID pid_m2 = { .kp=0.25f, .ki=2.0f, .kd=0.002f, .integ=0, .prev_err=0, .out_min=0.0f, .out_max=100.0f };

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

void motion_command(move_t move, int speed_percent) {
    motion_command_with_bias(move, speed_percent, 0.0f, 0.0f);
}

void motion_command_with_bias(move_t move, int speed_percent, float left_bias_cps, float right_bias_cps) {
    float cps = pct_to_cps(speed_percent);
    switch (move) {
        case MOVE_FORWARD:  dir_m1 = +1; dir_m2 = +1; tgt_cps_m1 = cps + right_bias_cps; tgt_cps_m2 = cps + left_bias_cps; break;
        case MOVE_BACKWARD: dir_m1 = -1; dir_m2 = -1; tgt_cps_m1 = cps + right_bias_cps; tgt_cps_m2 = cps + left_bias_cps; break;
        case MOVE_LEFT:     dir_m1 = +1; dir_m2 = -1; tgt_cps_m1 = cps; tgt_cps_m2 = cps; break;
        case MOVE_RIGHT:    dir_m1 = -1; dir_m2 = +1; tgt_cps_m1 = cps; tgt_cps_m2 = cps; break;
        case MOVE_STOP:
        default:            dir_m1 = 0;  dir_m2 = 0;  tgt_cps_m1 = 0; tgt_cps_m2 = 0; break;
    }
}

static void encoder_isr_cb(uint gpio, uint32_t events) {
    if (gpio == ENCODER_PIN_M1 && (events & GPIO_IRQ_EDGE_RISE)) { enc_count_m1++; enc_win_m1++; }
    else if (gpio == ENCODER_PIN_M2 && (events & GPIO_IRQ_EDGE_RISE)) { enc_count_m2++; enc_win_m2++; }
}

void encoder_init(void) {
    gpio_init(ENCODER_PIN_M1); gpio_set_dir(ENCODER_PIN_M1, GPIO_IN); gpio_pull_down(ENCODER_PIN_M1);
    gpio_init(ENCODER_PIN_M2); gpio_set_dir(ENCODER_PIN_M2, GPIO_IN); gpio_pull_down(ENCODER_PIN_M2);
    gpio_set_irq_enabled_with_callback(ENCODER_PIN_M1, GPIO_IRQ_EDGE_RISE, true, &encoder_isr_cb);
    gpio_set_irq_enabled(ENCODER_PIN_M2, GPIO_IRQ_EDGE_RISE, true);
}

void motor_init(void) {
    setup_pwm(M1_IN1); setup_pwm(M1_IN2);
    setup_pwm(M2_IN1); setup_pwm(M2_IN2);
    encoder_init();
    pid_reset(&pid_m1); pid_reset(&pid_m2);

    for (int i = 0; i < CPS_AVG_TAPS; ++i) { win_hist_m1[i] = 0; win_hist_m2[i] = 0; }
    win_sum_m1 = win_sum_m2 = 0; win_idx = 0;
}

bool motor_control_timer_cb(repeating_timer_t *t) {
    const float dt = CONTROL_PERIOD_MS / 1000.0f; // 0.01 s

    // Take 10ms window counts and reset
    uint32_t w1 = enc_win_m1, w2 = enc_win_m2;
    enc_win_m1 = enc_win_m2 = 0;

    // 100 ms moving average of windows
    win_sum_m1 -= win_hist_m1[win_idx];
    win_sum_m2 -= win_hist_m2[win_idx];
    win_hist_m1[win_idx] = (uint16_t)w1;
    win_hist_m2[win_idx] = (uint16_t)w2;
    win_sum_m1 += win_hist_m1[win_idx];
    win_sum_m2 += win_hist_m2[win_idx];
    win_idx = (win_idx + 1) % CPS_AVG_TAPS;

    // Average cps over 100 ms (10 * 10ms)
    float cps_m1_raw = (float)win_sum_m1 / (CPS_AVG_TAPS * dt);
    float cps_m2_raw = (float)win_sum_m2 / (CPS_AVG_TAPS * dt);

    // Extra IIR smoothing
    cps_m1_f = CPS_ALPHA * cps_m1_f + (1.0f - CPS_ALPHA) * cps_m1_raw;
    cps_m2_f = CPS_ALPHA * cps_m2_f + (1.0f - CPS_ALPHA) * cps_m2_raw;

    // Velocity PIDs on smoothed cps
    float out1 = pid_step(&pid_m1, tgt_cps_m1, cps_m1_f, dt);
    float out2 = pid_step(&pid_m2, tgt_cps_m2, cps_m2_f, dt);
    duty_m1 = out1; duty_m2 = out2;

    motor_set_signed(M1_IN1, M1_IN2, dir_m1, duty_m1);
    motor_set_signed(M2_IN1, M2_IN2, dir_m2, duty_m2);

    // ---- Telemetry (5 Hz) ----
    uint64_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms - last_telemetry_ms >= TELEMETRY_MS) {
        last_telemetry_ms = now_ms;

        float revs_m1 = (float)enc_count_m1 / TICKS_PER_REV;
        float revs_m2 = (float)enc_count_m2 / TICKS_PER_REV;
        float circ_m  = PI_F * WHEEL_DIAMETER_M;
        float dist_m1 = revs_m1 * circ_m;  // right motor distance
        float dist_m2 = revs_m2 * circ_m;  // left motor distance

        imu_state_t s = g_imu_last;
        float err  = g_heading_err_deg;
        float bias = g_bias_cps;
        bool  iok  = g_imu_ok;

        // One aligned line
        printf("STAT "
               "M1[cps=%6.1f tgt=%6.1f duty=%6.1f%% dir=%2d]  "
               "M2[cps=%6.1f tgt=%6.1f duty=%6.1f%% dir=%2d]  "
               "Dist[L=%7.3f R=%7.3f]  "
               "IMU[roll=%6.1f pitch=%6.1f head=%6.1f filt=%6.1f err=%6.1f bias=%6.1f %s]\n",
               cps_m1_raw, tgt_cps_m1, duty_m1, dir_m1,
               cps_m2_raw, tgt_cps_m2, duty_m2, dir_m2,
               dist_m2, dist_m1,
               iok ? s.roll_deg  : 0.0f,
               iok ? s.pitch_deg : 0.0f,
               iok ? s.heading_deg      : 0.0f,
               iok ? s.heading_deg_filt : 0.0f,
               iok ? err : 0.0f,
               iok ? bias : 0.0f,
               iok ? "" : "IMU=NA");
    }
    return true;
}

// ----- Helpers -----
void get_cps(float* cps_m1, float* cps_m2) {
    const float dt = CONTROL_PERIOD_MS / 1000.0f;
    *cps_m1 = (float)enc_win_m1 / dt;
    *cps_m2 = (float)enc_win_m2 / dt;
}

void get_cps_smoothed(float* out_m1, float* out_m2) {
    *out_m1 = cps_m1_f;
    *out_m2 = cps_m2_f;
}

void get_distance_m(float* d_m1, float* d_m2) {
    float circ_m  = PI_F * WHEEL_DIAMETER_M;
    *d_m1 = ((float)enc_count_m1 / TICKS_PER_REV) * circ_m;
    *d_m2 = ((float)enc_count_m2 / TICKS_PER_REV) * circ_m;
}

int get_dir_m1(void) { return dir_m1; }
int get_dir_m2(void) { return dir_m2; }
