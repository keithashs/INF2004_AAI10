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
// 1) Wheel-balance (encoder-difference) PID: keeps cps_R ~ cps_L (fast, aggressive)
static PID pid_track = {
    .kp = 0.90f, .ki = 1.8f, .kd = 0.00f,
    .integ = 0, .prev_err = 0,
    .out_min = -100.0f, .out_max = +100.0f
};

// 2) IMU heading PID (slow trim): slightly stronger/faster
static PID pid_heading = {
    .kp = 0.35f, .ki = 0.05f, .kd = 0.00f,
    .integ = 0, .prev_err = 0,
    .out_min = -100.0f, .out_max = +100.0f
};

// Timers
static repeating_timer_t control_timer_motor;   // 10 ms: motor PID & telemetry (in motor.c)
static repeating_timer_t control_timer_imu;     // 10 ms: encoder-balance + IMU trim (this file)

// Run state (shared with control cb via user_data)
static volatile bool running = false;

// Demo settings (you may change only run_speed_percent to tune overall speed)
static float initial_heading_deg = 0.0f;
static int   run_speed_percent   = 20;

// ---- Telemetry cache ----
volatile imu_state_t g_imu_last = {0};
volatile float       g_heading_err_deg = 0.0f;
volatile float       g_bias_cps        = 0.0f;
volatile bool        g_imu_ok          = false;

static inline bool btn_pressed(uint gpio) {
    return gpio_get(gpio) == 0;
}

static void buttons_init(void) {
    gpio_init(BTN_START); gpio_set_dir(BTN_START, GPIO_IN); gpio_pull_up(BTN_START);
    gpio_init(BTN_STOP);  gpio_set_dir(BTN_STOP,  GPIO_IN); gpio_pull_up(BTN_STOP);
}

static inline float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// ----------------- Fast control @ 100 Hz -----------------
// Does BOTH: (A) wheel-balance PID from encoders, (B) small IMU trim
static bool control_cb(repeating_timer_t* t) {
    volatile bool* p_run = (volatile bool*)t->user_data;
    if (!p_run || !*p_run) { motion_command(MOVE_STOP, 0); return true; }

    // Current smoothed cps from motor.c
    float cps_r, cps_l;
    get_cps_smoothed(&cps_r, &cps_l);

    // (A) Encoder-balance error: right minus left (we want 0)
    float diff_meas = cps_r - cps_l;
    const float dt = CONTROL_PERIOD_MS / 1000.0f;

    // positive bias -> LEFT faster, RIGHT slower
    float bias_track = pid_step(&pid_track, 0.0f, -diff_meas, dt);

    // (B) IMU heading small trim
    float bias_head = 0.0f;
    imu_state_t s;
    bool imu_ok = imu_read(&s) && s.ok;
    if (imu_ok) {
        float err_deg = wrap180(s.heading_deg_filt - initial_heading_deg);

        // smaller deadband for quicker correction
        if (fabsf(err_deg) < 1.0f) err_deg = 0.0f;

        bias_head = pid_step(&pid_heading, 0.0f, -err_deg, dt);

        // Telemetry cache
        g_imu_last = s;
        g_heading_err_deg = err_deg;
        g_imu_ok = true;
    } else {
        g_imu_ok = false;
    }

    // Total bias (CPS) = fast track + slow heading
    float base_cps = (MAX_CPS * (float)run_speed_percent) / 100.0f;

    // Keep track strong, but let heading do a bit more (30% of base)
    float lim_track = fmaxf(5.0f, 0.50f * base_cps);
    float lim_head  = fmaxf(2.0f, 0.30f * base_cps);

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

int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("\n=== Car Demo 1: Cascaded Heading (Enc diff + IMU trim) ===\n");

    buttons_init();

    if (!imu_init()) printf("IMU init failed! Check I2C wiring (GP2 SDA, GP3 SCL) and power.\n");
    else             printf("IMU ready.\n");

    motor_init();

    add_repeating_timer_ms(-CONTROL_PERIOD_MS, motor_control_timer_cb, NULL, &control_timer_motor);
    add_repeating_timer_ms(-CONTROL_PERIOD_MS, control_cb, (void*)&running, &control_timer_imu);

    while (true) {
        if (!running) {
            if (btn_pressed(BTN_START)) {
                sleep_ms(30);
                if (btn_pressed(BTN_START)) {
                    imu_state_t s;
                    if (imu_read(&s) && s.ok) {
                        initial_heading_deg = s.heading_deg_filt;
                        imu_reset_heading_filter(initial_heading_deg);
                    } else {
                        initial_heading_deg = 0.0f;
                    }
                    pid_reset(&pid_track);
                    pid_reset(&pid_heading);
                    motion_command(MOVE_FORWARD, run_speed_percent);
                    running = true;
                    printf("START -> %d%%, heading_ref=%.1f deg\n", run_speed_percent, initial_heading_deg);
                    while (btn_pressed(BTN_START)) tight_loop_contents();
                }
            } else {
                motor_all_stop();
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
