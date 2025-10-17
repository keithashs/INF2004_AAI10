#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include <stdio.h>
#include <math.h>

#include "config.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"

// Heading PID to keep straight (deg -> cps bias)
// Gains unchanged; clamp will be updated dynamically per speed below.
static PID pid_heading = {
    .kp = 6.0f, .ki = 0.0f, .kd = 0.5f,
    .integ = 0, .prev_err = 0,
    .out_min = -20.0f, .out_max = +20.0f
};

// Timers
static repeating_timer_t control_timer_motor;   // 10 ms: motor PID & telemetry (in motor.c)
static repeating_timer_t control_timer_imu;     // 10 ms: IMU read + heading PID (this file)

// Run state shared with the IMU/heading timer through user_data
static volatile bool running = false;

// Demo settings
static float initial_heading_deg = 0.0f;
static int   run_speed_percent   = 20;

// ---- Telemetry cache (for single-line print from motor.c) ----
volatile imu_state_t g_imu_last = {0};
volatile float       g_heading_err_deg = 0.0f;
volatile float       g_bias_cps        = 0.0f;
volatile bool        g_imu_ok          = false;

// ----------------- Buttons -----------------
static inline bool btn_pressed(uint gpio) {
    return gpio_get(gpio) == 0;
}

static void buttons_init(void) {
    gpio_init(BTN_START); gpio_set_dir(BTN_START, GPIO_IN); gpio_pull_up(BTN_START);
    gpio_init(BTN_STOP);  gpio_set_dir(BTN_STOP,  GPIO_IN); gpio_pull_up(BTN_STOP);
}

// ----------------- IMU + Heading control @ 100 Hz -----------------
static bool control_cb(repeating_timer_t* t) {
    volatile bool* p_run = (volatile bool*)t->user_data;
    if (!p_run || !*p_run) { motion_command(MOVE_STOP, 0); return true; }

    imu_state_t s;
    if (imu_read(&s) && s.ok) {
        // Error: actual - desired (positive if rotated CW relative to reference)
        float err = s.heading_deg_filt - initial_heading_deg;
        while (err > 180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;

        // --- Small deadband to avoid dither around zero ---
        if (fabsf(err) < 2.0f) err = 0.0f;

        // --- Scale heading clamp with current speed (≈60% of base CPS) ---
        float base_cps = (MAX_CPS * (float)run_speed_percent) / 100.0f;
        float bias_lim = fmaxf(5.0f, 0.6f * base_cps);   // at least 5 cps so it still steers when crawling
        pid_heading.out_min = -bias_lim;
        pid_heading.out_max = +bias_lim;

        const float dt = CONTROL_PERIOD_MS / 100.0f; // 10 ms -> 0.1 s
        float bias = pid_step(&pid_heading, 0.0f, err, dt);

        // positive bias -> LEFT faster, RIGHT slower (counter-steer)
        motion_command_with_bias(MOVE_FORWARD, run_speed_percent, +bias, -bias);

        // ---- Update telemetry cache (print happens in motor.c) ----
        g_imu_last = s;
        g_heading_err_deg = err;
        g_bias_cps = bias;
        g_imu_ok = true;
    }
    return true;
}

int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("\n=== Car Demo 1: PID Speed + IMU Heading (Robo Pico) ===\n");

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
