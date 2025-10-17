#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include <stdio.h>
#include <math.h>

#include "config.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"

// Heading PID to keep straight (deg -> cps bias). Output biases the wheel setpoints.
static PID pid_heading = {
    .kp = 6.0f, .ki = 0.0f, .kd = 0.5f,
    .integ = 0, .prev_err = 0,
    .out_min = -30.0f, .out_max = +30.0f
};
// tune out_min/max to a small fraction of MAX_CPS so heading fix doesn't saturate speed

// Timers
static repeating_timer_t control_timer_motor;   // 10 ms: motor PID & telemetry (in motor.c)
static repeating_timer_t control_timer_imu;     // 10 ms: IMU read + heading PID (this file)

// Run state shared with the IMU/heading timer through user_data
static volatile bool running = false;

// Demo settings
static float initial_heading_deg = 0.0f;
static int   run_speed_percent   = 100;

// ----------------- Buttons -----------------
static inline bool btn_pressed(uint gpio) {
    // Robo Pico buttons are typically pulled up; pressed == 0
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
        // Error: actual - desired (so positive err = rotated CW if heading increased)
        float heading_err = s.heading_deg_filt - initial_heading_deg;
        while (heading_err > 180.0f) heading_err -= 360.0f;
        while (heading_err < -180.0f) heading_err += 360.0f;

        const float dt = CONTROL_PERIOD_MS / 100.0f; // 10 ms -> 0.1 s
        // Drive error to zero: setpoint 0, measured = heading_err
        float bias_cps = pid_step(&pid_heading, 0.0f, heading_err, dt);

        // IMPORTANT: positive bias turns LEFT wheel faster, RIGHT slower (counter-steer)
        //             (flip signs vs earlier build that swung left)
        motion_command_with_bias(MOVE_FORWARD, run_speed_percent, +bias_cps, -bias_cps);

        static uint64_t last_imu_print = 0;
        uint64_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_imu_print >= TELEMETRY_MS) {
            last_imu_print = now_ms;
            float cps_r, cps_l; get_cps(&cps_r, &cps_l);
            float d_r, d_l;     get_distance_m(&d_r, &d_l);
            printf("IMU roll=%.1f pitch=%.1f head_raw=%.1f head_filt=%.1f err=%.1f  "
                   "biasCPS=%.1f  Lcps=%.1f Rcps=%.1f  Lm=%.3f Rm=%.3f\n",
                   s.roll_deg, s.pitch_deg, s.heading_deg, s.heading_deg_filt, heading_err,
                   bias_cps, cps_l, cps_r, d_l, d_r);
        }
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
