#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/rtc.h"
#include "pin.h"
#include "motor.h"
#include "ultrasonic.h"
#include "imu_lsm303.h"
#include "pid.h"
#include "filter.h"
#include "ir.h"

// ======= Demo 1 config =======
#define BASE_SPEED        0.50f   // nominal forward duty (0..1)
#define US_STOP_CM        20U     // stop if object nearer than this
#define LOOP_HZ           100.0f  // control loop rate
#define HEADING_ALPHA     0.2f    // LP filter for heading (0..1)
#define ACC_ALPHA         0.2f    // LP filter for accel (telemetry smoothing)

// simple helpers
static inline float clamp1(float x){ if(x>1) return 1; if(x<-1) return -1; return x; }
static float wrap_deg_err(float target, float current) {
    float e = target - current;
    while (e > 180.f) e -= 360.f;
    while (e < -180.f) e += 360.f;
    return e;
}

int main() {
    stdio_init_all();
    sleep_ms(500); // wait for USB CDC

    // Buttons
    gpio_init(BTN_START); gpio_set_dir(BTN_START, GPIO_IN); gpio_pull_up(BTN_START);
    gpio_init(BTN_STOP);  gpio_set_dir(BTN_STOP,  GPIO_IN); gpio_pull_up(BTN_STOP);

    motor_init();
    ultrasonic_init();
    ir_init();

    imu_t imu; imu_init(&imu);

    // Filters
    lp_t f_head; lp_init(&f_head, HEADING_ALPHA);
    lp_t f_ax; lp_init(&f_ax, ACC_ALPHA);
    lp_t f_ay; lp_init(&f_ay, ACC_ALPHA);
    lp_t f_az; lp_init(&f_az, ACC_ALPHA);

    // PID for heading correction: output is a differential term [-0.4..0.4]
    pid_t pid_heading; pid_init(&pid_heading, /*kp*/0.03f, /*ki*/0.00f, /*kd*/0.003f, -0.4f, 0.4f);

    const float dt = 1.0f / LOOP_HZ;
    absolute_time_t next = make_timeout_time_ms((uint32_t)(dt*1000.0f));

    // On first start, set reference heading (target)
    float target_heading = 0.0f;
    int   run_enabled = 0;
    uint32_t last_cm = 999;

    printf("Demo1 Basic Motion & Sensing (no WiFi)\n");

    while (true) {
        // Debounce start/stop
        if (!gpio_get(BTN_START)) { run_enabled = 1; sleep_ms(200); }
        if (!gpio_get(BTN_STOP))  { run_enabled = 0; motor_brake(); sleep_ms(200); }

        // Read sensors
        imu_read(&imu);
        float head_f = lp_apply(&f_head, imu.heading_deg);
        float ax_f = lp_apply(&f_ax, imu.ax);
        float ay_f = lp_apply(&f_ay, imu.ay);
        float az_f = lp_apply(&f_az, imu.az);

        // Lock target heading on first enable
        static int target_set = 0;
        if (run_enabled && !target_set) { target_heading = head_f; target_set = 1; }
        if (!run_enabled) target_set = 0;

        // Ultrasonic safety
        uint32_t cm = ultrasonic_read_cm();
        if (cm != 0) last_cm = cm;

        float left_cmd = 0, right_cmd = 0;

        if (run_enabled) {
            if (last_cm <= US_STOP_CM) {
                // Obstacle too close => hard stop
                left_cmd = right_cmd = 0.0f;
            } else {
                // Heading PID (keeps straight)
                float err = wrap_deg_err(target_heading, head_f);
                float diff = pid_step(&pid_heading, err, dt); // [-0.4..0.4]

                left_cmd  = clamp1(BASE_SPEED - diff);
                right_cmd = clamp1(BASE_SPEED + diff);
            }
        } else {
            left_cmd = right_cmd = 0.0f;
        }

        // Drive motors
        motor_set_left(left_cmd);
        motor_set_right(right_cmd);

        // Telemetry (USB CDC serial)
        // Raw vs filtered heading; accel raw vs filtered; PID terms and ultrasound
        printf("RUN:%d  US:%lucm  SPD:(L=%.2f R=%.2f)  "
               "HEAD raw=%.1f filt=%.1f tgt=%.1f err=%.1f  "
               "ACC raw(%.2f,%.2f,%.2f) filt(%.2f,%.2f,%.2f)\n",
               run_enabled, (unsigned long)last_cm, left_cmd, right_cmd,
               imu.heading_deg, head_f, target_heading, wrap_deg_err(target_heading, head_f),
               imu.ax, imu.ay, imu.az, ax_f, ay_f, az_f);

        sleep_until(next);
        next = delayed_by_ms(next, (uint32_t)(dt*1000.0f));
    }
    return 0;
}
