#include <stdio.h>
<<<<<<< Updated upstream
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
=======
#include "pico/stdlib.h"
#include "motor.h"
#include "ultrasonic.h"

// ===== Pin map (Cytron Robo Pico) =====
// Right motor (M1)
#define M1_IN1   8    // GP8
#define M1_IN2   9    // GP9
// Left motor (M2)
#define M2_IN1  11    // GP11
#define M2_IN2  10    // GP10

// IR Line Sensor D0 (your last builds used HIGH on black)
#define IR_DIGITAL_GPIO    0    // GP0

// Ultrasonic
#define US_TRIG  4
#define US_ECHO  5

// Buttons (active-low: onboard pull-ups)
#define BTN_START 21
#define BTN_STOP  20

// ===== Behaviour tuning =====
#define STOP_CM           15u   // stop if <= 15 cm
#define RESUME_CM         25u   // resume if >= 25 cm
#define LOOP_MS           60u   // main loop cadence
#define DEBOUNCE_MS       30u
#define EDGE_GUARD_MS     250u

// ===== Telemetry =====
#define TELEMETRY_MS      200u  // print US + IR every 200 ms

// ===== Base drive (always forward when allowed) =====
#define BASE_SPEED        0.30f   // forward base duty for both wheels

// ===== “Zig-zag” while on black (small alternating bias) =====
#define ZIG_DELTA         0.08f   // extra duty for the faster wheel
#define ZIG_PERIOD_MS     150u    // how often to flip zig direction (ms)

// ===== Off-line gentle correction (no spinning) =====
#define OFFLINE_DELTA     0.22f   // stronger, but still forward steer
#define OFFLINE_HOLD_MS   180u    // hold steer before rechecking

static inline void buttons_init(void) {
    gpio_init(BTN_START); gpio_set_dir(BTN_START, GPIO_IN); gpio_pull_up(BTN_START);
    gpio_init(BTN_STOP);  gpio_set_dir(BTN_STOP,  GPIO_IN); gpio_pull_up(BTN_STOP);
}

static bool button_pressed(uint pin, uint32_t *last_ms) {
    if (gpio_get(pin) == 0) { // active-low
        sleep_ms(DEBOUNCE_MS);
        if (gpio_get(pin) == 0) {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - *last_ms >= EDGE_GUARD_MS) {
                while (gpio_get(pin) == 0) { tight_loop_contents(); }
                *last_ms = now;
                return true;
            }
        }
    }
    return false;
}

static inline void line_sensor_init(void) {
    gpio_init(IR_DIGITAL_GPIO);
    gpio_set_dir(IR_DIGITAL_GPIO, GPIO_IN);
    // If your D0 is open-collector, uncomment:
    // gpio_pull_up(IR_DIGITAL_GPIO);
}

// HIGH means BLACK (match your previous code). Flip to ==0 if your module is LOW on black.
static inline bool on_black(void) {
    return gpio_get(IR_DIGITAL_GPIO) == 1;
}

int main(void) {
    stdio_init_all();
    sleep_ms(800);
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("ROBOCAR (2-pin HW PWM @20kHz): START GP21, STOP GP20\n");

    // === Motors (hardware PWM on both A/B pins)
    Motor left  = {0}, right = {0};
    motor_ab_init(&right, M1_IN1, M1_IN2, 20000.0f); // Right motor on M1
    motor_ab_init(&left,  M2_IN1, M2_IN2, 20000.0f); // Left  motor on M2
    motor_ab_brake(&left);
    motor_ab_brake(&right);

    // === IO + sensors
    buttons_init();
    line_sensor_init();
    setupUltrasonicPins(US_TRIG, US_ECHO);

    bool user_enable   = false;
    bool prox_blocked  = false;
    uint32_t last_cm   = 0;

    uint32_t t_last_start = 0, t_last_stop = 0;
    uint32_t t_last_telemetry = 0;

    // Zig-zag state
    bool zig_right = true;                    // which way to bias first
    uint32_t t_next_zig = 0;

    printf("Waiting for START…\n");
    printf("time_ms,US_cm,IR_D0,IR_state,left_cmd,right_cmd\n");

    while (true) {
        // --- buttons ---
        if (button_pressed(BTN_START, &t_last_start)) {
            user_enable = true;
            printf("[BTN] START -> enable\n");
        }
        if (button_pressed(BTN_STOP, &t_last_stop)) {
            user_enable = false;
            motor_ab_brake(&left);
            motor_ab_brake(&right);
            printf("[BTN] STOP  -> stop (brake)\n");
        }

        // --- ultrasonic safety (with hysteresis) ---
        uint32_t cm = ultrasonic_get_cm(US_TRIG, US_ECHO);
        if (cm == 0) { cm = last_cm ? last_cm : 100; } // treat timeout as far
        last_cm = cm;

        if (!prox_blocked && cm <= STOP_CM) {
            prox_blocked = true;
            motor_ab_brake(&left);
            motor_ab_brake(&right);
            printf("[US] %lu cm -> SAFETY STOP\n", (unsigned long)cm);
        } else if (prox_blocked && cm >= RESUME_CM) {
            prox_blocked = false;
            printf("[US] %lu cm -> SAFETY cleared\n", (unsigned long)cm);
        }

        const bool can_drive = user_enable && !prox_blocked;
        const uint32_t now   = to_ms_since_boot(get_absolute_time());

        float cmdL = 0.f, cmdR = 0.f;
        bool  black = on_black();

        if (!can_drive) {
            motor_ab_brake(&left);
            motor_ab_brake(&right);
            cmdL = cmdR = 0.f;
        } else if (black) {
            // --- Smooth zig-zag while on the line ---
            if (now >= t_next_zig) {
                zig_right = !zig_right;
                t_next_zig = now + ZIG_PERIOD_MS;
            }
            if (zig_right) {
                cmdL = BASE_SPEED + ZIG_DELTA;   // left a touch faster
                cmdR = BASE_SPEED - ZIG_DELTA;
            } else {
                cmdL = BASE_SPEED - ZIG_DELTA;
                cmdR = BASE_SPEED + ZIG_DELTA;   // right a touch faster
            }
            robot_movement(&left, &right, cmdL, cmdR);
        } else {
            // --- Off-line: gentle forward steer (no spin) in the opposite of current zig ---
            if (zig_right) {
                cmdL = BASE_SPEED - OFFLINE_DELTA;  // slow left wheel
                cmdR = BASE_SPEED + OFFLINE_DELTA;  // speed up right wheel
            } else {
                cmdL = BASE_SPEED + OFFLINE_DELTA;
                cmdR = BASE_SPEED - OFFLINE_DELTA;
            }

            // Clamp [0..1] and drive for a short hold, bailing out if line/safety flips
            if (cmdL < 0.f) cmdL = 0.f;
            if (cmdL > 1.f) cmdL = 1.f;
            if (cmdR < 0.f) cmdR = 0.f;
            if (cmdR > 1.f) cmdR = 1.f;

            uint32_t t0 = now;
            while (to_ms_since_boot(get_absolute_time()) - t0 < OFFLINE_HOLD_MS) {
                uint32_t cm2 = ultrasonic_get_cm(US_TRIG, US_ECHO);
                if (cm2 && cm2 <= STOP_CM) { motor_ab_brake(&left); motor_ab_brake(&right); prox_blocked = true; break; }
                if (on_black()) { break; } // re-acquired
                robot_movement(&left, &right, cmdL, cmdR);
                sleep_ms(10);
            }
        }

        // --- 200 ms telemetry ---
        if (now - t_last_telemetry >= TELEMETRY_MS) {
            int ir_raw = gpio_get(IR_DIGITAL_GPIO);
            printf("%lu,%lu,%d,%s,%.2f,%.2f\n",
                   (unsigned long)now,
                   (unsigned long)last_cm,
                   ir_raw,
                   black ? "BLACK" : "WHITE",
                   cmdL, cmdR);
            t_last_telemetry = now;
        }

        sleep_ms(LOOP_MS);
    }
>>>>>>> Stashed changes
}
