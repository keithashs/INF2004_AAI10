#include <stdio.h>
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

// IR Line Sensor (digital D0) — your module outputs HIGH on black
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


// ===== Duty targets =====
// On BLACK (on line) -> straight at 10%
#define DUTY_BLACK_L   0.50f
#define DUTY_BLACK_R   0.50f
// On WHITE (off line) -> gentle right turn.
// Left wheel 5%, Right wheel 2% (reduce right to bias turning right)
#define DUTY_WHITE_L   0.45f
#define DUTY_WHITE_R   0.40f

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
    // If D0 is open-collector, uncomment:
    // gpio_pull_up(IR_DIGITAL_GPIO);
}
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

        // --- choose duty based on line color ---
        const bool can_drive = user_enable && !prox_blocked;
        const bool black     = on_black();
        const uint32_t now   = to_ms_since_boot(get_absolute_time());

        if (!can_drive) {
            motor_ab_brake(&left);
            motor_ab_brake(&right);
        } else if (black) {
            // On the line -> straight at 10%
            robot_movement(&left, &right, DUTY_BLACK_L, DUTY_BLACK_R);
        } else {
            // Off the line -> slow right turn (left 5%, right 2%)
            robot_movement(&left, &right, DUTY_WHITE_L, DUTY_WHITE_R);
        }

        // --- 200 ms telemetry (unchanged semantics) ---
        if (now - t_last_telemetry >= TELEMETRY_MS) {
            int ir_raw = gpio_get(IR_DIGITAL_GPIO);
            printf("%lu,%lu,%d,%s,%.2f,%.2f\n",
                   (unsigned long)now,
                   (unsigned long)last_cm,
                   ir_raw,
                   black ? "BLACK" : "WHITE",
                   // last commanded speeds
                   black ? DUTY_BLACK_L : DUTY_WHITE_L,
                   black ? DUTY_BLACK_R : DUTY_WHITE_R);
            t_last_telemetry = now;
        }

        sleep_ms(LOOP_MS);
    }
}