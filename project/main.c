// main.c
#include <stdio.h>
#include "pico/stdlib.h"
#include "ultrasonic.h"

// ===== Pin map (Cytron Robo Pico, 2-pin drive per motor) =====
// Right motor (M1)
#define M1_IN1   8    // GP8
#define M1_IN2   9    // GP9
// Left motor (M2)
#define M2_IN1  11    // GP11
#define M2_IN2  10    // GP10

// Ultrasonic (change if needed)
#define US_TRIG  4
#define US_ECHO  5

// Buttons (active-low: use onboard pull-ups)
#define BTN_START 21   // press to enable movement
#define BTN_STOP  20   // press to force stop

// ===== Behaviour tuning =====
#define STOP_CM         15u   // stop if <= 15 cm
#define RESUME_CM       25u   // resume if >= 25 cm (hysteresis)
#define PING_PERIOD_MS  80u   // 60–100 ms between pings
#define DEBOUNCE_MS     30u
#define REPEAT_GUARD_MS 250u  // ignore repeats within this time

// ===== Motor helpers (direction only, full speed) =====
static inline void motor_drive_2pin(uint in1, uint in2, int dir) {
    // dir: +1 forward, -1 reverse, 0 stop (coast)
    if (dir > 0)      { gpio_put(in1, 1); gpio_put(in2, 0); }
    else if (dir < 0) { gpio_put(in1, 0); gpio_put(in2, 1); }
    else              { gpio_put(in1, 0); gpio_put(in2, 0); }
}

static inline void motors_init(void) {
    gpio_init(M1_IN1); gpio_set_dir(M1_IN1, GPIO_OUT); gpio_put(M1_IN1, 0);
    gpio_init(M1_IN2); gpio_set_dir(M1_IN2, GPIO_OUT); gpio_put(M1_IN2, 0);
    gpio_init(M2_IN1); gpio_set_dir(M2_IN1, GPIO_OUT); gpio_put(M2_IN1, 0);
    gpio_init(M2_IN2); gpio_set_dir(M2_IN2, GPIO_OUT); gpio_put(M2_IN2, 0);
}

static inline void robot_stop(void) {
    motor_drive_2pin(M1_IN1, M1_IN2, 0);
    motor_drive_2pin(M2_IN1, M2_IN2, 0);
}

// Signed “speed” is used as direction only: + = forward, − = reverse, 0 = stop
static inline void robot_move(float sL, float sR) {
    int dL = (sL > 0) - (sL < 0);
    int dR = (sR > 0) - (sR < 0);
    motor_drive_2pin(M2_IN1, M2_IN2, dL); // left motor
    motor_drive_2pin(M1_IN1, M1_IN2, dR); // right motor
}

// ===== Buttons =====
static inline void buttons_init(void) {
    gpio_init(BTN_START); gpio_set_dir(BTN_START, GPIO_IN); gpio_pull_up(BTN_START);
    gpio_init(BTN_STOP);  gpio_set_dir(BTN_STOP,  GPIO_IN); gpio_pull_up(BTN_STOP);
}

// debounced, edge-triggered press detector (active-low)
static bool button_pressed(uint pin, uint32_t *last_ms) {
    if (gpio_get(pin) == 0) {                // low = pressed
        sleep_ms(DEBOUNCE_MS);
        if (gpio_get(pin) == 0) {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - *last_ms >= REPEAT_GUARD_MS) {
                // wait for release so one press → one event
                while (gpio_get(pin) == 0) { tight_loop_contents(); }
                *last_ms = now;
                return true;
            }
        }
    }
    return false;
}

int main() {
    stdio_init_all();
    sleep_ms(800);
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("Robocar: START on GP21, STOP on GP20, ultrasonic safety + hysteresis\n");

    motors_init();
    buttons_init();
    setupUltrasonicPins(US_TRIG, US_ECHO);

    bool user_enable = false;   // set true by START, false by STOP
    bool prox_blocked = false;  // set by ultrasonic STOP/RESUME thresholds
    uint32_t last_cm = 0;

    uint32_t t_last_start = 0, t_last_stop = 0;

    // Idle until START is pressed
    robot_stop();
    printf("Waiting for START (GP21)...\n");

    while (true) {
        // ---- buttons ----
        if (button_pressed(BTN_START, &t_last_start)) {
            user_enable = true;
            printf("[BTN] START pressed -> enable movement\n");
        }
        if (button_pressed(BTN_STOP, &t_last_stop)) {
            user_enable = false;
            robot_stop();
            printf("[BTN] STOP pressed -> immediate stop\n");
        }

        // ---- ultrasonic safety (with hysteresis) ----
        uint32_t cm = ultrasonic_get_cm(US_TRIG, US_ECHO);
        if (cm == 0) { cm = last_cm ? last_cm : 100; }  // treat timeout as far
        last_cm = cm;

        if (!prox_blocked && cm <= STOP_CM) {
            prox_blocked = true;
            printf("[US] %lu cm -> SAFETY STOP active\n", (unsigned long)cm);
        } else if (prox_blocked && cm >= RESUME_CM) {
            prox_blocked = false;
            printf("[US] %lu cm -> SAFETY cleared\n", (unsigned long)cm);
        }

        // ---- decide & act ----
        bool should_run = user_enable && !prox_blocked;

        static bool was_running = false;
        if (should_run && !was_running) {
            robot_move(+1.0f, +1.0f);
            printf("RUN\n");
        } else if (!should_run && was_running) {
            robot_stop();
            printf("STOP (user=%d, safety=%d)\n", user_enable, !prox_blocked);
        }
        was_running = should_run;

        sleep_ms(PING_PERIOD_MS);
    }
}
