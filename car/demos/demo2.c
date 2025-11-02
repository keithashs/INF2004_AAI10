// ==============================
// main.c — Clean version (Demo 2) + Guard IR (DO) to stabilise search
// ==============================
// - Line following remains your original: 1x IR via ADC (GPIO28 / ADC2)
// - NEW guard IR (DO, e.g., GPIO1) used ONLY to:
//     (a) choose the *first* search direction when line is lost
//     (b) exit search early when ideal posture is reacquired (ADC on-line + guard white)
// - "Remember last turn" recovery logic preserved
// - No ultrasonic / obstacle detection
// ==============================

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

// === Your existing motor + encoder API headers ===
#include "motor.h"
#include "encoder.h"
#include "config.h"   // for config constants

// ================== CONFIG ==================

// Primary line sensor (ADC): GPIO 28 (ADC2)
#ifndef LINE_SENSOR_PIN
#define LINE_SENSOR_PIN 26
#endif

// Hysteresis thresholds (example: floor ~170, black ~4070)
#ifndef TH_LO
#define TH_LO   600      // definitely OFF line (white floor)
#endif
#ifndef TH_HI
#define TH_HI   3000     // definitely ON line (black tape)
#endif

// Guard IR (DO) used ONLY for search/recovery stabilisation
// Use your actual DO pin (e.g., GP1). Active-LOW on black for most modules.
#ifndef IR_GUARD_DO_GPIO
#define IR_GUARD_DO_GPIO 1
#endif

#ifndef LEFT
#define LEFT  0
#endif
#ifndef RIGHT
#define RIGHT 1
#endif

// Slow crawl speed (PID target, in cm/s) — your original Demo-2 setting
#ifndef SLOW_SPEED_CMPS
#define SLOW_SPEED_CMPS 2.0f
#endif

// Trim offset for drift correction (positive = reduce right motor)
#ifndef TRIM_OFFSET
#define TRIM_OFFSET -0.5f
#endif

// Base steering window for turn search (ms)
#ifndef STEER_DURATION
#define STEER_DURATION 220
#endif

// Debounce between “decisions” (ms)
#ifndef DEBOUNCE_DELAY_MS
#define DEBOUNCE_DELAY_MS 500
#endif

// Pivot PWM while searching
#ifndef SEARCH_PWM
#define SEARCH_PWM 50
#endif

// ================== Helpers ==================

// Hysteresis around ADC thresholds for stable ON/OFF line decision
static inline bool on_line_hys(uint16_t v) {
    static bool state = false;
    if (v >= TH_HI)      state = true;   // clearly black
    else if (v <= TH_LO) state = false;  // clearly white
    return state;
}

// Guard DO: return true when guard sees BLACK (active-LOW typical)
static inline bool guard_is_black(void) {
    return gpio_get(IR_GUARD_DO_GPIO) == 0;
}

static void init_line_sensor(void) {
    // Primary ADC line sensor on GPIO28/ADC2
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_select_input(2); // GPIO28 -> ADC2

    // Guard IR (DO)
    gpio_init(IR_GUARD_DO_GPIO);
    gpio_set_dir(IR_GUARD_DO_GPIO, GPIO_IN);
    gpio_pull_up(IR_GUARD_DO_GPIO);
}

// ================== Interrupts (encoders) ==================
// Make sure encoder.c exports read_encoder_pulse(...)
void driver_callbacks(uint gpio, uint32_t events) {
    switch (gpio) {
        case L_ENCODER_OUT: read_encoder_pulse(L_ENCODER_OUT, events); break;
        case R_ENCODER_OUT: read_encoder_pulse(R_ENCODER_OUT, events); break;
        default: break;
    }
}

static void init_interrupts(void) {
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true,  &driver_callbacks);
    gpio_set_irq_enabled_with_callback(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true,  &driver_callbacks);
}

// ================== Search helper (improved with guard DO) ==================
// Turn and poll sensors; exit early when ADC is ON-LINE *and* guard is WHITE.
static bool line_follow_turn_motor(int direction_0isLeft_1isRight, uint32_t steer_duration_ms) {
    disable_pid_control(); // we will hand the motion to turn helper

    const int motor_dir = (direction_0isLeft_1isRight == 0) ? LEFT : RIGHT;
    turn_motor_manual(motor_dir, CONTINUOUS_TURN, SEARCH_PWM, SEARCH_PWM);

    const uint64_t start = time_us_64();
    const uint64_t max_us = (uint64_t)steer_duration_ms * 1000ULL;

    while ((time_us_64() - start) < max_us) {
        uint16_t v = adc_read();
        bool on_line = on_line_hys(v);
        bool guard_white = !guard_is_black();

        // Ideal reacquisition posture: ADC sees line AND guard is white
        if (on_line && guard_white) {
            stop_motor_pid(); // also recentres any PID-owned outputs
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    stop_motor_pid();
    vTaskDelay(pdMS_TO_TICKS(50));
    return false;
}

// ================== Line-follow Task (Demo 2 + guard-stabilised search) ==================
static void lineFollowTask(void *pvParameters) {
    uint64_t last_decide_time = 0;

    // “Remember last turn” scaffolding
    int last_turn_dir = 0;   // 0 = Left, 1 = Right
    int initial_turn_direction   = last_turn_dir;
    int alternate_turn_direction = 1 - initial_turn_direction;

    bool needs_second_turn       = false;

    uint32_t initial_steer_duration = STEER_DURATION;
    uint32_t second_steer_duration  = initial_steer_duration + 60;

    int consecutive_reversals = 0;
    int non_reversal_turns    = 0;

    // NEW: drift hint from guard to pick the best first search direction
    // +1 => likely line is on RIGHT side, -1 => likely on LEFT, 0 => unknown
    int8_t drift_hint = 0;
    uint8_t drift_cnt = 0;
    const uint8_t DRIFT_OK = 3; // debounce samples (~3*loop ticks)

    printf("[LF] Slow-PID (ADC) line following started. Guard DO at GP%d.\n", IR_GUARD_DO_GPIO);
    printf("     Preferring %s first when unknown.\n", (last_turn_dir==0) ? "LEFT" : "RIGHT");

    for (;;) {
        const uint64_t now = time_us_64();
        if (now - last_decide_time < (uint64_t)DEBOUNCE_DELAY_MS * 1000ULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        last_decide_time = now;

        // Read ADC line & guard DO
        uint16_t raw = adc_read();
        bool on_line = on_line_hys(raw);
        bool guard_black = guard_is_black();

        // ------------------- Normal follow (unchanged) -------------------
        if (on_line) {
            // When solidly on line (center), go straight without correction
            // Only use PID near edges for gentle steering
            bool centered_on_line = (raw >= (TH_HI - (TH_HI - TH_LO)/4));
            
            if (centered_on_line) {
                // Go straight - disable PID, use manual equal speeds with trim
                disable_pid_control();
                float base_pwm = SLOW_SPEED_CMPS * 40;  // 100 PWM for 2 cm/s
                float left_pwm = base_pwm + TRIM_OFFSET;   // Increase left
                float right_pwm = base_pwm - TRIM_OFFSET;  // Decrease right
                forward_motor_manual(left_pwm, right_pwm);
            } else {
                // Near edge - use PID for smooth tracking
                forward_motor_pid(SLOW_SPEED_CMPS);
            }

            // Reset recovery/scaling variables
            initial_turn_direction   = last_turn_dir;
            alternate_turn_direction = 1 - initial_turn_direction;
            initial_steer_duration   = STEER_DURATION;
            second_steer_duration    = initial_steer_duration + 60;
            consecutive_reversals    = 0;
            non_reversal_turns       = 0;
            needs_second_turn        = false;

            // Build/decay drift hint near the loss boundary
            bool near_loss = (raw <= TH_LO + (TH_HI - TH_LO)/3);
            if (near_loss) {
                int8_t hint_now = guard_black ? +1 : -1; // guard black => tape near guard side
                if (drift_cnt < 255) drift_cnt++;
                if (drift_cnt >= DRIFT_OK) drift_hint = hint_now;
            } else {
                if (drift_cnt > 0) drift_cnt--;
                if (drift_cnt == 0) drift_hint = 0;
            }

        } else {
            // ------------------- LOST LINE: reverse & search -------------------

            // Small reverse to back off before pivot
            if (consecutive_reversals < 3 || non_reversal_turns >= 3) {
                stop_motor_pid();
                reverse_motor_manual(130, 130);
                vTaskDelay(pdMS_TO_TICKS(90));
                if (consecutive_reversals < 3) consecutive_reversals++;
            }

            // Choose the first search direction:
            // Use guard drift hint if available; else fall back to last_turn_dir
            int prefer_dir = (drift_hint > 0) ? RIGHT : (drift_hint < 0 ? LEFT : last_turn_dir);

            int initial_dir   = prefer_dir;
            int alternate_dir = 1 - initial_dir;

            bool found = false;

            if (!needs_second_turn) {
                found = line_follow_turn_motor(initial_dir, initial_steer_duration);
                needs_second_turn = !found;
                if (found) last_turn_dir = initial_dir;
            } else {
                found = line_follow_turn_motor(alternate_dir, second_steer_duration);
                needs_second_turn = false;
                if (found) last_turn_dir = alternate_dir;
            }

            // Escalate steer duration to cope with near-90° bends
            if (consecutive_reversals >= 3) {
                initial_steer_duration = (initial_steer_duration + 70 > 500)
                                           ? 500 : (initial_steer_duration + 70);
                second_steer_duration  = (initial_steer_duration + 60 > 520)
                                           ? 520 : (initial_steer_duration + 60);
                non_reversal_turns++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ================== Main ==================
int main(void) {
    stdio_init_all();
    sleep_ms(400);

    init_line_sensor();
    encoder_init();
    motor_init();
    init_interrupts();

    xTaskCreate(lineFollowTask, "LineFollow", configMINIMAL_STACK_SIZE * 4,
                NULL, tskIDLE_PRIORITY + 1, NULL);

    printf("[MAIN] Slow-PID line follow ready. TH_LO=%d, TH_HI=%d, speed=%.1f cm/s\n",
           TH_LO, TH_HI, SLOW_SPEED_CMPS);
    printf("[MAIN] Guard DO on GP%d. Search PWM=%d, steer=%dms (+%dms).\n",
           IR_GUARD_DO_GPIO, SEARCH_PWM, STEER_DURATION, 60);

    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
    return 0;
}