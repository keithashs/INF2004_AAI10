// ==============================
// demo2.c — Demo 2 + Demo 1 inner PID core (improved steering)
// ==============================
// - Line following (ADC on LEFT IR)
// - Guard IR (DO) to stabilise search
// - Demo 1 inner PID drive core used during ON-LINE drive
// - Continuous steering from ADC line error instead of fixed bias
// ==============================

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

// === Your motor + encoder headers ===
#include "motor.h"
#include "encoder.h"
#include "config.h"  // PID constants, TRACK_WIDTH, trims, etc.

// ================== CONFIG ==================

// Primary line sensor (ADC)
#ifndef LINE_SENSOR_PIN
#define LINE_SENSOR_PIN 27
#endif
#ifndef LINE_SENSOR_ADC_CH
#define LINE_SENSOR_ADC_CH 1
#endif

// Guard IR (DO) — RIGHT IR
#ifndef IR_GUARD_DO_GPIO
#define IR_GUARD_DO_GPIO 28
#endif

// Thresholds for line detection
#ifndef TH_LO
#define TH_LO  600
#endif
#ifndef TH_HI
#define TH_HI  3000
#endif

// Gain factor for line steering
#ifndef K_LINE_RAD_S
#define K_LINE_RAD_S  2.0f
#endif

#ifndef LEFT
#define LEFT  0
#endif
#ifndef RIGHT
#define RIGHT 1
#endif

#ifndef SLOW_SPEED_CMPS
#define SLOW_SPEED_CMPS 2.0f
#endif
#ifndef STEER_DURATION
#define STEER_DURATION 220
#endif
#ifndef DEBOUNCE_DELAY_MS
#define DEBOUNCE_DELAY_MS 500
#endif
#ifndef SEARCH_PWM
#define SEARCH_PWM 50
#endif

// ================== Helper functions ==================
static inline bool on_line_hys(uint16_t v) {
    static bool state = false;
    if (v >= TH_HI)      state = true;
    else if (v <= TH_LO) state = false;
    return state;
}

static inline bool guard_is_black(void) {
    return gpio_get(IR_GUARD_DO_GPIO) == 0;
}

static void init_line_sensor(void) {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_select_input(LINE_SENSOR_ADC_CH);

    gpio_init(IR_GUARD_DO_GPIO);
    gpio_set_dir(IR_GUARD_DO_GPIO, GPIO_IN);
    gpio_pull_up(IR_GUARD_DO_GPIO);
}

// ================== Encoder interrupts ==================
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

// ================== Demo 1 Drive Core (Inner PID + Straightness) ==================
typedef struct {
    float s_int;
    int lastL, lastR;
    float iL, iR, eL_prev, eR_prev;
} drive_core_t;

static inline float clampf_local(float v, float a, float b){ return v<a?a:(v>b?b:v); }
static inline int   clampi_local(int v, int a, int b){ return v<a?a:(v>b?b:v); }

static float pid_step(float kp, float ki, float kd, float err, float dt, float *i_acc, float *e_prev){
    *i_acc += err * dt;
    *i_acc = clampf_local(*i_acc, -SPID_IWIND_CLAMP, +SPID_IWIND_CLAMP);
    float d = (err - *e_prev) / dt;
    *e_prev = err;
    float u = kp*err + ki*(*i_acc) + kd*d;
    return clampf_local(u, SPID_OUT_MIN, SPID_OUT_MAX);
}

static void drive_core_step(drive_core_t *dc, float target_speed_cmps, float bias_rad_s){
    float diff_cmps = (0.5f * TRACK_WIDTH_M * bias_rad_s) * 100.0f;
    float vL_target = target_speed_cmps - diff_cmps;
    float vR_target = target_speed_cmps + diff_cmps;

    float vL_meas = get_left_speed();
    float vR_meas = get_right_speed();

    float uL = pid_step(SPID_KP, SPID_KI, SPID_KD, vL_target - vL_meas, DT_S, &dc->iL, &dc->eL_prev);
    float uR = pid_step(SPID_KP, SPID_KI, SPID_KD, vR_target - vR_meas, DT_S, &dc->iR, &dc->eR_prev);

    float s_err = (vR_meas - vL_meas);
    dc->s_int += s_err * DT_S;
    dc->s_int = clampf_local(dc->s_int, -STRAIGHT_I_CLAMP, STRAIGHT_I_CLAMP);
    float s_trim = STRAIGHT_KP * s_err + STRAIGHT_KI * dc->s_int;

    int pwmL = (int)lroundf(BASE_PWM_L + uL + s_trim + GLOBAL_S_TRIM_OFFSET);
    int pwmR = (int)lroundf(BASE_PWM_R + uR - s_trim - GLOBAL_S_TRIM_OFFSET);
    pwmL = clampi_local(pwmL, PWM_MIN_LEFT,  PWM_MAX_LEFT);
    pwmR = clampi_local(pwmR, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

    int dL = pwmL - dc->lastL, dR = pwmR - dc->lastR;
    if (dL >  MAX_PWM_STEP) pwmL = dc->lastL + MAX_PWM_STEP;
    if (dL < -MAX_PWM_STEP) pwmL = dc->lastL - MAX_PWM_STEP;
    if (dR >  MAX_PWM_STEP) pwmR = dc->lastR + MAX_PWM_STEP;
    if (dR < -MAX_PWM_STEP) pwmR = dc->lastR - MAX_PWM_STEP;

    forward_motor_manual(pwmL, pwmR);
    dc->lastL = pwmL;
    dc->lastR = pwmR;
}

// ================== Search helper ==================
static bool line_follow_turn_motor(int direction_0isLeft_1isRight, uint32_t steer_duration_ms) {
    disable_pid_control();
    const int motor_dir = (direction_0isLeft_1isRight == 0) ? LEFT : RIGHT;
    turn_motor_manual(motor_dir, CONTINUOUS_TURN, SEARCH_PWM, SEARCH_PWM);

    const uint64_t start = time_us_64();
    const uint64_t max_us = (uint64_t)steer_duration_ms * 1000ULL;

    while ((time_us_64() - start) < max_us) {
        uint16_t v = adc_read();
        bool on_line = on_line_hys(v);
        bool guard_white = !guard_is_black();
        if (on_line && guard_white) {
            stop_motor_pid();
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    stop_motor_pid();
    vTaskDelay(pdMS_TO_TICKS(50));
    return false;
}

// ================== Line-follow Task ==================
static void lineFollowTask(void *pvParameters) {
    uint64_t last_decide_time = 0;
    drive_core_t DC = { .s_int=0.f, .lastL=BASE_PWM_L, .lastR=BASE_PWM_R };

    int last_turn_dir = 0;
    int initial_turn_direction = last_turn_dir;
    int alternate_turn_direction = 1 - initial_turn_direction;

    bool needs_second_turn = false;
    uint32_t initial_steer_duration = STEER_DURATION;
    uint32_t second_steer_duration  = initial_steer_duration + 60;
    int consecutive_reversals = 0;
    int non_reversal_turns = 0;

    int8_t drift_hint = 0;
    uint8_t drift_cnt = 0;
    const uint8_t DRIFT_OK = 3;

    for (;;) {
        const uint64_t now = time_us_64();
        if (now - last_decide_time < (uint64_t)DEBOUNCE_DELAY_MS * 1000ULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        last_decide_time = now;

        uint16_t raw = adc_read();
        bool on_line = on_line_hys(raw);
        bool guard_black = guard_is_black();

        if (on_line) {
            // Convert ADC into signed error and steering bias
            float mid   = 0.5f * (TH_LO + TH_HI);
            float span  = fmaxf(50.0f, 0.5f * (TH_HI - TH_LO));
            float e_raw = ((float)raw - mid) / span;
            if (e_raw < -1.f) e_raw = -1.f;
            if (e_raw > +1.f) e_raw = +1.f;

            float bias_rad_s = K_LINE_RAD_S * e_raw;
            drive_core_step(&DC, SLOW_SPEED_CMPS, bias_rad_s);

            static uint32_t print_div = 0;
            if (print_div++ % 4 == 0) {
                float vL = get_left_speed();
                float vR = get_right_speed();
                printf("[SPEED] L=%.2f cm/s  R=%.2f cm/s | e_raw=%.2f | bias=%.2f\n", vL, vR, e_raw, bias_rad_s);
            }

            initial_turn_direction   = last_turn_dir;
            alternate_turn_direction = 1 - initial_turn_direction;
            initial_steer_duration   = STEER_DURATION;
            second_steer_duration    = initial_steer_duration + 60;
            consecutive_reversals    = 0;
            non_reversal_turns       = 0;
            needs_second_turn        = false;

            bool near_loss = (raw <= TH_LO + (TH_HI - TH_LO)/3);
            if (near_loss) {
                int8_t hint_now = guard_black ? +1 : -1;
                if (drift_cnt < 255) drift_cnt++;
                if (drift_cnt >= DRIFT_OK) drift_hint = hint_now;
            } else {
                if (drift_cnt > 0) drift_cnt--;
                if (drift_cnt == 0) drift_hint = 0;
            }

        } else {
            // LOST LINE
            if (consecutive_reversals < 3 || non_reversal_turns >= 3) {
                stop_motor_pid();
                reverse_motor_manual(130, 130);
                vTaskDelay(pdMS_TO_TICKS(90));
                if (consecutive_reversals < 3) consecutive_reversals++;
            }

            int prefer_dir = (drift_hint > 0) ? RIGHT : (drift_hint < 0 ? LEFT : last_turn_dir);
            int initial_dir = prefer_dir;
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

            if (consecutive_reversals >= 3) {
                initial_steer_duration = (initial_steer_duration + 70 > 500) ? 500 : (initial_steer_duration + 70);
                second_steer_duration  = (initial_steer_duration + 60 > 520) ? 520 : (initial_steer_duration + 60);
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

    printf("[MAIN] Demo2 + Demo1 inner PID ready.\n");
    printf("[MAIN] LINE on GP%d (ADC%d), GUARD on GP%d.\n", LINE_SENSOR_PIN, LINE_SENSOR_ADC_CH, IR_GUARD_DO_GPIO);
    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
    return 0;
}