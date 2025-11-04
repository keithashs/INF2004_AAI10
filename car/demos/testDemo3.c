// One-file: Curved line-follow + Code39 decisions (AO on GP28, DO on GP27)
// Patched: local PWM governor + safe steering scale + bend-aware base + tight slew
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

// --- Motor & barcode (your code) ---
#include "motor.h"        // forward_motor_manual(), turn_motor_manual(...)
#ifndef IR_SENSOR_PIN
#define IR_SENSOR_PIN 27  // barcode DO GPIO (your second IR module)
#endif
#include "barcode.h"      // barcode_init(), init_barcode_irq(), decoded_barcode_char, is_scanning_allowed

// ====================== Pins ======================
#define LINE_SENSOR_AO     28   // GP28 ADC2 (your line sensor AO)
#define LINE_SENSOR_ADC_CH 2    // ADC input 2

// ====================== Tunables ======================
// Line calibration (from your Step 1 values; update if lighting changes)
static float LINE_WHITE = 0.06f;   // white floor reflectance
static float LINE_BLACK = 0.85f;   // black line reflectance

// PD steering (gentle defaults)
#define KP_STEER           10.0f
#define KD_STEER           1.2f
#define ERR_LPF_ALPHA      0.18f   // 0..1 (lower = smoother)
#define STEER_GAIN_PWM     1.20f   // converts steer "units" to PWM counts

// Base PWM around mid (we’re not using wheel-speed PID here)
#define PWM_BASE_LEFT     100
#define PWM_BASE_RIGHT    100

// Global loop timing
#define LOOP_DT_MS           10
#define DT_S   ( (float)LOOP_DT_MS / 1000.0f )

// Loss detection
#define LOST_ERR_THRESH     1.05f
#define LOST_TIME_MS         180
#define SEARCH_PWM           300

// Barcode turn
#define TURN_ANGLE_DEG        90.0f
#define TURN_PWM              220

// =========== Local governor (DO NOT change motor.h; this only affects this file) ===========
#define LOCAL_PWM_MIN_L     90
#define LOCAL_PWM_MAX_L     120
#define LOCAL_PWM_MIN_R     90
#define LOCAL_PWM_MAX_R     120
#define LOCAL_MAX_STEP        3     // extra-smooth ramps for line follow only
#define BEND_SLOW_MAX        35     // up to -25 PWM as |ef|→1

// ====================== Globals ======================
extern volatile bool is_scanning_allowed; // from barcode.c
extern char decoded_barcode_char;         // last decoded ASCII

static float e_filt = 0.0f, e_prev = 0.0f;
static int lastL = PWM_MIN_LEFT, lastR = PWM_MIN_RIGHT;
static uint32_t sat_since_ms = 0;

// ====================== Helpers ======================
static inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
static inline int   clampi(int v, int lo, int hi){ return v<lo?lo:(v>hi?hi:v); }
static inline float adc_norm(uint16_t raw){ return (float)raw / 4095.0f; }

static inline float line_error_from_reflect(float r){
    float mid  = 0.5f*(LINE_BLACK + LINE_WHITE);
    float span = fmaxf(0.05f, (LINE_BLACK - LINE_WHITE));
    float e = (r - mid) / (0.5f*span);         // ~[-1,+1]
    return clampf(e, -1.2f, +1.2f);
}

static inline bool is_right_letter(char c){
    // Right: A,C,E,G,I,K,M,O,Q,S,U,W,Y
    switch (c){
        case 'A': case 'C': case 'E': case 'G': case 'I': case 'K':
        case 'M': case 'O': case 'Q': case 'S': case 'U': case 'W': case 'Y':
            return true;
        default: return false;
    }
}
static inline bool is_left_letter(char c){
    // Left: B,D,F,H,J,L,N,P,R,T,V,X,Z
    switch (c){
        case 'B': case 'D': case 'F': case 'H': case 'J': case 'L':
        case 'N': case 'P': case 'R': case 'T': case 'V': case 'X': case 'Z':
            return true;
        default: return false;
    }
}

// ====================== Tasks ======================
static void vBarcodeEventTask(void *pv){
    (void)pv;
    char last = 0;

    for(;;){
        if (decoded_barcode_char != 0 && decoded_barcode_char != last){
            last = decoded_barcode_char;
            printf("[BARCODE] %c\n", last);

            // Pause line-follow scan briefly while we act
            is_scanning_allowed = false;

            // Decide & turn
            if (is_right_letter(last)){
                printf("[ACTION] TURN RIGHT 90\n");
                turn_motor_manual(/*RIGHT*/1, TURN_ANGLE_DEG, TURN_PWM, TURN_PWM);
            } else if (is_left_letter(last)){
                printf("[ACTION] TURN LEFT 90\n");
                turn_motor_manual(/*LEFT*/0, TURN_ANGLE_DEG, TURN_PWM, TURN_PWM);
            } else {
                printf("[ACTION] (no-op for %c)\n", last);
            }

            // Clear decoded char and resume scanning
            decoded_barcode_char = 0;
            is_scanning_allowed = true;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void vLineFollowTask(void *pv){
    (void)pv;

    // --- ADC init (GP28/ADC2) ---
    adc_init();
    adc_gpio_init(LINE_SENSOR_AO);
    adc_select_input(LINE_SENSOR_ADC_CH);

    // Print config once to verify the build uses these numbers
    printf("[CFG] KP=%.2f KD=%.2f G=%.2f ALPHA=%.2f LOCAL[L:%d..%d R:%d..%d] MAXSTEP=%d\n",
           (double)KP_STEER, (double)KD_STEER, (double)STEER_GAIN_PWM,
           (double)ERR_LPF_ALPHA,
           LOCAL_PWM_MIN_L, LOCAL_PWM_MAX_L, LOCAL_PWM_MIN_R, LOCAL_PWM_MAX_R, LOCAL_MAX_STEP);

    TickType_t last_wake = xTaskGetTickCount();

    for(;;){
        // 1) Read reflectance and map to signed error
        uint16_t raw = adc_read();
        float r = adc_norm(raw);
        float e = line_error_from_reflect(r);

        // 2) Filter + derivative
        e_filt = (1.0f - ERR_LPF_ALPHA)*e_filt + ERR_LPF_ALPHA*e;
        float de_dt = (e_filt - e_prev)/DT_S;
        e_prev = e_filt;

        // 3) PD steering (PWM domain) + hard clamp
        float steer_cmd = KP_STEER*e_filt + KD_STEER*de_dt;
        float dPWM = STEER_GAIN_PWM * steer_cmd;

        // Clamp steering to ~30% of the smaller global span
        const int spanL = PWM_MAX_LEFT  - PWM_MIN_LEFT;
        const int spanR = PWM_MAX_RIGHT - PWM_MIN_RIGHT;
        const float dmax = 0.30f * (float)((spanL < spanR) ? spanL : spanR);
        if (dPWM >  dmax) dPWM =  dmax;
        if (dPWM < -dmax) dPWM = -dmax;

        // 3b) Bend-aware base speed (slow down up to BEND_SLOW_MAX when |ef|→1)
        int baseL = PWM_BASE_LEFT;
        int baseR = PWM_BASE_RIGHT;
        int bend_slow = (int)lroundf(BEND_SLOW_MAX * fminf(1.0f, fabsf(e_filt)));
        baseL -= bend_slow;
        baseR -= bend_slow;

        int pwmL = baseL - (int)lroundf(dPWM);
        int pwmR = baseR + (int)lroundf(dPWM);

        // 4) Line-loss detection (sustained saturation of |ef|)
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (fabsf(e_filt) > LOST_ERR_THRESH) {
            if (!sat_since_ms) sat_since_ms = now;
        } else {
            sat_since_ms = 0;
        }
        if (sat_since_ms && (now - sat_since_ms) > LOST_TIME_MS){
            // slow pivot search toward last seen side
            if (e_filt > 0) { pwmL = -SEARCH_PWM; pwmR =  SEARCH_PWM; }
            else            { pwmL =  SEARCH_PWM; pwmR = -SEARCH_PWM; }
        }

        // 5) LOCAL clamps + LOCAL slew (so Demo1 stays unchanged)
        if (pwmL < LOCAL_PWM_MIN_L) pwmL = LOCAL_PWM_MIN_L;
        if (pwmL > LOCAL_PWM_MAX_L) pwmL = LOCAL_PWM_MAX_L;
        if (pwmR < LOCAL_PWM_MIN_R) pwmR = LOCAL_PWM_MIN_R;
        if (pwmR > LOCAL_PWM_MAX_R) pwmR = LOCAL_PWM_MAX_R;

        int dL = pwmL - lastL, dR = pwmR - lastR;
        if (dL >  LOCAL_MAX_STEP) pwmL = lastL + LOCAL_MAX_STEP;
        if (dL < -LOCAL_MAX_STEP) pwmL = lastL - LOCAL_MAX_STEP;
        if (dR >  LOCAL_MAX_STEP) pwmR = lastR + LOCAL_MAX_STEP;
        if (dR < -LOCAL_MAX_STEP) pwmR = lastR - LOCAL_MAX_STEP;

        // 6) Finally, apply global safety clamps from motor.h and drive
        pwmL = clampi(pwmL, PWM_MIN_LEFT,  PWM_MAX_LEFT);
        pwmR = clampi(pwmR, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

        forward_motor_manual(pwmL, pwmR);

        lastL = pwmL; lastR = pwmR;

        // 7) Debug (10 Hz)
        static int div=0;
        if ((div++ % (1000/LOOP_DT_MS/10))==0){
            printf("[LINE] raw=%u r=%.2f e=%.2f ef=%.2f dPWM=%.1f  PWM[L=%d R=%d]\n",
                   raw, (double)r, (double)e, (double)e_filt, (double)dPWM, pwmL, pwmR);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_DT_MS));
    }
}

int main(void){
    stdio_init_all();
    sleep_ms(600);

    // Motors
    motor_init();         // sets up PWM on GP11/10/8/9 and starts PID task
    disable_pid_control();// ensure manual forward() isn’t overridden by pid_task

    // Barcode (GPIO27 interrupts + FreeRTOS worker)
    barcode_init();       // creates semaphore/task; sets is_scanning_allowed=true
    init_barcode_irq();   // attach ISR on IR_SENSOR_PIN (27)

    // Tasks
    xTaskCreate(vLineFollowTask,  "LineFollow",  2048, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vBarcodeEventTask,"BarcodeEvt",  1024, NULL, tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
}