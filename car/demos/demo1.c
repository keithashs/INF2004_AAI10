#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "config.h"


/* ============================ Helpers ============================ */
static inline float clampf(float v, float lo, float hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline int clampi(int v, int lo, int hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline float ema(float prev, float x, float a){
    return prev*(1.f - a) + x*a;
}
static inline float wrap_deg_pm180(float e){
    while (e > 180.f) e -= 360.f;
    while (e < -180.f) e += 360.f;
    return e;
}
static inline float wrap_deg_0_360(float e){
    while (e < 0.f) e += 360.f;
    while (e >= 360.f) e -= 360.f;
    return e;
}

/* ============================ Simple PID ============================ */
typedef struct {
    float kp, ki, kd;
    float integ, prev_err;
    float out_min, out_max;
    float integ_min, integ_max;
} pid_ctrl_t;

static inline void pid_init(pid_ctrl_t* p, float kp, float ki, float kd,
                            float out_min, float out_max, float integ_min, float integ_max){
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integ = 0.f; p->prev_err = 0.f;
    p->out_min = out_min; p->out_max = out_max;
    p->integ_min = integ_min; p->integ_max = integ_max;
}
static inline float pid_update(pid_ctrl_t* p, float err, float dt){
    p->integ += err * dt;
    p->integ = clampf(p->integ, p->integ_min, p->integ_max);
    float deriv = (err - p->prev_err) / dt;
    p->prev_err = err;
    float u = p->kp * err + p->ki * p->integ + p->kd * deriv;
    return clampf(u, p->out_min, p->out_max);
}

// ================= FreeRTOS Task =====================
static void vDriveTask(void *pvParameters) {
    // ---- IMU ----
    imu_t imu;
    imu.i2c      = i2c1;
    imu.i2c_baud = IMU_I2C_BAUD;
    imu.pin_sda  = IMU_SDA_PIN;
    imu.pin_scl  = IMU_SCL_PIN;
    if (!imu_init(&imu)) {
        printf("[CTRL] IMU init failed\n");
        vTaskDelete(NULL);
    }

    imu_set_mag_offsets(&imu, -191.5f, -71.0f, 87.0f); // IMU calibration
    printf("[CTRL] IMU OK\n");

    // ---- Motors & encoders ----
    motor_init();
    encoder_init();
    printf("[CTRL] Motors & Encoders OK  (PWM L[%d..%d] R[%d..%d])\n",
           PWM_MIN_LEFT, PWM_MAX_LEFT, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

    // ---- Inner (wheel) speed PIDs ----
    pid_ctrl_t pidL, pidR;
    pid_init(&pidL, SPID_KP, SPID_KI, SPID_KD,
             SPID_OUT_MIN, SPID_OUT_MAX, -SPID_IWIND_CLAMP, +SPID_IWIND_CLAMP);
    pid_init(&pidR, SPID_KP, SPID_KI, SPID_KD,
             SPID_OUT_MIN, SPID_OUT_MAX, -SPID_IWIND_CLAMP, +SPID_IWIND_CLAMP);

    // ---- Capture target heading (filtered) ----
    float filt_hdg = 0.f;
    for (int i = 0; i < 20; ++i) {
        float h = imu_update_and_get_heading(&imu);
        h += HEADING_OFFSET_DEG;
        h = wrap_deg_0_360(h);
        filt_hdg = ema(filt_hdg, h, 0.05f);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    const float target_heading_deg = filt_hdg;
    printf("[CTRL] Target heading = %.1f deg (corrected)\n", (double)target_heading_deg);

    // ---- Controllers' state ----
    float h_integ = 0.f, h_prev_err = 0.f; // outer IMU PID
    float s_int   = 0.0f;                  // straightness PI
    int   lastL   = PWM_MIN_LEFT;
    int   lastR   = PWM_MIN_RIGHT;

    // ---- Telemetry running state ----
    float dist_cm = 0.0f;

    // ---- Task timing ----
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // === 1) IMU heading (raw + filtered) ===
        float raw_hdg = imu_update_and_get_heading(&imu);
        raw_hdg += HEADING_OFFSET_DEG;
        raw_hdg = wrap_deg_0_360(raw_hdg);
        filt_hdg = ema(filt_hdg, raw_hdg, HDG_EMA_ALPHA);

        // Outer IMU PID -> delta_w (rad/s)
        float h_err = wrap_deg_pm180(target_heading_deg - filt_hdg);
        if (fabsf(h_err) < HEADING_DEADBAND_DEG) h_err = 0.f;

        float h_deriv = (h_err - h_prev_err) / DT_S;
        h_prev_err = h_err;

        if (KI_HEADING > 0.f){
            float h_iw = 150.0f / fmaxf(KI_HEADING, 1e-6f);
            h_integ += h_err * DT_S;
            h_integ = clampf(h_integ, -h_iw, +h_iw);
        }

        float delta_heading_rate_deg_s =
            KP_HEADING * h_err + KI_HEADING * h_integ + KD_HEADING * h_deriv;

        float delta_w = delta_heading_rate_deg_s * DEG2RAD * HEADING_RATE_SCALE;

        // === 2) Command v (cm/s) ===
        const float v_cmd_cmps = V_TARGET_CMPS;

        // === 3) (v, delta_w) -> left/right targets (cm/s) ===
        float diff_cmps = (0.5f * TRACK_WIDTH_M * delta_w) * 100.0f;
        float vL_target = v_cmd_cmps - diff_cmps;
        float vR_target = v_cmd_cmps + diff_cmps;

        // (4) Measured speeds (cm/s)
        float vL_meas = get_left_speed();
        float vR_meas = get_right_speed();
        static float vL_prev = 0.0f, vR_prev = 0.0f;
        if (vL_meas < 0.0f) vL_meas = vL_prev; else vL_prev = vL_meas;
        if (vR_meas < 0.0f) vR_meas = vR_prev; else vR_prev = vR_meas;

        // integrate distance (cm)
        float v_avg = 0.5f * (vL_meas + vR_meas);
        dist_cm += v_avg * DT_S;

        // (5) Inner wheel speed PIDs
        float uL = pid_update(&pidL, vL_target - vL_meas, DT_S);
        float uR = pid_update(&pidR, vR_target - vR_meas, DT_S);

        // (6) Straightness PI using speed mismatch
        float s_err = (vR_meas - vL_meas);
        s_int += s_err * DT_S;
        s_int = clampf(s_int, -STRAIGHT_I_CLAMP, STRAIGHT_I_CLAMP);
        float s_trim = STRAIGHT_KP * s_err + STRAIGHT_KI * s_int;

        // (7) Final PWM = BASE + PID delta ± straightness trim
        int pwmL = (int)lroundf(BASE_PWM_L + uL + WHEEL_TRIM_LEFT  + s_trim);
        int pwmR = (int)lroundf(BASE_PWM_R + uR + WHEEL_TRIM_RIGHT - s_trim);
        pwmL = clampi(pwmL, PWM_MIN_LEFT,  PWM_MAX_LEFT);
        pwmR = clampi(pwmR, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

        // (8) Slew limit
        int dL = pwmL - lastL, dR = pwmR - lastR;
        if (dL >  MAX_PWM_STEP) pwmL = lastL + MAX_PWM_STEP;
        if (dL < -MAX_PWM_STEP) pwmL = lastL - MAX_PWM_STEP;
        if (dR >  MAX_PWM_STEP) pwmR = lastR + MAX_PWM_STEP;
        if (dR < -MAX_PWM_STEP) pwmR = lastR - MAX_PWM_STEP;

        forward_motor_manual(pwmL, pwmR);
        lastL = pwmL; lastR = pwmR;

        // (9) Telemetry (~5 Hz)
        static int div = 0;
        if (++div >= (1000/LOOP_DT_MS/5)) {
            div = 0;
            printf("[CTRL] hdg=%.1f(raw=%.1f) herr=%.2f dW=%.3f  v=%.1f  "
                   "vL[t/m]=%.2f/%.2f  vR[t/m]=%.2f/%.2f  dist=%.1f  "
                   "s_err=%.2f s_int=%.2f str=%.2f  PWM[L=%d R=%d]\n",
                   (double)filt_hdg, (double)raw_hdg, (double)h_err, (double)delta_w,
                   (double)v_cmd_cmps,
                   (double)vL_target, (double)vL_meas,
                   (double)vR_target, (double)vR_meas,
                   (double)dist_cm,
                   (double)s_err, (double)s_int, (double)s_trim,
                   pwmL, pwmR);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_DT_MS));
    }
}

// ================= Main =================
int main(void) {
    stdio_init_all();
    sleep_ms(15000);

    printf("\n[BOOT] Demo1 single-file: FreeRTOS + MQTT. "
           "HeadingPID(%.2f/%.2f/%.2f)  SpeedPID(%.2f/%.2f/%.2f)\n",
           (double)KP_HEADING, (double)KI_HEADING, (double)KD_HEADING,
           (double)SPID_KP, (double)SPID_KI, (double)SPID_KD);

    xTaskCreate(vDriveTask, "DriveTask", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
    vTaskStartScheduler();

    // If scheduler returns
    while (true) {
        printf("Scheduler failed!\n");
        sleep_ms(1000);
    }
}