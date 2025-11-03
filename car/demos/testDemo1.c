// testdemo1_networked.c
// Patched inner-loop control + Wi-Fi + MQTT telemetry.
// - Signed speed PID, no base bias, simple per-wheel feed-forward.
// - Stronger heading loop defaults (tune at the bottom).
// - Network task handles Wi-Fi + MQTT reconnect; control task publishes JSON.

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"

#include "imu.h"
#include "motor.h"
#include "encoder.h"

/* ============================ User Config ========================= */
// --- Wi-Fi & MQTT ---
#define WIFI_SSID                 "Keithiphone"
#define WIFI_PASS                 "testong1"
// #define WIFI_SSID                 "Bin10"
// #define WIFI_PASS                 "6370lI2052"
#define WIFI_CONNECT_TIMEOUT_MS   20000

#define BROKER_IP_STR             "172.20.10.2"
#define BROKER_PORT               1883
#define MQTT_TOPIC_TELEM          "pico/demo1/telemetry"

// --- Geometry ---
#ifndef TRACK_WIDTH_M
#define TRACK_WIDTH_M        0.115f    // wheel center-to-center (m)
#endif

// --- Per-wheel trims (bias offsets, PWM units) ---
#ifndef WHEEL_TRIM_LEFT
#define WHEEL_TRIM_LEFT      0   // start at 0; tweak after encoder & FF tuning
#endif
#ifndef WHEEL_TRIM_RIGHT
#define WHEEL_TRIM_RIGHT     -5   // start at 0; tweak after encoder & FF tuning
#endif

// --- Target speed ---
#ifndef V_TARGET_MPS
#define V_TARGET_MPS         0.20f
#endif
#define V_TARGET_CMPS        (V_TARGET_MPS * 100.0f)

// --- Heading PID (outer loop) ---
#ifndef KP_HEADING
#define KP_HEADING           0.30f
#endif
#ifndef KI_HEADING
#define KI_HEADING           0.00f   // keep 0 during inner-loop ID
#endif
#ifndef KD_HEADING
#define KD_HEADING           0.18f
#endif
#define HEADING_DEADBAND_DEG 2.0f
#define HDG_EMA_ALPHA        0.20f
#define DEG2RAD              (float)(M_PI / 180.0f)
// While testing, keep offset 0 so captured target ≈ real facing.
#define HEADING_OFFSET_DEG   20.0f
#define HEADING_RATE_SCALE   0.02f   // drop to 0.10 if too aggressive

// --- Wheel-speed inner PID (signed output) ---
#ifndef PWM_MAX_RIGHT
#define PWM_MAX_RIGHT        255
#endif
#ifndef PWM_MAX_LEFT
#define PWM_MAX_LEFT         255
#endif
#ifndef PWM_MIN_LEFT
#define PWM_MIN_LEFT         80
#endif
#ifndef PWM_MIN_RIGHT
#define PWM_MIN_RIGHT        80
#endif

// Allow PID to command both up and down around FF (signed)
#define SPID_OUT_MIN         (-255.0f)
#define SPID_OUT_MAX         (float)(PWM_MAX_RIGHT)
#define SPID_KP              6.0f
#define SPID_KI              0.50f  // slightly lower than before; FF carries most of the load
#define SPID_KD              0.0f
#define SPID_IWIND_CLAMP     300.0f

// --- Straightness PI (bias cleanup only) ---
#define STRAIGHT_KP          1.2f
#define STRAIGHT_KI          0.30f
#define STRAIGHT_I_CLAMP     50.0f

// --- No base PWM; FF takes its place ---
#define BASE_PWM_L           0
#define BASE_PWM_R           0

// --- Slew limiter ---
#define MAX_PWM_STEP         6

// --- Timing ---
#define LOOP_DT_MS           10
#define DT_S                 ((float)LOOP_DT_MS / 1000.0f)

/* ============================ Feed-Forward ========================= */
// Simple linear model per wheel: PWM ≈ kS + kV * v_cmd (v in cm/s)
// Start with these; refine from logs (steady-state PWM vs. measured v)
#ifndef KS_L
#define KS_L 70.0f
#endif
#ifndef KV_L
#define KV_L 3.0f
#endif
#ifndef KS_R
#define KS_R 70.0f
#endif
#ifndef KV_R
#define KV_R 3.0f
#endif

// Gate to disable FF quickly if needed
#ifndef FF_ENABLE
#define FF_ENABLE 1
#endif

/* ============================ Encoder scale (temp) ================= */
// Temporary per-wheel scale (1.0 = no correction). Use the 1 m test to compute these.
#ifndef ENC_SCALE_L
#define ENC_SCALE_L 1.0f
#endif
#ifndef ENC_SCALE_R
#define ENC_SCALE_R 1.0f
#endif

/* ============================ Helpers ============================ */
static inline float clampf(float v, float lo, float hi){ if (v < lo) return lo; if (v > hi) return hi; return v; }
static inline int   clampi(int v, int lo, int hi){ if (v < lo) return lo; if (v > hi) return hi; return v; }
static inline float ema(float prev, float x, float a){ return prev*(1.f - a) + x*a; }
static inline float wrap_deg_pm180(float e){ while (e > 180.f) e -= 360.f; while (e < -180.f) e += 360.f; return e; }
static inline float wrap_deg_0_360(float e){ while (e < 0.f) e += 360.f; while (e >= 360.f) e -= 360.f; return e; }

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
    const float deriv = (err - p->prev_err) / fmaxf(dt, 1e-6f);
    p->prev_err = err;
    p->integ += err * dt;
    p->integ = clampf(p->integ, p->integ_min, p->integ_max);
    const float u = p->kp*err + p->ki*p->integ + p->kd*deriv;
    return clampf(u, p->out_min, p->out_max);
}

/* ============================ Wi-Fi / MQTT ========================= */
static mqtt_client_t *g_mqtt = NULL;
static ip_addr_t      g_broker_ip;
static SemaphoreHandle_t g_mqtt_mutex;

// Add a global flag
static volatile bool g_shutdown = false;

// Call this when you want to exit cleanly (button press, RPC, etc.)
static void mqtt_disconnect_now(void){
    if (!g_mqtt) return;
    // Tell lwIP we're about to touch the stack
    cyw43_arch_lwip_begin();
    mqtt_disconnect(g_mqtt);     // Sends MQTT DISCONNECT then closes TCP
    cyw43_arch_lwip_end();

    // Optional: give time for TCP close handshake
    vTaskDelay(pdMS_TO_TICKS(200));
}

// Example: trigger shutdown from anywhere
static void request_shutdown(void){
    g_shutdown = true;
}

static void mqtt_pub_cb(void *arg, err_t result){ (void)arg; (void)result; }
static void mqtt_conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status){
    (void)client; (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) printf("[MQTT] Connected\n");
    else printf("[MQTT] Disconnected status=%d\n", (int)status);
}
static bool wifi_connect_blocking(uint32_t timeout_ms){
    printf("[NET] Connecting to Wi-Fi SSID: %s\n", WIFI_SSID);
    int err = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS,
                                                 CYW43_AUTH_WPA2_AES_PSK, timeout_ms);
    if (err){ printf("[NET] Wi-Fi connect failed (err=%d)\n", err); return false; }
    printf("[NET] Wi-Fi connected\n"); return true;
}
static bool mqtt_connect_blocking(void){
    if (!g_mqtt) g_mqtt = mqtt_client_new();
    if (!g_mqtt){ printf("[MQTT] client_new failed\n"); return false; }

    struct mqtt_connect_client_info_t ci = {0};
    ci.client_id  = "pico-demo1";
    ci.keep_alive = 30;

    err_t er = mqtt_client_connect(g_mqtt, &g_broker_ip, BROKER_PORT, mqtt_conn_cb, NULL, &ci);
    if (er != ERR_OK){ printf("[MQTT] connect err=%d\n", er); return false; }
    vTaskDelay(pdMS_TO_TICKS(500));
    return mqtt_client_is_connected(g_mqtt);
}
static bool mqtt_is_connected(void){ return g_mqtt && mqtt_client_is_connected(g_mqtt); }
static err_t mqtt_publish_str(const char *topic, const char *payload){
    if (!mqtt_is_connected()) return ERR_CONN;
    if (g_mqtt_mutex) xSemaphoreTake(g_mqtt_mutex, portMAX_DELAY);
    cyw43_arch_lwip_begin();
    err_t r = mqtt_publish(g_mqtt, topic, (const u8_t*)payload, (u16_t)strlen(payload),
                           0 /*qos0*/, 0 /*retain*/, mqtt_pub_cb, NULL);
    cyw43_arch_lwip_end();
    if (g_mqtt_mutex) xSemaphoreGive(g_mqtt_mutex);
    return r;
}
static void vNetworkTask(void *param){
    (void)param;

    if (cyw43_arch_init()){ printf("[NET] cyw43 init failed\n"); vTaskDelete(NULL); }
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
    cyw43_arch_enable_sta_mode();

    while (!wifi_connect_blocking(WIFI_CONNECT_TIMEOUT_MS)){
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ip4_addr_set_u32(ip_2_ip4(&g_broker_ip), ipaddr_addr(BROKER_IP_STR));
    printf("[NET] Broker %s:%d\n", BROKER_IP_STR, BROKER_PORT);

    for (;;){
        if (g_shutdown){
            mqtt_disconnect_now();
            break; // exit task
        }

        if (!mqtt_is_connected()){
            printf("[MQTT] Connecting...\n");
            (void)mqtt_connect_blocking();
        }
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(750));
    }
}

/* ============================ Control Task ============================ */
static void vDriveTask(void *pvParameters) {
    (void)pvParameters;

    // ---- IMU ----
    imu_t imu; imu.i2c = i2c1; imu.i2c_baud = IMU_I2C_BAUD; imu.pin_sda = IMU_SDA_PIN; imu.pin_scl = IMU_SCL_PIN;
    imu.mx_off = imu.my_off = imu.mz_off = 0.f;
    if (!imu_init(&imu)){ printf("[CTRL] IMU init failed\n"); vTaskDelete(NULL); }
    printf("[CTRL] IMU OK\n");

    // ---- Motors & encoders ----
    motor_init();
    encoder_init();
    printf("[CTRL] Motors & Encoders OK  (PWM L[%d..%d] R[%d..%d])\n",
           PWM_MIN_LEFT, PWM_MAX_LEFT, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

    // ---- Wheel speed PIDs ----
    pid_ctrl_t pidL, pidR;
    pid_init(&pidL, SPID_KP, SPID_KI, SPID_KD, SPID_OUT_MIN, SPID_OUT_MAX, -SPID_IWIND_CLAMP, +SPID_IWIND_CLAMP);
    pid_init(&pidR, SPID_KP, SPID_KI, SPID_KD, SPID_OUT_MIN, SPID_OUT_MAX, -SPID_IWIND_CLAMP, +SPID_IWIND_CLAMP);

    // ---- Capture target heading (smoothed) ----
    float filt_hdg = 0.f;
    for (int i = 0; i < 20; ++i){
        float h = imu_update_and_get_heading(&imu);
        h += HEADING_OFFSET_DEG; h = wrap_deg_0_360(h); filt_hdg = ema(filt_hdg, h, HDG_EMA_ALPHA);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    const float target_heading_deg = filt_hdg;
    printf("[CTRL] Target heading = %.1f deg (corrected)\n", (double)target_heading_deg);

    // ---- Controller states ----
    float h_integ = 0.f, h_prev_err = 0.f;
    float s_int   = 0.0f;
    int   lastL   = PWM_MIN_LEFT;
    int   lastR   = PWM_MIN_RIGHT;

    float dist_cm = 0.0f;

    TickType_t last_wake = xTaskGetTickCount();
    int telem_div = 0;

    for (;;) {
        if (g_shutdown){
            forward_motor_manual(0,0);   // stop motors safely
            vTaskDelay(pdMS_TO_TICKS(50));
            vTaskDelete(NULL);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_DT_MS));

        // 1) IMU heading
        float raw_hdg = imu_update_and_get_heading(&imu);
        raw_hdg += HEADING_OFFSET_DEG; raw_hdg = wrap_deg_0_360(raw_hdg);
        filt_hdg = ema(filt_hdg, raw_hdg, HDG_EMA_ALPHA);

        float h_err = wrap_deg_pm180(target_heading_deg - filt_hdg);
        if (fabsf(h_err) < HEADING_DEADBAND_DEG) h_err = 0.f;
        float h_deriv = (h_err - h_prev_err) / DT_S; h_prev_err = h_err;
        if (KI_HEADING > 0.f){
            const float h_iw = 150.0f / fmaxf(KI_HEADING, 1e-6f);
            h_integ += h_err * DT_S; h_integ = clampf(h_integ, -h_iw, +h_iw);
        }
        float delta_heading_rate_deg_s = KP_HEADING * h_err + KI_HEADING * h_integ + KD_HEADING * h_deriv;
        float delta_w = delta_heading_rate_deg_s * DEG2RAD * HEADING_RATE_SCALE; // rad/s

        // 2) Nominal speed (cm/s)
        const float v_cmd_cmps = V_TARGET_CMPS;

        // 3) (v, delta_w) -> left/right targets (cm/s)
        float diff_cmps = (0.5f * TRACK_WIDTH_M * delta_w) * 100.0f;
        float vL_target = v_cmd_cmps - diff_cmps;
        float vR_target = v_cmd_cmps + diff_cmps;

        // 4) Measure speeds (cm/s)
        float vL_meas = ENC_SCALE_L * get_left_speed();
        float vR_meas = ENC_SCALE_R * get_right_speed();
        static float vL_prev = 0.0f, vR_prev = 0.0f;
        if (vL_meas < 0.0f) vL_meas = vL_prev; else vL_prev = vL_meas;
        if (vR_meas < 0.0f) vR_meas = vR_prev; else vR_prev = vR_meas;

        // integrate distance (cm)
        float v_avg = 0.5f * (vL_meas + vR_meas);
        dist_cm += v_avg * DT_S;

        // 5) Wheel speed PIDs -> delta PWM (signed)
        float uL = pid_update(&pidL, (vL_target - vL_meas), DT_S);
        float uR = pid_update(&pidR, (vR_target - vR_meas), DT_S);

        // 6) Straightness PI
        float s_err = (vR_meas - vL_meas);
        s_int += s_err * DT_S; s_int = clampf(s_int, -STRAIGHT_I_CLAMP, +STRAIGHT_I_CLAMP);
        float s_trim = STRAIGHT_KP * s_err + STRAIGHT_KI * s_int;

        // 7) Compose PWM = FF + PID + trims
        float ffL = FF_ENABLE ? (KS_L + KV_L * vL_target) : 0.0f;
        float ffR = FF_ENABLE ? (KS_R + KV_R * vR_target) : 0.0f;
        int pwmL = (int)lroundf(ffL + uL + WHEEL_TRIM_LEFT  + s_trim);
        int pwmR = (int)lroundf(ffR + uR + WHEEL_TRIM_RIGHT - s_trim);

        // clamp & slew
        pwmL = clampi(pwmL, PWM_MIN_LEFT, PWM_MAX_LEFT);
        pwmR = clampi(pwmR, PWM_MIN_RIGHT, PWM_MAX_RIGHT);
        if (pwmL > lastL + MAX_PWM_STEP) pwmL = lastL + MAX_PWM_STEP; else if (pwmL < lastL - MAX_PWM_STEP) pwmL = lastL - MAX_PWM_STEP;
        if (pwmR > lastR + MAX_PWM_STEP) pwmR = lastR + MAX_PWM_STEP; else if (pwmR < lastR - MAX_PWM_STEP) pwmR = lastR - MAX_PWM_STEP;
        lastL = pwmL; lastR = pwmR;

        // 8) Apply motors
        forward_motor_manual(pwmL, pwmR);

        // 9) Telemetry: USB + MQTT JSON (~5 Hz)
        if ((telem_div++ % (1000/LOOP_DT_MS/5)) == 0) {
            float herr_abs = fabsf(h_err);
            printf("[CTRL] hdg=%.1f(raw=%.1f) herr=%.2f  v=%.1f  "
                   "vL[t/m]=%.2f/%.2f  vR[t/m]=%.2f/%.2f  dist=%.1f  "
                   "s_err=%.2f s_int=%.2f str=%.2f  FF[L=%.0f R=%.0f] PID[L=%.0f R=%.0f] PWM[L=%d R=%d]\n",
                   (double)filt_hdg, (double)raw_hdg, (double)herr_abs, (double)v_cmd_cmps,
                   (double)vL_target, (double)vL_meas,
                   (double)vR_target, (double)vR_meas,
                   (double)dist_cm,
                   (double)s_err, (double)s_int, (double)s_trim,
                   (double)ffL, (double)ffR,
                   (double)uL, (double)uR,
                   pwmL, pwmR);

            if (mqtt_is_connected()){
                char json[224];
                uint32_t ts_ms = to_ms_since_boot(get_absolute_time());
                int n = snprintf(json, sizeof(json),
                    "{\"ts\":%u,\"vL\":%.2f,\"vR\":%.2f,\"vAvg\":%.2f,"
                    "\"dist\":%.1f,\"hdgRaw\":%.1f,\"hdg\":%.1f,"
                    "\"pwmL\":%d,\"pwmR\":%d}",
                    (unsigned)ts_ms,
                    (double)vL_meas, (double)vR_meas, (double)v_avg,
                    (double)dist_cm, (double)raw_hdg, (double)filt_hdg,
                    pwmL, pwmR);
                if (n > 0 && n < (int)sizeof(json)) {
                    err_t r = mqtt_publish_str(MQTT_TOPIC_TELEM, json);
                    if (r != ERR_OK) printf("[MQTT] publish err=%d\n", r);
                }
            }
        }
    }
}

/* ============================ Main ============================ */
int main(void){
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(10000);

    printf("\n[BOOT] demo1_networked: HeadingPID(%.2f/%.2f/%.2f)  SpeedPID(%.2f/%.2f/%.2f)\n",
           (double)KP_HEADING, (double)KI_HEADING, (double)KD_HEADING,
           (double)SPID_KP, (double)SPID_KI, (double)SPID_KD);

    g_mqtt_mutex = xSemaphoreCreateMutex();

    // Start network task (Wi-Fi + MQTT reconnect loop)
    xTaskCreate(vNetworkTask, "net", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);

    // Start control loop task
    xTaskCreate(vDriveTask, "drive", 4096, NULL, tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
}
