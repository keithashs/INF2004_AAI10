// ==============================
// IR Line Follow + Wheel Trim
// ==============================
// - Primary sensor: ADC on GPIO28 (ADC2)
// - Hysteresis thresholds: TH_LO, TH_HI
// - Follows BLACK by default; set FOLLOW_WHITE=1 to follow white
// - Integrates WHEEL_TRIM from straight calibration to maintain straightness
// ==============================

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"

#include "motor.h"
#include "encoder.h"

// ======== CONFIG ========
// --- Wi-Fi & MQTT ---
#define WIFI_SSID                 "Keithiphone"
#define WIFI_PASS                 "testong1"
#define WIFI_CONNECT_TIMEOUT_MS   20000

#define BROKER_IP_STR             "172.20.10.2"
#define BROKER_PORT               1883
#define MQTT_TOPIC_TELEM          "pico/demo2/telemetry"

#ifndef LINE_SENSOR_PIN
#define LINE_SENSOR_PIN 28           // GPIO28 -> ADC2
#endif

// Follow target color: 0 = BLACK on white floor (default), 1 = WHITE on black
#ifndef FOLLOW_WHITE
#define FOLLOW_WHITE 0
#endif

// Hysteresis thresholds (tune to your floor/tape)
#ifndef TH_LO
#define TH_LO   600      // definitely WHITE
#endif
#ifndef TH_HI
#define TH_HI   3000     // definitely BLACK
#endif

// Crawl speed for PID
#ifndef SLOW_SPEED_CMPS
#define SLOW_SPEED_CMPS 2.0f // drifts left line increase | drifts right negative
#endif

// Straightness correction (from your demo2.c tuning)
// +WHEEL_TRIM => LEFT slightly faster / RIGHT slightly slower (helps when car drifts LEFT)
// -WHEEL_TRIM => RIGHT faster / LEFT slower (helps when car drifts RIGHT)
#ifndef WHEEL_TRIM
#define WHEEL_TRIM (+0.02f)
#endif

// Search behaviour
#ifndef SEARCH_PWM
#define SEARCH_PWM 60
#endif
#ifndef STEER_DURATION
#define STEER_DURATION 80
#endif
#ifndef DEBOUNCE_DELAY_MS
#define DEBOUNCE_DELAY_MS 80
#endif
#ifndef REVERSE_MS
#define REVERSE_MS 80
#endif
#ifndef REACQUIRE_GOOD_SAMPLES
#define REACQUIRE_GOOD_SAMPLES 5
#endif

// ======== ADC init & helpers ========
static inline void init_line_adc(void) {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_select_input(2); // GPIO28 -> ADC2
}

// Hysteresis: return true if main sensor is on the target color
static inline bool adc_on_track(uint16_t v) {
    static bool state = false;
#if FOLLOW_WHITE
    if (v <= TH_LO)      state = true;    // on white
    else if (v >= TH_HI) state = false;   // off (black)
#else
    if (v >= TH_HI)      state = true;    // on black
    else if (v <= TH_LO) state = false;   // off (white)
#endif
    return state;
}

// ======== Trimmed forward helper ========
// Applies your calibrated WHEEL_TRIM to manual PWM drive
static inline uint16_t clamp_u16(int v, int lo, int hi){
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return (uint16_t)v;
}

static void forward_with_trim_manual(int basePWM){
    const int spanL = (PWM_MAX_LEFT  - PWM_MIN_LEFT);
    const int spanR = (PWM_MAX_RIGHT - PWM_MIN_RIGHT);
    int trimL = (int)( WHEEL_TRIM * 0.5f * spanL);
    int trimR = (int)(-WHEEL_TRIM * 0.5f * spanR);
    uint16_t pwmL = clamp_u16(basePWM + trimL, PWM_MIN_LEFT,  PWM_MAX_LEFT);
    uint16_t pwmR = clamp_u16(basePWM + trimR, PWM_MIN_RIGHT, PWM_MAX_RIGHT);
    forward_motor_manual(pwmL, pwmR);
}

// Pivot + poll ADC; exit early when we’ve seen N consecutive "on-track"
static bool search_pivot_and_probe(bool want_left, uint32_t ms) {
    disable_pid_control(); // manual control during search
    turn_motor_manual(want_left ? 0 /*LEFT*/ : 1 /*RIGHT*/,
                      CONTINUOUS_TURN, SEARCH_PWM, SEARCH_PWM);

    const uint64_t start = time_us_64();
    const uint64_t limit = (uint64_t)ms * 1000ULL;
    int good = 0;

    while ((time_us_64() - start) < limit) {
        uint16_t raw = adc_read();
        if (adc_on_track(raw)) {
            if (++good >= REACQUIRE_GOOD_SAMPLES) {
                stop_motor_pid();
                return true;
            }
        } else {
            good = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    stop_motor_pid();
    vTaskDelay(pdMS_TO_TICKS(40));
    return false;
}

/* ============================ Wi-Fi / MQTT ========================= */
static mqtt_client_t *g_mqtt = NULL;
static ip_addr_t      g_broker_ip;
static SemaphoreHandle_t g_mqtt_mutex;

// Shutdown flag for clean exit
static volatile bool g_shutdown = false;

// Clean disconnect
static void mqtt_disconnect_now(void){
    if (!g_mqtt) return;
    cyw43_arch_lwip_begin();
    mqtt_disconnect(g_mqtt);
    cyw43_arch_lwip_end();
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void request_shutdown(void){
    g_shutdown = true;
}

static void mqtt_pub_cb(void *arg, err_t result){ 
    (void)arg; (void)result; 
}

static void mqtt_conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status){
    (void)client; (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) 
        printf("[MQTT] Connected\n");
    else 
        printf("[MQTT] Disconnected status=%d\n", (int)status);
}

static bool wifi_connect_blocking(uint32_t timeout_ms){
    printf("[NET] Connecting to Wi-Fi SSID: %s\n", WIFI_SSID);
    int err = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS,
                                                 CYW43_AUTH_WPA2_AES_PSK, timeout_ms);
    if (err){ 
        printf("[NET] Wi-Fi connect failed (err=%d)\n", err); 
        return false; 
    }
    printf("[NET] Wi-Fi connected\n"); 
    return true;
}

static bool mqtt_connect_blocking(void){
    if (!g_mqtt) g_mqtt = mqtt_client_new();
    if (!g_mqtt){ 
        printf("[MQTT] client_new failed\n"); 
        return false; 
    }

    struct mqtt_connect_client_info_t ci = {0};
    ci.client_id  = "pico-demo2";
    ci.keep_alive = 30;

    err_t er = mqtt_client_connect(g_mqtt, &g_broker_ip, BROKER_PORT, 
                                   mqtt_conn_cb, NULL, &ci);
    if (er != ERR_OK){ 
        printf("[MQTT] connect err=%d\n", er); 
        return false; 
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    return mqtt_client_is_connected(g_mqtt);
}

static bool mqtt_is_connected(void){ 
    return g_mqtt && mqtt_client_is_connected(g_mqtt); 
}

static err_t mqtt_publish_str(const char *topic, const char *payload){
    if (!mqtt_is_connected()) return ERR_CONN;
    if (g_mqtt_mutex) xSemaphoreTake(g_mqtt_mutex, portMAX_DELAY);
    cyw43_arch_lwip_begin();
    err_t r = mqtt_publish(g_mqtt, topic, (const u8_t*)payload, 
                          (u16_t)strlen(payload),
                          0 /*qos0*/, 0 /*retain*/, mqtt_pub_cb, NULL);
    cyw43_arch_lwip_end();
    if (g_mqtt_mutex) xSemaphoreGive(g_mqtt_mutex);
    return r;
}

static void vNetworkTask(void *param){
    (void)param;

    if (cyw43_arch_init()){ 
        printf("[NET] cyw43 init failed\n"); 
        vTaskDelete(NULL); 
    }
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
            break;
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
    vTaskDelete(NULL);
}

// ======== Line-follow Task (ADC-only) ========
static void lineFollowTask(void *pvParameters) {
    (void)pvParameters;

    int last_turn_dir = 0;        // 0 = Left, 1 = Right (remember last success)
    uint32_t first_ms  = STEER_DURATION;
    uint32_t second_ms = STEER_DURATION + 60;
    bool needs_second  = false;

    printf("[LF] ADC-only line-follow + WHEEL_TRIM = %+.3f\n", (double)WHEEL_TRIM);
    printf("     Speed=%.1f cm/s, TH_LO=%d, TH_HI=%d, FOLLOW_%s\n",
           SLOW_SPEED_CMPS, TH_LO, TH_HI, FOLLOW_WHITE ? "WHITE" : "BLACK");

    uint64_t last_decide = 0;
    int telem_counter = 0;  // For periodic telemetry

    for (;;) {
        const uint64_t now = time_us_64();
        if (now - last_decide < (uint64_t)DEBOUNCE_DELAY_MS * 1000ULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        last_decide = now;

        uint16_t raw = adc_read();
        bool on_track = adc_on_track(raw);

        // Normal follow: apply PID drive + small trim bias
            float vL = get_left_speed();
            float vR = get_right_speed();

        if (on_track) {
            if (vL <= 0 || vR <= 0) {
                // PID start assist: manual PWM crawl with trim
                forward_with_trim_manual(PWM_MIN_LEFT + 25);
            } else {
                forward_motor_pid(SLOW_SPEED_CMPS);
            }

            first_ms  = STEER_DURATION;
            second_ms = STEER_DURATION + 60;
            needs_second = false;

        } else {
            // LOST: back off, then search with trim still applied
            stop_motor_pid();
            reverse_motor_manual(100, 100);
            vTaskDelay(pdMS_TO_TICKS(REVERSE_MS));

            int prefer_dir = last_turn_dir;
            bool found = false;
            if (!needs_second) {
                found = search_pivot_and_probe(prefer_dir == 0, first_ms);
                needs_second = !found;
                if (found) last_turn_dir = prefer_dir;
            } else {
                int alt_dir = 1 - prefer_dir;
                found = search_pivot_and_probe(alt_dir == 0, second_ms);
                needs_second = false;
                if (found) last_turn_dir = alt_dir;
            }

            if (!found) {
                if (first_ms  < 500) first_ms  += 70;
                if (second_ms < 520) second_ms += 70;
            }
        }
        // Publish telemetry every ~10 iterations (~300ms)
        if ((telem_counter++ % 10) == 0) {
            const char *state_str = on_track ? "TRACK" : "SEARCH";
            
            printf("[LF] ADC=%u %s vL=%.1f vR=%.1f trim=%+.3f\n",
                   raw, state_str, (double)vL, (double)vR, (double)WHEEL_TRIM);

            if (mqtt_is_connected()){
                char json[256];
                uint32_t ts_ms = to_ms_since_boot(get_absolute_time());
                int n = snprintf(json, sizeof(json),
                    "{\"ts\":%u,\"adc\":%u,\"onTrack\":%s,"
                    "\"vL\":%.2f,\"vR\":%.2f,\"state\":\"%s\","
                    "\"trim\":%.3f,\"speed\":%.1f,\"followColor\":\"%s\"}",
                    (unsigned)ts_ms,
                    raw,
                    on_track ? "true" : "false",
                    (double)vL, (double)vR,
                    state_str,
                    (double)WHEEL_TRIM,
                    (double)SLOW_SPEED_CMPS,
                    FOLLOW_WHITE ? "WHITE" : "BLACK");
                
                if (n > 0 && n < (int)sizeof(json)) {
                    err_t r = mqtt_publish_str(MQTT_TOPIC_TELEM, json);
                    if (r != ERR_OK) {
                        printf("[MQTT] publish err=%d\n", r);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// ======== Main ========
int main(void) {
    stdio_init_all();
    sleep_ms(400);

    init_line_adc();
    encoder_init();
    motor_init();

    g_mqtt_mutex = xSemaphoreCreateMutex();

    // Start network task (Wi-Fi + MQTT reconnect loop)
    xTaskCreate(vNetworkTask, "net", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);

    // Start line follow task
    xTaskCreate(lineFollowTask, "LineFollow",
                configMINIMAL_STACK_SIZE * 4,
                NULL, tskIDLE_PRIORITY + 1, NULL);

    printf("[MAIN] IR line-follow (ADC-only) with straightness trim ready.\n");
    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
    return 0;
}
