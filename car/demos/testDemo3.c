// ==============================
// demo2.c — IR Line Follow (ADC) + Width Scan on Obstacle
// ==============================
// - Keeps your exact demo2 ADC-only line-follow logic
// - When the ultrasonic at center (90°) detects an obstacle within threshold,
//   it pauses the robot, runs the edge-scan refinement from test_obstacle_width_cosine.c
//   to measure LEFT and RIGHT widths, prints results, then resumes line-follow.
// - Uses your calibrated headers (servo.h, ultrasonic.h, motor/encoder) as-is.
// ==============================

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

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
#include "servo.h"
#include "ultrasonic.h"

// ======== demo2 CONFIG (unchanged) ========
// --- Wi-Fi & MQTT ---
#define WIFI_SSID                 "Keithiphone"
#define WIFI_PASS                 "testong1"
// #define WIFI_SSID                 "Bin10"
// #define WIFI_PASS                 "6370lI2052"
#define WIFI_CONNECT_TIMEOUT_MS   20000

#define BROKER_IP_STR             "172.20.10.2"
#define BROKER_PORT               1883
#define MQTT_TOPIC_TELEM          "pico/demo3/telemetry"
#define MQTT_TOPIC_OBSTACLE       "pico/demo3/obstacle"

#ifndef LINE_SENSOR_PIN
#define LINE_SENSOR_PIN 28           // GPIO28 -> ADC2
#endif

#ifndef FOLLOW_WHITE
#define FOLLOW_WHITE 0
#endif

#ifndef TH_LO
#define TH_LO   600
#endif
#ifndef TH_HI
#define TH_HI   3000
#endif

#ifndef SLOW_SPEED_CMPS
#define SLOW_SPEED_CMPS 2.0f
#endif

#ifndef SEARCH_PWM
#define SEARCH_PWM 50
#endif
#ifndef STEER_DURATION
#define STEER_DURATION 80
#endif
#ifndef DEBOUNCE_DELAY_MS
#define DEBOUNCE_DELAY_MS 100
#endif
#ifndef REVERSE_MS
#define REVERSE_MS 80
#endif
#ifndef REACQUIRE_GOOD_SAMPLES
#define REACQUIRE_GOOD_SAMPLES 3
#endif

// ======== obstacle-width CONFIG ========
#define OBSTACLE_THRESHOLD_CM           25.0f   // Trigger threshold
#define SERVO_CENTER_ANGLE              90.0f
#define SERVO_RIGHT_ANGLE               150.0f
#define SERVO_LEFT_ANGLE                35.0f
#define SERVO_SCAN_STEP_DEGREE          1.0f
#define SERVO_REFINE_STEP_DEGREE        0.5f
#define MEASUREMENT_DELAY_MS            50
#define REFINEMENT_DELAY_MS             2000
#define MAX_DETECTION_DISTANCE_CM       100.0f
#define EDGE_CONFIRMATION_SAMPLES       5
#define CONSISTENCY_THRESHOLD_CM        10.0f

// ============== ADC init & helpers ==============
static inline void init_line_adc(void) {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_select_input(2); // GPIO28 -> ADC2
}

// Hysteresis: return true if sensor is on target color (demo2)
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

// Pivot + poll ADC (demo2v4)
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

// ============== Obstacle width helpers (from test_obstacle_width_cosine.c) ==============

static float calculate_width_component(float adjacent, float hypotenuse) {
    float h2 = hypotenuse * hypotenuse;
    float a2 = adjacent   * adjacent;
    if (h2 <= a2) {
        printf("    [WARNING] Hypotenuse (%.2f) <= adjacent (%.2f), returning 0\n", 
               hypotenuse, adjacent);
        return 0.0f;
    }
    return sqrtf(h2 - a2);
}

static bool take_consistent_samples(float *avg_distance_out) {
    float samples[EDGE_CONFIRMATION_SAMPLES];
    int valid = 0;

    for (int i = 0; i < EDGE_CONFIRMATION_SAMPLES; i++) {
        sleep_ms(100);
        float s = ultrasonic_get_distance_cm();
        if (s > 0 && s <= MAX_DETECTION_DISTANCE_CM) {
            samples[valid++] = s;
            printf("      Sample %d: %.2f cm\n", i + 1, s);
        } else {
            printf("      Sample %d: invalid\n", i + 1);
        }
    }
    if (valid < 3) return false;

    float mn = 1e9f, mx = 0.f, sum = 0.f;
    for (int i = 0; i < valid; i++) {
        sum += samples[i];
        if (samples[i] < mn) mn = samples[i];
        if (samples[i] > mx) mx = samples[i];
    }
    float var = mx - mn;
    *avg_distance_out = sum / valid;
    return (var <= CONSISTENCY_THRESHOLD_CM);
}

static float scan_left_side(float adjacent) {
    printf("\n[LEFT_SCAN] Waiting 2 seconds before scanning...\n");
    sleep_ms(2000);
    printf("[LEFT_SCAN] Starting from 35° scanning toward 90°...\n");
    printf("[LEFT_SCAN] Looking for ECHO (edge detection)...\n");
    printf("[LEFT_SCAN] Angle │ Status\n");
    printf("[LEFT_SCAN] ──────┼────────────────\n");

    float current_angle = SERVO_LEFT_ANGLE;
    bool  edge_found = false;
    float edge_distance = 0.0f;

    while (current_angle <= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(MEASUREMENT_DELAY_MS);

        float d = ultrasonic_get_distance_cm();
        if (d > 0 && d <= MAX_DETECTION_DISTANCE_CM) {
            printf("[LEFT_SCAN] %.1f° │ ✓ ECHO! Confirming...\n", current_angle);
            float avg;
            if (take_consistent_samples(&avg)) {
                printf("[LEFT_SCAN] ✓✓ EDGE FOUND at %.1f°! Starting refinement...\n", current_angle);
                edge_found = true;
                edge_distance = avg;
                break;
            } else {
                printf("[LEFT_SCAN] ✗ Inconsistent, continuing...\n");
            }
        } else {
            printf("[LEFT_SCAN] %.1f° │ No echo\n", current_angle);
        }
        current_angle += SERVO_SCAN_STEP_DEGREE;
    }
    if (!edge_found) { printf("[LEFT_SCAN] No edge found!\n"); return 0.0f; }

    printf("\n[LEFT_REFINE] Starting edge refinement (moving back toward 35°)...\n");
    while (current_angle >= SERVO_LEFT_ANGLE) {
        current_angle -= SERVO_REFINE_STEP_DEGREE;
        servo_set_angle(current_angle);
        printf("[LEFT_REFINE] Testing %.1f° (waiting 2s)...\n", current_angle);
        sleep_ms(REFINEMENT_DELAY_MS);

        float avg;
        if (take_consistent_samples(&avg)) {
            printf("[LEFT_REFINE] ✓ Still edge at %.1f°, avg: %.2f cm\n", current_angle, avg);
            edge_distance = avg;
        } else {
            printf("[LEFT_REFINE] ✗ Edge lost! Final edge at %.1f°\n", current_angle + SERVO_REFINE_STEP_DEGREE);
            current_angle += SERVO_REFINE_STEP_DEGREE;
            break;
        }
    }

    float left_width = calculate_width_component(adjacent, edge_distance);
    printf("[LEFT_REFINE] Final edge angle: %.1f°\n", current_angle);
    printf("[LEFT_REFINE] Final distance: %.2f cm\n", edge_distance);
    printf("[LEFT_REFINE] LEFT width: %.2f cm\n", left_width);
    return left_width;
}

static float scan_right_side(float adjacent) {
    printf("\n[RIGHT_SCAN] Waiting 2 seconds before scanning...\n");
    sleep_ms(2000);
    printf("[RIGHT_SCAN] Starting from 150° scanning toward 90°...\n");
    printf("[RIGHT_SCAN] Looking for ECHO (edge detection)...\n");
    printf("[RIGHT_SCAN] Angle │ Status\n");
    printf("[RIGHT_SCAN] ──────┼────────────────\n");

    float current_angle = SERVO_RIGHT_ANGLE;
    bool  edge_found = false;
    float edge_distance = 0.0f;

    while (current_angle >= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(MEASUREMENT_DELAY_MS);

        float d = ultrasonic_get_distance_cm();
        if (d > 0 && d <= MAX_DETECTION_DISTANCE_CM) {
            printf("[RIGHT_SCAN] %.1f° │ ✓ ECHO! Confirming...\n", current_angle);
            float avg;
            if (take_consistent_samples(&avg)) {
                printf("[RIGHT_SCAN] ✓✓ EDGE FOUND at %.1f°! Starting refinement...\n", current_angle);
                edge_found = true;
                edge_distance = avg;
                break;
            } else {
                printf("[RIGHT_SCAN] ✗ Inconsistent, continuing...\n");
            }
        } else {
            printf("[RIGHT_SCAN] %.1f° │ No echo\n", current_angle);
        }
        current_angle -= SERVO_SCAN_STEP_DEGREE;
    }
    if (!edge_found) { printf("[RIGHT_SCAN] No edge found!\n"); return 0.0f; }

    printf("\n[RIGHT_REFINE] Starting edge refinement (moving back toward 150°)...\n");
    while (current_angle <= SERVO_RIGHT_ANGLE) {
        current_angle += SERVO_REFINE_STEP_DEGREE;
        servo_set_angle(current_angle);
        printf("[RIGHT_REFINE] Testing %.1f° (waiting 2s)...\n", current_angle);
        sleep_ms(REFINEMENT_DELAY_MS);

        float avg;
        if (take_consistent_samples(&avg)) {
            printf("[RIGHT_REFINE] ✓ Still edge at %.1f°, avg: %.2f cm\n", current_angle, avg);
            edge_distance = avg;
        } else {
            printf("[RIGHT_REFINE] ✗ Edge lost! Final edge at %.1f°\n", current_angle - SERVO_REFINE_STEP_DEGREE);
            current_angle -= SERVO_REFINE_STEP_DEGREE;
            break;
        }
    }

    float right_width = calculate_width_component(adjacent, edge_distance);
    printf("[RIGHT_REFINE] Final edge angle: %.1f°\n", current_angle);
    printf("[RIGHT_REFINE] Final distance: %.2f cm\n", edge_distance);
    printf("[RIGHT_REFINE] RIGHT width: %.2f cm\n", right_width);
    return right_width;
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
    ci.client_id  = "pico-demo3";
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

static void run_width_scan_sequence(float adjacent) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║                 *** OBSTACLE DETECTED ***                          ║\n");
    printf("║           Perpendicular Distance: %.2f cm (adjacent)               ║\n", adjacent);
    printf("╚════════════════════════════════════════════════════════════════════╝\n");

    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(500);

    float left_w  = scan_left_side(adjacent);
    sleep_ms(500);
    float right_w = scan_right_side(adjacent);

    float total_w = left_w + right_w;
    printf("\n[RESULT] LEFT=%.2f cm, RIGHT=%.2f cm  →  TOTAL WIDTH=%.2f cm\n",
           left_w, right_w, total_w);
    
    // *** NEW: Publish obstacle width data to MQTT ***
    if (mqtt_is_connected()) {
        char json[200];
        uint32_t ts_ms = to_ms_since_boot(get_absolute_time());
        int n = snprintf(json, sizeof(json),
            "{\"ts\":%u,\"adjacent\":%.2f,\"leftWidth\":%.2f,\"rightWidth\":%.2f,\"totalWidth\":%.2f}",
            (unsigned)ts_ms,
            (double)adjacent,
            (double)left_w,
            (double)right_w,
            (double)total_w);
        
        if (n > 0 && n < (int)sizeof(json)) {
            err_t r = mqtt_publish_str(MQTT_TOPIC_OBSTACLE, json);
            if (r == ERR_OK) {
                printf("[MQTT] Published obstacle data: %s\n", json);
            } else {
                printf("[MQTT] Publish failed, err=%d\n", r);
            }
        }
    } else {
        printf("[MQTT] Not connected, skipping telemetry publish\n");
    }
    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(300);
}

// ============== Line-follow Task (demo2) with obstacle hook ==============
static void lineFollowTask(void *pvParameters) {
    (void)pvParameters;

    int last_turn_dir = 0;
    uint32_t first_ms  = STEER_DURATION;
    uint32_t second_ms = STEER_DURATION + 60;
    bool needs_second  = false;

    printf("[LF] ADC-only line-follow. Speed=%.1f cm/s, TH_LO=%d, TH_HI=%d, FOLLOW_%s\n",
           SLOW_SPEED_CMPS, TH_LO, TH_HI, FOLLOW_WHITE ? "WHITE" : "BLACK");

    uint64_t last_decide = 0;

    for (;;) {
        const uint64_t now = time_us_64();
        if (now - last_decide < (uint64_t)DEBOUNCE_DELAY_MS * 1000ULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        last_decide = now;

        // --- New: quick obstacle check at center ---
        servo_set_angle(SERVO_CENTER_ANGLE);
        float center_cm = ultrasonic_get_distance_cm();
        if (center_cm > 0.0f && center_cm <= OBSTACLE_THRESHOLD_CM) {
            stop_motor_pid();
            run_width_scan_sequence(center_cm);
        }

        uint16_t raw = adc_read();
        bool on_track = adc_on_track(raw);

        if (on_track) {
            forward_motor_pid(SLOW_SPEED_CMPS);
            first_ms  = STEER_DURATION;
            second_ms = STEER_DURATION + 60;
            needs_second = false;

        } else {
            stop_motor_pid();
            reverse_motor_manual(130, 130);
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

    servo_init();
    ultrasonic_init();
    servo_set_angle(SERVO_CENTER_ANGLE);

    g_mqtt_mutex = xSemaphoreCreateMutex();

    // Start network task (Wi-Fi + MQTT reconnect loop)
    xTaskCreate(vNetworkTask, "net", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);

    xTaskCreate(lineFollowTask, "LineFollow",
                configMINIMAL_STACK_SIZE * 5,
                NULL, tskIDLE_PRIORITY + 1, NULL);

    printf("[MAIN] IR line-follow + width-scan ready.\n");
    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
    return 0;
}