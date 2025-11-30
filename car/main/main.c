// ==============================
// Integrated Edge Following + Obstacle Detection (TURN AFTER SCAN)
// - demo2.c: IR Edge Following with Aggressive Corner Response
// - testDemo3.c: Ultrasonic obstacle detection + width scan
// - NEW: Turn 90 degrees right after obstacle scan
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
#include "ir_linefollow.h"
#include "imu.h"

// ======== CONFIG ========
// --- Wi-Fi & MQTT ---
#define WIFI_SSID                 "Keithiphone"
#define WIFI_PASS                 "testong1"
// #define WIFI_SSID                 "Jared"
// #define WIFI_PASS                 "1teddygodie"
// #define WIFI_SSID                 "Oppo"
// #define WIFI_PASS                 "happy1234"
#define WIFI_CONNECT_TIMEOUT_MS   20000

#define BROKER_IP_STR             "172.20.10.3"
// #define BROKER_IP_STR             "10.22.173.48"
// #define BROKER_IP_STR             "10.86.216.48"
#define BROKER_PORT               1883
#define MQTT_TOPIC_TELEM          "pico/main/telemetry"
#define MQTT_TOPIC_STATUS         "pico/main/status"
#define MQTT_TOPIC_SENSOR         "pico/main/sensor"
#define MQTT_TOPIC_MOTOR          "pico/main/motor"
#define MQTT_TOPIC_OBSTACLE       "pico/main/obstacle"


// ======== SERVO SCAN ANGLES ========
#define SERVO_CENTER_ANGLE          90.0f
#define SERVO_RIGHT_ANGLE           135.0f
#define SERVO_LEFT_ANGLE            10.0f

// ======== OBSTACLE DETECTION CONFIG ========
#define OBSTACLE_THRESHOLD_CM       20.0f
#define OBSTACLE_CHECK_INTERVAL     5

// ======== GENTLE LINE SEARCH CONFIG ========
#define GENTLE_TURN_PWM_LEFT        90
#define GENTLE_TURN_PWM_RIGHT       125
#define GENTLE_SEARCH_MAX_TIME_MS   20000
#define LINE_EDGE_MIN_ADC           800    // Lowered for earlier detection
#define LINE_EDGE_MAX_ADC           2500
#define LINE_CAPTURE_DURATION_MS    4000   // Moderate duration to prevent overshoot
#define LINE_CAPTURE_BASE_PWM       95     // Very low base speed for precise control
#define TRANSITION_DURATION_MS      2000   // Smooth ramp-up after capture

// ======== HEADING FILTER CONFIG (IMU) ========
#define HDG_EMA_ALPHA        0.20f
#define HEADING_OFFSET_DEG   0.0f   // keep 0 so logged heading ≈ real facing

/* ============================ Wi-Fi / MQTT ========================= */
static mqtt_client_t *g_mqtt = NULL;
static ip_addr_t      g_broker_ip;
static SemaphoreHandle_t g_mqtt_mutex;
static volatile bool g_shutdown = false;

static int8_t g_current_edge_direction = EDGE_DIRECTION;
static float  g_error_history_sum = 0.0f;
static int    g_error_history_count = 0;

// Avoidance cooldown to prevent snapping back to old line
static volatile int g_avoidance_cooldown_steps = 0;

// ======== IMU GLOBALS ========
static imu_t g_imu;
static bool  g_imu_ok       = false;
static float g_hdg_deg      = 0.0f;  // filtered heading (deg 0..360)
static float g_hdg_raw_deg  = 0.0f;  // raw heading before EMA

// ======== Small helpers ========
static inline float wrap_deg_0_360(float e){
    while (e < 0.0f)   e += 360.0f;
    while (e >= 360.0f) e -= 360.0f;
    return e;
}
static inline float ema(float prev, float x, float a){
    return prev*(1.0f - a) + x*a;
}

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
    ci.client_id  = "pico-main";
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
    u16_t len = strlen(payload);
    err_t r = mqtt_publish(g_mqtt, topic, (const u8_t*)payload,
                          (u16_t)len, 0, 0, mqtt_pub_cb, NULL);
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

/* ========== Gentle Left-Turning Arc Search for Line ========== */

static void gentle_search_for_line(void) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════╗\n");
    printf("║  GENTLE LEFT-TURNING ARC SEARCH FOR LINE         ║\n");
    printf("╚════════════════════════════════════════════════════╝\n");

    disable_pid_control();

    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    bool found = false;

    uint16_t baseline_adc = read_line_adc();
    printf("[GENTLE] Baseline ADC: %u (white surface)\n", baseline_adc);
    printf("[GENTLE] Target ADC: %d (black line edge)\n", TARGET_EDGE_VALUE);
    printf("[GENTLE] Left motor PWM: %d, Right motor PWM: %d\n",
           GENTLE_TURN_PWM_LEFT, GENTLE_TURN_PWM_RIGHT);

    // Publish search start event
    char msg[256];
    snprintf(msg, sizeof(msg),
             "{\"event\":\"gentle_search_start\",\"baseline_adc\":%u,\"target_adc\":%d,"
             "\"pwm_left\":%d,\"pwm_right\":%d,\"max_time_ms\":%u}",
             baseline_adc, TARGET_EDGE_VALUE,
             GENTLE_TURN_PWM_LEFT, GENTLE_TURN_PWM_RIGHT,
             GENTLE_SEARCH_MAX_TIME_MS);
    mqtt_publish_str(MQTT_TOPIC_TELEM, msg);

    int reading_count = 0;

    printf("[GENTLE] Starting gentle left arc search...\n");

    while ((to_ms_since_boot(get_absolute_time()) - start_ms) < GENTLE_SEARCH_MAX_TIME_MS) {
        uint16_t adc_raw = read_line_adc();

        int deviation_from_white = abs((int)adc_raw - (int)baseline_adc);

        bool on_edge = (adc_raw >= LINE_EDGE_MIN_ADC && adc_raw <= LINE_EDGE_MAX_ADC);
        bool approaching_line = (adc_raw >= LINE_EDGE_MIN_ADC);

        if (reading_count++ % 10 == 0) {
            printf("[GENTLE] t=%lums ADC=%u deviation=%d approaching=%s on_edge=%s\n",
                   (unsigned long)(to_ms_since_boot(get_absolute_time()) - start_ms),
                   adc_raw, deviation_from_white,
                   approaching_line ? "YES" : "no",
                   on_edge ? "YES" : "no");

            snprintf(msg, sizeof(msg),
                     "{\"event\":\"gentle_search\",\"time_ms\":%lu,\"adc\":%u,"
                     "\"deviation\":%d,\"approaching\":%s,\"on_edge\":%s}",
                     (unsigned long)(to_ms_since_boot(get_absolute_time()) - start_ms),
                     adc_raw, deviation_from_white,
                     approaching_line ? "true" : "false",
                     on_edge ? "true" : "false");
            mqtt_publish_str(MQTT_TOPIC_TELEM, msg);
        }

        if (approaching_line || on_edge) {
            printf("[GENTLE] ✓✓ BLACK LINE DETECTED! ADC=%u\n", adc_raw);
            printf("[GENTLE] Changed from baseline %u to %u (Δ=%+d)\n",
                   baseline_adc, adc_raw, (int)adc_raw - (int)baseline_adc);

            snprintf(msg, sizeof(msg),
                     "{\"event\":\"line_detected\",\"adc\":%u,\"baseline\":%u,"
                     "\"delta\":%d,\"time_ms\":%lu}",
                     adc_raw, baseline_adc,
                     (int)adc_raw - (int)baseline_adc,
                     (unsigned long)(to_ms_since_boot(get_absolute_time()) - start_ms));
            mqtt_publish_str(MQTT_TOPIC_TELEM, msg);

            found = true;
            break;
        }

        // Gentle left turn: left motor slower, right motor faster
        forward_motor_manual(100, 120);
        sleep_ms(20);
    }

    stop_motor_pid();
    sleep_ms(100);

    if (found) {
        uint16_t detection_adc = read_line_adc();
        printf("[GENTLE] ✓ Line detected! ADC=%u\n", detection_adc);

        // === FAST DIRECTIONAL ANALYSIS: Determine which way to turn ===
        // After 90° right turn, we're perpendicular to track
        // Use quick, gentle sampling to avoid overshooting
        
        printf("[GENTLE] Fast direction analysis...\n");
        stop_motor_pid();
        sleep_ms(50);  // Shorter stabilization
        
        // Baseline position
        uint16_t baseline_pos = read_line_adc();
        
        // Quick left sample with VERY gentle turn (50ms only)
        forward_motor_manual(90, 120);  // Gentler than before
        sleep_ms(50);  // Much shorter
        uint16_t left_sample = read_line_adc();
        
        stop_motor_pid();
        sleep_ms(30);
        
        // Return to baseline position
        forward_motor_manual(120, 90);
        sleep_ms(50);
        stop_motor_pid();
        sleep_ms(30);
        
        // Quick right sample with VERY gentle turn (50ms only)
        forward_motor_manual(120, 90);  // Gentler than before
        sleep_ms(50);  // Much shorter
        uint16_t right_sample = read_line_adc();
        
        stop_motor_pid();
        sleep_ms(50);
        
        // Return to center
        forward_motor_manual(90, 120);
        sleep_ms(50);
        stop_motor_pid();
        sleep_ms(30);
        
        // Final position check
        uint16_t final_pos = read_line_adc();
        
        // Determine direction: if left turn increased ADC more, line is on left side
        int left_delta = (int)left_sample - (int)baseline_pos;
        int right_delta = (int)right_sample - (int)baseline_pos;
        bool line_on_left = (left_delta > right_delta + 150);
        const char* preferred_direction = line_on_left ? "LEFT" : "RIGHT";
        
        printf("[GENTLE] Direction: base=%u left=%u(Δ%+d) right=%u(Δ%+d) final=%u → %s\n",
               baseline_pos, left_sample, left_delta, right_sample, right_delta,
               final_pos, preferred_direction);
        
        // Publish direction analysis
        char msg[256];
        snprintf(msg, sizeof(msg),
                 "{\"event\":\"direction_analysis\",\"detection_adc\":%u,\"baseline\":%u,"
                 "\"left_sample\":%u,\"left_delta\":%d,\"right_sample\":%u,\"right_delta\":%d,"
                 "\"final_pos\":%u,\"preferred\":\"%s\"}",
                 detection_adc, baseline_pos, left_sample, left_delta,
                 right_sample, right_delta, final_pos, preferred_direction);
        mqtt_publish_str(MQTT_TOPIC_TELEM, msg);

        // IMMEDIATE REACTION based on detection position
        if (detection_adc > 2500) {
            // Detected DEEP IN BLACK - perpendicular crossing!
            printf("[GENTLE] PERPENDICULAR CROSSING! ADC=%u - HARD RIGHT NOW!\n", detection_adc);
            
            // EMERGENCY RIGHT TURN to find edge
            forward_motor_manual(180, 60);  // Hard right
            sleep_ms(150);  // Brief but aggressive
            
            // Check if we found edge
            uint16_t check_adc = read_line_adc();
            printf("[GENTLE] After emergency turn: ADC=%u\n", check_adc);
            
            // If still in black, continue turning
            while (check_adc > 2000) {
                forward_motor_manual(165, 70);
                sleep_ms(50);
                check_adc = read_line_adc();
                printf("[GENTLE] Still in black: ADC=%u, continuing right turn...\n", check_adc);
                
                // Safety timeout
                static int turn_cycles = 0;
                if (++turn_cycles > 10) {
                    printf("[GENTLE] Emergency turn timeout!\n");
                    break;
                }
            }
            
            stop_motor_pid();
            sleep_ms(100);
            detection_adc = read_line_adc();
            printf("[GENTLE] Repositioned to ADC=%u\n", detection_adc);
        }

        // Now execute controlled approach to settle on edge
        printf("[GENTLE] Starting controlled approach from ADC=%u (prefer turn %s)...\n",
               detection_adc, preferred_direction);
        
        // Phase 1.5: HIGH-SPEED controlled approach (10ms sampling = 100Hz update rate)
        uint32_t approach_start = to_ms_since_boot(get_absolute_time());
        int approach_stable = 0;
        bool edge_locked = false;
        uint32_t max_approach_time = 1500;  // Extended to 1.5s for better recovery
        
        // Adjust search behavior based on preferred direction
        int search_left_pwm_l = line_on_left ? 70 : 80;
        int search_left_pwm_r = line_on_left ? 145 : 135;
        
        while ((to_ms_since_boot(get_absolute_time()) - approach_start) < max_approach_time) {
            uint16_t adc = read_line_adc();
            
            // HIGH-FREQUENCY corrections based on current position
            if (adc < 800) {
                // Far on white → Need to find line with preferred direction
                forward_motor_manual(search_left_pwm_l, search_left_pwm_r);
                if (reading_count++ % 10 == 0) {
                    printf("[APPROACH] FAR WHITE(ADC=%u) → SEARCH\n", adc);
                }
                approach_stable = 0;
            } else if (adc >= 800 && adc < 1400) {
                // Approaching from white → moderate left to reach edge
                forward_motor_manual(85, 130);
                if (reading_count++ % 10 == 0) {
                    printf("[APPROACH] APPROACHING(ADC=%u)\n", adc);
                }
                approach_stable = 0;
            } else if (adc >= 1400 && adc < 1600) {
                // Near edge from below → gentle left
                forward_motor_manual(95, 115);
                approach_stable++;
            } else if (adc >= 1600 && adc <= 2000) {
                // PERFECT EDGE ZONE → minimal movement to hold position
                // Bias slightly toward preferred direction to ensure forward travel
                if (line_on_left) {
                    forward_motor_manual(102, 112);  // Slight right bias (line on left)
                } else {
                    forward_motor_manual(112, 102);  // Slight left bias (line on right)
                }
                approach_stable++;
                edge_locked = true;
                if (approach_stable >= 10) {  // Need more stable readings at high sampling
                    printf("[APPROACH] ✓ EDGE LOCKED at ADC=%u (direction: %s)!\n",
                           adc, preferred_direction);
                    break;
                }
            } else if (adc > 2000 && adc <= 2300) {
                // Near edge from above → gentle right
                forward_motor_manual(115, 95);
                approach_stable = 0;
            } else if (adc > 2300) {
                // Too far in black → strong right turn to exit
                forward_motor_manual(145, 75);
                if (reading_count++ % 10 == 0) {
                    printf("[APPROACH] DEEP BLACK(ADC=%u) → RIGHT\n", adc);
                }
                approach_stable = 0;
            }
            
            // Telemetry at lower frequency but control at high frequency
            if (reading_count++ % 20 == 0) {
                const char* zone = (adc < 800) ? "WHITE" : 
                                   (adc < 1400) ? "APPROACH" :
                                   (adc >= 1600 && adc <= 2000) ? "EDGE-LOCK" : 
                                   (adc > 2300) ? "BLACK" : "EDGE";
                printf("[APPROACH] ADC=%u (%s) stable=%d\n", adc, zone, approach_stable);
            }
            
            sleep_ms(10);  // HIGH SAMPLING RATE: 100Hz (was 20ms = 50Hz)
        }
        
        stop_motor_pid();
        sleep_ms(100);
        
        uint16_t post_approach_adc = read_line_adc();
        printf("[GENTLE] After approach: ADC=%u (target: %d) locked=%s direction=%s\n",
               post_approach_adc, TARGET_EDGE_VALUE, edge_locked ? "YES" : "NO",
               preferred_direction);

        // PHASE 2: Aggressive edge-following
        printf("[GENTLE] Phase 2: Aggressive edge-following for %d ms\n",
               LINE_CAPTURE_DURATION_MS);

        PIDController capture_pid;
        pid_init(&capture_pid, KP_STEER, KI_STEER, KD_STEER);
        
        // Pre-warm PID with current position
        uint16_t warmup_adc = read_line_adc();
        float warmup_error = (float)((int)warmup_adc - TARGET_EDGE_VALUE);
        pid_compute(&capture_pid, warmup_error, 0.02f);
        printf("[CAPTURE] PID pre-warmed with error=%.0f\n", (double)warmup_error);

        uint64_t last_capture_time = time_us_64();
        uint32_t capture_start = to_ms_since_boot(get_absolute_time());
        int stable_readings = 0;
        const int REQUIRED_STABLE = 10;

        while ((to_ms_since_boot(get_absolute_time()) - capture_start) <
               LINE_CAPTURE_DURATION_MS) {
            uint64_t now = time_us_64();
            float dt = (now - last_capture_time) / 1e6f;
            if (dt < 0.001f) dt = 0.02f;
            last_capture_time = now;

            uint16_t adc_raw = read_line_adc();
            float raw_error = (float)((int)adc_raw - TARGET_EDGE_VALUE);
            float error = raw_error;

            float abs_error = abs_float(error);
            bool in_corner = (abs_error > CORNER_ERROR_THRESHOLD);
            float steer_correction = pid_compute(&capture_pid, error, dt);

            if (in_corner) {
                // Reduced corner gain to prevent wild swings
                float corner_intensity =
                    (abs_error - CORNER_ERROR_THRESHOLD) / 800.0f;  // Was 500
                if (corner_intensity > 1.0f) corner_intensity = 1.0f;
                float gain = 1.0f + 0.3f * corner_intensity;  // Max 1.3x gain (was 1.6x)
                steer_correction *= gain;
            }

            // Use fixed lower base speed for capture phase
            int base_pwm = LINE_CAPTURE_BASE_PWM;  // 110 PWM
            
            if (in_corner) {
                base_pwm -= (CORNER_SPEED_REDUCTION / 2);  // Gentler slow-down
                if (base_pwm < 80) base_pwm = 80;  // Prevent stalling
            }

            // Gentler PWM scaling during capture
            float effective_scale = in_corner ? 18.0f : 15.0f;  // Reduced from 25-32.5
            int pwm_adjustment = (int)(steer_correction * effective_scale);
            int pwmL = base_pwm + pwm_adjustment;
            int pwmR = base_pwm - pwm_adjustment;

            // Clamp BEFORE enforcing differential
            if (pwmL < 60) pwmL = 60;   // Prevent motor stall
            if (pwmL > 180) pwmL = 180;
            if (pwmR < 60) pwmR = 60;
            if (pwmR > 180) pwmR = 180;
            
            // Ensure minimum speed difference for turns
            int actual_diff = pwmL - pwmR;
            int abs_diff = (actual_diff < 0) ? -actual_diff : actual_diff;
            if (abs_diff < 30 && abs_error > 400) {  // Only enforce on large errors
                int sign = (actual_diff > 0) ? 1 : -1;
                int boost = (30 - abs_diff) / 2;
                pwmL += sign * boost;
                pwmR -= sign * boost;
                
                // Re-clamp after adjustment
                if (pwmL < 60) pwmL = 60;
                if (pwmL > 180) pwmL = 180;
                if (pwmR < 60) pwmR = 60;
                if (pwmR > 180) pwmR = 180;
            }

            disable_pid_control();
            forward_motor_manual(pwmL, pwmR);

            // Check for stable line lock (stricter criteria)
            if (abs_error < 150) {  // Tighter tolerance
                stable_readings++;
                if (stable_readings >= REQUIRED_STABLE) {
                    printf("[CAPTURE] ✓✓ LINE LOCK CONFIRMED after %d stable readings!\n",
                           stable_readings);
                    break;
                }
            } else {
                stable_readings = 0;  // Reset completely on instability
            }

            // Telemetry at reduced frequency
            if (reading_count++ % 10 == 0) {
                const char *mode = in_corner ? "CORNER" : "STRAIGHT";
                const char *direction =
                    (error > 0) ? "LEFT" : (error < 0) ? "RIGHT" : "STRAIGHT";
                const char *zone =
                    (adc_raw < 400) ? "WHITE" :
                    (adc_raw > 2200) ? "BLACK" : "EDGE";
                int pwm_diff = pwmL - pwmR;

                printf("[CAPTURE-%s] ADC=%u(%s) err=%+.0f %s pwmL=%d pwmR=%d "
                       "diff=%d stable=%d\n",
                       mode, adc_raw, zone, (double)error,
                       direction, pwmL, pwmR, pwm_diff, stable_readings);

                snprintf(msg, sizeof(msg),
                         "{\"event\":\"line_capture\",\"adc\":%u,\"zone\":\"%s\","
                         "\"error\":%.0f,\"mode\":\"%s\",\"direction\":\"%s\","
                         "\"pwm_left\":%d,\"pwm_right\":%d,\"diff\":%d,"
                         "\"stable\":%d}",
                         adc_raw, zone, (double)error, mode, direction,
                         pwmL, pwmR, pwm_diff, stable_readings);
                mqtt_publish_str(MQTT_TOPIC_TELEM, msg);
            }

            sleep_ms(10);  // HIGH CAPTURE SAMPLING: 100Hz (was 20ms = 50Hz)
        }

        stop_motor_pid();
        sleep_ms(200);

        printf("[GENTLE] ✓ Line capture complete! Starting transition phase...\n");
        uint16_t final_adc = read_line_adc();
        printf("[GENTLE] Final ADC: %u (target: %d)\n",
               final_adc, TARGET_EDGE_VALUE);

        // ===== TRANSITION PHASE: Gradual speed ramp-up for smooth rejoin =====
        printf("[GENTLE] Phase 3: Transition to full-speed edge following (%d ms)\n",
               TRANSITION_DURATION_MS);

        PIDController transition_pid;
        pid_init(&transition_pid, KP_STEER, KI_STEER, KD_STEER);

        uint64_t last_transition_time = time_us_64();
        uint32_t transition_start = to_ms_since_boot(get_absolute_time());
        int transition_stable = 0;

        while ((to_ms_since_boot(get_absolute_time()) - transition_start) <
               TRANSITION_DURATION_MS) {
            uint64_t now = time_us_64();
            float dt = (now - last_transition_time) / 1e6f;
            if (dt < 0.001f) dt = 0.02f;
            last_transition_time = now;

            uint16_t adc_raw = read_line_adc();
            float raw_error = (float)((int)adc_raw - TARGET_EDGE_VALUE);
            float error = raw_error;
            float abs_error = abs_float(error);

            bool in_corner = (abs_error > CORNER_ERROR_THRESHOLD);
            float steer_correction = pid_compute(&transition_pid, error, dt);

            if (in_corner) {
                float corner_intensity =
                    (abs_error - CORNER_ERROR_THRESHOLD) / 500.0f;
                if (corner_intensity > 1.0f) corner_intensity = 1.0f;
                float gain = 1.0f + (CORNER_GAIN_MULTIPLIER - 1.0f) *
                                      corner_intensity;
                steer_correction *= gain;
            }

            // Gradual speed ramp: start at 80 PWM, ramp to full speed (PWM_MIN_LEFT)
            float progress = (float)(to_ms_since_boot(get_absolute_time()) - transition_start) / TRANSITION_DURATION_MS;
            int base_pwm = (int)(80 + (PWM_MIN_LEFT - 80) * progress);

            if (in_corner) {
                base_pwm -= CORNER_SPEED_REDUCTION;
                if (base_pwm < PWM_MIN_LEFT) base_pwm = PWM_MIN_LEFT;
            }

            float effective_scale =
                in_corner ? (PWM_SCALE_FACTOR * 1.3f) : PWM_SCALE_FACTOR;
            int pwm_adjustment = (int)(steer_correction * effective_scale);
            int pwmL = base_pwm + pwm_adjustment;
            int pwmR = base_pwm - pwm_adjustment;

            pwmL = clamp_pwm_left(pwmL);
            pwmR = clamp_pwm_right(pwmR);

            disable_pid_control();
            forward_motor_manual(pwmL, pwmR);

            // Check stability
            if (abs_error < 250) {
                transition_stable++;
            } else {
                transition_stable = 0;
            }

            // Early exit if very stable
            if (transition_stable >= 20) {
                printf("[TRANSITION] ✓ Stable lock achieved early!\n");
                break;
            }

            if (reading_count++ % 10 == 0) {
                printf("[TRANSITION] ADC=%u err=%+.0f base_pwm=%d pwmL=%d pwmR=%d stable=%d\n",
                       adc_raw, (double)error, base_pwm, pwmL, pwmR, transition_stable);
            }

            sleep_ms(15);  // Faster transition sampling: 67Hz (was 20ms = 50Hz)
        }

        stop_motor_pid();
        sleep_ms(100);

        printf("[GENTLE] ✓✓ FULL REJOIN COMPLETE! Returning to normal edge following.\n\n");
        uint16_t rejoin_adc = read_line_adc();
        printf("[GENTLE] Rejoin ADC: %u (target: %d)\n\n",
               rejoin_adc, TARGET_EDGE_VALUE);

        snprintf(msg, sizeof(msg),
                 "{\"event\":\"line_rejoin_complete\",\"final_adc\":%u,"
                 "\"rejoin_adc\":%u,\"target\":%d,\"stable_readings\":%d,"
                 "\"transition_stable\":%d,\"direction\":\"%s\",\"line_side\":\"%s\"}",
                 final_adc, rejoin_adc, TARGET_EDGE_VALUE, stable_readings,
                 transition_stable, preferred_direction, line_on_left ? "left" : "right");
        mqtt_publish_str(MQTT_TOPIC_TELEM, msg);

        // Reset avoidance cooldown to allow immediate corrections
        g_avoidance_cooldown_steps = 0;

    } else {
        printf("[GENTLE] ✗ Line not found after %d ms - stopping.\n\n",
               GENTLE_SEARCH_MAX_TIME_MS);

        snprintf(msg, sizeof(msg),
                 "{\"event\":\"gentle_search_failed\",\"max_time_ms\":%u}",
                 GENTLE_SEARCH_MAX_TIME_MS);
        mqtt_publish_str(MQTT_TOPIC_TELEM, msg);
    }
}


/* ======== Obstacle Width Scan + MQTT Publish ======== */

static void run_width_scan_and_publish(float adjacent) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║ *** OBSTACLE DETECTED ***                                         ║\n");
    printf("║ Perpendicular Distance: %.2f cm                                   ║\n", adjacent);
    printf("╚════════════════════════════════════════════════════════════════════╝\n");

    if (adjacent > MIN_DISTANCE_FOR_WIDTH_SCAN) {
        printf("\n[SCAN] Distance %.2f cm > %.2f cm - TOO FAR for reliable width measurement\n",
               adjacent, MIN_DISTANCE_FOR_WIDTH_SCAN);
        printf("[SCAN] Skipping width scan - ultrasonic not reliable at this distance\n");

        if (mqtt_is_connected()) {
            char json[200];
            uint32_t ts_ms = to_ms_since_boot(get_absolute_time());
            int n = snprintf(json, sizeof(json),
                             "{\"ts\":%u,\"adjacent\":%.2f,\"leftWidth\":0.00,"
                             "\"rightWidth\":0.00,\"totalWidth\":0.00,\"note\":\"too_far\"}",
                             (unsigned)ts_ms, (double)adjacent);
            if (n > 0 && n < (int)sizeof(json)) {
                mqtt_publish_str(MQTT_TOPIC_OBSTACLE, json);
            }
        }

        servo_set_angle(SERVO_CENTER_ANGLE);
        return;
    }

    printf("[SCAN] Distance %.2f cm < %.2f cm - GOOD for width measurement\n",
           adjacent, MIN_DISTANCE_FOR_WIDTH_SCAN);
    printf("Waiting %dms for system to stabilize.\n", INITIAL_STOP_DELAY_MS);
    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(INITIAL_STOP_DELAY_MS);

    // Use ultrasonic module's scan helpers
    float left_w  = ultrasonic_scan_left_side(adjacent);
    sleep_ms(500);
    float right_w = ultrasonic_scan_right_side(adjacent);
    float total_w = left_w + right_w;

    printf("\n[RESULT] LEFT=%.2f cm, RIGHT=%.2f cm → TOTAL WIDTH=%.2f cm\n",
           left_w, right_w, total_w);

    if (mqtt_is_connected()) {
        char json[200];
        uint32_t ts_ms = to_ms_since_boot(get_absolute_time());
        int n = snprintf(json, sizeof(json),
                         "{\"ts\":%u,\"adjacent\":%.2f,\"leftWidth\":%.2f,"
                         "\"rightWidth\":%.2f,\"totalWidth\":%.2f}",
                         (unsigned)ts_ms,
                         (double)adjacent,
                         (double)left_w,
                         (double)right_w,
                         (double)total_w);
        if (n > 0 && n < (int)sizeof(json)) {
            mqtt_publish_str(MQTT_TOPIC_OBSTACLE, json);
        }
    }

    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(300);

    printf("\n[SCAN] Obstacle measurement complete\n");
    printf("[SCAN] LEFT: %.2f cm, RIGHT: %.2f cm, TOTAL: %.2f cm\n",
           left_w, right_w, total_w);
}

/* ======== Edge Following Task ======== */

static void edgeFollowTask(void *pvParameters) {
    (void)pvParameters;

    PIDController steer_pid;
    pid_init(&steer_pid, KP_STEER, KI_STEER, KD_STEER);

    printf("[EDGE] Edge following initialized\n");
    if (g_imu_ok) {
        printf("[IMU] Heading telemetry enabled (hdg in JSON)\n");
    } else {
        printf("[IMU] NOT available - hdg will stay 0.0\n");
    }

    uint64_t last_time = time_us_64();
    int telem_counter = 0;
    int obstacle_check_counter = 0;

    servo_set_angle(SERVO_CENTER_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(500));

    for (;;) {
        uint64_t now = time_us_64();
        float dt = (now - last_time) / 1e6f;
        if (dt < 0.001f) dt = 0.02f;
        last_time = now;

        // --- IMU update: keep heading fresh while robot moves ---
        if (g_imu_ok) {
            float raw = imu_update_and_get_heading(&g_imu);
            raw += HEADING_OFFSET_DEG;
            raw = wrap_deg_0_360(raw);
            g_hdg_raw_deg = raw;
            g_hdg_deg = ema(g_hdg_deg, raw, HDG_EMA_ALPHA);
        }

        obstacle_check_counter++;
        if (obstacle_check_counter >= OBSTACLE_CHECK_INTERVAL) {
            obstacle_check_counter = 0;

            float center_cm = ultrasonic_get_distance_cm();
            if (center_cm > 0.0f && center_cm <= OBSTACLE_THRESHOLD_CM) {
                printf("[EDGE] Obstacle at %.1f cm, stopping...\n", center_cm);
                disable_pid_control();
                stop_motor_pid();

                // Run width scan and publish EXACT same JSON as testDemo3.c
                run_width_scan_and_publish(center_cm);

                // Execute 90-degree right turn
                motor_turn_right_90();

                // Search for line with gentle arc
                gentle_search_for_line();

                printf("[EDGE] Line rejoin complete, resuming normal edge following...\n");
                servo_set_angle(SERVO_CENTER_ANGLE);
                vTaskDelay(pdMS_TO_TICKS(200));
                last_time = time_us_64();
            }
        }

        // Edge following logic
        uint16_t adc_raw = read_line_adc();
        float raw_error = (float)((int)adc_raw - TARGET_EDGE_VALUE);
        float error;

        if (LINE_FOLLOWING_MODE == 0) {
            error = raw_error;
        } else {
            error = g_current_edge_direction * raw_error;
            g_error_history_sum += raw_error;
            g_error_history_count++;

            if (g_error_history_count >= 50) {
                float avg_error = g_error_history_sum / g_error_history_count;
                if (abs_float(avg_error) > 400.0f) {
                    g_current_edge_direction *= -1;
                }
                g_error_history_sum = 0.0f;
                g_error_history_count = 0;
            }
        }

        float abs_error = abs_float(error);
        bool in_corner = (abs_error > CORNER_ERROR_THRESHOLD);
        float steer_correction = pid_compute(&steer_pid, error, dt);

        if (in_corner) {
            float corner_intensity =
                (abs_error - CORNER_ERROR_THRESHOLD) / 500.0f;
            if (corner_intensity > 1.0f) corner_intensity = 1.0f;
            float gain = 1.0f + (CORNER_GAIN_MULTIPLIER - 1.0f) *
                                  corner_intensity;
            steer_correction *= gain;
        }

        if (g_avoidance_cooldown_steps > 0) {
            g_avoidance_cooldown_steps--;
            if (steer_correction < 0.0f) {
                steer_correction *= 0.3f;
            }
        }

        int base_pwm;
        if (in_corner) {
            base_pwm = PWM_MIN_LEFT - CORNER_SPEED_REDUCTION;
            if (base_pwm < PWM_MIN_LEFT) base_pwm = PWM_MIN_LEFT;
        } else {
            base_pwm = PWM_MIN_LEFT;
        }

        float effective_scale =
            in_corner ? (PWM_SCALE_FACTOR * 1.3f) : PWM_SCALE_FACTOR;
        int pwm_adjustment = (int)(steer_correction * effective_scale);
        int pwmL = base_pwm + pwm_adjustment;
        int pwmR = base_pwm - pwm_adjustment;

        if (in_corner) {
            int actual_diff = pwmL - pwmR;
            int abs_diff = (actual_diff < 0) ? -actual_diff : actual_diff;
            if (abs_diff < MIN_CORNER_PWM_DIFF) {
                int sign = (actual_diff > 0) ? 1 : -1;
                int needed = (MIN_CORNER_PWM_DIFF - abs_diff) / 2;
                pwmL += sign * needed;
                pwmR -= sign * needed;
            }
        }

        pwmL = clamp_pwm_left(pwmL);
        pwmR = clamp_pwm_right(pwmR);
        float vL = get_left_speed();
        float vR = get_right_speed();

        disable_pid_control();
        forward_motor_manual(pwmL, pwmR);

        if ((telem_counter++ % 10) == 0) {
            const char *mode = in_corner ? "CORNER" : "STRAIGHT";
            const char *follow_mode =
                (LINE_FOLLOWING_MODE == 0) ? "CENTER" : "EDGE";
            const char *edge_dir =
                (g_current_edge_direction > 0) ? "LEFT" : "RIGHT";
            bool turning_left = (error > 0);
            bool turning_right = (error < 0);
            const char *direction =
                turning_left ? "LEFT" : (turning_right ? "RIGHT" : "STRAIGHT");
            int pwm_diff = pwmL - pwmR;

            printf("[%s-%s] ADC=%u err=%+.0f %s %s pwmL=%d pwmR=%d diff=%d hdg=%.1f\n",
                   follow_mode, edge_dir, adc_raw, (double)error,
                   mode, direction, pwmL, pwmR, pwm_diff,
                   (double)g_hdg_deg);

            if (mqtt_is_connected()){
                char json[384];
                uint32_t ts_ms = to_ms_since_boot(get_absolute_time());
                int n;

                // ----- MAIN TELEMETRY (add "hdg") -----
                n = snprintf(json, sizeof(json),
                    "{\"timestamp\":%u,\"adc\":%u,\"target\":%d,\"error\":%.1f,"
                    "\"mode\":\"%s\",\"direction\":\"%s\",\"follow\":\"%s\","
                    "\"edge\":\"%s\",\"pwmL\":%d,\"pwmR\":%d,\"diff\":%d,"
                    "\"speedL\":%.2f,\"speedR\":%.2f,\"inCorner\":%s,"
                    "\"hdg\":%.1f}",
                    (unsigned)ts_ms, adc_raw, TARGET_EDGE_VALUE,
                    (double)error, mode, direction, follow_mode, edge_dir,
                    pwmL, pwmR, pwm_diff,
                    (double)vL, (double)vR, in_corner ? "true" : "false",
                    (double)g_hdg_deg);   // <<< heading deg from IMU
                if (n > 0 && n < (int)sizeof(json)) {
                    mqtt_publish_str(MQTT_TOPIC_TELEM, json);
                }

                // SENSOR JSON
                n = snprintf(json, sizeof(json),
                    "{\"adc\":%u,\"error\":%.1f,\"target\":%d}",
                    adc_raw, (double)error, TARGET_EDGE_VALUE);
                if (n > 0 && n < (int)sizeof(json)) {
                    mqtt_publish_str(MQTT_TOPIC_SENSOR, json);
                }

                // MOTOR JSON
                n = snprintf(json, sizeof(json),
                    "{\"pwmL\":%d,\"pwmR\":%d,\"speedL\":%.2f,\"speedR\":%.2f}",
                    pwmL, pwmR, (double)vL, (double)vR);
                if (n > 0 && n < (int)sizeof(json)) {
                    mqtt_publish_str(MQTT_TOPIC_MOTOR, json);
                }

                // STATUS JSON (periodic)
                if (telem_counter % 50 == 0) {
                    n = snprintf(json, sizeof(json),
                        "{\"mode\":\"%s\",\"direction\":\"%s\",\"follow\":\"%s\","
                        "\"edge\":\"%s\",\"corner\":%s}",
                        mode, direction, follow_mode, edge_dir,
                        in_corner ? "true" : "false");
                    if (n > 0 && n < (int)sizeof(json)) {
                        mqtt_publish_str(MQTT_TOPIC_STATUS, json);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(400);

    init_line_adc();
    encoder_init();
    motor_init();
    servo_init();
    ultrasonic_init();
    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(300);

    // ---------- IMU INIT (like testDemo1.c, but only for heading telemetry) ----------
    g_imu.i2c      = i2c1;
    g_imu.i2c_baud = IMU_I2C_BAUD;
    g_imu.pin_sda  = IMU_SDA_PIN;
    g_imu.pin_scl  = IMU_SCL_PIN;
    g_imu.mx_off = g_imu.my_off = g_imu.mz_off = 0.0f;

    g_imu_ok = imu_init(&g_imu);
    if (g_imu_ok) {
        printf("[IMU] Init OK, priming heading filter...\n");
        float filt = 0.0f;
        for (int i = 0; i < 20; ++i) {
            float h = imu_update_and_get_heading(&g_imu);
            h += HEADING_OFFSET_DEG;
            h = wrap_deg_0_360(h);
            filt = ema(filt, h, HDG_EMA_ALPHA);
            sleep_ms(10);
        }
        g_hdg_deg = filt;
        g_hdg_raw_deg = filt;
        printf("[IMU] Initial heading = %.1f deg\n", (double)g_hdg_deg);
    } else {
        printf("[IMU] Init FAILED - hdg will stay 0.0 in telemetry\n");
    }

    g_mqtt_mutex = xSemaphoreCreateMutex();

    printf("\n========================================\n");
    printf("Edge Following + Obstacle Scan + Turn\n");
    printf("========================================\n");
    printf("Behavior: Scan obstacle → Turn 90° right\n");
    printf("Turn duration: %d ms at PWM %d\n", TURN_90_DURATION_MS, TURN_PWM_SPEED);
    printf("========================================\n\n");

    xTaskCreate(vNetworkTask, "net", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(edgeFollowTask, "EdgeFollow",
                configMINIMAL_STACK_SIZE * 4,
                NULL, tskIDLE_PRIORITY + 1, NULL);

    printf("[MAIN] Starting edge following with obstacle avoidance...\n");
    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
    return 0;
}