// ==============================
// Integrated Edge Following + Obstacle Detection (IMPROVED)
// - demo2.c: IR Edge Following with Aggressive Corner Response
// - testDemo3.c: Ultrasonic obstacle detection + width scan (left/right) using servo
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

// ======== CONFIG (from demo2.c) ========
// --- Wi-Fi & MQTT ---
// #define WIFI_SSID                 "Keithiphone"
// #define WIFI_PASS                 "testong1"
#define WIFI_SSID                 "Jared"
#define WIFI_PASS                 "1teddygodie"
#define WIFI_CONNECT_TIMEOUT_MS   20000

// #define BROKER_IP_STR             "172.20.10.2"
#define BROKER_IP_STR             "10.22.173.149"
#define BROKER_PORT               1883
#define MQTT_TOPIC_TELEM          "pico/demo3/telemetry"
#define MQTT_TOPIC_STATUS         "pico/demo3/status"
#define MQTT_TOPIC_SENSOR         "pico/demo3/sensor"
#define MQTT_TOPIC_MOTOR          "pico/demo3/motor"
#define MQTT_TOPIC_OBSTACLE       "pico/demo3/obstacle"

#ifndef LINE_SENSOR_PIN
#define LINE_SENSOR_PIN 28           // GPIO28 -> ADC2
#endif

// ======== EDGE FOLLOWING PARAMETERS (from demo2.c) ========
#ifndef TARGET_EDGE_VALUE
#define TARGET_EDGE_VALUE 1800  // ADC value: edge mode=boundary, center mode=middle of line
#endif

// Line following mode: 0=center-line (bidirectional), 1=edge (single direction)
#ifndef LINE_FOLLOWING_MODE
#define LINE_FOLLOWING_MODE 0  // Use center-line for figure-8 tracks
#endif

// Edge direction: +1=left edge (clockwise), -1=right edge (counterclockwise)
// Only used if LINE_FOLLOWING_MODE=1
#ifndef EDGE_DIRECTION
#define EDGE_DIRECTION 1
#endif

#ifndef BASE_SPEED_CMPS
#define BASE_SPEED_CMPS 2.0f
#endif

// PID gains for normal operation
#ifndef KP_STEER
#define KP_STEER 0.055f  // Reduced from 0.02 for smoother response
#endif

#ifndef KI_STEER
#define KI_STEER 0.0002f  // Reduced from 0.0006 to minimize windup
#endif

#ifndef KD_STEER
#define KD_STEER 0.0010f  // Reduced from 0.012 to dampen noise
#endif

// INCREASED maximum steering correction
#ifndef MAX_STEER_CORRECTION
#define MAX_STEER_CORRECTION 2.0f  // INCREASED to allow aggressive corner turns
#endif

#ifndef ERROR_DEADBAND
#define ERROR_DEADBAND 30
#endif

// PWM scaling
#ifndef PWM_SCALE_FACTOR
#define PWM_SCALE_FACTOR 25.0f  // Base scaling
#endif

// Corner detection and handling
#ifndef CORNER_ERROR_THRESHOLD
#define CORNER_ERROR_THRESHOLD 300  // Increased from 250 - only trigger on sharp turns
#endif

#ifndef CORNER_SPEED_REDUCTION
#define CORNER_SPEED_REDUCTION 8  // Reduced from 8 - less speed change
#endif

// Exponential steering gain for corners
#ifndef CORNER_GAIN_MULTIPLIER
#define CORNER_GAIN_MULTIPLIER 1.6f  // Reduced from 1.8 - gentler corner boost
#endif

// Minimum PWM difference to ensure turning happens
#ifndef MIN_CORNER_PWM_DIFF
#define MIN_CORNER_PWM_DIFF 40  // Reduced from 20 - less forced differential
#endif

// ======== OBSTACLE DETECTION PARAMETERS (from testDemo3.c) ========
#define OBSTACLE_THRESHOLD_CM       25.0f
#define SERVO_CENTER_ANGLE          90.0f
#define SERVO_RIGHT_ANGLE           150.0f
#define SERVO_LEFT_ANGLE            35.0f
#define SERVO_SCAN_STEP_DEGREE      1.0f
#define SERVO_REFINE_STEP_DEGREE    0.5f
#define MEASUREMENT_DELAY_MS        50
#define REFINEMENT_DELAY_MS         2000
#define MAX_DETECTION_DISTANCE_CM   100.0f
#define EDGE_CONFIRMATION_SAMPLES   5
#define CONSISTENCY_THRESHOLD_CM    10.0f

// NEW: Obstacle check frequency (check every N loop iterations)
#define OBSTACLE_CHECK_INTERVAL     25  // Check every 25 iterations (~500ms)

// ======== ADC init (from demo2.c) ========
static inline void init_line_adc(void) {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_select_input(2);
}

// ======== PID Controller (from demo2.c) ========
typedef struct {
    float integral;
    float prev_error;
    float kp;
    float ki;
    float kd;
} PIDController;

static void pid_init(PIDController *pid, float kp, float ki, float kd) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

static float pid_compute(PIDController *pid, float error, float dt) {
    if (error > -ERROR_DEADBAND && error < ERROR_DEADBAND) {
        error = 0.0f;
    }
    
    pid->integral += error * dt;
    
    const float max_integral = MAX_STEER_CORRECTION / (pid->ki + 0.0001f);
    if (pid->integral > max_integral) pid->integral = max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;
    
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    if (output > MAX_STEER_CORRECTION) output = MAX_STEER_CORRECTION;
    if (output < -MAX_STEER_CORRECTION) output = -MAX_STEER_CORRECTION;
    
    return output;
}

/* ============================ Wi-Fi / MQTT ========================= */
static mqtt_client_t *g_mqtt = NULL;
static ip_addr_t      g_broker_ip;
static SemaphoreHandle_t g_mqtt_mutex;
static volatile bool g_shutdown = false;

// Edge-following state for bidirectional support
static int8_t g_current_edge_direction = EDGE_DIRECTION;  // +1 left, -1 right
static float  g_error_history_sum = 0.0f;  // Running sum for auto-detection
static int    g_error_history_count = 0;

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

// ======== Motor PWM clamps ========
#ifndef PWM_MIN_LEFT
#define PWM_MIN_LEFT  120
#endif
#ifndef PWM_MAX_LEFT
#define PWM_MAX_LEFT  255
#endif
#ifndef PWM_MIN_RIGHT
#define PWM_MIN_RIGHT 120
#endif
#ifndef PWM_MAX_RIGHT
#define PWM_MAX_RIGHT 255
#endif

static inline int clamp_pwm_left(int v) {
    if (v < PWM_MIN_LEFT)  v = PWM_MIN_LEFT;
    if (v > PWM_MAX_LEFT)  v = PWM_MAX_LEFT;
    return v;
}

static inline int clamp_pwm_right(int v) {
    if (v < PWM_MIN_RIGHT) v = PWM_MIN_RIGHT;
    if (v > PWM_MAX_RIGHT) v = PWM_MAX_RIGHT;
    return v;
}

static inline float abs_float(float x) {
    return x < 0 ? -x : x;
}

/* ========== Obstacle Width Scan Functions (from testDemo3.c) ========== */

static float calculate_width_component(float adjacent, float hypotenuse) {
    float h2 = hypotenuse * hypotenuse;
    float a2 = adjacent * adjacent;
    if (h2 <= a2) {
        printf(" [WARNING] Hypotenuse (%.2f) <= adjacent (%.2f), returning 0\n",
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
            printf(" Sample %d: %.2f cm\n", i + 1, s);
        } else {
            printf(" Sample %d: invalid\n", i + 1);
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
    bool edge_found = false;
    float edge_distance = 0.0f;

    while (current_angle <= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(MEASUREMENT_DELAY_MS);
        float d = ultrasonic_get_distance_cm();
        if (d > 0 && d <= MAX_DETECTION_DISTANCE_CM) {
            printf("[LEFT_SCAN] %.1f° │ ✓ ECHO! Confirming...\n", current_angle);
            float avg;
            if (take_consistent_samples(&avg)) {
                printf("[LEFT_SCAN] ✓✓ EDGE FOUND at %.1f°! Starting refinement...\n",
                       current_angle);
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

    if (!edge_found) {
        printf("[LEFT_SCAN] No edge found!\n");
        return 0.0f;
    }

    printf("\n[LEFT_REFINE] Starting edge refinement (moving back toward 35°)...\n");
    while (current_angle >= SERVO_LEFT_ANGLE) {
        current_angle -= SERVO_REFINE_STEP_DEGREE;
        servo_set_angle(current_angle);
        printf("[LEFT_REFINE] Testing %.1f° (waiting 2s)...\n", current_angle);
        sleep_ms(REFINEMENT_DELAY_MS);

        float avg;
        if (take_consistent_samples(&avg)) {
            printf("[LEFT_REFINE] ✓ Still edge at %.1f°, avg: %.2f cm\n",
                   current_angle, avg);
            edge_distance = avg;
        } else {
            printf("[LEFT_REFINE] ✗ Edge lost! Final edge at %.1f°\n",
                   current_angle + SERVO_REFINE_STEP_DEGREE);
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
    bool edge_found = false;
    float edge_distance = 0.0f;

    while (current_angle >= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(MEASUREMENT_DELAY_MS);
        float d = ultrasonic_get_distance_cm();
        if (d > 0 && d <= MAX_DETECTION_DISTANCE_CM) {
            printf("[RIGHT_SCAN] %.1f° │ ✓ ECHO! Confirming...\n", current_angle);
            float avg;
            if (take_consistent_samples(&avg)) {
                printf("[RIGHT_SCAN] ✓✓ EDGE FOUND at %.1f°! Starting refinement...\n",
                       current_angle);
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

    if (!edge_found) {
        printf("[RIGHT_SCAN] No edge found!\n");
        return 0.0f;
    }

    printf("\n[RIGHT_REFINE] Starting edge refinement (moving back toward 150°)...\n");
    while (current_angle <= SERVO_RIGHT_ANGLE) {
        current_angle += SERVO_REFINE_STEP_DEGREE;
        servo_set_angle(current_angle);
        printf("[RIGHT_REFINE] Testing %.1f° (waiting 2s)...\n", current_angle);
        sleep_ms(REFINEMENT_DELAY_MS);

        float avg;
        if (take_consistent_samples(&avg)) {
            printf("[RIGHT_REFINE] ✓ Still edge at %.1f°, avg: %.2f cm\n",
                   current_angle, avg);
            edge_distance = avg;
        } else {
            printf("[RIGHT_REFINE] ✗ Edge lost! Final edge at %.1f°\n",
                   current_angle - SERVO_REFINE_STEP_DEGREE);
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

static void run_width_scan_sequence(float adjacent) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║ *** OBSTACLE DETECTED ***                                         ║\n");
    printf("║ Perpendicular Distance: %.2f cm (adjacent)                        ║\n", adjacent);
    printf("╚════════════════════════════════════════════════════════════════════╝\n");

    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(500);

    float left_w  = scan_left_side(adjacent);
    sleep_ms(500);
    float right_w = scan_right_side(adjacent);
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
            err_t r = mqtt_publish_str(MQTT_TOPIC_OBSTACLE, json);
            if (r == ERR_OK) {
                printf("[MQTT] Published obstacle data: %s\n", json);
            } else {
                printf("[MQTT] Publish failed, err=%d\n", r);
            }
        }
    } else {
        printf("[MQTT] Not connected, skipping obstacle telemetry publish\n");
    }

    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(300);
}

/* ======== Edge Following Task with Periodic Obstacle Detection ======== */

static void edgeFollowTask(void *pvParameters) {
    (void)pvParameters;

    PIDController steer_pid;
    pid_init(&steer_pid, KP_STEER, KI_STEER, KD_STEER);

    printf("[EDGE] Aggressive corner following initialized\n");
    printf(" Target: %d, Corner threshold: %d\n",
           TARGET_EDGE_VALUE, CORNER_ERROR_THRESHOLD);
    printf(" Corner gain multiplier: %.1fx\n", (double)CORNER_GAIN_MULTIPLIER);
    printf(" Obstacle check interval: every %d iterations (~%dms)\n", 
           OBSTACLE_CHECK_INTERVAL, OBSTACLE_CHECK_INTERVAL * 20);

    uint64_t last_time = time_us_64();
    int telem_counter = 0;
    int obstacle_check_counter = 0;  // NEW: Counter for periodic obstacle checks

    // Initialize servo at center position once
    servo_set_angle(SERVO_CENTER_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(500));

    for (;;) {
        uint64_t now = time_us_64();
        float dt = (now - last_time) / 1e6f;
        if (dt < 0.001f) dt = 0.02f;
        last_time = now;

        // ---- Periodic obstacle detection (IMPROVED - less intrusive) ----
        obstacle_check_counter++;
        if (obstacle_check_counter >= OBSTACLE_CHECK_INTERVAL) {
            obstacle_check_counter = 0;
            
            // Quick obstacle check without moving servo (it should already be at center)
            float center_cm = ultrasonic_get_distance_cm();
            if (center_cm > 0.0f && center_cm <= OBSTACLE_THRESHOLD_CM) {
                printf("[EDGE] Obstacle within %.1f cm, stopping and scanning width...\n",
                       center_cm);
                disable_pid_control();
                stop_motor_pid();
                
                // Run the full width scan
                run_width_scan_sequence(center_cm);
                
                // After scan, reset servo to center and give system time to stabilize
                servo_set_angle(SERVO_CENTER_ANGLE);
                vTaskDelay(pdMS_TO_TICKS(200));
                
                // Reset timing to avoid dt spike
                last_time = time_us_64();
            }
        }

        // ---- Edge following logic (from demo2.c) - UNCHANGED ----
        // Read ADC value
        uint16_t adc_raw = adc_read();

        // Compute error based on following mode
        float raw_error = (float)((int)adc_raw - TARGET_EDGE_VALUE);
        float error;

        if (LINE_FOLLOWING_MODE == 0) {
            // CENTER-LINE MODE (bidirectional - recommended for figure-8)
            // Positive error = sensor sees white (off line to the left) → turn right
            // Negative error = sensor sees black (on line or right side) → turn left
            // This works for both clockwise and counterclockwise
            error = raw_error;
        } else {
            // EDGE MODE (single direction, requires correct EDGE_DIRECTION)
            // Apply edge direction: +1 for left edge, -1 for right edge
            error = g_current_edge_direction * raw_error;

            // Optional: Auto-detect if we're following the wrong edge
            // If error is consistently large and same sign, we might be on wrong edge
            g_error_history_sum += raw_error;
            g_error_history_count++;

            if (g_error_history_count >= 50) {  // Every 1 second
                float avg_error = g_error_history_sum / g_error_history_count;
                // If average error magnitude is large, consider flipping edge
                if (abs_float(avg_error) > 400.0f) {
                    printf("[EDGE] Detected wrong edge direction, flipping...\n");
                    g_current_edge_direction *= -1;
                }
                g_error_history_sum = 0.0f;
                g_error_history_count = 0;
            }
        }

        float abs_error = abs_float(error);

        // Detect corner
        bool in_corner = (abs_error > CORNER_ERROR_THRESHOLD);

        // Compute base steering correction from PID
        float steer_correction = pid_compute(&steer_pid, error, dt);

        // Apply exponential gain for corners
        if (in_corner) {
            // Calculate how far into corner we are (0 to 1)
            float corner_intensity = (abs_error - CORNER_ERROR_THRESHOLD) / 500.0f;
            if (corner_intensity > 1.0f) corner_intensity = 1.0f;

            // Apply exponential multiplier (1.0x to CORNER_GAIN_MULTIPLIER)
            float gain = 1.0f + (CORNER_GAIN_MULTIPLIER - 1.0f) * corner_intensity;
            steer_correction *= gain;

            // Removed forced minimum - let PID decide correction naturally
        }

        // Adaptive base speed
        int base_pwm;
        if (in_corner) {
            base_pwm = PWM_MIN_LEFT - CORNER_SPEED_REDUCTION;
            if (base_pwm < PWM_MIN_LEFT) base_pwm = PWM_MIN_LEFT;
        } else {
            base_pwm = PWM_MIN_LEFT;
        }

        // Apply steering correction with higher scaling in corners
        float effective_scale = in_corner ? (PWM_SCALE_FACTOR * 1.3f) : PWM_SCALE_FACTOR;
        int pwm_adjustment = (int)(steer_correction * effective_scale);

        // CORRECTED STEERING LOGIC:
        // positive error (white) = drifted left → speed up left, slow right → turn right back to edge
        // negative error (black) = drifted right → slow left, speed up right → turn left back to edge
        int pwmL = base_pwm + pwm_adjustment;
        int pwmR = base_pwm - pwm_adjustment;

        // In corners, enforce minimum PWM difference to ensure turning
        if (in_corner) {
            int actual_diff = pwmL - pwmR;
            int abs_diff = (actual_diff < 0) ? -actual_diff : actual_diff;

            if (abs_diff < MIN_CORNER_PWM_DIFF) {
                // Need more differential
                int sign = (actual_diff > 0) ? 1 : -1;
                int needed = (MIN_CORNER_PWM_DIFF - abs_diff) / 2;
                pwmL += sign * needed;
                pwmR -= sign * needed;
            }
        }

        // Clamp PWM values
        pwmL = clamp_pwm_left(pwmL);
        pwmR = clamp_pwm_right(pwmR);

        // Get current speeds
        float vL = get_left_speed();
        float vR = get_right_speed();

        // Drive motors
        disable_pid_control();
        forward_motor_manual(pwmL, pwmR);

        // Telemetry
        if ((telem_counter++ % 10) == 0) {
            const char *mode = in_corner ? "CORNER" : "STRAIGHT";
            const char *follow_mode = (LINE_FOLLOWING_MODE == 0) ? "CENTER" : "EDGE";
            const char *edge_dir = (g_current_edge_direction > 0) ? "LEFT" : "RIGHT";

            // Determine turn direction for telemetry
            bool turning_left = (error > 0);
            bool turning_right = (error < 0);
            const char *direction = turning_left ? "LEFT" : (turning_right ? "RIGHT" : "STRAIGHT");

            int pwm_diff = pwmL - pwmR;
            printf("[%s-%s] ADC=%u err=%+.0f %s %s pwmL=%d pwmR=%d diff=%d\n",
                   follow_mode, edge_dir, adc_raw, (double)error, mode, direction, pwmL, pwmR, pwm_diff);

            if (mqtt_is_connected()){
                char json[384];
                uint32_t ts_ms = to_ms_since_boot(get_absolute_time());
                int n;

                // Publish comprehensive telemetry
                n = snprintf(json, sizeof(json),
                    "{\"timestamp\":%u,\"adc\":%u,\"target\":%d,\"error\":%.1f,"
                    "\"mode\":\"%s\",\"direction\":\"%s\",\"follow\":\"%s\",\"edge\":\"%s\","
                    "\"pwmL\":%d,\"pwmR\":%d,\"diff\":%d,"
                    "\"speedL\":%.2f,\"speedR\":%.2f,\"inCorner\":%s}",
                    (unsigned)ts_ms, adc_raw, TARGET_EDGE_VALUE,
                    (double)error, mode, direction, follow_mode, edge_dir,
                    pwmL, pwmR, pwm_diff,
                    (double)vL, (double)vR, in_corner ? "true" : "false");
                if (n > 0 && n < (int)sizeof(json)) {
                    mqtt_publish_str(MQTT_TOPIC_TELEM, json);
                }

                // Publish separate sensor data for easier charting
                n = snprintf(json, sizeof(json),
                    "{\"adc\":%u,\"error\":%.1f,\"target\":%d}",
                    adc_raw, (double)error, TARGET_EDGE_VALUE);
                if (n > 0 && n < (int)sizeof(json)) {
                    mqtt_publish_str(MQTT_TOPIC_SENSOR, json);
                }

                // Publish motor data
                n = snprintf(json, sizeof(json),
                    "{\"pwmL\":%d,\"pwmR\":%d,\"speedL\":%.2f,\"speedR\":%.2f}",
                    pwmL, pwmR, (double)vL, (double)vR);
                if (n > 0 && n < (int)sizeof(json)) {
                    mqtt_publish_str(MQTT_TOPIC_MOTOR, json);
                }

                // Publish status every 50 cycles (less frequent)
                if (telem_counter % 50 == 0) {
                    n = snprintf(json, sizeof(json),
                        "{\"mode\":\"%s\",\"direction\":\"%s\",\"follow\":\"%s\",\"edge\":\"%s\",\"corner\":%s}",
                        mode, direction, follow_mode, edge_dir, in_corner ? "true" : "false");
                    if (n > 0 && n < (int)sizeof(json)) {
                        mqtt_publish_str(MQTT_TOPIC_STATUS, json);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
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
    
    // Initialize servo at center position
    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(300);

    g_mqtt_mutex = xSemaphoreCreateMutex();

    printf("\n========================================\n");
    printf("Edge Following + Obstacle Detection\n");
    printf("========================================\n");
    printf("Mode: %s\n", (LINE_FOLLOWING_MODE == 0) ? "CENTER-LINE" : "EDGE");
    if (LINE_FOLLOWING_MODE == 1) {
        printf("Edge direction: %s\n", (EDGE_DIRECTION > 0) ? "LEFT (CW)" : "RIGHT (CCW)");
    }
    printf("Target ADC: %d\n", TARGET_EDGE_VALUE);
    printf("Corner threshold: %d (error above this)\n", CORNER_ERROR_THRESHOLD);
    printf("Corner gain: %.1fx normal\n", (double)CORNER_GAIN_MULTIPLIER);
    printf("Min corner PWM diff: %d\n", MIN_CORNER_PWM_DIFF);
    printf("Obstacle check: every %dms\n", OBSTACLE_CHECK_INTERVAL * 20);

    printf("\nCalibration readings:\n");
    for (int i = 0; i < 10; i++) {
        uint16_t adc_val = adc_read();
        printf("  %d: %u\n", i+1, adc_val);
        sleep_ms(200);
    }
    printf("========================================\n\n");

    xTaskCreate(vNetworkTask, "net", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(edgeFollowTask, "EdgeFollow",
                configMINIMAL_STACK_SIZE * 4,
                NULL, tskIDLE_PRIORITY + 1, NULL);

    printf("[MAIN] Starting edge following with obstacle detection...\n");
    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
    return 0;
}