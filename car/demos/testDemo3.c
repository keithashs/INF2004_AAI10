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

// ======== CONFIG (from demo2.c) ========
// --- Wi-Fi & MQTT ---
#define WIFI_SSID                 "Keithiphone"
#define WIFI_PASS                 "testong1"
// #define WIFI_SSID                 "Jared"
// #define WIFI_PASS                 "1teddygodie"
#define WIFI_CONNECT_TIMEOUT_MS   20000
#define AVOID_EXTRA_LATERAL_CM       2.0f   // Reduced from 5.0 for tighter turns
#define SPIN_SEARCH_PWM              140
#define SPIN_SEARCH_MAX_TIME_MS      3000

// Dynamic turn angle parameters - precise turns based on obstacle width
#define MIN_OBSTACLE_WIDTH_CM        5.0f   // Minimum width for scaling
#define MAX_OBSTACLE_WIDTH_CM        25.0f  // Maximum width for scaling
#define MIN_TURN_DURATION_MS         400    // Very tight turn for small obstacles
#define MAX_TURN_DURATION_MS         800    // Moderate turn for large obstacles

#define BROKER_IP_STR             "172.20.10.3"
// #define BROKER_IP_STR             "10.22.173.149"
#define BROKER_PORT               1883
#define MQTT_TOPIC_TELEM          "pico/demo3/telemetry"
#define MQTT_TOPIC_STATUS         "pico/demo3/status"
#define MQTT_TOPIC_SENSOR         "pico/demo3/sensor"
#define MQTT_TOPIC_MOTOR          "pico/demo3/motor"
#define MQTT_TOPIC_OBSTACLE       "pico/demo3/obstacle"
// ======== TURN + LINE RE-ENTRY PARAMETERS ========
#define FORWARD_AFTER_REALIGN_PWM      150   // PWM for straight drive after last 90° left
#define LINE_DETECT_ERROR_THRESHOLD    150   // Very sensitive - any significant deviation
#define FORWARD_SEARCH_MAX_TIME_MS     12000 // Extended to 12 seconds

#define FORWARD_SEARCH_PWM           110     // Very slow for maximum detection window
#define LINE_EDGE_DERIVATIVE_THRESHOLD 50    // Extremely sensitive to any ADC change   

#ifndef LINE_SENSOR_PIN
#define LINE_SENSOR_PIN 28
#endif

// ======== EDGE FOLLOWING PARAMETERS (from demo2.c) ========
#ifndef TARGET_EDGE_VALUE
#define TARGET_EDGE_VALUE 1800
#endif

#ifndef LINE_FOLLOWING_MODE
#define LINE_FOLLOWING_MODE 0
#endif

#ifndef EDGE_DIRECTION
#define EDGE_DIRECTION 1
#endif

#ifndef BASE_SPEED_CMPS
#define BASE_SPEED_CMPS 2.0f
#endif

#ifndef KP_STEER
#define KP_STEER 0.055f
#endif

#ifndef KI_STEER
#define KI_STEER 0.0002f
#endif

#ifndef KD_STEER
#define KD_STEER 0.0010f
#endif

#ifndef MAX_STEER_CORRECTION
#define MAX_STEER_CORRECTION 2.0f
#endif

#ifndef ERROR_DEADBAND
#define ERROR_DEADBAND 30
#endif

#ifndef PWM_SCALE_FACTOR
#define PWM_SCALE_FACTOR 25.0f
#endif

#ifndef CORNER_ERROR_THRESHOLD
#define CORNER_ERROR_THRESHOLD 300
#endif

#ifndef CORNER_SPEED_REDUCTION
#define CORNER_SPEED_REDUCTION 8
#endif

#ifndef CORNER_GAIN_MULTIPLIER
#define CORNER_GAIN_MULTIPLIER 1.6f
#endif

#ifndef MIN_CORNER_PWM_DIFF
#define MIN_CORNER_PWM_DIFF 40
#endif

// ======== OBSTACLE DETECTION PARAMETERS ========
#define OBSTACLE_THRESHOLD_CM       20.0f   // Increased to account for stopping distance
#define SERVO_CENTER_ANGLE          90.0f
#define SERVO_RIGHT_ANGLE           150.0f
#define SERVO_LEFT_ANGLE            35.0f
#define SERVO_SCAN_STEP_DEGREE      1.0f
#define SERVO_REFINE_STEP_DEGREE    0.5f

#define MEASUREMENT_DELAY_MS        200
#define REFINEMENT_DELAY_MS         3000
#define SERVO_SETTLE_DELAY_MS       300
#define INITIAL_STOP_DELAY_MS       1000

#define MIN_DISTANCE_FOR_WIDTH_SCAN 20.0f
#define MAX_REASONABLE_WIDTH_CM     30.0f
#define MAX_EDGE_DISTANCE_CM        40.0f
#define MAX_DISTANCE_DIFF_FACTOR    2.5f

#define MAX_DETECTION_DISTANCE_CM   100.0f
#define EDGE_CONFIRMATION_SAMPLES   10
#define MIN_VALID_SAMPLES           7
#define CONSISTENCY_THRESHOLD_CM    5.0f
#define OUTLIER_THRESHOLD_CM        15.0f

#define OBSTACLE_CHECK_INTERVAL     5    // Check every 5 loops for faster response

// ======== NEW: TURN PARAMETERS ========
#define TURN_90_DURATION_MS         1000 
#define TURN_45_DURATION_MS   (TURN_90_DURATION_MS / 2)
#define TURN_PWM_SPEED              110    // PWM for turning

// NEW: Forward movement after turn (calibrate these)
#define FORWARD_PWM_SPEED           150    // PWM when moving forward
#define FORWARD_MS_PER_CM           70     // ~time per cm (tune experimentally)

// ======== GENTLE LINE SEARCH PARAMETERS ========
#define GENTLE_TURN_PWM_LEFT        110    // Slower left motor for gentle right arc
#define GENTLE_TURN_PWM_RIGHT       145    // Faster right motor for gentle right arc
#define GENTLE_SEARCH_MAX_TIME_MS   8000   // Maximum time for gentle search
#define LINE_FOUND_THRESHOLD        200    // ADC change indicating line found
#define LINE_CAPTURE_DURATION_MS    1500   // Time to actively track line after detection
#define LINE_CAPTURE_BASE_PWM       130    // Base PWM during line capture phase

// ======== ADC init ========
static inline void init_line_adc(void) {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_select_input(2);
}

// ======== PID Controller ========
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

static int8_t g_current_edge_direction = EDGE_DIRECTION;
static float  g_error_history_sum = 0.0f;
static int    g_error_history_count = 0;

// NEW: after-avoidance cooldown to prevent snapping back to old line
static volatile int g_avoidance_cooldown_steps = 0;   // counts edgeFollow loop iterations


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
    ci.client_id  = "pico-demo3";
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

/* ========== NEW: 90 Degree Right Turn Function ========== */

/* ========== NEW: 90 Degree Right Turn Only ========== */

static void turn_right_90_degrees(void) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════╗\n");
    printf("║   EXECUTING 90 DEGREE RIGHT TURN                  ║\n");
    printf("╚════════════════════════════════════════════════════╝\n");
    
    disable_pid_control();
    
    // ---- PRECISE 90 DEGREE RIGHT TURN ----
    printf("[TURN] Turning right 90 degrees...\n");
    printf("[TURN] Duration: %u ms at PWM %d\n", TURN_90_DURATION_MS, TURN_PWM_SPEED);
    
    // Right turn: left motor forward, right motor slow/stopped
    forward_motor_manual(TURN_PWM_SPEED, PWM_MIN_RIGHT);
    sleep_ms(TURN_90_DURATION_MS);
    stop_motor_pid();
    sleep_ms(300);

    printf("[TURN] 90-degree right turn complete\n\n");
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
    
    uint16_t baseline_adc = adc_read();
    printf("[GENTLE] Baseline ADC: %u (white surface)\n", baseline_adc);
    printf("[GENTLE] Target ADC: %d (black line edge)\n", TARGET_EDGE_VALUE);
    printf("[GENTLE] Left motor PWM: %d, Right motor PWM: %d\n", 
           GENTLE_TURN_PWM_LEFT, GENTLE_TURN_PWM_RIGHT);
    
    int reading_count = 0;
    uint16_t prev_adc = baseline_adc;
    
    // Continuous gentle left-turning arc while moving forward
    printf("[GENTLE] Starting gentle left arc search...\n");
    
    while ((to_ms_since_boot(get_absolute_time()) - start_ms) < GENTLE_SEARCH_MAX_TIME_MS) {
        uint16_t adc_raw = adc_read();
        
        // Calculate how far we've deviated from white surface
        int deviation_from_white = abs((int)adc_raw - (int)baseline_adc);
        
        // CORRECTED: Black line has HIGH ADC values (around 1800), white is LOW (around 200)
        // Check if we're approaching the black line (ADC RISING toward TARGET)
        bool approaching_line = (adc_raw > baseline_adc + LINE_FOUND_THRESHOLD);
        
        // Also check if we've crossed into the edge following range
        int error_from_target = abs((int)adc_raw - TARGET_EDGE_VALUE);
        bool on_edge = (error_from_target < 400);  // Within edge following range (more sensitive)
        
        if (reading_count++ % 10 == 0) {
            printf("[GENTLE] t=%lums ADC=%u deviation=%d approaching=%s on_edge=%s\n",
                   (unsigned long)(to_ms_since_boot(get_absolute_time()) - start_ms),
                   adc_raw, deviation_from_white,
                   approaching_line ? "YES" : "no",
                   on_edge ? "YES" : "no");
        }
        
        // Line found if either condition is met
        if (approaching_line || on_edge) {
            printf("[GENTLE] ✓✓ BLACK LINE DETECTED! ADC=%u\n", adc_raw);
            printf("[GENTLE] Changed from baseline %u to %u (Δ=%+d)\n",
                   baseline_adc, adc_raw, (int)adc_raw - (int)baseline_adc);
            found = true;
            break;
        }
        
        prev_adc = adc_raw;
        
        // Gentle left turn: left motor slower, right motor faster
        // This creates a wide, gentle arc to the left
        forward_motor_manual(GENTLE_TURN_PWM_LEFT, GENTLE_TURN_PWM_RIGHT);
        sleep_ms(50);  // Slower sampling for smoother motion
    }
    
    stop_motor_pid();
    sleep_ms(100);

    if (found) {
        printf("[GENTLE] ✓ Line found! Beginning active line capture...\n");
        
        // PHASE 2: Active line tracking - keep following the detected line
        printf("[GENTLE] Phase 2: Active line tracking for %d ms\n", LINE_CAPTURE_DURATION_MS);
        uint32_t capture_start = to_ms_since_boot(get_absolute_time());
        int stable_readings = 0;
        const int REQUIRED_STABLE = 15;  // Need 15 good readings to confirm lock
        
        while ((to_ms_since_boot(get_absolute_time()) - capture_start) < LINE_CAPTURE_DURATION_MS) {
            uint16_t adc_now = adc_read();
            int error = (int)adc_now - TARGET_EDGE_VALUE;
            
            // Simple proportional steering toward the target
            float steer_factor = (float)error / 500.0f;  // Normalize error
            if (steer_factor > 1.0f) steer_factor = 1.0f;
            if (steer_factor < -1.0f) steer_factor = -1.0f;
            
            // Calculate motor speeds to track the line
            int pwm_diff = (int)(steer_factor * 40.0f);  // Up to ±40 PWM differential
            int pwmL = LINE_CAPTURE_BASE_PWM + pwm_diff;
            int pwmR = LINE_CAPTURE_BASE_PWM - pwm_diff;
            
            // Clamp
            if (pwmL < PWM_MIN_LEFT) pwmL = PWM_MIN_LEFT;
            if (pwmL > 180) pwmL = 180;
            if (pwmR < PWM_MIN_RIGHT) pwmR = PWM_MIN_RIGHT;
            if (pwmR > 180) pwmR = 180;
            
            forward_motor_manual(pwmL, pwmR);
            
            // Check if we're maintaining good lock on the line
            int abs_error = (error < 0) ? -error : error;
            if (abs_error < 250) {
                stable_readings++;
                if (stable_readings >= REQUIRED_STABLE) {
                    printf("[GENTLE] ✓✓ LINE LOCK CONFIRMED after %d stable readings!\n", stable_readings);
                    break;
                }
            } else {
                stable_readings = 0;  // Reset if we lose the line
            }
            
            if (reading_count++ % 5 == 0) {
                printf("[CAPTURE] ADC=%u err=%+d steer=%.2f pwmL=%d pwmR=%d stable=%d/%d\n",
                       adc_now, error, (double)steer_factor, pwmL, pwmR, stable_readings, REQUIRED_STABLE);
            }
            
            sleep_ms(50);
        }
        
        stop_motor_pid();
        sleep_ms(200);
        
        printf("[GENTLE] ✓ Line capture complete! Transitioning to edge following.\n");
        printf("[GENTLE] Final ADC: %u (target: %d)\n\n", adc_read(), TARGET_EDGE_VALUE);
        
    } else {
        printf("[GENTLE] ✗ Line not found after %d ms - stopping.\n\n", 
               GENTLE_SEARCH_MAX_TIME_MS);
    }
}



/* ========== Obstacle Width Scan Functions ========== */

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

static int compare_floats(const void *a, const void *b) {
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

static float calculate_median(float *arr, int n) {
    if (n == 0) return 0.0f;
    
    float temp[EDGE_CONFIRMATION_SAMPLES];
    for (int i = 0; i < n; i++) {
        temp[i] = arr[i];
    }
    
    qsort(temp, n, sizeof(float), compare_floats);
    
    if (n % 2 == 0) {
        return (temp[n/2 - 1] + temp[n/2]) / 2.0f;
    } else {
        return temp[n/2];
    }
}

static bool take_consistent_samples(float *avg_distance_out) {
    float samples[EDGE_CONFIRMATION_SAMPLES];
    int valid = 0;

    for (int i = 0; i < EDGE_CONFIRMATION_SAMPLES; i++) {
        sleep_ms(MEASUREMENT_DELAY_MS);
        float s = ultrasonic_get_distance_cm();
        if (s > 0 && s <= MAX_DETECTION_DISTANCE_CM) {
            samples[valid++] = s;
            printf(" Sample %d: %.2f cm ✓\n", i + 1, s);
        } else {
            printf(" Sample %d: invalid ✗\n", i + 1);
        }
    }

    if (valid < MIN_VALID_SAMPLES) {
        printf(" Only %d/%d valid samples\n", valid, EDGE_CONFIRMATION_SAMPLES);
        return false;
    }

    float median = calculate_median(samples, valid);
    printf(" Median: %.2f cm\n", median);

    float filtered[EDGE_CONFIRMATION_SAMPLES];
    int filtered_count = 0;
    
    for (int i = 0; i < valid; i++) {
        float diff = abs_float(samples[i] - median);
        if (diff <= OUTLIER_THRESHOLD_CM) {
            filtered[filtered_count++] = samples[i];
        }
    }

    if (filtered_count < MIN_VALID_SAMPLES - 2) {
        printf(" Only %d samples after filtering\n", filtered_count);
        return false;
    }

    float sum = 0.0f;
    float min_val = 1e9f;
    float max_val = 0.0f;
    
    for (int i = 0; i < filtered_count; i++) {
        sum += filtered[i];
        if (filtered[i] < min_val) min_val = filtered[i];
        if (filtered[i] > max_val) max_val = filtered[i];
    }

    float avg = sum / filtered_count;
    float variance = max_val - min_val;

    if (variance > CONSISTENCY_THRESHOLD_CM) {
        printf(" Variance %.2f cm exceeds threshold\n", variance);
        return false;
    }

    *avg_distance_out = avg;
    printf(" ✓ CONSISTENT: %.2f cm\n", avg);
    return true;
}

static bool validate_edge_distance(float center_distance, float edge_distance, const char* side) {
    float max_expected = center_distance * MAX_DISTANCE_DIFF_FACTOR;
    
    if (edge_distance > MAX_EDGE_DISTANCE_CM) {
        printf(" [VALIDATE-%s] REJECTED: %.2f cm > max allowed (%.2f cm)\n", 
               side, edge_distance, MAX_EDGE_DISTANCE_CM);
        return false;
    }
    
    if (edge_distance > max_expected) {
        printf(" [VALIDATE-%s] REJECTED: %.2f cm > expected max (%.2f cm based on center %.2f cm)\n",
               side, edge_distance, max_expected, center_distance);
        return false;
    }
    
    printf(" [VALIDATE-%s] PASSED: %.2f cm is reasonable\n", side, edge_distance);
    return true;
}

static float scan_left_side(float adjacent) {
    printf("\n[LEFT_SCAN] Starting from 35° scanning toward 90°...\n");
    sleep_ms(2000);

    float current_angle = SERVO_LEFT_ANGLE;
    bool edge_found = false;
    float edge_distance = 0.0f;

    while (current_angle <= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(SERVO_SETTLE_DELAY_MS);
        
        float d = ultrasonic_get_distance_cm();
        if (d > 0 && d <= MAX_DETECTION_DISTANCE_CM) {
            printf("[LEFT_SCAN] %.1f° │ ECHO at %.2f cm, confirming...\n", current_angle, d);
            float avg;
            if (take_consistent_samples(&avg)) {
                if (validate_edge_distance(adjacent, avg, "LEFT")) {
                    printf("[LEFT_SCAN] ✓✓ VALID EDGE at %.1f°!\n", current_angle);
                    edge_found = true;
                    edge_distance = avg;
                    break;
                } else {
                    printf("[LEFT_SCAN] ✗ Invalid edge distance, continuing...\n");
                }
            } else {
                printf("[LEFT_SCAN] ✗ Inconsistent, continuing...\n");
            }
        } else {
            printf("[LEFT_SCAN] %.1f° │ No echo\n", current_angle);
        }
        current_angle += SERVO_SCAN_STEP_DEGREE;
    }

    if (!edge_found) {
        printf("[LEFT_SCAN] No valid edge found!\n");
        return 0.0f;
    }

    float left_width = calculate_width_component(adjacent, edge_distance);
    
    if (left_width > MAX_REASONABLE_WIDTH_CM) {
        printf("[LEFT_SCAN] ✗ REJECTED: Width %.2f cm > max reasonable (%.2f cm)\n",
               left_width, MAX_REASONABLE_WIDTH_CM);
        return 0.0f;
    }
    
    printf("[LEFT_SCAN] Final distance: %.2f cm\n", edge_distance);
    printf("[LEFT_SCAN] LEFT width: %.2f cm ✓\n", left_width);
    return left_width;
}

static float scan_right_side(float adjacent) {
    printf("\n[RIGHT_SCAN] Starting from 150° scanning toward 90°...\n");
    sleep_ms(2000);

    float current_angle = SERVO_RIGHT_ANGLE;
    bool edge_found = false;
    float edge_distance = 0.0f;

    while (current_angle >= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(SERVO_SETTLE_DELAY_MS);
        
        float d = ultrasonic_get_distance_cm();
        if (d > 0 && d <= MAX_DETECTION_DISTANCE_CM) {
            printf("[RIGHT_SCAN] %.1f° │ ECHO at %.2f cm, confirming...\n", current_angle, d);
            float avg;
            if (take_consistent_samples(&avg)) {
                if (validate_edge_distance(adjacent, avg, "RIGHT")) {
                    printf("[RIGHT_SCAN] ✓✓ VALID EDGE at %.1f°!\n", current_angle);
                    edge_found = true;
                    edge_distance = avg;
                    break;
                } else {
                    printf("[RIGHT_SCAN] ✗ Invalid edge distance, continuing...\n");
                }
            } else {
                printf("[RIGHT_SCAN] ✗ Inconsistent, continuing...\n");
            }
        } else {
            printf("[RIGHT_SCAN] %.1f° │ No echo\n", current_angle);
        }
        current_angle -= SERVO_SCAN_STEP_DEGREE;
    }

    if (!edge_found) {
        printf("[RIGHT_SCAN] No valid edge found!\n");
        return 0.0f;
    }

    float right_width = calculate_width_component(adjacent, edge_distance);
    
    if (right_width > MAX_REASONABLE_WIDTH_CM) {
        printf("[RIGHT_SCAN] ✗ REJECTED: Width %.2f cm > max reasonable (%.2f cm)\n",
               right_width, MAX_REASONABLE_WIDTH_CM);
        return 0.0f;
    }
    
    printf("[RIGHT_SCAN] Final distance: %.2f cm\n", edge_distance);
    printf("[RIGHT_SCAN] RIGHT width: %.2f cm ✓\n", right_width);
    return right_width;
}

static void run_width_scan_sequence(float adjacent) {
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
    printf("Waiting %dms for system to stabilize...\n", INITIAL_STOP_DELAY_MS);
    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(INITIAL_STOP_DELAY_MS);

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
            mqtt_publish_str(MQTT_TOPIC_OBSTACLE, json);
        }
    }

    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(300);
    
    printf("\n[SCAN] Obstacle measurement complete\n");
    printf("[SCAN] LEFT: %.2f cm, RIGHT: %.2f cm, TOTAL: %.2f cm\n",
           left_w, right_w, total_w);
    
    // Step 1: Execute 90-degree right turn
    turn_right_90_degrees();

    // Step 2: Search for line with gentle left-turning arc
    gentle_search_for_line();
    
    printf("[SCAN] Obstacle avoidance sequence complete\n\n");
}

/* ======== Edge Following Task ======== */

static void edgeFollowTask(void *pvParameters) {
    (void)pvParameters;

    PIDController steer_pid;
    pid_init(&steer_pid, KP_STEER, KI_STEER, KD_STEER);

    printf("[EDGE] Edge following initialized\n");

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

        obstacle_check_counter++;
        if (obstacle_check_counter >= OBSTACLE_CHECK_INTERVAL) {
            obstacle_check_counter = 0;
            
            float center_cm = ultrasonic_get_distance_cm();
            if (center_cm > 0.0f && center_cm <= OBSTACLE_THRESHOLD_CM) {
                printf("[EDGE] Obstacle at %.1f cm, stopping...\n", center_cm);
                disable_pid_control();
                stop_motor_pid();
                
                // This will scan and then turn 90 degrees right
                run_width_scan_sequence(center_cm);
                
                servo_set_angle(SERVO_CENTER_ANGLE);
                vTaskDelay(pdMS_TO_TICKS(200));
                last_time = time_us_64();
            }
        }

        // Edge following logic (unchanged)
        uint16_t adc_raw = adc_read();
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
            float corner_intensity = (abs_error - CORNER_ERROR_THRESHOLD) / 500.0f;
            if (corner_intensity > 1.0f) corner_intensity = 1.0f;
            float gain = 1.0f + (CORNER_GAIN_MULTIPLIER - 1.0f) * corner_intensity;
            steer_correction *= gain;
        }
        if (g_avoidance_cooldown_steps > 0) {
            g_avoidance_cooldown_steps--;

            // During cooldown, heavily limit LEFT turns (negative correction)
            if (steer_correction < 0.0f) {
                steer_correction *= 0.3f;   // or even set to 0.0f if you want no left turn
            }
        }
        int base_pwm;
        if (in_corner) {
            base_pwm = PWM_MIN_LEFT - CORNER_SPEED_REDUCTION;
            if (base_pwm < PWM_MIN_LEFT) base_pwm = PWM_MIN_LEFT;
        } else {
            base_pwm = PWM_MIN_LEFT;
        }

        float effective_scale = in_corner ? (PWM_SCALE_FACTOR * 1.3f) : PWM_SCALE_FACTOR;
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
            const char *follow_mode = (LINE_FOLLOWING_MODE == 0) ? "CENTER" : "EDGE";
            const char *edge_dir = (g_current_edge_direction > 0) ? "LEFT" : "RIGHT";
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

                n = snprintf(json, sizeof(json),
                    "{\"adc\":%u,\"error\":%.1f,\"target\":%d}",
                    adc_raw, (double)error, TARGET_EDGE_VALUE);
                if (n > 0 && n < (int)sizeof(json)) {
                    mqtt_publish_str(MQTT_TOPIC_SENSOR, json);
                }

                n = snprintf(json, sizeof(json),
                    "{\"pwmL\":%d,\"pwmR\":%d,\"speedL\":%.2f,\"speedR\":%.2f}",
                    pwmL, pwmR, (double)vL, (double)vR);
                if (n > 0 && n < (int)sizeof(json)) {
                    mqtt_publish_str(MQTT_TOPIC_MOTOR, json);
                }

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