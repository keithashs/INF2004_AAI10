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

// ======== CONFIG ========
// --- Wi-Fi & MQTT ---
#define WIFI_SSID                 "Keithiphone"
#define WIFI_PASS                 "testong1"
// #define WIFI_SSID                 "Jared"
// #define WIFI_PASS                 "1teddygodie"
#define WIFI_CONNECT_TIMEOUT_MS   20000
#define AVOID_EXTRA_LATERAL_CM       2.0f
#define SPIN_SEARCH_PWM              140
#define SPIN_SEARCH_MAX_TIME_MS      3000

#define BROKER_IP_STR             "172.20.10.3"
// #define BROKER_IP_STR             "10.22.173.149"
#define BROKER_PORT               1883
#define MQTT_TOPIC_TELEM          "pico/main/telemetry"
#define MQTT_TOPIC_STATUS         "pico/main/status"
#define MQTT_TOPIC_SENSOR         "pico/main/sensor"
#define MQTT_TOPIC_MOTOR          "pico/main/motor"
#define MQTT_TOPIC_OBSTACLE       "pico/main/obstacle"

// Dynamic turn angle parameters - precise turns based on obstacle width
#define MIN_OBSTACLE_WIDTH_CM        5.0f   // Minimum width for scaling
#define MAX_OBSTACLE_WIDTH_CM        25.0f  // Maximum width for scaling
#define MIN_TURN_DURATION_MS         400    // Very tight turn for small obstacles
#define MAX_TURN_DURATION_MS         800    // Moderate turn for large obstacles

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
#define TURN_90_DURATION_MS         900 
#define TURN_45_DURATION_MS   (TURN_90_DURATION_MS / 2)
#define TURN_PWM_SPEED              150    // PWM for turning

// NEW: Forward movement after turn (calibrate these)
#define FORWARD_PWM_SPEED           150    // PWM when moving forward
#define FORWARD_MS_PER_CM           70     // ~time per cm (tune experimentally)

// ======== GENTLE LINE SEARCH PARAMETERS ========
#define GENTLE_TURN_PWM_LEFT        90    // Slower left motor for gentle right arc
#define GENTLE_TURN_PWM_RIGHT       125    // Faster right motor for gentle right arc
#define GENTLE_SEARCH_MAX_TIME_MS   20000   // Maximum time for gentle search
#define LINE_EDGE_MIN_ADC           1000   // Minimum ADC to consider as edge/line (detect earlier!)
#define LINE_EDGE_MAX_ADC           2500   // Maximum ADC before we're too deep in black
#define LINE_CAPTURE_DURATION_MS    4000   // Time to actively track line after detection
#define LINE_CAPTURE_BASE_PWM       135    // Base PWM during line capture phase

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
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status){
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Connected to broker\n");
    } else {
        printf("[MQTT] Connection failed: %d\n", (int)status);
    }
}

static bool mqtt_is_connected(void){
    return (g_mqtt && mqtt_client_is_connected(g_mqtt));
}

static void mqtt_publish_str(const char *topic, const char *message){
    if (!mqtt_is_connected()) return;
    if (!xSemaphoreTake(g_mqtt_mutex, pdMS_TO_TICKS(100))) return;

    cyw43_arch_lwip_begin();
    err_t e = mqtt_publish(g_mqtt, topic, message, strlen(message),
                           0, 0, NULL, NULL);
    cyw43_arch_lwip_end();

    if (e != ERR_OK) {
        printf("[MQTT] Publish fail: %d\n", (int)e);
    }
    xSemaphoreGive(g_mqtt_mutex);
}

static void vNetworkTask(void *pvParameters) {
    (void)pvParameters;

    if (cyw43_arch_init()) {
        printf("[NET] cyw43_arch_init FAIL!\n");
        vTaskDelete(NULL);
        return;
    }
    cyw43_arch_enable_sta_mode();

    printf("[NET] Connecting to Wi-Fi '%s'...\n", WIFI_SSID);
    int timeout_count = 0;
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, WIFI_CONNECT_TIMEOUT_MS)) {
        printf("[NET] Wi-Fi connect timeout, retry #%d\n", ++timeout_count);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("[NET] Wi-Fi connected!\n");
    printf("[NET] Resolving broker IP '%s'...\n", BROKER_IP_STR);
    if (!ipaddr_aton(BROKER_IP_STR, &g_broker_ip)) {
        printf("[NET] Invalid broker IP\n");
        cyw43_arch_deinit();
        vTaskDelete(NULL);
        return;
    }

    printf("[NET] Broker IP = %s\n", ipaddr_ntoa(&g_broker_ip));

    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico_main";
    ci.keep_alive = 60;

    cyw43_arch_lwip_begin();
    g_mqtt = mqtt_client_new();
    if (!g_mqtt) {
        printf("[MQTT] mqtt_client_new FAIL\n");
        cyw43_arch_lwip_end();
        cyw43_arch_deinit();
        vTaskDelete(NULL);
        return;
    }

    err_t err = mqtt_client_connect(g_mqtt, &g_broker_ip, BROKER_PORT,
                                     mqtt_connection_cb, NULL, &ci);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("[MQTT] connect FAIL: %d\n", (int)err);
        cyw43_arch_deinit();
        vTaskDelete(NULL);
        return;
    }

    printf("[NET] MQTT connect initiated\n");

    for (;;) {
        if (g_shutdown) {
            printf("[NET] Shutting down...\n");
            mqtt_disconnect_now();
            sleep_ms(100);
            cyw43_arch_deinit();
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ======== OBSTACLE AVOIDANCE ======== */

// Turn right 90 degrees
static void turn_right_90_degrees(void) {
    printf("[TURN] Executing 90-degree right turn for %d ms at PWM %d...\n", 
           TURN_90_DURATION_MS, TURN_PWM_SPEED);

    // Turn right: left forward, right reverse
    disable_pid_control();
    turn_motor(1 /*RIGHT*/, TURN_PWM_SPEED, TURN_PWM_SPEED);
    
    sleep_ms(TURN_90_DURATION_MS);
    
    stop_motor();
    printf("[TURN] 90-degree turn complete\n");
    sleep_ms(300);
}

// Gentle line search with left-turning arc
static void gentle_search_for_line(void) {
    printf("[SEARCH] Starting gentle left-arc search for line...\n");
    
    uint64_t start_time_ms = to_ms_since_boot(get_absolute_time());
    bool line_found = false;
    bool line_capture_active = false;
    uint64_t line_first_seen_ms = 0;
    
    disable_pid_control();
    
    // Initial gentle right turn to search
    forward_motor_manual(GENTLE_TURN_PWM_LEFT, GENTLE_TURN_PWM_RIGHT);
    
    while (true) {
        uint64_t now_ms = to_ms_since_boot(get_absolute_time());
        uint64_t elapsed_ms = now_ms - start_time_ms;
        
        if (elapsed_ms > GENTLE_SEARCH_MAX_TIME_MS) {
            printf("[SEARCH] Timeout - no line found after %u ms\n", (unsigned)elapsed_ms);
            stop_motor();
            break;
        }
        
        uint16_t adc = read_line_adc();
        
        if (!line_capture_active) {
            // Detection phase
            if (adc >= LINE_EDGE_MIN_ADC && adc <= LINE_EDGE_MAX_ADC) {
                if (!line_found) {
                    printf("[SEARCH] Line edge detected! ADC=%u at %u ms\n", 
                           adc, (unsigned)elapsed_ms);
                    line_found = true;
                    line_first_seen_ms = now_ms;
                    line_capture_active = true;
                    
                    // Enter capture mode - increase right motor slightly to follow
                    forward_motor_manual(LINE_CAPTURE_BASE_PWM, LINE_CAPTURE_BASE_PWM + 15);
                }
            }
        } else {
            // Capture phase - actively tracking line
            uint64_t capture_elapsed_ms = now_ms - line_first_seen_ms;
            
            if (capture_elapsed_ms > LINE_CAPTURE_DURATION_MS) {
                printf("[SEARCH] Line capture complete after %u ms\n", 
                       (unsigned)capture_elapsed_ms);
                stop_motor();
                
                // Set cooldown to prevent immediate snap-back to old line
                g_avoidance_cooldown_steps = 30;  // ~30 loop iterations of cooldown
                break;
            }
            
            // Adjust motors based on ADC to stay on edge
            if (adc < LINE_EDGE_MIN_ADC) {
                // Too far from line, turn more right
                forward_motor_manual(LINE_CAPTURE_BASE_PWM - 10, LINE_CAPTURE_BASE_PWM + 20);
            } else if (adc > LINE_EDGE_MAX_ADC) {
                // Too much on line, turn more left
                forward_motor_manual(LINE_CAPTURE_BASE_PWM + 10, LINE_CAPTURE_BASE_PWM + 5);
            } else {
                // Good range, maintain course
                forward_motor_manual(LINE_CAPTURE_BASE_PWM, LINE_CAPTURE_BASE_PWM + 15);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    if (line_found) {
        printf("[SEARCH] Line successfully captured and tracked\n");
    }
    
    sleep_ms(200);
}

// Measure distance at specific angle with validation
static float measure_distance_at_angle(float angle, int samples) {
    servo_set_angle(angle);
    sleep_ms(SERVO_SETTLE_DELAY_MS);
    
    float valid_readings[EDGE_CONFIRMATION_SAMPLES];
    int valid_count = 0;
    
    for (int i = 0; i < samples; i++) {
        float dist = ultrasonic_get_distance_cm();
        
        if (dist > 0.0f && dist <= MAX_DETECTION_DISTANCE_CM) {
            bool is_outlier = false;
            for (int j = 0; j < valid_count; j++) {
                if (abs_float(dist - valid_readings[j]) > OUTLIER_THRESHOLD_CM) {
                    is_outlier = true;
                    break;
                }
            }
            
            if (!is_outlier) {
                valid_readings[valid_count++] = dist;
            }
        }
        
        if (i < samples - 1) {
            sleep_ms(50);
        }
    }
    
    if (valid_count < MIN_VALID_SAMPLES) {
        return -1.0f;
    }
    
    // Calculate average
    float sum = 0.0f;
    for (int i = 0; i < valid_count; i++) {
        sum += valid_readings[i];
    }
    float avg = sum / valid_count;
    
    // Check consistency
    for (int i = 0; i < valid_count; i++) {
        if (abs_float(valid_readings[i] - avg) > CONSISTENCY_THRESHOLD_CM) {
            return -1.0f;
        }
    }
    
    return avg;
}

// Binary search to find edge
static float find_edge_binary_search(float start_angle, float end_angle, 
                                      float center_dist, bool search_left) {
    float low = start_angle;
    float high = end_angle;
    float last_valid_angle = search_left ? SERVO_CENTER_ANGLE : SERVO_CENTER_ANGLE;
    float last_valid_dist = center_dist;
    
    int iterations = 0;
    const int max_iterations = 8;
    
    while ((high - low) > 2.0f && iterations < max_iterations) {
        float mid = (low + high) / 2.0f;
        float dist = measure_distance_at_angle(mid, EDGE_CONFIRMATION_SAMPLES);
        
        if (dist < 0.0f) {
            if (search_left) {
                high = mid;
            } else {
                low = mid;
            }
            iterations++;
            continue;
        }
        
        float dist_increase = dist - center_dist;
        bool found_edge = (dist_increase > (center_dist * 0.5f) && 
                           dist > MAX_EDGE_DISTANCE_CM);
        
        if (found_edge) {
            last_valid_angle = mid;
            last_valid_dist = dist;
            
            if (search_left) {
                high = mid;
            } else {
                low = mid;
            }
        } else {
            if (search_left) {
                low = mid;
            } else {
                high = mid;
            }
        }
        
        iterations++;
    }
    
    return last_valid_angle;
}

// Calculate obstacle width from edges
static float calculate_obstacle_width(float left_angle, float right_angle, float distance_cm) {
    if (distance_cm <= 0.0f) {
        return -1.0f;
    }
    
    float left_rad = (left_angle - SERVO_CENTER_ANGLE) * (3.14159265359f / 180.0f);
    float right_rad = (right_angle - SERVO_CENTER_ANGLE) * (3.14159265359f / 180.0f);
    
    float left_x = distance_cm * sinf(left_rad);
    float right_x = distance_cm * sinf(right_rad);
    
    float width = abs_float(left_x - right_x);
    
    if (width < 0.0f || width > MAX_REASONABLE_WIDTH_CM) {
        return -1.0f;
    }
    
    return width;
}

// Main width scan sequence
static void run_width_scan_sequence(float initial_distance) {
    printf("\n[SCAN] ===== Obstacle Width Measurement =====\n");
    printf("[SCAN] Initial distance: %.2f cm\n", initial_distance);
    
    stop_motor();
    sleep_ms(INITIAL_STOP_DELAY_MS);
    
    float center_dist = measure_distance_at_angle(SERVO_CENTER_ANGLE, EDGE_CONFIRMATION_SAMPLES);
    if (center_dist < 0.0f || center_dist < MIN_DISTANCE_FOR_WIDTH_SCAN) {
        printf("[SCAN] Invalid center distance, aborting scan\n");
        servo_set_angle(SERVO_CENTER_ANGLE);
        return;
    }
    
    printf("[SCAN] Center distance confirmed: %.2f cm\n", center_dist);
    
    // Find edges
    float left_edge_angle = find_edge_binary_search(
        SERVO_CENTER_ANGLE, SERVO_LEFT_ANGLE, center_dist, true);
    
    float right_edge_angle = find_edge_binary_search(
        SERVO_CENTER_ANGLE, SERVO_RIGHT_ANGLE, center_dist, false);
    
    printf("[SCAN] Left edge at %.1f°, Right edge at %.1f°\n", 
           left_edge_angle, right_edge_angle);
    
    // Calculate widths
    float left_w = calculate_obstacle_width(left_edge_angle, SERVO_CENTER_ANGLE, center_dist);
    float right_w = calculate_obstacle_width(SERVO_CENTER_ANGLE, right_edge_angle, center_dist);
    float total_w = calculate_obstacle_width(left_edge_angle, right_edge_angle, center_dist);
    
    // Publish results
    if (mqtt_is_connected()) {
        char json[256];
        int n = snprintf(json, sizeof(json),
                         "{\"center_dist\":%.2f,\"left_angle\":%.1f,\"right_angle\":%.1f,"
                         "\"left_width\":%.2f,\"right_width\":%.2f,\"total_width\":%.2f}",
                         (double)center_dist,
                         (double)left_edge_angle,
                         (double)right_edge_angle,
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