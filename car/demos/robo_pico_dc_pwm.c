// motor_calibration.c
// Simple calibration: Run motors at constant PWM for 10s, display encoder telemetry
// All functions in one file - no PID, no IMU, just raw PWM and encoder measurements

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// ==================== Pin Definitions ====================
// Motor pins - TB6612FNG driver
#define PWM_M2A  10    // Left motor reverse
#define PWM_M2B  11    // Left motor forward
#define PWM_M1A  8     // Right motor forward
#define PWM_M1B  9     // Right motor reverse

// Encoder pins
#define L_ENCODER_OUT 6
#define R_ENCODER_OUT 16

// Button pins
#define EXECUTE_BTN   21
#define EXECUTE_BTN2  20

// ==================== Encoder Constants ====================
#define PULSES_PER_REVOLUTION 20.0f
#define WHEEL_CIRCUMFERENCE   20.42f   // cm (wheel diameter = 6.5cm, π×d = 20.42cm)

// ==================== PWM Constants ====================
#define PWM_WRAP_VALUE 255
#define PWM_CLOCK_DIV  125.0f

// Target PWM for approximately 20cm/s (tune this based on results)
// Starting estimate: 130-140 should give around 15-25 cm/s
#define TARGET_PWM_LEFT  115
#define TARGET_PWM_RIGHT 115

// Test duration
#define TEST_DURATION_SEC 5

// ==================== Encoder Data Structures ====================
typedef struct {
    volatile uint32_t pulse_count;
    volatile uint64_t timestamp_us;
} EncoderData;

typedef struct {
    volatile EncoderData data;
    volatile EncoderData last;
    SemaphoreHandle_t    mutex;
} Encoder;

static Encoder left_encoder  = { .data = {0,0}, .last = {0,0}, .mutex = NULL };
static Encoder right_encoder = { .data = {0,0}, .last = {0,0}, .mutex = NULL };

// ==================== Encoder Functions ====================
void encoder_isr(uint gpio, uint32_t events) {
    Encoder *enc = NULL;
    
    if (gpio == L_ENCODER_OUT) {
        enc = &left_encoder;
    } else if (gpio == R_ENCODER_OUT) {
        enc = &right_encoder;
    }
    
    if (!enc) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (xSemaphoreTakeFromISR(enc->mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
        enc->last = enc->data;
        enc->data.pulse_count++;
        enc->data.timestamp_us = time_us_64();
        xSemaphoreGiveFromISR(enc->mutex, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void encoder_init(void) {
    // Initialize encoder input pins
    gpio_init(L_ENCODER_OUT);
    gpio_set_dir(L_ENCODER_OUT, GPIO_IN);
    gpio_pull_up(L_ENCODER_OUT);
    
    gpio_init(R_ENCODER_OUT);
    gpio_set_dir(R_ENCODER_OUT, GPIO_IN);
    gpio_pull_up(R_ENCODER_OUT);
    
    // Create mutexes
    left_encoder.mutex  = xSemaphoreCreateMutex();
    right_encoder.mutex = xSemaphoreCreateMutex();
    
    // Setup interrupts - rising edge detection
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &encoder_isr);
    gpio_set_irq_enabled(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true);
    
    printf("[ENCODER] Initialized\n");
}

float get_distance_cm(Encoder *enc) {
    float distance = 0.0f;
    
    if (xSemaphoreTake(enc->mutex, portMAX_DELAY) == pdTRUE) {
        // Distance per pulse
        float dpp = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
        distance = dpp * (float)enc->data.pulse_count;
        xSemaphoreGive(enc->mutex);
    }
    
    return distance;
}

float get_speed_cmps(Encoder *enc) {
    float speed = -1.0f;  // Invalid speed marker
    
    if (xSemaphoreTake(enc->mutex, portMAX_DELAY) == pdTRUE) {
        EncoderData now = enc->data;
        EncoderData last = enc->last;
        xSemaphoreGive(enc->mutex);
        
        // Calculate time difference (in seconds)
        float dt = (float)((int64_t)now.timestamp_us - (int64_t)last.timestamp_us) / 1e6f;
        
        // Calculate pulse difference
        float dp = (float)(now.pulse_count - last.pulse_count);
        
        // Check if data is fresh (within last 1 second)
        float age = (float)(time_us_64() - now.timestamp_us) / 1e6f;
        
        if (dt > 0.0f && age < 1.0f && dp > 0.0f) {
            float dpp = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
            float distance = dpp * dp;
            speed = distance / dt;  // cm/s
        } else {
            speed = 0.0f;  // Stopped
        }
    }
    
    return speed;
}

uint32_t get_pulse_count(Encoder *enc) {
    uint32_t count = 0;
    
    if (xSemaphoreTake(enc->mutex, portMAX_DELAY) == pdTRUE) {
        count = enc->data.pulse_count;
        xSemaphoreGive(enc->mutex);
    }
    
    return count;
}

void reset_encoders(void) {
    if (xSemaphoreTake(left_encoder.mutex, portMAX_DELAY) == pdTRUE) {
        left_encoder.data.pulse_count = 0;
        left_encoder.data.timestamp_us = 0;
        left_encoder.last = left_encoder.data;
        xSemaphoreGive(left_encoder.mutex);
    }
    
    if (xSemaphoreTake(right_encoder.mutex, portMAX_DELAY) == pdTRUE) {
        right_encoder.data.pulse_count = 0;
        right_encoder.data.timestamp_us = 0;
        right_encoder.last = right_encoder.data;
        xSemaphoreGive(right_encoder.mutex);
    }
}

// ==================== Motor Functions ====================
void motor_pwm_init(void) {
    // Configure all motor pins as PWM
    gpio_set_function(PWM_M2A, GPIO_FUNC_PWM);
    gpio_set_function(PWM_M2B, GPIO_FUNC_PWM);
    gpio_set_function(PWM_M1A, GPIO_FUNC_PWM);
    gpio_set_function(PWM_M1B, GPIO_FUNC_PWM);
    
    // Get PWM slices for each pin
    uint slice_m2a = pwm_gpio_to_slice_num(PWM_M2A);
    uint slice_m2b = pwm_gpio_to_slice_num(PWM_M2B);
    uint slice_m1a = pwm_gpio_to_slice_num(PWM_M1A);
    uint slice_m1b = pwm_gpio_to_slice_num(PWM_M1B);
    
    // Configure PWM slices
    uint slices[] = {slice_m2a, slice_m2b, slice_m1a, slice_m1b};
    for (int i = 0; i < 4; i++) {
        pwm_set_wrap(slices[i], PWM_WRAP_VALUE);
        pwm_set_clkdiv(slices[i], PWM_CLOCK_DIV);
        pwm_set_enabled(slices[i], true);
    }
    
    // Start with motors stopped
    pwm_set_gpio_level(PWM_M2A, 0);
    pwm_set_gpio_level(PWM_M2B, 0);
    pwm_set_gpio_level(PWM_M1A, 0);
    pwm_set_gpio_level(PWM_M1B, 0);
    
    printf("[MOTOR] PWM Initialized (wrap=%d, div=%.1f)\n", PWM_WRAP_VALUE, PWM_CLOCK_DIV);
}

void set_motor_left_forward(uint16_t pwm_value) {
    pwm_set_gpio_level(PWM_M2B, pwm_value);  // Forward
    pwm_set_gpio_level(PWM_M2A, 0);          // Reverse OFF
}

void set_motor_right_forward(uint16_t pwm_value) {
    pwm_set_gpio_level(PWM_M1A, pwm_value);  // Forward
    pwm_set_gpio_level(PWM_M1B, 0);          // Reverse OFF
}

void stop_motors(void) {
    pwm_set_gpio_level(PWM_M2A, 0);
    pwm_set_gpio_level(PWM_M2B, 0);
    pwm_set_gpio_level(PWM_M1A, 0);
    pwm_set_gpio_level(PWM_M1B, 0);
}

// ==================== Button Functions ====================
void button_init(void) {
    gpio_init(EXECUTE_BTN);
    gpio_set_dir(EXECUTE_BTN, GPIO_IN);
    gpio_pull_up(EXECUTE_BTN);
    
    gpio_init(EXECUTE_BTN2);
    gpio_set_dir(EXECUTE_BTN2, GPIO_IN);
    gpio_pull_up(EXECUTE_BTN2);
    
    printf("[BUTTON] Initialized (active LOW)\n");
}

bool is_button_pressed(void) {
    return (gpio_get(EXECUTE_BTN) == 0) || (gpio_get(EXECUTE_BTN2) == 0);
}

// ==================== Calibration Task ====================
void calibration_task(void *params) {
    (void)params;
    
    printf("\n=================================================\n");
    printf("   MOTOR CALIBRATION MODE\n");
    printf("=================================================\n");
    printf("Target Speed: ~20 cm/s\n");
    printf("Test Duration: %d seconds\n", TEST_DURATION_SEC);
    printf("PWM Left: %d, PWM Right: %d\n", TARGET_PWM_LEFT, TARGET_PWM_RIGHT);
    printf("\nPress button to start...\n");
    printf("=================================================\n\n");
    
    // Wait for button press
    while (!is_button_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Debounce
    vTaskDelay(pdMS_TO_TICKS(200));
    
    printf("\n[CAL] Starting test in 3 seconds...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("[CAL] 2...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("[CAL] 1...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Reset encoders
    reset_encoders();
    
    // Start motors
    printf("\n[CAL] MOTORS ON!\n");
    printf("=================================================\n");
    printf("Time(s) | L_Pulses | R_Pulses | L_Dist(cm) | R_Dist(cm) | L_Speed(cm/s) | R_Speed(cm/s) | Avg_Speed(cm/s)\n");
    printf("---------+----------+----------+------------+------------+---------------+---------------+----------------\n");
    
    set_motor_left_forward(TARGET_PWM_LEFT);
    set_motor_right_forward(TARGET_PWM_RIGHT);
    
    uint64_t start_time_us = time_us_64();
    uint32_t elapsed_sec = 0;
    
    // Run test for specified duration
    while (elapsed_sec < TEST_DURATION_SEC) {
        vTaskDelay(pdMS_TO_TICKS(500));  // Update every 500ms
        
        uint64_t now_us = time_us_64();
        float elapsed_s = (float)(now_us - start_time_us) / 1e6f;
        elapsed_sec = (uint32_t)elapsed_s;
        
        // Read encoder data
        uint32_t left_pulses = get_pulse_count(&left_encoder);
        uint32_t right_pulses = get_pulse_count(&right_encoder);
        
        float left_dist = get_distance_cm(&left_encoder);
        float right_dist = get_distance_cm(&right_encoder);
        
        float left_speed = get_speed_cmps(&left_encoder);
        float right_speed = get_speed_cmps(&right_encoder);
        
        // Calculate average
        float avg_speed = 0.0f;
        if (left_speed >= 0.0f && right_speed >= 0.0f) {
            avg_speed = (left_speed + right_speed) / 2.0f;
        }
        
        // Print telemetry
        printf("  %5.1f  |  %6lu  |  %6lu  |   %7.2f  |   %7.2f  |     %8.2f  |     %8.2f  |     %9.2f\n",
               (double)elapsed_s,
               (unsigned long)left_pulses,
               (unsigned long)right_pulses,
               (double)left_dist,
               (double)right_dist,
               (double)left_speed,
               (double)right_speed,
               (double)avg_speed);
    }
    
    // Stop motors
    stop_motors();
    printf("=================================================\n");
    printf("[CAL] MOTORS OFF - Test complete!\n\n");
    
    // Final summary
    uint32_t final_left_pulses = get_pulse_count(&left_encoder);
    uint32_t final_right_pulses = get_pulse_count(&right_encoder);
    float final_left_dist = get_distance_cm(&left_encoder);
    float final_right_dist = get_distance_cm(&right_encoder);
    float avg_left_speed = final_left_dist / (float)TEST_DURATION_SEC;
    float avg_right_speed = final_right_dist / (float)TEST_DURATION_SEC;
    
    printf("=================================================\n");
    printf("   CALIBRATION RESULTS\n");
    printf("=================================================\n");
    printf("Test Duration:     %d seconds\n", TEST_DURATION_SEC);
    printf("PWM Values:        L=%d, R=%d\n", TARGET_PWM_LEFT, TARGET_PWM_RIGHT);
    printf("\nLeft Wheel:\n");
    printf("  Total Pulses:    %lu\n", (unsigned long)final_left_pulses);
    printf("  Total Distance:  %.2f cm\n", (double)final_left_dist);
    printf("  Average Speed:   %.2f cm/s\n", (double)avg_left_speed);
    printf("\nRight Wheel:\n");
    printf("  Total Pulses:    %lu\n", (unsigned long)final_right_pulses);
    printf("  Total Distance:  %.2f cm\n", (double)final_right_dist);
    printf("  Average Speed:   %.2f cm/s\n", (double)avg_right_speed);
    printf("\nDifference:\n");
    printf("  Pulse Diff:      %ld (L-R)\n", (long)(final_left_pulses - final_right_pulses));
    printf("  Distance Diff:   %.2f cm\n", (double)(final_left_dist - final_right_dist));
    printf("  Speed Diff:      %.2f cm/s\n", (double)(avg_left_speed - avg_right_speed));
    printf("=================================================\n\n");
    
    // Infinite loop - test complete
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==================== Main ====================
int main(void) {
    // Initialize stdio
    stdio_init_all();
    
    // Wait for USB serial connection
    sleep_ms(3000);
    
    printf("\n\n");
    printf("=================================================\n");
    printf("   MOTOR & ENCODER CALIBRATION TOOL\n");
    printf("=================================================\n");
    printf("Version: 1.0\n");
    printf("Hardware: Raspberry Pi Pico + TB6612FNG\n");
    printf("Encoder: 20-tick discs\n");
    printf("Wheels: 6.5cm diameter (20.42cm circumference)\n");
    printf("=================================================\n\n");
    
    // Initialize hardware
    button_init();
    motor_pwm_init();
    encoder_init();
    
    printf("[INIT] All hardware initialized\n");
    printf("[INIT] Creating FreeRTOS task...\n\n");
    
    // Create calibration task
    xTaskCreate(calibration_task, 
                "CalTask", 
                configMINIMAL_STACK_SIZE * 4, 
                NULL, 
                tskIDLE_PRIORITY + 1, 
                NULL);
    
    // Start FreeRTOS scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    while (true) {
        tight_loop_contents();
    }
    
    return 0;
}