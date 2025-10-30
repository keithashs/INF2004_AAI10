#include "encoder.h"

static Encoder left_encoder  = { .data = {0,0}, .last = {0,0}, .mutex = NULL };
static Encoder right_encoder = { .data = {0,0}, .last = {0,0}, .mutex = NULL };

void read_encoder_pulse(uint gpio, uint32_t events) {
    Encoder *enc = NULL;
    if (gpio == L_ENCODER_OUT)      enc = &left_encoder;
    else if (gpio == R_ENCODER_OUT) enc = &right_encoder;
    if (!enc) return;

    BaseType_t hpw = pdFALSE;
    if (xSemaphoreTakeFromISR(enc->mutex, &hpw) == pdTRUE) {
        enc->last = enc->data;
        enc->data.pulse_count++;
        enc->data.timestamp_us = time_us_64();
        xSemaphoreGiveFromISR(enc->mutex, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

static inline float distance_cm_from_pulses(uint32_t pulses) {
    const float dpp = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
    return dpp * (float)pulses;
}

static float get_distance_from(Encoder *enc) {
    float d = 0.f;
    if (xSemaphoreTake(enc->mutex, portMAX_DELAY) == pdTRUE) {
        d = distance_cm_from_pulses(enc->data.pulse_count);
        xSemaphoreGive(enc->mutex);
    }
    return d;
}

float get_left_distance(void)   { return get_distance_from(&left_encoder); }
float get_right_distance(void)  { return get_distance_from(&right_encoder); }
float get_average_distance(void){ return 0.5f*(get_left_distance()+get_right_distance()); }

static float get_speed_from(Encoder *enc) {
    float v = INVALID_SPEED;
    if (xSemaphoreTake(enc->mutex, portMAX_DELAY) == pdTRUE) {
        const EncoderData now = enc->data;
        const EncoderData last= enc->last;
        xSemaphoreGive(enc->mutex);

        const float dp = (float)(now.pulse_count - last.pulse_count);
        const float dt = (float)((int64_t)now.timestamp_us - (int64_t)last.timestamp_us) / 1e6f;
        const float age= (float)(time_us_64() - now.timestamp_us) / 1e6f;

        if (dt > 0.f && age < 1.0f && dp > 0.f) {
            v = distance_cm_from_pulses((uint32_t)dp) / dt; // cm/s
        }
    }
    return v;
}

float get_left_speed(void)   { return get_speed_from(&left_encoder); }
float get_right_speed(void)  { return get_speed_from(&right_encoder); }
float get_average_speed(void){
    float vl = get_left_speed(), vr = get_right_speed();
    if (vl <= INVALID_SPEED || vr <= INVALID_SPEED) return INVALID_SPEED;
    return 0.5f*(vl+vr);
}

static void reset_one(Encoder *enc) {
    if (xSemaphoreTake(enc->mutex, portMAX_DELAY) == pdTRUE) {
        enc->data.pulse_count = 0;
        enc->data.timestamp_us = 0;
        enc->last = enc->data;
        xSemaphoreGive(enc->mutex);
    }
}

void reset_left_encoder(void)  { reset_one(&left_encoder); }
void reset_right_encoder(void) { reset_one(&right_encoder); }
void reset_encoders(void)      { reset_one(&left_encoder); reset_one(&right_encoder); }

void encoder_init(void) {
    gpio_init(L_ENCODER_POW); gpio_set_dir(L_ENCODER_POW, GPIO_OUT);
    gpio_init(L_ENCODER_OUT); gpio_set_dir(L_ENCODER_OUT, GPIO_IN); gpio_pull_up(L_ENCODER_OUT);

    gpio_init(R_ENCODER_POW); gpio_set_dir(R_ENCODER_POW, GPIO_OUT);
    gpio_init(R_ENCODER_OUT); gpio_set_dir(R_ENCODER_OUT, GPIO_IN); gpio_pull_up(R_ENCODER_OUT);

    gpio_put(L_ENCODER_POW, 1);
    gpio_put(R_ENCODER_POW, 1);

    left_encoder.mutex  = xSemaphoreCreateMutex();
    right_encoder.mutex = xSemaphoreCreateMutex();

    // Register the common ISR once; enable it for both pins.
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &read_encoder_pulse);
    gpio_set_irq_enabled(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true);
}