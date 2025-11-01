#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "semphr.h"

// Pins
// #define L_ENCODER_POW 26
#define L_ENCODER_OUT 6
// #define R_ENCODER_POW 17
#define R_ENCODER_OUT 16

// Wheel/disc geometry
#define PULSES_PER_REVOLUTION 20.0f
#define WHEEL_CIRCUMFERENCE   20.0f   // cm
#define WHEEL_TO_WHEEL_DISTANCE 10.8f // cm

#define INVALID_SPEED -1.0f

typedef struct {
    volatile uint32_t pulse_count;
    volatile uint64_t timestamp_us;
} EncoderData;

typedef struct {
    volatile EncoderData data;
    volatile EncoderData last;
    SemaphoreHandle_t    mutex;
} Encoder;

void  encoder_init(void);
void  read_encoder_pulse(uint gpio, uint32_t events);

// distances (cm)
float get_left_distance(void);
float get_right_distance(void);
float get_average_distance(void);

// speeds (cm/s)
float get_left_speed(void);
float get_right_speed(void);
float get_average_speed(void);

// reset
void  reset_left_encoder(void);
void  reset_right_encoder(void);
void  reset_encoders(void);

#endif