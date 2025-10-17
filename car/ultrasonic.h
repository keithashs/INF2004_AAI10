<<<<<<< Updated upstream
#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <stdint.h>

void ultrasonic_init(void);
/** Returns distance in centimeters; returns 0 if timeout/no echo. */
uint32_t ultrasonic_read_cm(void);

=======
#pragma once
#include "pico/types.h"
#include "pico/stdlib.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void     setupUltrasonicPins(uint trigPin, uint echoPin);
uint32_t ultrasonic_get_cm  (uint trigPin, uint echoPin);
uint32_t getInch(uint trigPin, uint echoPin);

#ifdef __cplusplus
} // extern "C"
>>>>>>> Stashed changes
#endif
