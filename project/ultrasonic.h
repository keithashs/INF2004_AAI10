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
#endif
