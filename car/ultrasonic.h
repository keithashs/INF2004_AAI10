#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <stdint.h>

void ultrasonic_init(void);
/** Returns distance in centimeters; returns 0 if timeout/no echo. */
uint32_t ultrasonic_read_cm(void);

#endif
