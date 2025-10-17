#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

void motor_init(void);

/** Speed in range [-1.0 .. +1.0]; internally maps to 2-pin drive with PWM. */
void motor_set_left(float speed);
void motor_set_right(float speed);

/** Convenience: stop both. */
void motor_brake(void);

#endif
