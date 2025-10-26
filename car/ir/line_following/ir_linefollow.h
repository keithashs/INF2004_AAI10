#ifndef IR_H
#define IR_H

#include "pico/stdlib.h"

// IR sensor pins (change if needed)
#define IR_LEFT_PIN   1
#define IR_RIGHT_PIN  28

// If IR sensor outputs LOW on black line, set to 1
// If it outputs HIGH on black line, set to 0
#define IR_BLACK_IS_LOW  1

// Initialize IR sensor GPIOs
void ir_init(void);

// Read digital states (raw 0 or 1)
int ir_read_left_raw(void);
int ir_read_right_raw(void);

// Check whether each sensor detects the black line
int ir_left_is_black(void);
int ir_right_is_black(void);

#endif
