#ifndef IR_H
#define IR_H
#include <stdbool.h>
#include <stdint.h>
void ir_init(void);
bool ir_read_digital(void);
uint16_t ir_read_adc_raw(void);
#endif
