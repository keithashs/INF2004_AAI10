#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pin.h"

void ir_init(void) {
    gpio_init(IR_DIGITAL_GPIO); gpio_set_dir(IR_DIGITAL_GPIO, GPIO_IN); gpio_pullup(IR_DIGITAL_GPIO);
    adc_init(); adc_gpio_init(IR_ADC_GPIO); // ADC0 on GP26
}
bool ir_read_digital(void) { return gpio_get(IR_DIGITAL_GPIO); }
uint16_t ir_read_adc_raw(void) { adc_select_input(0); return adc_read(); }
