#include "ir.h"

static inline int is_black(int raw)
{
    return IR_BLACK_IS_LOW ? (raw == 0) : (raw == 1);
}

void ir_init(void)
{
    gpio_init(IR_LEFT_PIN);
    gpio_set_dir(IR_LEFT_PIN, GPIO_IN);
    //gpio_pull_up(IR_LEFT_PIN);

    gpio_init(IR_RIGHT_PIN);
    gpio_set_dir(IR_RIGHT_PIN, GPIO_IN);
    //gpio_pull_up(IR_RIGHT_PIN);
}

int ir_read_left_raw(void)
{
    return gpio_get(IR_LEFT_PIN);
}

int ir_read_right_raw(void)
{
    return gpio_get(IR_RIGHT_PIN);
}

int ir_left_is_black(void)
{
    return is_black(ir_read_left_raw());
}

int ir_right_is_black(void)
{
    return is_black(ir_read_right_raw());
}
