#include "encoder.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "config.h"   // expects ENCODER_PIN_M1 / ENCODER_PIN_M2

// ----- Internal counters -----
// Total counts (monotonic since last reset)
static volatile uint32_t enc_tot_m1 = 0;
static volatile uint32_t enc_tot_m2 = 0;

// Last snapshot used to produce "window" deltas
static volatile uint32_t prev_tot_m1 = 0;
static volatile uint32_t prev_tot_m2 = 0;

// Shared IRQ for both encoder pins
static void encoder_gpio_irq(uint gpio, uint32_t events) {
    (void)events;
    if (gpio == ENCODER_PIN_M1) {
        enc_tot_m1++;
    } else if (gpio == ENCODER_PIN_M2) {
        enc_tot_m2++;
    }
}

void encoder_init(void) {
    // Configure as inputs with pull-ups (common for reflective encoders)
    gpio_init(ENCODER_PIN_M1);
    gpio_set_dir(ENCODER_PIN_M1, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_M1);

    gpio_init(ENCODER_PIN_M2);
    gpio_set_dir(ENCODER_PIN_M2, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_M2);

    // Reset counters
    uint32_t s = save_and_disable_interrupts();
    enc_tot_m1 = enc_tot_m2 = 0;
    prev_tot_m1 = prev_tot_m2 = 0;
    restore_interrupts(s);

    // Enable IRQ on both edges to count stripes
    gpio_set_irq_enabled_with_callback(ENCODER_PIN_M1,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_gpio_irq);
    gpio_set_irq_enabled(ENCODER_PIN_M2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void encoder_reset_all(void) {
    uint32_t s = save_and_disable_interrupts();
    enc_tot_m1 = enc_tot_m2 = 0;
    prev_tot_m1 = prev_tot_m2 = 0;
    restore_interrupts(s);
}

void encoder_sample_window(uint16_t *win_m1, uint16_t *win_m2) {
    // Compute deltas since last sample (IRQ-safe)
    uint32_t s = save_and_disable_interrupts();
    uint32_t cur_m1 = enc_tot_m1;
    uint32_t cur_m2 = enc_tot_m2;
    uint32_t d1 = cur_m1 - prev_tot_m1;
    uint32_t d2 = cur_m2 - prev_tot_m2;
    prev_tot_m1 = cur_m1;
    prev_tot_m2 = cur_m2;
    restore_interrupts(s);

    if (win_m1) *win_m1 = (uint16_t)d1;
    if (win_m2) *win_m2 = (uint16_t)d2;
}

uint32_t encoder_total_m1(void) { return enc_tot_m1; }
uint32_t encoder_total_m2(void) { return enc_tot_m2; }
