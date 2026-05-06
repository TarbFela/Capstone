#ifndef ADPC_GPIO_PWM_H
#define ADPC_GPIO_PWM_H

#include "pico/stdlib.h"
#include "hardware/gpio.h
#include "hardware/pwm.h"

typedef struct {
    int pwm_a_pin;
    int pwm_c_pin;
    int ph_en_pin;
    uint ch_A;
    uint ch_C;
    uint slice;
} mphb2_gpio_pwm_t;

// initialize gpio pins, pwm channels, and populate the info struct.
void mphb_gpio_init(mphb2_gpio_pwm_t *s, int pwm_ab_pin, int pwm_cd_pin, int ph_en_pin);

#endif