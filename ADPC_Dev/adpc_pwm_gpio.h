#ifndef ADPC_PWM_GPIO_H
#define ADPC_PWM_GPIO_H 

#inlcude "hardware/gpio.h"
#include "hardware/pwm.h"

void mphb_gpio_init(int pwm_ab_pin, int pwm_cd_pin, int ph_en_pin);

#endif
