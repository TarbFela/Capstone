//
// Created by The Tragedy of Darth Wise on 12/8/25.
//

#ifndef CAPSTONE_PWM_H
#define CAPSTONE_PWM_H

#include "hardware/pwm.h"
#include "hardware/gpio.h"

#define PWM1_GPIO_PIN 0
#define PWM2_GPIO_PIN 1
#define PWM3_GPIO_PIN 2
#define PWM4_GPIO_PIN 3

#define capstone_pwm_set_hi_level(d) pwm_set_gpio_level(PWM1_GPIO_PIN,d);pwm_set_gpio_level(PWM3_GPIO_PIN,d);
#define capstone_pwm_set_lo_level(d) pwm_set_gpio_level(PWM2_GPIO_PIN,d);pwm_set_gpio_level(PWM4_GPIO_PIN,d);



void capstone_pwm_init();

void capstone_pwm_start();
void capstone_pwm_stop();




#endif //CAPSTONE_PWM_H
