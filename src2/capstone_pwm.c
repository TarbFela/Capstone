//
// Created by The Tragedy of Darth Wise on 12/8/25.
//
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "capstone_pwm.h"

void capstone_pwm_init() {
    uint gpio1 = PWM1_GPIO_PIN;
    uint gpio2 = PWM2_GPIO_PIN;
    uint gpio3 = PWM3_GPIO_PIN;
    uint gpio4 = PWM4_GPIO_PIN;

    uint slice_num1 = pwm_gpio_to_slice_num(gpio1);
    uint slice_num2 = pwm_gpio_to_slice_num(gpio2);
    uint slice_num3 = pwm_gpio_to_slice_num(gpio3);
    uint slice_num4 = pwm_gpio_to_slice_num(gpio4);
    pwm_set_irq_enabled(slice_num1,false);
    pwm_set_irq_enabled(slice_num2,false);
    pwm_set_irq_enabled(slice_num3,false);
    pwm_set_irq_enabled(slice_num4,false);

    uint chan_num1 = pwm_gpio_to_channel(gpio1);
    uint chan_num2 = pwm_gpio_to_channel(gpio2);
    uint chan_num3 = pwm_gpio_to_channel(gpio3);
    uint chan_num4 = pwm_gpio_to_channel(gpio4);

    gpio_set_function(PWM1_GPIO_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM2_GPIO_PIN, GPIO_FUNC_PWM);

    gpio_set_function(PWM3_GPIO_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM4_GPIO_PIN, GPIO_FUNC_PWM);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, 1);

    config.top = 1000; //125MHz --> 125kHz
    pwm_init(slice_num1, &config, false);
    pwm_init(slice_num2, &config, false);
    pwm_init(slice_num3, &config, false);
    pwm_init(slice_num4, &config, false);

    pwm_set_counter(slice_num3, 500); // 50% offset...
    pwm_set_gpio_level(PWM2_GPIO_PIN, 0);
    pwm_set_gpio_level(PWM4_GPIO_PIN, 0);

    pwm_set_mask_enabled(0x1 << slice_num1 | 0x1 << slice_num2 | 0x1 << slice_num3 | 0x1 << slice_num4);
}
