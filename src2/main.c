#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
//#include <math.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"

#include "capstone_pwm.h"


#define PMIC_I2C_SDA_PIN 8
#define PMIC_I2C_SCL_PIN 9
#define PMIC_I2C i2c0

int main(void) {
    stdio_init_all();
    sleep_ms(5000);

    capstone_pwm_init();
    printf("Hello Capstone World!\n");

    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PMIC_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PMIC_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PMIC_I2C_SDA_PIN);
    gpio_pull_up(PMIC_I2C_SCL_PIN);

    char ui = 0;
    int latch = 0;
    uint16_t D1_thresh = 0;
    uint16_t D1_thresh_setting_offset = 0;
    uint16_t D1_thresh_setting_multiplier = 2;
    while(1) {
        scanf("%c",&ui);
        if(ui=='q') break;
        if(ui == ' ') {
            if(latch) {
                latch = 0;
                pwm_set_gpio_level(PWM1_GPIO_PIN, 0);
                pwm_set_gpio_level(PWM2_GPIO_PIN, 0);
                pwm_set_gpio_level(PWM3_GPIO_PIN, 0);
                pwm_set_gpio_level(PWM4_GPIO_PIN, 0);
                printf("PWM OFF\n");
            }
            else {
                latch = 1;
                pwm_set_gpio_level(PWM1_GPIO_PIN, D1_thresh);
                pwm_set_gpio_level(PWM2_GPIO_PIN, 150);
                pwm_set_gpio_level(PWM3_GPIO_PIN, D1_thresh);
                pwm_set_gpio_level(PWM4_GPIO_PIN, 150);
                printf("PWM ON, D = %.2f\n",100.0*(float)D1_thresh/1000);
            }
        }
        if(ui == 'o') {
            if(D1_thresh_setting_offset) {
                D1_thresh_setting_offset = 0;
                printf("BIAS SET TO 0 \n");
            }
            else {
                D1_thresh_setting_offset = 150;
                printf("BIAS SET TO 50 \n");
            }
        }
        if(ui == 'm') {
            D1_thresh_setting_multiplier = D1_thresh_setting_multiplier*1.5 + 1;
            if(D1_thresh_setting_multiplier>100) D1_thresh_setting_multiplier=1;
            printf("MULTIPLIER SET TO %d\n",D1_thresh_setting_multiplier);
        }
        if(ui >= '0' && ui <= '9') {
            D1_thresh = ((uint16_t)ui - '0')*D1_thresh_setting_multiplier + D1_thresh_setting_offset;
            printf("D = %.2f\n",100.0*(float)D1_thresh/1000);
            if(latch) {
                pwm_set_gpio_level(PWM1_GPIO_PIN, D1_thresh);
                pwm_set_gpio_level(PWM3_GPIO_PIN,D1_thresh);
            }
        }
        if(ui == 'p' || ui == 'l') {
            if(ui=='p') D1_thresh+= D1_thresh_setting_multiplier/4;
            if(ui=='l') D1_thresh+= -D1_thresh_setting_multiplier/4;
            printf("D = %.2f\n",100.0*(float)D1_thresh/1000);
            if(latch) {
                pwm_set_gpio_level(PWM1_GPIO_PIN, D1_thresh);
                pwm_set_gpio_level(PWM3_GPIO_PIN,D1_thresh);
            }
        }
        if(ui == 'i') {
            printf("Writing to INA236...\n");
            int INA236B_msg[3] = {7,0xAB,0xCD};
            i2c_write_timeout_us(i2c0, 0x48, INA236B_msg, 3, 1, 1000000);

        }

    }

    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
    return 0;
}