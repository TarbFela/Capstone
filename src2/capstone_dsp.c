//
// Created by The Tragedy of Darth Wise on 12/12/25.
//

#include "capstone_dsp.h"
#include "hardware/pwm.h"
#include "capstone_pwm.h"
#include "hardware/gpio.h"

void __not_in_flash_func(process_ISNS)(uint16_t *sample, void *other_stuff) {
//    static int y_k_IS = 0;
//    int PI_setpoint = ((PI_controller_t *)(other_stuff))->PI_SP;
//    PI_setpoint *= 62;
//    int err_IS = PI_setpoint - (*sample);
//    y_k_IS += err_IS * 11;
//    if(y_k_IS < -1000000000) y_k_IS = -1000000000;
//    if(y_k_IS > 1000000000) y_k_IS = 1000000000;
//
//    int d = ((8*y_k_IS)>>19) + (err_IS>>7);
//
//    if(d>320) d = 320;
//    if(d<110) d = 110;
//
//    pwm_set_gpio_level(PWM1_GPIO_PIN,d);
//    pwm_set_gpio_level(PWM3_GPIO_PIN,d);




//    ((PI_controller_t *)(other_stuff))->d = d;
//    ((PI_controller_t *)(other_stuff))->y_k_IS = y_k_IS;
}
void __not_in_flash_func(process_TSNS)(uint16_t *sample, void *other_stuff) {
}

void PI_controller_init(PI_controller_t *pic,
                        int PI_SP,
                        int y_k_IS_SP_scalar,
                        int y_k_IS_ADC_scalar,
                        int err_k_IS_SP_scalar,
                        int err_k_IS_ADC_scalar,
                        int y_k_IS_abs_bounds,
                        int d_bound_lower,
                        int d_bound_upper) {
    pic->PI_SP = PI_SP;
    pic->y_k_IS_SP_scalar = y_k_IS_SP_scalar;
    pic->y_k_IS_ADC_scalar = y_k_IS_ADC_scalar;
    pic->err_k_IS_SP_scalar = err_k_IS_SP_scalar;
    pic->err_k_IS_ADC_scalar = err_k_IS_ADC_scalar;
    pic->y_k_IS_abs_bounds = y_k_IS_abs_bounds;
    pic->d_bound_lower = d_bound_lower;
    pic->d_bound_upper = d_bound_upper;
    pic->y_k_IS = 0;
    pic->controller_paused = 0;
}

void __not_in_flash_func(PI_controller_DSP)(PI_controller_t *pic, uint16_t sample) {
//
//    pic->y_k_IS += pic->y_k_IS_SP_scalar * pic->PI_SP - pic->y_k_IS_ADC_scalar * sample;
//    if(pic->y_k_IS > pic->y_k_IS_abs_bounds) pic->y_k_IS = pic->y_k_IS_abs_bounds;
//    if(pic->y_k_IS < -pic->y_k_IS_abs_bounds) pic->y_k_IS = -pic->y_k_IS_abs_bounds;
//
//    pic->d_IS = pic->y_k_IS + pic->err_k_IS_SP_scalar * pic->PI_SP - pic->err_k_IS_ADC_scalar * sample;
//    int d = (pic->d_IS >> 16);
//    if(d > pic->d_bound_upper) d = pic->d_bound_upper;
//    if(d < pic->d_bound_lower) d = pic->d_bound_lower;

//    pic->d = d;
    pic->y_k_IS += 569*pic->PI_SP - 9*sample;
    if(pic->y_k_IS>50000000) pic->y_k_IS=50000000;
    if(pic->y_k_IS<-50000000) pic->y_k_IS=-50000000;
    pic->d_IS = pic->y_k_IS + 27314*pic->PI_SP - 440*sample;
    pic->d = pic->d_IS>>16;
    if(pic->d>400) pic->d = 400;
    if(pic->d<110) pic->d = 110;
    /*
            y_k_IS += 569*PI_setpoint - 9*sample;
            if(y_k_IS>500000000) y_k_IS=5000000;
            if(y_k_IS<-500000000) y_k_IS=-5000000;
            d_IS = y_k_IS + 27314*PI_setpoint - 440*sample;
            d = d_IS>>16;

            if(!((i)%0x3FFFF)) {
                printf("d: %04d ykIS: %08d Imeas: %0.3fA\n",d,y_k_IS,(float)sample * 20.0*3.3/4096.0);
            }

            if(d>330) d = 200;
            if(d<110) d = 110;
            */
}


