//
// Created by The Tragedy of Darth Wise on 12/12/25.
//

#ifndef CAPSTONE_PWM_CAPSTONE_DSP_H
#define CAPSTONE_PWM_CAPSTONE_DSP_H

#include <stdint.h>
#include <stdio.h>

// "IS" refers to "integer scaling" where I am scaling up the values to improve precision in stored values, which are bit-shifted down to usbale duty cycle values.
// "IS" variables *already factor in* PI constants (ki = 0.41677, kp = 0.00041677)
typedef struct {
    int PI_SP;
    int err_IS;
    int y_k_IS;
    int d_IS;
    int d;
    int y_k_IS_SP_scalar;
    int y_k_IS_ADC_scalar;
    int err_k_IS_SP_scalar;
    int err_k_IS_ADC_scalar;
    int y_k_IS_abs_bounds;
    int d_bound_lower;
    int d_bound_upper;
} PI_controller_t;

typedef struct {
    uint16_t **adc_dma_buffs;
} Global_DMA_IRQ_Info_t;

void process_ISNS(uint16_t *sample, void *other_stuff);
void process_TSNS(uint16_t *sample, void *other_stuff);

void PI_controller_init(PI_controller_t *pic,
                        int PI_SP,
                        int y_k_IS_SP_scalar,
                        int y_k_IS_ADC_scalar,
                        int err_k_IS_SP_scalar,
                        int err_k_IS_ADC_scalar,
                        int y_k_IS_abs_bounds,
                        int d_bound_lower,
                        int d_bound_upper);

void PI_controller_DSP(PI_controller_t *pic, uint16_t sample);

static void (*sample_processors[])(uint16_t *, void *) = {process_ISNS, process_TSNS};

#endif //CAPSTONE_PWM_CAPSTONE_DSP_H
