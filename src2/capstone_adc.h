//
// Created by The Tragedy of Darth Wise on 12/12/25.
//

#ifndef CAPSTONE_PWM_CAPSTONE_ADC_H
#define CAPSTONE_PWM_CAPSTONE_ADC_H

#include "pico/stdlib.h"
#include <stdlib.h>
#include <stdbool.h>

#include "hardware/adc.h"
#include "hardware/dma.h"

#define ISNS_ADC_PIN 28
#define TSNS_ADC_PIN 27

#define ADC_BUFFER_SIZE 256
#define ADC_BUFFER_SIZE_WRAP_MASK 0xFF



typedef struct {
    uint adc_dma_daisy_chain[2];
    uint16_t *adc_dma_buffer;
    dma_channel_hw_t *adc_dma_daisy_chain_hw[2];
    dma_channel_config dma_cfg[2];
} capstone_adc_struct_t;

void capstone_adc_init(capstone_adc_struct_t *cas, void (handler_function)(void));

void capstone_adc_start(capstone_adc_struct_t *cas);
void capstone_adc_stop(capstone_adc_struct_t *cas);

#endif //CAPSTONE_PWM_CAPSTONE_ADC_H
