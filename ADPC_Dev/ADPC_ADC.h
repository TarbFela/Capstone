#ifndef ADPC_ADC_H
#define ADPC_ADC_H

#include "../src2/mcp3x6xR_driver/mcp3x6xR.h"
#include "../src2/mcp3x6xR_driver/mcp_pio.h"


extern mcp_info_t mcp_1;
extern mcp_pio_t mpio_1;

int adpc_adc_init(void (*dma_handler)(void));
int adpc_adc_start();

#endif