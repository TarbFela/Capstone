#ifndef ADPC_ADC_H
#define ADPC_ADC_H

#include "mcp3x6xR_driver/mcp3x6xR.h"
#include "mcp3x6xR_driver/mcp_pio.h"


extern mcp_info_t mcp;
extern mcp_pio_t mpio;

int adpc_adc_init(void (*dma_handler)(void));
int adpc_adc_start();

#endif