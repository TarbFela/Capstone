#ifndef ADPC_APP_FUNCS_H
#define ADPC_APP_FUNCS_H

#include "adpc_app_funcs.h"


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "tusb.h"

#include <string.h>

#include "mcp3x6xR_driver/mcp3x6xR.h"
#include "mcp3x6xR_driver/mcp_pio.h"
#include "ada4255_driver/ada4255.h"

#include "../ADPC_Dev/ADPC_cfg.h"
#include "../ADPC_Dev/ADPC_ADC.h"
#include "../ADPC_Dev/adpc_gpio_pwm.h"

#include "adpc_app.h"

extern ada_info_t ada;

// if none, 0, if A, 1, if B, 2
extern volatile int dma0_last_written;
extern volatile int dma1_last_written;


void dma_irq_handler_1(void);
void dma_irq_handler_0(void);
app_result_t adpc_init();

app_result_t app_cmd_rstream(app_state_t *s);


#endif
