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


ada_info_t ada;

// if none, 0, if A, 1, if B, 2
volatile int dma0_last_written = 0;
volatile int dma1_last_written = 0;


void dma_irq_handler_1(void) {
    // clear the correct interrupt
    int culprit_is_a = dma_hw->ints1 & (1u << mpio_1.dma_a);
    if (culprit_is_a) {
        dma_hw->ints1 = 0x1 << (mpio_1.dma_a);
        dma1_last_written = 1;
    }
    else {
        dma_hw->ints1 = 0x1 << (mpio_1.dma_b);
        dma1_last_written = 2;
    }
}

void dma_irq_handler_0(void) {
    // clear the correct interrupt
    int culprit_is_a = dma_hw->ints0 & (1u << mpio_0.dma_a);
    if (culprit_is_a) {
        dma_hw->ints0 = 0x1 << (mpio_0.dma_a);
        dma0_last_written = 1;
    }
    else {
        dma_hw->ints0 = 0x1 << (mpio_0.dma_b);
        dma0_last_written = 2;
    }
}

app_result_t adpc_init() {
    printf("Initializing ADPC ADC(s)\n");
    int as = adpc_adc_init(dma_irq_handler_1, dma_irq_handler_0);
    if(as != 0) {
        printf("ADPC INITIALIZATION FAILED!\nError Code: %d\n",as);
        return -1;
    }

    printf("Initializing ADPC PGIA\n");
    ada_spi_init(&ada, PGIA_SPI, PGIA_PIN_MOSI, PGIA_PIN_MISO ,PGIA_PIN_CS, PGIA_PIN_SCK);
    int ada_status = ada_input_select(&ada, ADA_INPUT_2);
    if(ada_status) {
        printf("PGIA INITIALIZATION FAILED!\nError Code: %d\n",ada_status);
        return ada_status;
    }
    ada_status = ada_input_gain_select(&ada, ADA_INPUT_GAIN_32);
    if(ada_status) {
        printf("PGIA INITIALIZATION FAILED!\nError Code: %d\n",ada_status);
        return ada_status;
    }

    printf("Initializing MPHB connection(s)\n");
    mphb_gpio_init(HB1B);
    mphb_gpio_init(HB2B);
    mphb_setup_multiphase_masked((1U<<HB1B) | (1U<<HB2B));

    printf("initialized!\n");
    return APP_OK;
}