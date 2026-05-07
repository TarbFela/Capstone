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

#include <string.h>

#include "mcp3x6xR_driver/mcp3x6xR.h"
#include "mcp3x6xR_driver/mcp_pio.h"
//#include "ada4255_driver/ada4255.h"

#include "../ADPC_Dev/ADPC_cfg.h"

#include "../ADPC_Dev/ADPC_ADC.h"

#include "tusb.h"


volatile int dma0_done = 0;
// if none, 0, if A, 1, if B, 2
volatile int dma_last_written = 0;

volatile int cc = 1;

void dma_irq_handler_1(void) {
    // clear the correct interrupt
    int culprit_is_a = dma_hw->ints0 & (1u << mpio_1.dma_a);
    if (culprit_is_a) {
        dma_hw->ints0 = 0x1 << (mpio_1.dma_a);
        dma_last_written = 1;
    }
    else {
        dma_hw->ints0 = 0x1 << (mpio_1.dma_b);
        dma_last_written = 2;
    }
    if(cc == 0) {
        mcp_pio_stop(&mpio_1);
        //printf("DONE!\n");
        dma0_done = 1;
        cc = 0;
    }
    else {
        //printf("%d...",cc);
        cc--;
    }
}





int main() {
    stdio_init_all();
    char ui[64];

    scanf(" %c",ui);
    printf("[input recieved]\n");
    sleep_ms(100);
    printf("Initializing ADPC ADC(s)\n");
    int as = adpc_adc_init(dma_irq_handler_1, dma_irq_handler_1);
    if(as != 0) {
        printf("ADPC INITIALIZATION FAILED!\nError Code: %d\n",as);
        goto reboot;
    }
//
//    printf("Initializing ADPC PGIA\n");
//    ada_info_t ada;
//    ada_spi_init(&ada, PGIA_SPI, PGIA_PIN_MOSI, PGIA_PIN_MISO ,PGIA_PIN_CS, PGIA_PIN_SCK);
//    int ada_status = ada_input_select(&ada, ADA_INPUT_2);
//    if(ada_status) {
//        printf("PGIA INITIALIZATION FAILED!\nError Code: %d\n",ada_status);
//    }
//    ada_status = ada_input_gain_select(&ada, ADA_INPUT_GAIN_32);
//    if(ada_status) {
//        printf("PGIA INITIALIZATION FAILED!\nError Code: %d\n",ada_status);
//    }

    printf("initialized!\n");

sample:
    printf("To reboot enter 'q'. To read samples, enter a number.\n");
    scanf(" %s",ui);
    printf("[input recieved]\n");
    if (ui[0] == 'q') goto reboot;
    cc = atoi(ui);
    if (cc == 0) {
        printf("Invalid input.\n");
        goto sample;
    }
    sleep_ms(100);
    if(ui[0] == 'q') goto reboot;

    printf("Taking single conversion...\n");
    uint16_t res[2] = {0,0};
    mcp_single_conversion(&mcp_0, res);
    printf("result = %04X %04X\n",res[0],res[1]);

    goto sample;



reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}