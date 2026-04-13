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

#include "mcp3x6xR_driver/mcp3x6xR.h"
#include "../src2/ADPC_cfg.h"
//#define ADC_1_PIN_MOSI      11
//#define ADC_1_PIN_MISO      12
//#define ADC_1_PIN_CS        13
//#define ADC_1_PIN_SCK       10
//#define ADC_1_PIN_IRQ       15
//#define ADC_1_SPI           spi1

#include "mcp3x6xR_driver/mcp_pio.h"

#include "ADPC_ADC.h"



volatile int dma_done = 0;
void dma_irq_handler(void) {
    static int cc = 0;
    // clear the correct interrupt
    int culprit_is_a = dma_hw->ints0 & (1u << mpio.dma_a);
    if (culprit_is_a) {
        dma_hw->ints0 = 0x1 << (mpio.dma_a);
        //dma_hw->ch[mpio.dma_a].write_addr = (io_rw_32)mpio.buff;
    }
    else {
        dma_hw->ints0 = 0x1 << (mpio.dma_b);
        //dma_hw->ch[mpio.dma_b].write_addr = (io_rw_32)(mpio.buff + DMA_BUFF_SIZE);
    }
    if(cc > 10) {
        mcp_pio_stop(&mpio);
        printf("DONE!\n");
        dma_done = 1;
        cc = 0;
    }
    else {
        //printf("%d...",cc);
        cc++;
    }
}




int main() {
    stdio_init_all();
    char ui[64];

    scanf(" %c",ui);
    printf("[input recieved]\n");
    printf("Initializing ADPC ADC(s)\n");
    int as = adpc_adc_init(dma_irq_handler);
    if(as != 0) {
        printf("ADPC INITIALIZATION FAILED!\nError Code: %d\n",as);
    }

sample:
    dma_done = 0;
    int tii = 0;
    adpc_adc_start();
    while(!dma_done) {
        sleep_ms(20);
        if(tii++ > 100) {
            printf("TIMEOUT!\n");
            goto reboot;
        }
    }
    printf("Samples:\n");
    for(int i = 0; i<DMA_BUFF_SIZE*2; i++) {
        printf("%10ld\t",dma_buff);
        if((i%8)==7) printf("\t[%d]\n",i);
    }

    printf("Done. Enter 'q' to exit. Enter any other character to re-read.\n");
    scanf(" %c",ui);
    if(ui[0]!='q') goto sample;

reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}