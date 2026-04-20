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
#include "mcp3x6xR_driver/mcp3x6xR.h"
#include "../ADPC_Dev/ADPC_cfg.h"
//#define ADC_1_PIN_MOSI      11
//#define ADC_1_PIN_MISO      12
//#define ADC_1_PIN_CS        13
//#define ADC_1_PIN_SCK       10
//#define ADC_1_PIN_IRQ       15
//#define ADC_1_SPI           spi1

#include "mcp3x6xR_driver/mcp_pio.h"

#include "../ADPC_Dev/ADPC_ADC.h"

#include "tusb.h"


volatile int dma_done = 0;
// if none, 0, if A, 1, if B, 2
volatile int dma_last_written = 0;

void dma_irq_handler(void) {
    static int cc = 0;
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
    if(cc > 10) {
        mcp_pio_stop(&mpio_1);
        //printf("DONE!\n");
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
    sleep_ms(100);
    printf("Initializing ADPC ADC(s)\n");
    int as = adpc_adc_init(dma_irq_handler);
    if(as != 0) {
        printf("ADPC INITIALIZATION FAILED!\nError Code: %d\n",as);
        goto reboot;
    }
    printf("initialized!\n");

    printf("[awaiting user input]\n");
    scanf(" %c",ui);
    printf("[input recieved]\n");
    sleep_ms(100);
    if(ui[0] == 'q') goto reboot;


sample:
    dma_done = 0;
    int pii = 0;
    uint bsent = 0;
    int dma_last_printed = dma_last_written;
    printf("STREAMING RAW DATA:\n");
    sleep_ms(100);
    // Mute stdio over USB
    stdio_set_driver_enabled(&stdio_usb, false);
    if(adpc_adc_start() != 0) {
        printf("ADPC START ERROR!\n");
        goto reboot;
    }
    while(!dma_done) {
        while(dma_last_written == dma_last_printed) tight_loop_contents();
        // Blast raw data
        bsent += tud_cdc_write(dma_buff + (dma_last_written-1)*DMA_BUFF_SIZE, DMA_BUFF_SIZE*sizeof(uint32_t));
        tud_cdc_write_flush();
        while (tud_cdc_write_available() < CFG_TUD_CDC_TX_BUFSIZE) tud_task();
        //tud_task();
        dma_last_printed = dma_last_written;
        pii ++;
    }
    // Restore stdio
    stdio_set_driver_enabled(&stdio_usb, true);
    printf("\nEND RAW DATA STREAM\n");
    printf("Streamed %d batches. Wrote %d bytes.\n",pii, bsent);
    printf("Samples:\n");
    for(int i = 0; i<DMA_BUFF_SIZE*2; i++) {
        printf("[%01ld]",dma_buff[i]>>29);
        sign_extend_24_to_32(dma_buff[i]);
        printf(" %-10ld  ",dma_buff[i]);
        if((i%8)==7) printf("\n",i);
    }

    printf("Done. Enter 'q' to exit. Enter any other character to re-read.\n");
    scanf(" %c",ui);
    if(ui[0]!='q') goto sample;

reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}