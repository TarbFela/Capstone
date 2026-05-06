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
#include "ada4255_driver/ada4255.h"

#include "../ADPC_Dev/ADPC_cfg.h"
#include "../ADPC_Dev/ADPC_ADC.h"
#include "../ADPC_Dev/adpc_gpio_pwm.h"

#include "tusb.h"


volatile int dma_done = 0;
// if none, 0, if A, 1, if B, 2
volatile int dma_last_written = 0;

volatile int cc = 1;

void dma_irq_handler(void) {
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
        dma_done = 1;
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
    int as = adpc_adc_init(dma_irq_handler);
    if(as != 0) {
        printf("ADPC INITIALIZATION FAILED!\nError Code: %d\n",as);
        goto reboot;
    }

    printf("Initializing ADPC PGIA\n");
    ada_info_t ada;
    ada_spi_init(&ada, PGIA_SPI, PGIA_PIN_MOSI, PGIA_PIN_MISO ,PGIA_PIN_CS, PGIA_PIN_SCK);
    int ada_status = ada_input_select(&ada, ADA_INPUT_2);
    if(ada_status) {
        printf("PGIA INITIALIZATION FAILED!\nError Code: %d\n",ada_status);
    }
    ada_status = ada_input_gain_select(&ada, ADA_INPUT_GAIN_32);
    if(ada_status) {
        printf("PGIA INITIALIZATION FAILED!\nError Code: %d\n",ada_status);
    }

    printf("Initializing MPHB connection(s)\n");
    mphb2_gpio_pwm_t hb_1B;
    mphb_gpio_init(&hb_1B, PWM_B_1_PIN, PWM_D_1_PIN, PH_EN_B1_PIN);


    printf("initialized!\n");




    printf("To reboot enter 'q'.\n"
           "To read samples, enter 'r'.\n"
           "To control PWM, enter 0-9 or 'p' or 'l'\n"
           "To enable/disable PWM, enter 'e' or 'd' (resp.)\n");

    int level = 0;
    int pwm_enabled = 0;
    while(1) {
        ui[0] = getchar_timeout_us(1000);
        if((int8_t)*ui == PICO_ERROR_TIMEOUT) {
            continue;
        }
        else {
            printf("RECIEVED [%c]\n",*ui);
        }
        if(*ui == 'q') {
            mphb_set_levels(&hb_1B,0,0);
            goto reboot;
        }
        if(*ui == ' ') {
            pwm_enabled = !pwm_enabled;
            printf("Setting PWM state to %s\n",pwm_enabled ? "ENABLED" : "DISABLED");
            if (!pwm_enabled) {
                mphb_set_levels(&hb_1B,0,0);
            }
            else {
                mphb_set_levels(&hb_1B,300,300);
            }
            sleep_us(100); // let the PWM go to zero and then disable it.
            pwm_set_enabled(hb_1B.slice, pwm_enabled);
        }
        if(*ui == 'e') {
            gpio_put(PH_EN_B1_PIN, 1);
            printf("ENABLE PIN ON\n");
        }
        if(*ui == 'd') {
            gpio_put(PH_EN_B1_PIN, 0);
            printf("ENABLE PIN OFF\n");
        }
        else if ((*ui >= '0') && (*ui <= '9')) {
            level = (*ui-'0')*15;
            printf("%d offset\n",level);
            mphb_set_levels(&hb_1B,300+level,300-level);
        }
        else if (*ui == 'p' || *ui == 'l') {
            level += (*ui == 'p') ? 5 : -5;
            printf("%d offset\n",level);
            mphb_set_levels(&hb_1B,300+level,300-level);
        }
        else if (*ui == 'r') {
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
        }
    }





reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}