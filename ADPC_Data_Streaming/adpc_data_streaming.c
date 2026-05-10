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
#include "adpc_app_funcs.h"


int main() {
    stdio_init_all();

    char uic;
    scanf(" %c",&uic);
    printf("[input recieved]\n");
    if(uic == 'q') goto reboot;
    sleep_ms(100);


    while(1) {
        if(app_shell_task(&ui_state)==APP_REBOOT) goto reboot;
    }
//
//    while(1) {
//        ui[0] = getchar_timeout_us(1000);
//        if((int8_t)*ui == PICO_ERROR_TIMEOUT) {
//            continue;
//        }
//        else {
//            printf("RECIEVED [%c]\n",*ui);
//        }
//        if(*ui == 'q') {
//            mphb_set_levels_all(0,0);
//            goto reboot;
//        }
//        if(*ui == ' ') {
//            pwm_enabled = !pwm_enabled;
//            printf("Setting PWM state to %s\n",pwm_enabled ? "ENABLED" : "DISABLED");
//            if (pwm_enabled) {
//                mphb_set_dlevel_all( level);
//                mphb_set_pwm_en(HB1B,true);
//                mphb_set_pwm_en(HB2B,true);
//            }
//            else {
//                mphb_set_levels_all(0,0);
//                mphb_set_pwm_en(HB1B,false);
//                mphb_set_pwm_en(HB2B,false);
//            }
//            sleep_us(100); // let the PWM go to zero and then disable it.
//
//        }
//        if(*ui == 'e') {
//            mphb_set_ph_en(HB1B, true);
//            mphb_set_ph_en(HB2B, true);
//            printf("ENABLE PIN ON\n");
//        }
//        if(*ui == 'd') {
//            mphb_set_ph_en(HB1B, false);
//            mphb_set_ph_en(HB2B, false);
//            printf("ENABLE PIN OFF\n");
//        }
//        else if ((*ui >= '0') && (*ui <= '9')) {
//            level = (*ui-'0')*5;
//            printf("%d offset\n",level);
//            mphb_set_dlevel_all( level);
//        }
//        else if (*ui == 'p' || *ui == 'l') {
//            level += (*ui == 'p') ? 1 : -1;
//            printf("%d offset\n",level);
//            mphb_set_dlevel_all( level);
//        }
//        else if (*ui == 'r' || *ui == 'R') {
//            int pii = 0;
//            uint bsent = 0;
//
//            int dma0_last_printed = dma0_last_written;
//            int dma1_last_printed = dma1_last_written;
//            printf("STREAMING RAW DATA:\n");
//            sleep_ms(100);
//            // Mute stdio over USB
//            if(adpc_adc_start(&mpio_0) != 0) {
//                printf("ADPC START ERROR!\n");
//                goto reboot;
//            }
//            if(adpc_adc_start(&mpio_1) != 0) {
//                printf("ADPC START ERROR!\n");
//                goto reboot;
//            }
//            stdio_set_driver_enabled(&stdio_usb, false);
//            int ui_exit_signal = 0;
//            while(1) {
//                while(dma0_last_written == dma0_last_printed && dma1_last_written == dma1_last_printed) {
//                    if( tud_cdc_available() ) {
//                        uint32_t uii = tud_cdc_read_char();
//                        if(uii!=-1) *ui = (char)uii;
//                        if(*ui == 'e') {
//                            mphb_set_ph_en(HB1B, true);
////                            printf("ENABLE PIN ON\n");
//                        }
//                        if(*ui == 'd') {
//                            mphb_set_ph_en(HB1B, false);
////                            printf("ENABLE PIN OFF\n");
//                        }
//                        else if ((*ui >= '0') && (*ui <= '9')) {
//                            level = (*ui-'0')*5;
////                            printf("%d offset\n",level);
//                            mphb_set_dlevel_all( level);
//                        }
//                        else if (*ui == 'p' || *ui == 'l') {
//                            level += (*ui == 'p') ? 1 : -1;
////                            printf("%d offset\n",level);
//                            mphb_set_dlevel_all( level);
//                        }
//                        else {
//                            mcp_pio_stop(&mpio_0);
//                            mcp_pio_stop(&mpio_1);
//                            ui_exit_signal = 1;
//                            break;
//                        }
//                    }
//                };
//                if(ui_exit_signal) break;
//                if(dma0_last_written != dma0_last_printed) {
//                    // Blast raw data
//                    bsent += tud_cdc_write(mpio_0.buff + (dma0_last_written - 1) * DMA_BUFF_SIZE,
//                                           DMA_BUFF_SIZE * sizeof(uint32_t));
//                    tud_cdc_write_flush();
//                    while (tud_cdc_write_available() < CFG_TUD_CDC_TX_BUFSIZE) tud_task();
//                    dma0_last_printed = dma0_last_written;
//                }
//                if(dma1_last_written != dma1_last_printed) {
//                    // Blast raw data
//                    bsent += tud_cdc_write(mpio_1.buff + (dma1_last_written - 1) * DMA_BUFF_SIZE,
//                                           DMA_BUFF_SIZE * sizeof(uint32_t));
//                    tud_cdc_write_flush();
//                    while (tud_cdc_write_available() < CFG_TUD_CDC_TX_BUFSIZE) tud_task();
//                    dma1_last_printed = dma1_last_written;
//                }
//                pii ++;
//            }
//            // Restore stdio
//            stdio_set_driver_enabled(&stdio_usb, true);
//            printf("\nEND RAW DATA STREAM\n");
//            if(ui_exit_signal) printf("Interrupted by user input.\n");
//            printf("Streamed %d batches. Wrote %d bytes.\n",pii, bsent);
//        }
//    }





reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}