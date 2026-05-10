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
#include "adpc_core1.h"

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
//    int32_t *buff = (int32_t *)mpio_0.buff + (dma0_last_written - 1) * DMA_BUFF_SIZE;
//    for(int i = 0; i<DMA_BUFF_SIZE; i++) {
//        buff[i] >>= 16;
//    }
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
    int ada_status = ada_input_select(&ada, ADA_INPUT_1);
    if(ada_status) {
        printf("PGIA INITIALIZATION FAILED!\nError Code: %d\n",ada_status);
        return ada_status;
    }
    ada_status = ada_input_gain_select(&ada, ADA_INPUT_GAIN_32);
    if(ada_status) {
        printf("PGIA INITIALIZATION FAILED!\nError Code: %d\n",ada_status);
        return ada_status;
    }

    sleep_ms(109); // idk if its necessary...

    printf("Starting ADC PIOs...\n");
    if(adpc_adc_start(&mpio_0) != 0) {
        printf("ADPC START ERROR!\n");
        return APP_ERROR;
    }
    if(adpc_adc_start(&mpio_1) != 0) {
        printf("ADPC START ERROR!\n");
        return APP_ERROR;
    }
    printf("ADC PIOs initialized.\n");

    printf("Initializing MPHB connection(s)\n");
    for(mphb_port_t i = HB1A; i<=HB3B; i++) {
        mphb_gpio_init(i);
//        printf("\tMPHB %s CONNECTION: %s\n",mphb_port_names[i], mphb_detect_connection(i) ? "CONNECTED" : "NO CONNECTION");
    }
    mphb_gpio_init(HB3A);
    mphb_gpio_init(HB1B);
    mphb_gpio_init(HB2B);
    mphb_gpio_init(HB3B);
    mphb_setup_multiphase_masked((1U<<HB3A) | (1U<<HB1B) | (1U<<HB2B) | (1U<<HB3B));

    printf("initialized!\n");
    return APP_OK;
}

app_result_t app_cmd_rstream(app_state_t *s) {
    s->is_streaming = true;

    printf("STREAMING RAW DATA:\n");
    sleep_ms(100);
    // Mute stdio over USB
    stdio_set_driver_enabled(&stdio_usb, false);
    int dma0_last_printed = dma0_last_written;
    int dma1_last_printed = dma1_last_written;
    int ui_exit_signal = 0;
    uint bsent = 0, pii = 0;
    while(1) {
        while(dma0_last_written == dma0_last_printed && dma1_last_written == dma1_last_printed) {
            if( tud_cdc_available() ) {
                uint32_t uii = tud_cdc_read_char();
                char ui;
                if(uii==-1) continue;
                ui = (char)uii;
                if(ui == 'q') {
                    ui_exit_signal = 1;
                    break;
                }
                if(app_dispatch_single_char(s, ui) != APP_OK) {
                    ui_exit_signal = 1;
                    break;
                }
            }
        };
        if(ui_exit_signal) break;
        if(dma0_last_written != dma0_last_printed) {
            // Blast raw data
            bsent += tud_cdc_write(mpio_0.buff + (dma0_last_written - 1) * DMA_BUFF_SIZE,
                                   DMA_BUFF_SIZE * sizeof(uint32_t));
            tud_cdc_write_flush();
            while (tud_cdc_write_available() < CFG_TUD_CDC_TX_BUFSIZE) tud_task();
            dma0_last_printed = dma0_last_written;
        }
        if(dma1_last_written != dma1_last_printed) {
            // Blast raw data
            bsent += tud_cdc_write(mpio_1.buff + (dma1_last_written - 1) * DMA_BUFF_SIZE,
                                   DMA_BUFF_SIZE * sizeof(uint32_t));
            tud_cdc_write_flush();
            while (tud_cdc_write_available() < CFG_TUD_CDC_TX_BUFSIZE) tud_task();
            dma1_last_printed = dma1_last_written;
        }
        pii ++;
    }
    // Restore stdio
    stdio_set_driver_enabled(&stdio_usb, true);
    sleep_ms(100);
    printf("\nEND RAW DATA STREAM\n");
    printf("Streamed %d batches. Wrote %d bytes.\n",pii, bsent);
    s->is_streaming = false;
    return APP_OK;
}

app_result_t app_cmd_isp(app_state_t *s, float sp) {
    int32_t sp_scaled = sp * 162.4203245 - -25.41180842;

    s->current_setpoint = sp;
    if (!s->is_streaming) printf("[set ictl setpoint to %4.2f amps]\n", (float)sp_scaled * 0.006156071217 + 0.1572961424);

    return APP_OK;
}

app_result_t app_cmd_ictl(app_state_t *s) {
    if(s->cl_ictl) {
        multicore_fifo_push_timeout_us(MC_FIFO_STOP_FLAG, 100);
        if(!s->is_streaming) printf("Core 1 pushed...\n");
        multicore_fifo_pop_blocking();
        sleep_ms(100);
        multicore_reset_core1();
        return APP_OK;
    }
    else {
        if(!s->is_streaming) printf("Launching Core 1\n");
        multicore_launch_core1(core1_ictl);
        return APP_OK;
    }
}

app_result_t app_cmd_irun(app_state_t *s) {
    // start CL control if it isn't yet running
    mphb_set_ph_en_all(true);
    if(!ui_state.cl_ictl) {
        if(app_cmd_ictl(s) != APP_OK) return APP_ERROR;
    }
    uint64_t ts = (int)(s->current_program.timestep * 1000);
    uint64_t t = time_us_64();
    bool ui_break = false;
    for( int i = 0; i < s->current_program.N; i++) {
        while(time_us_64() - t <= ts) {
            if(stdio_getchar_timeout_us(10) != PICO_ERROR_TIMEOUT) {
                ui_break = true;
                break;
            }
        }
        t += ts;
        if(ui_break) break;
        if(app_cmd_isp(s, s->current_program.setpoints[i]) != APP_OK) break;
    }
    //stop CL control
    app_cmd_ictl(s);
    //turn off outputs
    mphb_set_ph_en_all(false);
    return APP_OK;
}

app_result_t app_cmd_irun_streaming(app_state_t *s) {
    mphb_set_ph_en_all(true);
    // start CL control if it isn't yet running
    if(!ui_state.cl_ictl) {
        if(app_cmd_ictl(s) != APP_OK) return APP_ERROR;
    }

    s->is_streaming = true;
    printf("STREAMING RAW DATA:\n");
    sleep_ms(100);
    // Mute stdio over USB
    stdio_set_driver_enabled(&stdio_usb, false);
    int dma0_last_printed = dma0_last_written;
    int dma1_last_printed = dma1_last_written;
    uint bsent = 0, pii = 0;

    uint64_t ts = (int)(s->current_program.timestep * 1000);
    uint64_t t = time_us_64();
    bool ui_break = false;
    for( int i = 0; i < s->current_program.N; i++) {
        while(time_us_64() - t <= ts) {
            if(stdio_getchar_timeout_us(10) != PICO_ERROR_TIMEOUT) {
                ui_break = true;
                break;
            }
            if(dma0_last_written != dma0_last_printed || dma1_last_written != dma1_last_printed) {
                if (dma0_last_written != dma0_last_printed) {
                    // Blast raw data
                    bsent += tud_cdc_write(mpio_0.buff + (dma0_last_written - 1) * DMA_BUFF_SIZE,
                                           DMA_BUFF_SIZE * sizeof(uint32_t));
                    tud_cdc_write_flush();
                    while (tud_cdc_write_available() < CFG_TUD_CDC_TX_BUFSIZE) tud_task();
                    dma0_last_printed = dma0_last_written;
                }
                if (dma1_last_written != dma1_last_printed) {
                    // Blast raw data
                    bsent += tud_cdc_write(mpio_1.buff + (dma1_last_written - 1) * DMA_BUFF_SIZE,
                                           DMA_BUFF_SIZE * sizeof(uint32_t));
                    tud_cdc_write_flush();
                    while (tud_cdc_write_available() < CFG_TUD_CDC_TX_BUFSIZE) tud_task();
                    dma1_last_printed = dma1_last_written;
                }
                pii++;
            }
        }
        t += ts;
        if(ui_break) break;
        if(app_cmd_isp(s, s->current_program.setpoints[i]) != APP_OK) break;
    }
    // Restore stdio
    stdio_set_driver_enabled(&stdio_usb, true);
    sleep_ms(100);
    printf("\nEND RAW DATA STREAM\n");
    printf("Streamed %d batches. Wrote %d bytes.\n",pii, bsent);
    s->is_streaming = false;

    //stop CL control
    app_cmd_ictl(s);
    //turn off outputs
    mphb_set_ph_en_all(false);
    return APP_OK;
}