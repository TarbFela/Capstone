#include "adpc_core1.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include <string.h>
#include "tusb.h"

#include "mcp3x6xR_driver/mcp3x6xR.h"
#include "mcp3x6xR_driver/mcp_pio.h"
#include "ada4255_driver/ada4255.h"

#include "../ADPC_Dev/ADPC_cfg.h"
#include "../ADPC_Dev/ADPC_ADC.h"
#include "../ADPC_Dev/adpc_gpio_pwm.h"

#include "adpc_app_funcs.h"

volatile ictl_info_t ictlInfo = {
        .accum = 0,
        .i_coeff = 0.25,
        .p_coeff = 0.75,
};

void core1_ictl(void) {
    gpio_init(ADPC_PIN_LED);
    gpio_set_dir(ADPC_PIN_LED, GPIO_OUT);

    ui_state.cl_ictl = true;

//    int ictl_targ = 500;

//    while(1) {
//        int lw_current = dma0_last_written;
//        while (lw_current == dma0_last_written) tight_loop_contents();
//        int32_t sum = 0;
//        for (int i = 0; i < DMA_BUFF_SIZE; i++)
//            sum += (int32_t) mpio_0.buff[(dma0_last_written - 1) * DMA_BUFF_SIZE + i];
//        int32_t avg = sum / DMA_BUFF_SIZE;
//        ui_state.level += (avg-ictl_targ)/8;
//        if (ui_state.level < -100) ui_state.level = -100;
//        if (ui_state.level > -10) ui_state.level = -10;
//        mphb_set_dlevel_all(ui_state.level);
//        sio_hw->gpio_hi_togl |= (1U << (ADPC_PIN_LED - 32));
//
//        if(multicore_fifo_rvalid()) {
//            if( multicore_fifo_pop_blocking_inline() == MC_FIFO_STOP_FLAG) break;
//        }
//    }

    // this converts from units of AMPS to PWM LEVEL (out of 1000 or MPHB_PWM_WRAP)
    float ictl_sp = ui_state.current_setpoint * 162.4203245 - -25.41180842;
    int lw_current;
start:
    lw_current = dma0_last_written;
    uint dma_ch_writing = dma0_last_written == 2 ? mpio_0.dma_a : mpio_0.dma_b;
    uint index_old = 0;
    while(1) {
        while (lw_current == dma0_last_written &&
               (DMA_BUFF_SIZE - dma_channel_hw_addr(dma_ch_writing)->transfer_count) == index_old) tight_loop_contents();
        if(lw_current != dma0_last_written) goto start;


        // DSP stuff
        int *base = (int *)(mpio_0.buff);
        int *val = (int *)(mpio_0.buff + DMA_BUFF_SIZE*(2-lw_current) + (DMA_BUFF_SIZE - dma_channel_hw_addr(dma_ch_writing)->transfer_count - 1));
        uint32_t offset = (uint32_t)(val-base);
        float avg = 0;
        for(int i =0; i<5; i++) {
            if((int32_t)(offset) - i > 0) avg += *(int *)(base+offset - i);
            else avg += *base; // not ideal; doesn't actually wrap the buffer. Whatever.
        }
        avg /= 5;

        ictl_sp = ui_state.current_setpoint * 162.4203245 - -25.41180842;

        float err = (avg - ictl_sp);
        float incr = err * ictlInfo.i_coeff;
        ictlInfo.accum += incr;
        if (ictlInfo.accum < -50000) ictlInfo.accum = -50000;
        if (ictlInfo.accum > 5000) ictlInfo.accum = 5000;

        float level = (ictlInfo.accum + ictlInfo.p_coeff * err) / 100000;
        if(level > 0.3) level = 0.3;
        if(level < -0.3) level = -0.3;
        ui_state.level = level; // convert to duty cycle (scale 100) from PWM level with scale 1000

        if((offset&0x7F) == 0) {
            if(!ui_state.is_streaming) printf("ERR %.3f INCR %.3f ACC %.3f LVL %.3f\n", err, incr, ictlInfo.accum, ui_state.level);
        }
        mphb_set_dlevel_all_spatial_dithering(ui_state.level);
        //mphb_set_dlevel_all(ui_state.level);


        sio_hw->gpio_hi_togl |= (1U<<(ADPC_PIN_LED-32));

        // TODO: optimize the modulo...
        index_old = (index_old + 1)%(DMA_BUFF_SIZE);
        if(multicore_fifo_rvalid()) {
            int cmd = multicore_fifo_pop_blocking_inline();
            if( cmd == MC_FIFO_STOP_FLAG) break;
        }
    }

    ui_state.cl_ictl = false;
    if(!ui_state.is_streaming) printf("Exiting core 1.\n");
    multicore_fifo_push_blocking(1);
}