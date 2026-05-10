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

    int ictl_sp = ui_state.current_setpoint * 162.4203245 - -25.41180842;
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
        int avg = 0;
        for(int i =0; i<5; i++) {
            if((int32_t)(offset) - i > 0) avg += *(int *)(base+offset - i);
            else avg += *base; // not ideal; doesn't actually wrap the buffer. Whatever.
        }
        avg /= 5;

        ictl_sp = ui_state.current_setpoint * 162.4203245 - -25.41180842;

        int err = (avg - ictl_sp);
        int incr = 0;
        if(err > 50) incr = 1;
        if(err < -50) incr = -1;
        if(err > 200) incr += 1;
        if(err < -200) incr += -1;
        // slower, finer-grain integral control
        if((offset&0x7F) == 0) {
            if(err > 4) incr += 1;
            if(err < -4) incr += -1;
            if(!ui_state.is_streaming) printf("%d %d %d\n", err, incr, ui_state.level);
        }
        ui_state.level += incr;
        if (ui_state.level < -110) ui_state.level = -110;
        if (ui_state.level > -10) ui_state.level = -10;
        mphb_set_dlevel_all(ui_state.level);


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