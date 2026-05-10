#include "adpc_dsp.h"

#include "hardware/dma.h"
#include "ADPC_ADC.h"

void adpc_dsp_dma_handler_1(void) {
    int culprit_is_a = dma_hw->ints0 & (1u << mpio_1.dma_a);
    if (culprit_is_a) {
        dma_hw->ints0 = 0x1 << (mpio_1.dma_a);
        //dma_hw->ch[mpio.dma_a].write_addr = (io_rw_32)mpio.buff;
    }
    else {
        dma_hw->ints0 = 0x1 << (mpio_1.dma_b);
        //dma_hw->ch[mpio.dma_b].write_addr = (io_rw_32)(mpio.buff + DMA_BUFF_SIZE);
    }
}
