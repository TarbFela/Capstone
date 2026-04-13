#include "mcp_pio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "mcp_pio.pio.h"

/*
 * Expects GPIOs to be initialized for SPI and expects the mcp info struct to already be built up.
 */
void mcp_pio_init(mcp_pio_t *s,mcp_info_t *mcp,uint32_t *sample_buff, void (*dma_handler)(void)) {
    PIO pio = pio0;
    int sm = 0;
    uint32_t offset = pio_add_program(pio, &mcp_conversions_read_program);

    pio_sm_config c = mcp_conversions_read_program_get_default_config(offset);
    sm_config_set_clkdiv(&c, 125); //1Mhz is ok for now?
    sm_config_set_in_pins(&c, (mcp->miso));
    sm_config_set_sideset_pins(&c, (mcp->sck));
    sm_config_set_jmp_pin(&c, (mcp->nirq));

    sm_config_set_in_shift(&c, false, true, 32);

    pio_sm_set_pins_with_mask(pio, sm, 0u << (mcp->sck), 1u << (mcp->sck));
    pio_sm_set_pindirs_with_mask(pio, sm,
                                 1 << (mcp->sck),
                                 (1 << (mcp->sck)) | (1 << (mcp->miso)) | (1 << (mcp->nirq))
    );

    pio_sm_init(pio, sm, offset, &c);

    uint dma_a = dma_claim_unused_channel(true);
    uint dma_b = dma_claim_unused_channel(true);

    // TODO: FINISH THE DAISY CHAIN CODE!! THIS IS NOT FUNCTIONAL OR COMPLETE CURRENTLY!!!
    dma_channel_config cfg_a = dma_channel_get_default_config(dma_a);
    channel_config_set_transfer_data_size(&cfg_a, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_a, false);
    channel_config_set_write_increment(&cfg_a, true);
    //dma_channel_set_transfer_count(&dc,100,false);
    channel_config_set_dreq(&cfg_a, pio_get_dreq(pio,sm,false));
//    channel_config_set_chain_to(&cfg_a,dma_b);
    dma_channel_configure(
            dma_a, &cfg_a, sample_buff,
            &pio->rxf[sm],
            100, // 100 samples TODO: daisychain your DMAs to continuously trigger DSP and read forever.
            false
            );

//
//    dma_channel_config cfg_b = dma_channel_get_default_config(dma_b);
//    channel_config_set_transfer_data_size(&cfg_b, DMA_SIZE_32);
//    channel_config_set_read_increment(&cfg_b, false);
//    channel_config_set_write_increment(&cfg_b, true);
//    //dma_channel_set_transfer_count(&dc,100,false);
//    channel_config_set_dreq(&cfg_b, pio_get_dreq(pio,sm,false));
//    channel_config_set_chain_to(&cfg_b,dma_a);
//    dma_channel_configure(
//            dma_b, &cfg_b, sample_buff,
//            &pio->rxf[sm],
//            100, // 100 samples TODO: daisychain your DMAs to continuously trigger DSP and read forever.
//            false
//    );

    irq_set_exclusive_handler(DMA_IRQ_0,dma_handler);

    s->pio = pio;
    s->sm = sm;
    s->mcp_info = mcp;
    s->dma = dma_a;
    s->buff = sample_buff;
}

/*
 * Converts GPIO pins from SIO to PIO functionality, then starts the PIO.
 */
void mcp_pio_start(mcp_pio_t *s) {
    pio_sm_set_pindirs_with_mask(s->pio, s->sm,
                                 1 << (s->mcp_info->sck),
                                 (1 << (s->mcp_info->sck)) | (1 << (s->mcp_info->miso)) | (1 << (s->mcp_info->nirq))
    );
    gpio_set_function(s->mcp_info->miso, GPIO_FUNC_PIO0);
    gpio_set_function(s->mcp_info->sck, GPIO_FUNC_PIO0);
    gpio_set_function(s->mcp_info->nirq, GPIO_FUNC_PIO0);

    pio_sm_restart(s->pio, s->sm);
    pio_sm_clear_fifos(s->pio, s->sm);

    dma_channel_set_irq0_enabled(s->dma, true);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_set_transfer_count(s->dma,100,false);
    dma_channel_set_write_addr(s->dma,s->buff,true);

    pio_sm_set_enabled(s->pio, s->sm, true);
}

/*
 * Reverses actions of mcp_pio_start
 */
void mcp_pio_stop(mcp_pio_t *s) {
    pio_sm_set_enabled(s->pio, s->sm, false);
    irq_set_enabled(DMA_IRQ_0, false);
    dma_channel_set_irq0_enabled(s->dma, false);
    dma_channel_abort(s->dma);

    pio_sm_set_pindirs_with_mask(s->pio, s->sm, 0,
                                 (1 << s->mcp_info->sck) | (1 << s->mcp_info->miso) | (1 << s->mcp_info->nirq));

    gpio_set_function(s->mcp_info->miso, GPIO_FUNC_SPI);
    gpio_set_function(s->mcp_info->sck, GPIO_FUNC_SPI);
}
