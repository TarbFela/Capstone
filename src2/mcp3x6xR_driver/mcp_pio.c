#include "mcp_pio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "mcp_pio.pio.h"

#include <stdio.h>


volatile uint32_t dma_buff_adc_0[DMA_BUFF_SIZE * 2] __attribute__((aligned(DMA_BUFF_SIZE*sizeof(uint32_t))));
volatile uint32_t dma_buff_adc_1[DMA_BUFF_SIZE * 2] __attribute__((aligned(DMA_BUFF_SIZE*sizeof(uint32_t))));

const uint32_t dma_buff_alignment_bytes = 31 - __builtin_clz(DMA_BUFF_SIZE*sizeof(uint32_t));

/*
 * Expects GPIOs to be initialized for SPI and expects the mcp info struct to already be built up.
 */
void mcp_pio_init(mcp_pio_t *s,mcp_info_t *mcp,uint32_t *sample_buff, void (*dma_handler)(void)) {
    // I couldn't figure out the PIO "claim unused channel", so we're doing something uglier. Sorry!
    PIO pio;
    static int call_count = 0;
    uint irqn;
    if(call_count == 0) {
        pio = pio0;
        call_count++;
        irqn = DMA_IRQ_0;
    } else if (call_count == 1) {
        pio = pio1;
        call_count++;
        irqn = DMA_IRQ_1;
    } else {
        panic("\n\tPANIC!! PIO INIT MAY ONLY BE CALLED TWICE.\n");
    }
    //printf("claiming sm...\n");
    int sm = pio_claim_unused_sm(pio, true);
    //printf("adding program...\n");
    uint32_t offset = pio_add_program(pio, &mcp_conversions_read_program);
    //printf("reading program config...\n");
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
    //printf("pio sm init...\n");
    pio_sm_init(pio, sm, offset, &c);

    //printf("dma init...\n");
    uint dma_a = dma_claim_unused_channel(true);
    uint dma_b = dma_claim_unused_channel(true);

    // TODO: FINISH THE DAISY CHAIN CODE!! THIS IS NOT FUNCTIONAL OR COMPLETE CURRENTLY!!!
    dma_channel_config cfg_a = dma_channel_get_default_config(dma_a);
    channel_config_set_transfer_data_size(&cfg_a, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_a, false);
    channel_config_set_write_increment(&cfg_a, true);
    channel_config_set_ring(&cfg_a, 1, dma_buff_alignment_bytes);


    //dma_channel_set_transfer_count(&dc,100,false);
    channel_config_set_dreq(&cfg_a, pio_get_dreq(pio,sm,false));
    channel_config_set_chain_to(&cfg_a,dma_b);
    dma_channel_configure(
            dma_a, &cfg_a, sample_buff,
            &pio->rxf[sm],
            DMA_BUFF_SIZE,
            false
            );


    dma_channel_config cfg_b = dma_channel_get_default_config(dma_b);
    channel_config_set_transfer_data_size(&cfg_b, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_b, false);
    channel_config_set_write_increment(&cfg_b, true);
    channel_config_set_ring(&cfg_b, 1, dma_buff_alignment_bytes);
    channel_config_set_dreq(&cfg_b, pio_get_dreq(pio,sm,false));
    channel_config_set_chain_to(&cfg_b,dma_a);
    dma_channel_configure(
            dma_b, &cfg_b, sample_buff + DMA_BUFF_SIZE,
            &pio->rxf[sm],
            DMA_BUFF_SIZE,
            false
    );

    //printf("set handler...\n");

    irq_set_exclusive_handler(irqn,dma_handler);

    //printf("populate struct...\n");
    s->pio = pio;
    s->sm = sm;
    s->mcp_info = mcp;
    s->dma_a = dma_a;
    s->dma_b = dma_b;
    s->buff = sample_buff;
    s->irqn = irqn;
}

/*
 * Converts GPIO pins from SIO to PIO functionality, then starts the PIO.
 */
void mcp_pio_start(mcp_pio_t *s) {
    pio_sm_set_pindirs_with_mask(s->pio, s->sm,
                                 1 << (s->mcp_info->sck),
                                 (1 << (s->mcp_info->sck)) | (1 << (s->mcp_info->miso)) | (1 << (s->mcp_info->nirq))
    );
    // this logic makes an assumption. Beware.
    gpio_set_function(s->mcp_info->miso, (s->pio == pio0) ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1);
    gpio_set_function(s->mcp_info->sck, (s->pio == pio0) ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1);
    gpio_set_function(s->mcp_info->nirq, (s->pio == pio0) ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1);

    pio_sm_restart(s->pio, s->sm);
    pio_sm_clear_fifos(s->pio, s->sm);

    if(s->irqn == DMA_IRQ_0) {
        dma_channel_set_irq0_enabled(s->dma_a, true);
        dma_channel_set_irq0_enabled(s->dma_b, true);
    }
    else if (s->irqn == DMA_IRQ_1) {
        dma_channel_set_irq1_enabled(s->dma_a, true);
        dma_channel_set_irq1_enabled(s->dma_b, true);
    }
    irq_set_enabled(s->irqn, true);
    dma_channel_set_transfer_count(s->dma_a,DMA_BUFF_SIZE,false);
    dma_channel_set_transfer_count(s->dma_b,DMA_BUFF_SIZE,false);
    dma_channel_set_write_addr(s->dma_b,s->buff + DMA_BUFF_SIZE,false);
    dma_channel_set_write_addr(s->dma_a,s->buff,true);

    pio_sm_set_enabled(s->pio, s->sm, true);
}

/*
 * Reverses actions of mcp_pio_start
 */
void __not_in_flash_func(mcp_pio_stop)(mcp_pio_t *s) {
    pio_sm_set_enabled(s->pio, s->sm, false);
    irq_set_enabled(s->irqn, false);

    if(s->irqn == DMA_IRQ_0) {
        dma_channel_set_irq0_enabled(s->dma_a, false);
        dma_channel_set_irq0_enabled(s->dma_b, false);
    }
    else if (s->irqn == DMA_IRQ_1) {
        dma_channel_set_irq1_enabled(s->dma_a, false);
        dma_channel_set_irq1_enabled(s->dma_b, false);
    }
    dma_channel_abort(s->dma_a);
    dma_channel_abort(s->dma_b);

    pio_sm_set_pindirs_with_mask(s->pio, s->sm, 0,
                                 (1 << s->mcp_info->sck) | (1 << s->mcp_info->miso) | (1 << s->mcp_info->nirq));

    gpio_set_function(s->mcp_info->miso, GPIO_FUNC_SPI);
    gpio_set_function(s->mcp_info->sck, GPIO_FUNC_SPI);
}
