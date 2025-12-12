//
// Created by The Tragedy of Darth Wise on 12/12/25.
//
#include "capstone_adc.h"

capstone_adc_struct_t *capstone_adc_init() {
    adc_init();
    adc_run(false);
    adc_gpio_init(ISNS_ADC_PIN);
    adc_gpio_init(TSNS_ADC_PIN);
    adc_select_input(ISNS_ADC_PIN
                     -26);
    adc_fifo_setup(
            true,    // Write each completed conversion to the sample FIFO
            true,    // Enable DMA data request (DREQ)
            1,       // DREQ (and IRQ) asserted when at least 1 sample present
            false,   // We won't see the ERR bit because of 8 bit reads; disable.
            false     // Shift each sample to 8 bits when pushing to FIFO?
    );

    // ROUND ROBIN ORDER:
    // AINSEL initial value is ISNS (28)
    // subsequent RR goes TSNS, ISNS, TSNS, ISNS, [...]
    // So, actual order is: ISNS, TSNS, ISNS, TSNS, [...]
    adc_set_round_robin(0x1<<(ISNS_ADC_PIN-26) | 0x1<<(TSNS_ADC_PIN-26));
    // 96kHz = 48MHz / 500
    adc_set_clkdiv(1000);

    uint adc_dma_daisy_chain[2];
    adc_dma_daisy_chain[0] = dma_claim_unused_channel(true);
    adc_dma_daisy_chain[1] = dma_claim_unused_channel(true);
    uint16_t *adc_dma_buffer;
    //TODO: check for malloc failure.
    adc_dma_buffer = (uint16_t *)malloc(sizeof(uint16_t) * ADC_BUFFER_SIZE);

    dma_channel_hw_t *adc_dma_daisy_chain_hw[2];
    adc_dma_daisy_chain_hw[0] = &dma_hw->ch[adc_dma_daisy_chain[0]];
    adc_dma_daisy_chain_hw[1] = &dma_hw->ch[adc_dma_daisy_chain[1]];

    dma_channel_config cfg0 = dma_channel_get_default_config(adc_dma_daisy_chain[0]);
    channel_config_set_transfer_data_size(&cfg0, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg0, false);
    channel_config_set_write_increment(&cfg0, true);
    channel_config_set_dreq(&cfg0, DREQ_ADC);
    channel_config_set_chain_to(&cfg0, adc_dma_daisy_chain[1]);

    dma_channel_config cfg1 = dma_channel_get_default_config(adc_dma_daisy_chain[1]);
    channel_config_set_transfer_data_size(&cfg1, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg1, false);
    channel_config_set_write_increment(&cfg1, true);
    channel_config_set_dreq(&cfg1, DREQ_ADC);
    channel_config_set_chain_to(&cfg1, adc_dma_daisy_chain[0]);

    capstone_adc_struct_t *cas;
    cas = (capstone_adc_struct_t *)malloc(sizeof(capstone_adc_struct_t));
    cas->adc_dma_buffer = adc_dma_buffer;
    cas->adc_dma_daisy_chain[0] = adc_dma_daisy_chain[0];
    cas->adc_dma_daisy_chain[1] = adc_dma_daisy_chain[1];
    cas->adc_dma_daisy_chain_hw[0] = adc_dma_daisy_chain_hw[0];
    cas->adc_dma_daisy_chain_hw[1] = adc_dma_daisy_chain_hw[1];
    cas->dma_cfg[0] = cfg0;
    cas->dma_cfg[1] = cfg1;
    return cas;
}

void capstone_adc_start(capstone_adc_struct_t *cas) {
    dma_channel_configure(cas->adc_dma_daisy_chain[0], &(cas->dma_cfg[0]),
                          cas->adc_dma_buffer,    // dst
                          &adc_hw->fifo,  // src
                          ADC_BUFFER_SIZE,  // transfer count
                          false            // DON'T start immediately
    );
    dma_channel_configure(cas->adc_dma_daisy_chain[1], &(cas->dma_cfg[1]),
                          cas->adc_dma_buffer,    // dst
                          &adc_hw->fifo,  // src
                          ADC_BUFFER_SIZE,  // transfer count
                          false            // DON'T start immediately
    );

    adc_run(true);
    dma_channel_start(cas->adc_dma_daisy_chain[0]);
}

void capstone_adc_stop(capstone_adc_struct_t *cas) {
    adc_run(false);
    adc_fifo_drain();
    dma_channel_abort(cas->adc_dma_daisy_chain[0]);
    dma_channel_abort(cas->adc_dma_daisy_chain[1]);
}
