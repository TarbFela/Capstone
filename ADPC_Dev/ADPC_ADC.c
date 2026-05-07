#include "ADPC_ADC.h"
#include "ADPC_cfg.h"
#include "../src2/mcp3x6xR_driver/mcp_pio.h"
#include "../src2/mcp3x6xR_driver/mcp3x6xR.h"

#include "hardware/pwm.h"

#include <stdio.h>

mcp_info_t mcp_0;
mcp_pio_t mpio_0;
mcp_info_t mcp_1;
mcp_pio_t mpio_1;

enum adpc_adc_status_code {GOOD = 0, NO_POWER = -1, CONFIG_FAILED = -2};


// TODO: Input configuration
// TODO: Scan mode
// TODO: Scan mode timing
// TODO: Scan mode buffers
// TODO: Other configs etc
int adpc_adc_init(void (*dma_handler_1)(void), void (*dma_handler_0)(void)) {
    mcp_cfg_t cfg;

    /********************************************
     *  ADC 0: ISNS, 16-bit MCP3462R            *
     ********************************************/
    gpio_disable_pulls(ADC_0_PIN_IRQ);
    gpio_pull_up(ADC_0_PIN_IRQ);
    gpio_init(ADC_0_PIN_IRQ);
    mcp_spi_init(&mcp_0, ADC_0_SPI, ADC_0_PIN_MOSI,ADC_0_PIN_MISO,ADC_0_PIN_CS,ADC_0_PIN_SCK,ADC_0_PIN_IRQ, ADC_MCLK_PIN);
    mcp_pio_init(&mpio_0, &mcp_0, dma_buff_adc_0, dma_handler_0);
    cfg.cfgs[0] = MCP_CFG0_VREF_SEL_INTERNAL | MCP_CFG0_NO_PARTIAL_SHUTDOWN | MCP_CFG0_CLK_SEL_INTERNAL | MCP_CFG0_ADC_MODE_STDBY;
    cfg.cfgs[1] = MCP_CFG1_AMCLK_PRESCALE_NONE | MCP_CFG1_OSR_2048;
    cfg.cfgs[2] = MCP_CFG2_BIAS_CURRENT_SEL_1 | MCP_CFG2_ADC_GAIN_SEL_1 | MCP_CFG2_AUTO_ZERO_REF_EN | 0x1;
    // TODO: investigate using 16-bit data format for some DSP instructions, maybe?
    cfg.cfgs[3] = MCP_CFG3_CONV_MODE_CONTINUOUS | MCP4_CFG3_DATA_FORMAT_32_LJ;
    cfg.input_mode = MCP_MUX_MODE;
    cfg.scan_sel = 0;
    cfg.mux_sel = MCP_MUX_P_SEL(MCP_MUX_VAL_CH0) | MCP_MUX_N_SEL(MCP_MUX_VAL_CH1);
    mcp_configure(&mcp_0, &cfg);

    /********************************************
     *  ADC 1: TSNS and VSNS, 24-bit MCP3562R   *
     ********************************************/
    gpio_disable_pulls(ADC_1_PIN_IRQ);
    gpio_init(ADC_1_PIN_IRQ);
    mcp_spi_init(&mcp_1, ADC_1_SPI, ADC_1_PIN_MOSI,ADC_1_PIN_MISO,ADC_1_PIN_CS,ADC_1_PIN_SCK,ADC_1_PIN_IRQ, ADC_MCLK_PIN);
    mcp_pio_init(&mpio_1, &mcp_1, dma_buff_adc_1, dma_handler_1);
    cfg.cfgs[0] = MCP_CFG0_VREF_SEL_INTERNAL | MCP_CFG0_NO_PARTIAL_SHUTDOWN | MCP_CFG0_CLK_SEL_INTERNAL | MCP_CFG0_ADC_MODE_STDBY;
    cfg.cfgs[1] = MCP_CFG1_AMCLK_PRESCALE_NONE | MCP_CFG1_OSR_512;
    cfg.cfgs[2] = MCP_CFG2_BIAS_CURRENT_SEL_1 | MCP_CFG2_ADC_GAIN_SEL_1 | MCP_CFG2_AUTO_ZERO_REF_EN | 0x1;
    cfg.cfgs[3] = MCP_CFG3_CONV_MODE_CONTINUOUS | MCP5_CFG3_DATA_FORMAT_32_CHID_SGN4_24;
    cfg.input_mode = MCP_SCAN_MODE;
    cfg.scan_sel = MCP_SCAN_SEL_BIT_DIFF_A | MCP_SCAN_SEL_BIT_DIFF_B;
    mcp_configure(&mcp_1, &cfg);
    mcp_timer_set(&mcp_1,0);
    mcp_scan_set_dly(&mcp_1,0);



    mcp_mclk_init(&mcp_0, 10000000);
    mcp_mclk_init(&mcp_1, 10000000);
    return GOOD;
}

// TODO: check communication somehow.
int adpc_adc_start(mcp_pio_t *s) {
    // write ADC mode
    uint8_t tx[5], rx[5];
    tx[0] = s->mcp_info->cfg.cfgs[0] | MCP_CFG0_ADC_MODE_CONV;
    mcp_write_regs(s->mcp_info, tx, 1, MCP_REG_ADDR_CONFIG0);

    // prepare to perform static read of ADC register
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_STAT(MCP_REG_ADDR_ADCDATA);
    // wait for first IRQ pin signal
    while(gpio_get(s->mcp_info->nirq));

    gpio_put(s->mcp_info->cs,0); sleep_us(5);
    spi_write_read_blocking(s->mcp_info->spi, tx, rx, 5);

    mcp_pio_start(s);
    return GOOD;
}
