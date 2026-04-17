#include "ADPC_ADC.h"
#include "ADPC_cfg.h"
#include "../src2/mcp3x6xR_driver/mcp_pio.h"
#include "../src2/mcp3x6xR_driver/mcp3x6xR.h"

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
int adpc_adc_init(void (*dma_handler)(void)) {
    gpio_disable_pulls(ADC_1_PIN_IRQ);
    gpio_init(ADC_1_PIN_IRQ);
    mcp_spi_init(&mcp_1, ADC_1_SPI, ADC_1_PIN_MOSI,ADC_1_PIN_MISO,ADC_1_PIN_CS,ADC_1_PIN_SCK,ADC_1_PIN_IRQ);

    mcp_pio_init(&mpio_1, &mcp_1, dma_buff, dma_handler);

    mcp_cfg_t cfg;
    cfg.cfgs[0] = MCP_CFG0_VREF_SEL_INTERNAL | MCP_CFG0_NO_PARTIAL_SHUTDOWN | MCP_CFG0_CLK_SEL_INTERNAL | MCP_CFG0_ADC_MODE_STDBY;
    cfg.cfgs[1] = MCP_CFG1_AMCLK_PRESCALE_NONE | MCP_CFG1_OSR_256;
    cfg.cfgs[2] = MCP_CFG2_BIAS_CURRENT_SEL_1 | MCP_CFG2_ADC_GAIN_SEL_1 | MCP_CFG2_AUTO_ZERO_REF_EN | 0x1;
    cfg.cfgs[3] = MCP_CFG3_CONV_MODE_CONTINUOUS | MCP5_CFG3_DATA_FORMAT_32_CHID_SGN4_24;
    cfg.input_mode = MCP_SCAN_MODE;
    cfg.scan_sel = MCP_SCAN_SEL_BIT_DIFF_A | MCP_SCAN_SEL_BIT_DIFF_B;
    cfg.scan_dly = MCP_SCAN_DLY_0;
    cfg.scan_timer = 0;
    mcp_configure(&mcp_1, &cfg);


    return GOOD;
}

// TODO: check communication somehow.

int adpc_adc_start() {
    mcp_status_t status;

    // write ADC mode (start conversions)
    status = mcp_write_reg_masked(&mcp_1, MCP_CFG0_ADC_MODE_BITS,MCP_CFG0_ADC_MODE_CONV,MCP_REG_ADDR_CONFIG0);
    if (status & MCP_STATUS_ERROR_FLAG) return -1;


    // prepare to perform static read of ADC register
    uint8_t tx = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_STAT(MCP_REG_ADDR_ADCDATA);
    // wait for first IRQ pin signal
    while(!gpio_get(ADC_1_PIN_IRQ)) tight_loop_contents();

    gpio_put(ADC_1_PIN_CS,0); sleep_us(5);
    spi_write_read_blocking(mcp_1.spi, &tx, &status, 1);

    mcp_pio_start(&mpio_1);
    return GOOD;
}
