#include "ADPC_ADC.h"
#include "../src2/ADPC_cfg.h"
#include "mcp3x6xR_driver/mcp_pio.h"
#include "mcp3x6xR_driver/mcp3x6xR.h"

#include <stdio.h>

mcp_info_t mcp;
mcp_pio_t mpio;

enum adpc_adc_status_code {GOOD = 0, NO_POWER = -1, CONFIG_FAILED = -2};


// TODO: Input configuration
// TODO: Scan mode
// TODO: Scan mode timing
// TODO: Scan mode buffers
// TODO: Other configs etc
int adpc_adc_init(void (*dma_handler)(void)) {
    gpio_disable_pulls(ADC_1_PIN_IRQ);
    gpio_init(ADC_1_PIN_IRQ);
    mcp_spi_init(&mcp, ADC_1_SPI, ADC_1_PIN_MOSI,ADC_1_PIN_MISO,ADC_1_PIN_CS,ADC_1_PIN_SCK,ADC_1_PIN_IRQ);
    mcp_pio_init(&mpio, &mcp, dma_buff, dma_handler);

    uint8_t tx[16], rx[16];
    for(int i =0; i<16; i++) {
        tx[i] = 0;
        rx[i] = 0;
    }

    // read all config regs.
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_INCR(MCP_REG_ADDR_CONFIG0);
    gpio_put(ADC_1_PIN_CS,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 5);
    sleep_us(100); gpio_put(ADC_1_PIN_CS,1);
    // if the status byte is zero, the ADC is not communicating.
    if(rx[0] == 00) return NO_POWER;


    // write all config regs
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_CONFIG0);
    tx[1] = 0xE2;
    tx[2] = 0x0C;
    tx[3] = 0x8B;
    tx[4] = MCP_CFG3_CONV_MODE_CONTINUOUS | MCP5_CFG3_DATA_FORMAT_32_SGN;
    gpio_put(ADC_1_PIN_CS,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 5);
    sleep_us(100); gpio_put(ADC_1_PIN_CS,1);
    for(int i =0; i<16; i++) {
        tx[i] = 0;
        rx[i] = 0;
    }

    // read all config regs again
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_INCR(MCP_REG_ADDR_CONFIG0);
    gpio_put(ADC_1_PIN_CS,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 5);
    sleep_us(100); gpio_put(ADC_1_PIN_CS,1);

    // check that the write was performed correctly.
    if (    (rx[1] != (MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_CONFIG0)))
        &&  (rx[2] != 0x0C)
        &&  (rx[3] != 0xE2)
        &&  (rx[4] != 0x8B)
            ) return CONFIG_FAILED;
    for(int i = 1; i<5; i++) printf("rx[%d] 0x%02X\n",i,rx[i]);

    // write MUX
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_MUX);
    tx[1] = MCP_MUX_N_SEL(MCP_MUX_VAL_Int_Temp_Diode_M) | MCP_MUX_P_SEL(MCP_MUX_VAL_Int_Temp_Diode_P);
    gpio_put(ADC_1_PIN_CS,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 2);
    sleep_us(100); gpio_put(ADC_1_PIN_CS,1);
    for(int i =0; i<16; i++) {
        tx[i] = 0;
        rx[i] = 0;
    }

    return GOOD;
}

void adpc_adc_start() {
    // write ADC mode
    uint8_t tx[2], rx[2];
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_CONFIG0);
    tx[1] = MCP_CFG0_VREF_SEL_INTERNAL | MCP_CFG0_NO_PARTIAL_SHUTDOWN | MCP_CFG0_CLK_SEL_INTERNAL | MCP_CFG0_ADC_MODE_CONV;
    gpio_put(ADC_1_PIN_CS,0); sleep_us(5);
    spi_write_read_blocking(spi1, tx, rx, 2);
    sleep_us(5); gpio_put(ADC_1_PIN_CS,1);
    // prepare to perform static read of ADC register
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_STAT(MCP_REG_ADDR_ADCDATA);
    // wait for first IRQ pin signal
    while(!gpio_get(ADC_1_PIN_IRQ));

    gpio_put(ADC_1_PIN_CS,0); sleep_us(5);
    spi_write_blocking(spi1, tx, 1);

    mcp_pio_start(&mpio);
}
