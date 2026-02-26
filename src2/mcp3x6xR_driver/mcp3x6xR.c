#include "mcp3x6xR.h"

#include <stdio.h>
#include <stdint.h>

#include "stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#define MCP_SPI_BAUDRATE 48000
#define MCP_SLEEPTIME_US 500
#define MCP_CS_DESELECT 1
#define MCP_CS_SELECT 0
#define MCP_SPI spi1

mcp_status_t mcp_spi_init(mcp_info_t *s, int mosi_pin, int miso_pin, int cs_pin, int sck_pin) {
    gpio_init(cs_pin);
    gpio_put(cs_pin, MCP_CS_DESELECT);
    gpio_set_dir(cs_pin, GPIO_OUT);

    sleep_ms(10); //needless.

    spi_init(MCP_SPI, 80*1000); // 80kHz
    // drive device in 0,0 mode
    spi_set_format(MCP_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(sck_pin,GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin,GPIO_FUNC_SPI);
    gpio_set_function(miso_pin,GPIO_FUNC_SPI);

    s->cs = cs_pin;
    s->miso = miso_pin;
    s->mosi = mosi_pin;
    s->sck = sck_pin;

    return 0;
}

mcp_status_t mcp_read_cfgn(mcp_info_t *s, uint8_t *dst, int cfg_n) {
    if(cfg_n<0 || cfg_n>3) return -1;

    uint8_t cmd[2] = {MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_STAT(MCP_REG_ADDR_CONFIG0 + cfg_n), 0};
    uint8_t rxbuff[2] = {0xFF,0xFF};

    gpio_put(s->cs,MCP_CS_SELECT);
    sleep_us(MCP_SLEEPTIME_US);

    spi_write_read_blocking(MCP_SPI, cmd, rxbuff, 2);

    sleep_us(MCP_SLEEPTIME_US);
    gpio_put(s->cs,MCP_CS_DESELECT);

    *dst = rxbuff[1];
    return (mcp_status_t)rxbuff[0];
}

mcp_status_t mcp_write_cfgn(mcp_info_t *s, uint8_t val, int cfg_n) {
    if(cfg_n<0 || cfg_n>3) return -1;

    uint8_t cmd[2] = {MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_CONFIG0 + cfg_n), val};
    uint8_t rx[2];

    gpio_put(s->cs,MCP_CS_SELECT);
    sleep_us(MCP_SLEEPTIME_US);

    spi_write_read_blocking(MCP_SPI, cmd, rx, 2);

    sleep_us(MCP_SLEEPTIME_US);
    gpio_put(s->cs,MCP_CS_DESELECT);

    return rx[0];
}

