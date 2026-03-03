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
    return rxbuff[0];
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

mcp_status_t mcp_single_conversion(mcp_info_t *s, uint16_t *dst) {
    // initiate conversion.
    uint8_t tx[3] = {MCP_CMD_DEV_ADDR | MCP_CMD_ADC_CONV_START_FAST, 0x0, 0x0};
    uint8_t rx[3] = {0xFF,0xFF,0xFF};

    gpio_put(s->cs,MCP_CS_SELECT);
    sleep_us(MCP_SLEEPTIME_US);

    spi_write_read_blocking(MCP_SPI, tx, rx, 1);

    sleep_us(MCP_SLEEPTIME_US);
    gpio_put(s->cs,MCP_CS_DESELECT);


    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_DUMMY;
    // wait
    do {
        rx[0] = 0xFF;
        gpio_put(s->cs, MCP_CS_SELECT);
        sleep_us(MCP_SLEEPTIME_US);
        spi_write_read_blocking(MCP_SPI, tx, rx, 1);
        sleep_us(MCP_SLEEPTIME_US);
        gpio_put(s->cs, MCP_CS_DESELECT);
    } while(rx[0]&MCP_STAT_nDR_STATUS_MASK);


    // read out data
    rx[0] = 0xFF;
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_INCR(MCP_REG_ADDR_ADCDATA);

    gpio_put(s->cs,MCP_CS_SELECT);
    sleep_us(MCP_SLEEPTIME_US);

    // length of 3: one status byte, two ADC bytes.
    spi_write_read_blocking(MCP_SPI, tx, rx, 3);

    sleep_us(MCP_SLEEPTIME_US);
    gpio_put(s->cs,MCP_CS_DESELECT);

    *dst = rx[1]<<8 | rx[2];
    return rx[0];
}

mcp_status_t mcp_mux_sel(mcp_info_t *s, mcp_mux_vals_t mux_p, mcp_mux_vals_t mux_n) {
    uint8_t tx[2] = {
            MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_MUX),
            MCP_MUX_N_SEL(mux_n) | MCP_MUX_P_SEL(mux_p)
    };
    uint8_t rx[2] = {0xFF,0xFF};

    gpio_put(s->cs,MCP_CS_SELECT);
    sleep_us(MCP_SLEEPTIME_US);

    spi_write_read_blocking(MCP_SPI, tx, rx, 2);

    sleep_us(MCP_SLEEPTIME_US);
    gpio_put(s->cs,MCP_CS_DESELECT);

    return rx[0];
}

