#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "MCP4251_driver.h"

void MCP4251_init() {
    gpio_init(MCP_SPI_CS_PIN);
    gpio_put(MCP_SPI_CS_PIN, MCP_CS_DESELECT);
    gpio_set_dir(MCP_SPI_CS_PIN, GPIO_OUT);

    spi_init(MCP_SPI, 5*1000*1000); // 5MHz
    // drive device in 0,0 mode
    spi_set_format(MCP_SPI, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(MCP_SPI_SCK_PIN,GPIO_FUNC_SPI);
    gpio_set_function(MCP_SPI_TX_PIN,GPIO_FUNC_SPI);
    gpio_set_function(MCP_SPI_RX_PIN,GPIO_FUNC_SPI);

    // Attempt to put the device into 0,0 mode
    sleep_ms(1);
    gpio_put(MCP_SPI_CS_PIN, MCP_CS_SELECT);
    sleep_ms(1);
    gpio_put(MCP_SPI_CS_PIN, MCP_CS_DESELECT);
}

const uint16_t MCP_SPI_WIPER_ADDRESSES[2] = {0x0000, 0x1000};
void MCP4251_write_value(uint8_t value, uint8_t wiper_number) {
    // confine wiper number to 0-1 and value to 0-255
    uint16_t cmd = MCP_SPI_WIPER_ADDRESSES[wiper_number & 0x1] | (value & 0xFF);
    gpio_put(MCP_SPI_CS_PIN, MCP_CS_SELECT);
    spi_write16_blocking(MCP_SPI, &cmd, 1);
    gpio_put(MCP_SPI_CS_PIN, MCP_CS_DESELECT);
}
