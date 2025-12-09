//
// Created by The Tragedy of Darth Wise on 5/26/25.
//

#ifndef MCP4251_DRIVER_MCP4251_DRIVER_H
#define MCP4251_DRIVER_MCP4251_DRIVER_H

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define MCP_SPI spi1
#define MCP_CS_SELECT 0
#define MCP_CS_DESELECT 1
// pinout info for SPI:
// PIN   GPIO    SPI
// 14    10      SCK
// 15    11      TX
// 16    12      RX
// 17    13      CS
#define MCP_SPI_SCK_PIN 10
#define MCP_SPI_TX_PIN 11
#define MCP_SPI_RX_PIN 12
#define MCP_SPI_CS_PIN 13

void MCP4251_init();
void MCP4251_write_value(uint8_t value, uint8_t wiper_number);

#endif //MCP4251_DRIVER_MCP4251_DRIVER_H
