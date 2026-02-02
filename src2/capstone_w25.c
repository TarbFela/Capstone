//
// Created by The Tragedy of Darth Wise on 12/17/25.
//


#include "capstone_w25.h"


void W25_Write_Enable() {
    // write enable
    gpio_put(W25_SPI_CS_PIN,W25_CS_SELECT); sleep_us(W25_CS_SLEEPTIME_US);
    uint8_t cmd_buff[1] = {WRITE_ENABLE};
    spi_write_blocking(W25_SPI, &cmd_buff[0], 1);
    //sleep_us(5);
    gpio_put(W25_SPI_CS_PIN, W25_CS_DESELECT);; sleep_us(W25_CS_SLEEPTIME_US);
    //sleep_us(5);
}

// returns 0xFF if something is wrong, otherwise returns status register values
uint8_t W25_Read_Status_1() {
    uint8_t status_reg = 0xFF;
    // read status
    gpio_put(W25_SPI_CS_PIN,W25_CS_SELECT); sleep_us(W25_CS_SLEEPTIME_US);
    uint8_t cmd_buff[1] = {READ_STATUS_REG_1};
    spi_write_blocking(W25_SPI, &cmd_buff[0], 1);
    spi_read_blocking(W25_SPI, 0, &status_reg, 1);
    //sleep_us(5);
    gpio_put(W25_SPI_CS_PIN, W25_CS_DESELECT);; sleep_us(W25_CS_SLEEPTIME_US);
    //sleep_us(5);
    return status_reg;
}

void W25_Block_While_Busy() {
    while((W25_Read_Status_1() & W25_STATUS_1_BUSY) == W25_STATUS_1_BUSY);
}

// clear a sector. Note that the chip will be busy for a while, so if you want to use it immediately use the blocking version. This is just more CPU-time friendly
void W25_Clear_Sector_No_Blocking(uint32_t address) {
    W25_Write_Enable();
    // clear sector
    gpio_put(W25_SPI_CS_PIN,W25_CS_SELECT); sleep_us(W25_CS_SLEEPTIME_US);
    uint8_t cmd_buff[4] = { SECTOR_ERASE, (address>>16)&0xFF, (address >> 8)&0xFF, address&0xFF};
    spi_write_blocking(W25_SPI, &cmd_buff[0], 4); // sector erase plus 24-bit address
    //sleep_us(5);
    gpio_put(W25_SPI_CS_PIN, W25_CS_DESELECT);; sleep_us(W25_CS_SLEEPTIME_US);
    //sleep_us(5);
}

// clear a sector and wait for it to be done clearing it
void W25_Clear_Sector_Blocking(uint32_t address) {
    W25_Clear_Sector_No_Blocking(address);
    W25_Block_While_Busy();
}

// Write N bytes to address (N < 256)
// note that exceeding the page-alignment (256bytes) will cause a wraparound.
// note also that you must wait for the write to finish before using the W25 again
void W25_Program_Page_No_Blocking(uint32_t address, uint8_t *data, uint16_t n_bytes) {
    W25_Write_Enable();
    // write data
    gpio_put(W25_SPI_CS_PIN,W25_CS_SELECT); sleep_us(W25_CS_SLEEPTIME_US);
    uint8_t cmd_buff[4] = { PAGE_PROGRAM, (address>>16)&0xFF, (address >> 8)&0xFF, address&0xFF};
    spi_write_blocking(W25_SPI, &cmd_buff[0], 4);
    spi_write_blocking(W25_SPI, data, n_bytes);
    gpio_put(W25_SPI_CS_PIN, W25_CS_DESELECT);; sleep_us(W25_CS_SLEEPTIME_US);
}

void W25_Program_Page_Blocking(uint32_t address, uint8_t *data, uint16_t n_bytes) {
    W25_Program_Page_No_Blocking(address, data, n_bytes);
    W25_Block_While_Busy();

}

void W25_Read_Data(uint32_t address, uint8_t *dst, uint32_t n_bytes) {
    // read data
    gpio_put(W25_SPI_CS_PIN,W25_CS_SELECT); sleep_us(W25_CS_SLEEPTIME_US);
    uint8_t cmd_buff[4] = { READ_DATA, (address>>16)&0xFF, (address >> 8)&0xFF, address&0xFF};
    spi_write_blocking(W25_SPI, &cmd_buff[0], 4);
    spi_read_blocking(W25_SPI,0,dst,n_bytes);
    gpio_put(W25_SPI_CS_PIN, W25_CS_DESELECT);; sleep_us(W25_CS_SLEEPTIME_US);
}

void W25_Init() {
    gpio_init(W25_SPI_CS_PIN);
    gpio_put(W25_SPI_CS_PIN, W25_CS_DESELECT);; sleep_us(W25_CS_SLEEPTIME_US);
    gpio_set_dir(W25_SPI_CS_PIN, GPIO_OUT);

    spi_init(W25_SPI, 5*100*1000); // 500kHz
    // drive device in 0,0 mode
    spi_set_format(W25_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(W25_SPI_SCK_PIN,GPIO_FUNC_SPI);
    gpio_set_function(W25_SPI_TX_PIN,GPIO_FUNC_SPI);
    gpio_set_function(W25_SPI_RX_PIN,GPIO_FUNC_SPI);
}