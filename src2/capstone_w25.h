//
// Created by The Tragedy of Darth Wise on 12/17/25.
//

#ifndef CAPSTONE_PWM_CAPSTONE_W25_H
#define CAPSTONE_PWM_CAPSTONE_W25_H


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#include "hardware/spi.h"

// TODO: HUGE, NEED TO DO ERROR PROPOGATION WITH AN ENUM. HOWEVER IT IS 5AM SO I WILL NOT DO IT IN TIME FOR DESIGN REVIEW.


#define W25_SPI spi1
#define W25_SPI_SCK_PIN     14
#define W25_SPI_TX_PIN      15
#define W25_SPI_RX_PIN      12
#define W25_SPI_CS_PIN      13

#define W25_CS_DESELECT     1
#define W25_CS_SELECT       0

// commands
#define READ_STATUS_REG_1   0x05
#define WRITE_ENABLE        0x06
#define WRITE_DISABLE       0x04
#define READ_DATA           0x03
#define SECTOR_ERASE        0x20
#define PAGE_PROGRAM        0x02

// status bit masks
#define W25_STATUS_1_BUSY   0x01
#define W25_STATUS_1_WEN    0x02

typedef struct {
    uint8_t n_pages_written;
    uint8_t n_pages_read;
} W25_filesystem_t;

void W25_Write_Enable();

// returns 0xFF if something is wrong, otherwise returns status register values
uint8_t W25_Read_Status_1();

void W25_Block_While_Busy();

// clear a sector. Note that the chip will be busy for a while, so if you want to use it immediately use the blocking version. This is just more CPU-time friendly
void W25_Clear_Sector_No_Blocking(uint32_t address);

// clear a sector and wait for it to be done clearing it
void W25_Clear_Sector_Blocking(uint32_t address);

// Write N bytes to address (N < 256)
// note that exceeding the page-alignment (256bytes) will cause a wraparound.
// note also that you must wait for the write to finish before using the W25 again
void W25_Program_Page_No_Blocking(uint32_t address, uint8_t *data, uint16_t n_bytes);

void W25_Program_Page_Blocking(uint32_t address, uint8_t *data, uint16_t n_bytes);

void W25_Read_Data(uint32_t address, uint8_t *dst, uint32_t n_bytes);

void W25_Init();
#endif //CAPSTONE_PWM_CAPSTONE_W25_H
