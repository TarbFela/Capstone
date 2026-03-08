#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "pico/multicore.h"

#include "mcp3x6xR_driver/mcp3x6xR.h"

int main() {
    stdio_init_all();
    char ui[64];
    mcp_info_t mcp;
    uint8_t tx[16], rx[16];
    for(int i =0; i<16; i++) {
        tx[i] = 0;
        rx[i] = 0;
    }

    // wait for user input.
    scanf(" %c",ui);
    mcp_spi_init(&mcp, 11,12,13,10);
    printf("MCP STRUCT:\n\tCS %d\n\tMOSI %d\n\tMISO %d\n\tSCK %d\n",mcp.cs,mcp.mosi,mcp.miso,mcp.sck);

    printf("provide a character to continue...\n");
    scanf(" %c",ui);

    // read all config regs.
    printf("Reading Config Registers\n");
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_INCR(MCP_REG_ADDR_CONFIG0);
    gpio_put(13,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 5);
    sleep_us(100); gpio_put(13,1);
    for(int i = 0; i<5; i++) {printf("RX[%d]: 0x%02X\n",i,rx[i]);}

    printf("provide a character to continue...\n");
    scanf(" %c",ui);

    // write all config regs
    printf("Writing Config Registers\n");
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_CONFIG0);
    tx[1] = 0xE2;
    tx[2] = 0x0C;
    tx[3] = 0x8B;
    tx[4] = 0x80;
    gpio_put(13,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 5);
    sleep_us(100); gpio_put(13,1);
    for(int i = 0; i<1; i++) {printf("RX[%d]: 0x%02X\n",i,rx[i]);}
    for(int i =0; i<16; i++) {
        tx[i] = 0;
        rx[i] = 0;
    }

    printf("provide a character to continue...\n");
    scanf(" %c",ui);

    // read all config regs again
    printf("Reading Config Registers\n");
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_INCR(MCP_REG_ADDR_CONFIG0);
    gpio_put(13,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 5);
    sleep_us(100); gpio_put(13,1);
    for(int i = 0; i<5; i++) {printf("RX[%d]: 0x%02X\n",i,rx[i]);}

    // write MUX
    printf("Writing MUX settings\n");
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_MUX);
    tx[1] = 0x01;
    gpio_put(13,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 2);
    sleep_us(100); gpio_put(13,1);
    for(int i = 0; i<1; i++) {printf("RX[%d]: 0x%02X\n",i,rx[i]);}
    for(int i =0; i<16; i++) {
        tx[i] = 0;
        rx[i] = 0;
    }

sample:
    // write ADC mode
    printf("Writing Config Registers\n");
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_CONFIG0);
    tx[1] = 0xE3;
    gpio_put(13,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 2);
    sleep_us(100); gpio_put(13,1);
    for(int i = 0; i<1; i++) {printf("RX[%d]: 0x%02X\n",i,rx[i]);}
    for(int i =0; i<16; i++) {
        tx[i] = 0;
        rx[i] = 0;
    }
    // read the status a couple times or until we get some data ready.


    printf("Reading status.\n");
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_DUMMY;
    for(int i = 0; i<100; i++) {
        rx[0] = 0;
        gpio_put(13, 0);
        sleep_us(100);
        spi_write_read_blocking(spi1, tx, rx, 1);
        sleep_us(100);
        gpio_put(13, 1);
        printf("[%02X] ",rx[0]);
        if(rx[0] == 0x13) {
            printf("\nConversion Ready. Reading ADC_DATA register.\n");
            tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_INCR(MCP_REG_ADDR_ADCDATA);
            gpio_put(13,0); sleep_us(100);
            spi_write_read_blocking(spi1, tx, rx, 3);
            sleep_us(100); gpio_put(13,1);
            for(int i = 0; i<3; i++) {printf("RX[%d]: 0x%02X\n",i,rx[i]);}
            break;
        }
    }

    printf("Done. Enter 'q' to exit. Enter any other character to re-read.\n");
    scanf(" %c",ui);
    if(ui[0]!='q') goto sample;

reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}