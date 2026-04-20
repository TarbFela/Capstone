#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "pico/multicore.h"

#include "ada4255_driver/ada4255.h"
#include "../ADPC_Dev/ADPC_cfg.h"

int main() {
    stdio_init_all();
    char ui[64];
    ada_info_t ada;
    uint8_t tx[16], rx[16];
    for(int i =0; i<16; i++) {
        tx[i] = 0;
        rx[i] = 0;
    }

    gpio_init(PGIA_PIN_CS);
    gpio_put(PGIA_PIN_CS,ADA_CS_DESELECT);
    gpio_set_dir(PGIA_PIN_CS,GPIO_OUT);

    // wait for user input.
    scanf(" %c",ui);

run:
    gpio_init(ADC_1_PIN_CS);
    gpio_put(ADC_1_PIN_CS, ADA_CS_DESELECT);
    gpio_set_dir(ADC_1_PIN_CS,GPIO_OUT);
    ada_spi_init(&ada, ADC_1_SPI, ADC_1_PIN_MOSI,ADC_1_PIN_MISO,ADC_1_PIN_CS,ADC_1_PIN_SCK);

    printf("provide a character to continue...\n");
    scanf(" %c",ui);

    // read all config regs.
    printf("Reading Registers.\n");
    tx[0] = ADA_CMD_READ | 0x00;
    tx[1] = 0;
    gpio_put(PGIA_PIN_CS,0); sleep_us(5);
    spi_write_blocking(PGIA_SPI, tx, 1);
    spi_read_blocking(PGIA_SPI,0,rx,10);
    sleep_us(5); gpio_put(PGIA_PIN_CS,1);
    for(int i = 0;i<10; i++) {
        printf("Read: 0x%02X\n", rx[i]);
    }
    printf("provide a character to continue...\n");
    scanf(" %c",ui);

    printf("CHECKING DIGITAL ERROR REGISTER...\n");
    tx[0] = ADA_CMD_READ | ADA_ADDR_DIGITAL_ERR;
    gpio_put(PGIA_PIN_CS,0); sleep_us(5);
    spi_write_blocking(PGIA_SPI, tx, 1);
    spi_read_blocking(PGIA_SPI,0,rx,1);
    sleep_us(5); gpio_put(PGIA_PIN_CS,1);
    printf("Read: 0x%02X\n", rx[0]);

    printf("CLEARING DIGITAL ERROR REGISTER...\n");
    tx[0] = ADA_CMD_WRITE | ADA_ADDR_DIGITAL_ERR;
    tx[1] = ADA_DIGITAL_ERR_REG_SPI_SCLK_CNT_ERR;
    gpio_put(PGIA_PIN_CS,0); sleep_us(5);
    spi_write_blocking(PGIA_SPI, tx, 2);
    sleep_us(5); gpio_put(PGIA_PIN_CS,1);

    printf("CHECKING DIGITAL ERROR REGISTER...\n");
    tx[0] = ADA_CMD_READ | ADA_ADDR_DIGITAL_ERR;
    gpio_put(PGIA_PIN_CS,0); sleep_us(5);
    spi_write_blocking(PGIA_SPI, tx, 1);
    spi_read_blocking(PGIA_SPI,0,rx,1);
    sleep_us(5); gpio_put(PGIA_PIN_CS,1);
    printf("Read: 0x%02X\n", rx[0]);

    printf("WRITING INPUT MIX...\n");
    tx[0] = ADA_CMD_WRITE | ADA_ADDR_INPUT_MUX;
    tx[1] = ADA_INPUT_MUX_REG_SW_B1 | ADA_INPUT_MUX_REG_SW_B2;
    gpio_put(PGIA_PIN_CS,0); sleep_us(5);
    spi_write_blocking(PGIA_SPI, tx, 2);
    sleep_us(5); gpio_put(PGIA_PIN_CS,1);

    printf("CHECKING DIGITAL ERROR REGISTER...\n");
    tx[0] = ADA_CMD_READ | ADA_ADDR_DIGITAL_ERR;
    gpio_put(PGIA_PIN_CS,0); sleep_us(5);
    spi_write_blocking(PGIA_SPI, tx, 1);
    spi_read_blocking(PGIA_SPI,0,rx,1);
    sleep_us(5); gpio_put(PGIA_PIN_CS,1);
    printf("Read: 0x%02X\n", rx[0]);

    printf("CHECKING INPUT MUX...\n");
    tx[0] = ADA_CMD_READ | ADA_ADDR_INPUT_MUX;
    gpio_put(PGIA_PIN_CS,0); sleep_us(5);
    spi_write_blocking(PGIA_SPI, tx, 1);
    spi_read_blocking(PGIA_SPI,0,rx,1);
    sleep_us(5); gpio_put(PGIA_PIN_CS,1);
    printf("Read: 0x%02X\n", rx[0]);


    printf("Done. Enter 'q' to exit. Enter any other character to re-read.\n");
    scanf(" %c",ui);
    if(ui[0]!='q') goto run;

reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}