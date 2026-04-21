#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "pico/multicore.h"

#include "ada4255_driver/ada4255.h"
#include "../ADPC_Dev/ADPC_cfg.h"

#define ADA_SLEEPTIME_US 5

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

    printf("provide a character to continue...\n");
    scanf(" %c",ui);

    printf("initializing...\n");
    ada_spi_init(&ada, ADC_1_SPI, ADC_1_PIN_MOSI,ADC_1_PIN_MISO,PGIA_PIN_CS,ADC_1_PIN_SCK);


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


    // read all config regs.
    printf("Reading Registers.\n");
    tx[0] = ADA_CMD_READ | 0x00;
    tx[1] = 0;
    gpio_put(ada.cs_pin,0); sleep_us(ADA_SLEEPTIME_US);
    spi_write_blocking(ada.spi, tx, 1);
    spi_read_blocking(ada.spi,0,rx,10);
    sleep_us(ADA_SLEEPTIME_US); gpio_put(ada.cs_pin,1);
    for(int i = 0;i<10; i++) {
        printf("Read: 0x%02X\n", rx[i]);
    }

    printf("Writing input mux.\n");
    int result = ada_input_select(&ada,ADA_INPUT_2);
    printf("%s", (result==0) ? "Success.\n" : "Input Mux Write Failure.\n");


    printf("provide a character to continue...\n");
    scanf(" %c",ui);


    printf("Done. Enter 'q' to exit. Enter any other character to re-read.\n");
    scanf(" %c",ui);
    if(ui[0]!='q') goto run;

reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}