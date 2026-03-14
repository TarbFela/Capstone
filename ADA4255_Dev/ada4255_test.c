#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "pico/multicore.h"

#include "ada4255_driver/ada4255.h"
#include "../src2/ADPC_cfg.h"

int main() {
    stdio_init_all();
    char ui[64];
    ada_info_t ada;
    uint8_t tx[16], rx[16];
    for(int i =0; i<16; i++) {
        tx[i] = 0;
        rx[i] = 0;
    }

    // wait for user input.
    scanf(" %c",ui);
    ada_spi_init(&ada, ADC_1_SPI, ADC_1_PIN_MOSI,ADC_1_PIN_MISO,ADC_1_PIN_CS,ADC_1_PIN_SCK);

    printf("provide a character to continue...\n");
    scanf(" %c",ui);

    // read all config regs.
    printf("Reading Clock Sync Register.\n");
    tx[0] = ADA_CMD_READ | ADA_ADDR_SYNC_CFG;
    gpio_put(PGIA_PIN_CS,0); sleep_us(100);
    spi_write_blocking(PGIA_SPI, tx, 1);
    spi_read_blocking(PGIA_SPI,0,rx,1);
    sleep_us(100); gpio_put(PGIA_PIN_CS,1);
    printf("Read: 0x%02X\n",rx[0]);

    printf("provide a character to continue...\n");
    scanf(" %c",ui);

    printf("Done. Enter 'q' to exit. Enter any other character to re-read.\n");
    scanf(" %c",ui);
    //if(ui[0]!='q') goto sample;

reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}