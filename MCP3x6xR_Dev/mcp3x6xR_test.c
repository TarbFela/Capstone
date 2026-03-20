#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/multicore.h"


#include "mcp3x6xR_driver/mcp3x6xR.h"
#include "../src2/ADPC_cfg.h"
#include "mcp3x6xr_read.pio.h"

//
//volatile uint32_t adc_irq_counter = 0;
//
//void adc_gpio_irq_handler(uint gpio, uint32_t events) {
//    adc_irq_counter++;
//    printf("\t\tIRQ\n");
//}

void mcp3x6xr_pio_init(PIO pio, uint sm) {
    // Load program
    uint offset = pio_add_program(pio, &mcp3x6xr_read_program);

    // --- Reassign pins from SPI → PIO ---
    gpio_set_function(ADC_1_PIN_SCK,  PIO_FUNCSEL_NUM(pio, ADC_1_PIN_SCK));
    gpio_set_function(ADC_1_PIN_MISO, PIO_FUNCSEL_NUM(pio, ADC_1_PIN_MISO));
    gpio_set_function(ADC_1_PIN_IRQ,  GPIO_FUNC_SIO);  // stays as input for WAIT

    // Ensure directions
    gpio_set_dir(ADC_1_PIN_SCK, GPIO_OUT);
    gpio_set_dir(ADC_1_PIN_MISO, GPIO_IN);
    gpio_set_dir(ADC_1_PIN_IRQ, GPIO_IN);

    // --- Config ---
    pio_sm_config c = mcp3x6xr_read_program_get_default_config(offset);

    // SCK via sideset
    sm_config_set_sideset_pins(&c, ADC_1_PIN_SCK);

    // MISO input
    sm_config_set_in_pins(&c, ADC_1_PIN_MISO);

    // IRQ pin for WAIT
    sm_config_set_jmp_pin(&c, ADC_1_PIN_IRQ);

    // Shift config: MSB-first, autopush at 32 bits
    sm_config_set_in_shift(&c,
                           true,   // shift right
                           true,   // autopush
                           32
    );

    // Clock divider (adjust to meet ADC timing)
    sm_config_set_clkdiv(&c, 4.0f);

    // Init SM
    pio_sm_init(pio, sm, offset, &c);

    // Set pin directions inside PIO
    pio_sm_set_consecutive_pindirs(pio, sm, ADC_1_PIN_SCK, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, ADC_1_PIN_MISO, 1, false);

    // Start
    pio_sm_set_enabled(pio, sm, true);
}

int main() {
    stdio_init_all();
    char ui[64];
    mcp_info_t mcp;
    uint8_t tx[16], rx[16];
    for(int i =0; i<16; i++) {
        tx[i] = 0;
        rx[i] = 0;
    }

    gpio_disable_pulls(ADC_1_PIN_IRQ);
    gpio_init(ADC_1_PIN_IRQ);



    // wait for user input.
    scanf(" %c",ui);
    mcp_spi_init(&mcp, ADC_1_SPI, ADC_1_PIN_MOSI,ADC_1_PIN_MISO,ADC_1_PIN_CS,ADC_1_PIN_SCK);
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
        if((rx[0]&MCP_STAT_nDR_STATUS_MASK) == 0) {
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