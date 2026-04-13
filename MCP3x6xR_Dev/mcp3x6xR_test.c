#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"

#include "mcp3x6xR_driver/mcp3x6xR.h"
#include "../src2/ADPC_cfg.h"
//#define ADC_1_PIN_MOSI      11
//#define ADC_1_PIN_MISO      12
//#define ADC_1_PIN_CS        13
//#define ADC_1_PIN_SCK       10
//#define ADC_1_PIN_IRQ       15
//#define ADC_1_SPI           spi1

#include "mcp_pio.h"

//volatile uint32_t adc_irq_counter = 0;
//
//void adc_gpio_irq_handler(uint gpio, uint32_t events) {
//    adc_irq_counter++;
//    printf("\t\tIRQ\n");
//}

mcp_pio_t mpio;

volatile int dma_done = 0;
void dma_irq_handler(void) {
    dma_done = 1;
    mcp_pio_stop(&mpio);
    // clear the correct interrupt
    dma_hw->ints0 = 0x1 << (mpio.dma);
    printf("DMA DONE!!\n");
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
    //gpio_set_irq_enabled_with_callback(ADC_1_PIN_IRQ, GPIO_IRQ_EDGE_FALL, true, &adc_gpio_irq_handler);


    // wait for user input.
    scanf(" %c",ui);
    printf("INITIALIZING...\n");
    mcp_spi_init(&mcp, ADC_1_SPI, ADC_1_PIN_MOSI,ADC_1_PIN_MISO,ADC_1_PIN_CS,ADC_1_PIN_SCK,ADC_1_PIN_IRQ);
    printf("MCP STRUCT:\n\tCS %d\n\tMOSI %d\n\tMISO %d\n\tSCK %d\n",mcp.cs,mcp.mosi,mcp.miso,mcp.sck);

    uint32_t dma_buff[100];
    mcp_pio_init(&mpio, &mcp, dma_buff, dma_irq_handler);

    printf("provide a character to continue...\n");
    scanf(" %c",ui);

    // read all config regs.
    printf("Reading Config Registers\n");
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_INCR(MCP_REG_ADDR_CONFIG0);
    gpio_put(ADC_1_PIN_CS,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 5);
    sleep_us(100); gpio_put(ADC_1_PIN_CS,1);
    for(int i = 0; i<5; i++) {printf("RX[%d]: 0x%02X\n",i,rx[i]);}

    printf("provide a character to continue...\n");
    scanf(" %c",ui);

    // write all config regs
    printf("Writing Config Registers\n");
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_CONFIG0);
    tx[1] = 0xE2;
    tx[2] = 0x0C;
    tx[3] = 0x8B;
    tx[4] = MCP_CFG3_CONV_MODE_CONTINUOUS | MCP5_CFG3_DATA_FORMAT_32_SGN;
    gpio_put(ADC_1_PIN_CS,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 5);
    sleep_us(100); gpio_put(ADC_1_PIN_CS,1);
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
    gpio_put(ADC_1_PIN_CS,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 5);
    sleep_us(100); gpio_put(ADC_1_PIN_CS,1);
    for(int i = 0; i<5; i++) {printf("RX[%d]: 0x%02X\n",i,rx[i]);}

    // write MUX
    printf("Writing MUX settings\n");
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_MUX);
    tx[1] = MCP_MUX_N_SEL(MCP_MUX_VAL_Int_Temp_Diode_M) | MCP_MUX_P_SEL(MCP_MUX_VAL_Int_Temp_Diode_P);
    gpio_put(ADC_1_PIN_CS,0); sleep_us(100);
    spi_write_read_blocking(spi1, tx, rx, 2);
    sleep_us(100); gpio_put(ADC_1_PIN_CS,1);
    for(int i = 0; i<1; i++) {printf("RX[%d]: 0x%02X\n",i,rx[i]);}
    for(int i =0; i<16; i++) {
        tx[i] = 0;
        rx[i] = 0;
    }


    uint32_t adc_data[256];
sample:
    // clear samples
    for(int i = 0; i<256; i++) adc_data[i] = 0;
    // write ADC mode
    printf("Writing Config Registers\n");
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_CONFIG0);
    tx[1] = MCP_CFG0_VREF_SEL_INTERNAL | MCP_CFG0_NO_PARTIAL_SHUTDOWN | MCP_CFG0_CLK_SEL_INTERNAL | MCP_CFG0_ADC_MODE_CONV;
    gpio_put(ADC_1_PIN_CS,0); sleep_us(5);
    spi_write_read_blocking(spi1, tx, rx, 2);
    sleep_us(5); gpio_put(ADC_1_PIN_CS,1);
//    for(int i = 0; i<1; i++) {printf("RX[%d]: 0x%02X\n",i,rx[i]);}
//    for(int i =0; i<16; i++) {
//        tx[i] = 0;
//        rx[i] = 0;
//    }
//
    printf(".");
    // prepare to perform static read of ADC register
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_STAT(MCP_REG_ADDR_ADCDATA);
    // wait for first IRQ pin signal
    while(!gpio_get(ADC_1_PIN_IRQ));

    gpio_put(ADC_1_PIN_CS,0); sleep_us(5);
    spi_write_blocking(spi1, tx, 1);
    printf(".");

    mcp_pio_start(&mpio);
    printf(".");

    int ti = 0;
    dma_done = 0;
    while(1) {
        if((ti++) > 100) {
            printf("TIMEOUT.\n");
            mcp_pio_stop(&mpio);
            break;
        }
        if(dma_done) break;
        sleep_ms(50);
    }

    sleep_us(5); gpio_put(ADC_1_PIN_CS,1);

    // stop the ADC.
    printf("Writing Config Registers\n");
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_CONFIG0);
    tx[1] = MCP_CFG0_VREF_SEL_INTERNAL | MCP_CFG0_NO_PARTIAL_SHUTDOWN | MCP_CFG0_CLK_SEL_INTERNAL | MCP_CFG0_ADC_MODE_STDBY;
    gpio_put(ADC_1_PIN_CS,0); sleep_us(5);
    spi_write_read_blocking(spi1, tx, rx, 2);
    sleep_us(5); gpio_put(ADC_1_PIN_CS,1);

    if(ti>100) goto reboot;

    printf("Samples:\n");
    for(int i = 0; i<100; i++) {
        printf("%10ld\t",mpio.buff[i]);
        if((i%4)==3) printf("\n");
    }
//    printf( "GPIO NIRQ DIRECTION: %s\n"
//            ,gpio_get_dir(ADC_1_PIN_IRQ) ? "OUT" : "IN"
//            );

    printf("Done. Enter 'q' to exit. Enter any other character to re-read.\n");
    scanf(" %c",ui);
    if(ui[0]!='q') goto sample;

reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}