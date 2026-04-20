#include "ada4255.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <stdio.h>

#define ADA_SLEEPTIME_US 5


void ada_spi_init(ada_info_t *s, spi_inst_t *spi, int mosi_pin, int miso_pin, int cs_pin, int sck_pin) {
    s->spi = spi;
    s->cs_pin = cs_pin;

    if(gpio_get_dir(cs_pin) != GPIO_OUT) {
        gpio_init(cs_pin);
        gpio_put(cs_pin, ADA_CS_DESELECT);
        gpio_set_dir(cs_pin, GPIO_OUT);
    }

    // TODO: figure out how to arbitrate multiple spi inits on the same spi channel
    spi_init(spi, 500*1000); // 80kHz
    // drive device in 0,0 mode
    spi_set_format(s->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(sck_pin,GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin,GPIO_FUNC_SPI);
    gpio_set_function(miso_pin,GPIO_FUNC_SPI);

    sleep_ms(100);

    for (int i = 0; i < 2; i++) {
        uint8_t rst[2] = {ADA_CMD_WRITE | ADA_ADDR_RESET, ADA_RESET_REG_RESET};
        gpio_put(s->cs_pin, ADA_CS_SELECT); sleep_us(ADA_SLEEPTIME_US);
        spi_write_blocking(s->spi, rst, 2);
        sleep_us(ADA_SLEEPTIME_US); gpio_put(s->cs_pin, ADA_CS_DESELECT);
        sleep_ms(200);
    }
}


// for now this just applies a set configuration to the device
void ada_configure(ada_info_t *s) {
    uint8_t tx[2];
    tx[0] = ADA_CMD_WRITE | ADA_ADDR_INPUT_MUX;
    tx[1] = ADA_INPUT_MUX_REG_SW_B1 | ADA_INPUT_MUX_REG_SW_B2;

    gpio_put(s->cs_pin, ADA_CS_SELECT); sleep_us(ADA_SLEEPTIME_US);
    spi_write_blocking(s->spi, tx, 2);
    sleep_us(ADA_SLEEPTIME_US); gpio_put(s->cs_pin, ADA_CS_DESELECT);

    tx[0] = ADA_CMD_READ | ADA_ADDR_INPUT_MUX;
    tx[1] = 0;
    gpio_put(s->cs_pin, ADA_CS_SELECT); sleep_us(ADA_SLEEPTIME_US);
    spi_write_blocking(s->spi,tx,1);
    spi_read_blocking(s->spi, 0, tx+1, 1);
    sleep_us(ADA_SLEEPTIME_US); gpio_put(s->cs_pin, ADA_CS_DESELECT);

    printf("rx: %02X\n",tx[1]);

    tx[0] = ADA_CMD_WRITE | ADA_ADDR_GAIN_MUX;
    tx[1] = ADA_GAIN_REG_MUX_INPUT_GAIN_32 | ADA_GAIN_REG_MUX_OUTPUT_SCALING_1;

    gpio_put(s->cs_pin, ADA_CS_SELECT); sleep_us(ADA_SLEEPTIME_US);
    spi_write_blocking(s->spi, tx, 2);
    sleep_us(ADA_SLEEPTIME_US); gpio_put(s->cs_pin, ADA_CS_DESELECT);

    tx[0] = ADA_CMD_READ | ADA_ADDR_GAIN_MUX;
    tx[1] = 0;
    gpio_put(s->cs_pin, ADA_CS_SELECT); sleep_us(ADA_SLEEPTIME_US);
    spi_write_blocking(s->spi,tx,1);
    spi_read_blocking(s->spi, 0, tx+1, 1);
    sleep_us(ADA_SLEEPTIME_US); gpio_put(s->cs_pin, ADA_CS_DESELECT);

    printf("rx: %02X\n",tx[1]);
}



