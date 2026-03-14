#include "ada4255.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"


void ada_spi_init(ada_info_t *s, spi_inst_t *spi, int mosi_pin, int miso_pin, int cs_pin, int sck_pin) {
    gpio_init(cs_pin);
    gpio_put(cs_pin, ADA_CS_DESELECT);
    gpio_set_dir(cs_pin, GPIO_OUT);

    // TODO: figure out how to arbitrate multiple spi inits on the same spi channel
    spi_init(spi, 80*1000); // 80kHz
    // drive device in 0,0 mode
    spi_set_format(s->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(sck_pin,GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin,GPIO_FUNC_SPI);
    gpio_set_function(miso_pin,GPIO_FUNC_SPI);

    // TODO: move these to a larger init function.
    s->spi = spi;
    s->cs_pin = cs_pin;
}



