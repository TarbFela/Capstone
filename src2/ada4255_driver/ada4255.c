#include "ada4255.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <stdio.h>

#define ADA_SLEEPTIME_US 5

// TODO: don't brute-force the error bit clears.
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
        sleep_ms(150);
    }

    uint8_t derr = ada_check_digital_error(s);
    if(derr) {
        ada_clear_digital_error(s,derr);
    }

    uint8_t aerr = ada_check_analog_error(s);
    if(aerr) {
        ada_clear_analog_error(s,derr);
    }

}

// returns 0 if input select was verified
// -1 for bad input
int ada_input_select(ada_info_t *s, uint input) {
    uint8_t data = 0;
    switch (input) {
        case ADA_INPUT_1:
            data = ADA_INPUT_MUX_REG_SW_A1 | ADA_INPUT_MUX_REG_SW_A2;
            break;
        case ADA_INPUT_2:
            data = ADA_INPUT_MUX_REG_SW_B1 | ADA_INPUT_MUX_REG_SW_B2;
            break;
        case ADA_INPUT_TEST_MUX:
            data = ADA_INPUT_MUX_REG_SW_C1 | ADA_INPUT_MUX_REG_SW_C2;
            break;
        case ADA_INPUT_SHORT:
            data = ADA_INPUT_MUX_REG_SW_D12;
            break;
        default:
            return -1;
    }
//    printf("is data: 0x%02X\n",data);
    ada_write_reg(s, ADA_ADDR_INPUT_MUX, data);
    uint8_t read = ada_read_reg(s, ADA_ADDR_INPUT_MUX);
//    printf("is read: 0x%02X\n",read);
//    printf("same? %s\n",(data==read) ? "YES" : "NO");
    if(read != data) return -2;
    return 0;
}

// does not check that addr is valid.
// Returns read data (one byte).
uint8_t ada_read_reg(ada_info_t *s, uint8_t addr) {
    uint8_t tx = ADA_CMD_READ | addr;
    uint8_t rx = 0;
    gpio_put(s->cs_pin,0); sleep_us(ADA_SLEEPTIME_US);
    spi_write_blocking(s->spi, &tx, 1);
    spi_read_blocking(s->spi,0,&rx,1);
    sleep_us(ADA_SLEEPTIME_US); gpio_put(s->cs_pin,1);
    return rx;
}

// does not check that addr is valid.
void ada_write_reg(ada_info_t *s, uint8_t addr, uint8_t val) {
    uint8_t tx[2] =
            {ADA_CMD_WRITE | addr,
             val
            };
    gpio_put(s->cs_pin,0); sleep_us(ADA_SLEEPTIME_US);
    spi_write_blocking(s->spi, tx, 2);
    sleep_us(ADA_SLEEPTIME_US); gpio_put(s->cs_pin,1);
}

uint8_t ada_check_digital_error(ada_info_t *s) {
    return ada_read_reg(s,ADA_ADDR_DIGITAL_ERR);
}

void ada_clear_digital_error(ada_info_t *s, uint8_t bits) {
    ada_write_reg(s,ADA_ADDR_DIGITAL_ERR,bits);
}

uint8_t ada_check_analog_error(ada_info_t *s) {
    return ada_read_reg(s,ADA_ADDR_ANALOG_ERR);
}

void ada_clear_analog_error(ada_info_t *s, uint8_t bits) {
    ada_write_reg(s,ADA_ADDR_ANALOG_ERR,bits);
}





