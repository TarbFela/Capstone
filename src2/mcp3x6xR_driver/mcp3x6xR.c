#include "mcp3x6xR.h"

#include <stdio.h>
#include <stdint.h>

#include "stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#define MCP_SPI_BAUDRATE 48000
#define MCP_SLEEPTIME_US 500
#define MCP_CS_DESELECT 1
#define MCP_CS_SELECT 0

mcp_status_t mcp_spi_init(mcp_info_t *s, spi_inst_t *spi, int mosi_pin, int miso_pin, int cs_pin, int sck_pin, int nirq_pin) {
    gpio_init(cs_pin);
    gpio_put(cs_pin, MCP_CS_DESELECT);
    gpio_set_dir(cs_pin, GPIO_OUT);

    spi_init(spi, 80*1000); // 80kHz
    // drive device in 0,0 mode
    spi_set_format(s->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(sck_pin,GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin,GPIO_FUNC_SPI);
    gpio_set_function(miso_pin,GPIO_FUNC_SPI);

    s->cs = cs_pin;
    s->miso = miso_pin;
    s->mosi = mosi_pin;
    s->sck = sck_pin;
    s->nirq = nirq_pin;

    return 0;
}


mcp_status_t mcp_read_regs(mcp_info_t *s, uint8_t *dst, uint n, int reg_addr) {
    uint8_t cmd[2] = {MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_INCR(reg_addr)};
    uint8_t status = 0xFF;

    gpio_put(s->cs,MCP_CS_SELECT); sleep_us(MCP_SLEEPTIME_US);
    spi_write_read_blocking(s->spi, cmd, &status, 1);
    spi_read_blocking(s->spi, 0,dst,n);
    sleep_us(MCP_SLEEPTIME_US); gpio_put(s->cs,MCP_CS_DESELECT);

    return status;
}

mcp_status_t mcp_write_regs(mcp_info_t *s, uint8_t *vals, uint n, int reg_addr) {
    uint8_t cmd = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(reg_addr);
    uint8_t status = 0xFF;

    gpio_put(s->cs,MCP_CS_SELECT); sleep_us(MCP_SLEEPTIME_US);
    spi_write_read_blocking(s->spi, &cmd, &status, 1);
    spi_write_blocking(s->spi,vals,n);
    sleep_us(MCP_SLEEPTIME_US); gpio_put(s->cs,MCP_CS_DESELECT);

    return status;
}
//
//// write regs and then verify them
//// returns 0 for all matched, 1 for mismatch
//int mcp_write_regs_verify(mcp_info_t *s, uint8_t *vals, uint n, int reg_addr) {
//    mcp_write_regs(s,vals,n,reg_addr);
//    uint8_t *read_vals;
//    read_vals = (uint8_t *)calloc(sizeof(uint8_t) * n);
//    mcp_read_regs(s,read_vals,n,reg_addr);
//
//    int ret;
//    for(int i = 0; i<n; i++) {
//        ret |= (vals[i] != read_vals[i]);
//    }
//    free(read_vals);
//    return ret;
//}

// Read–modify–write
// only works on 8-bit registers.
// mask and val should both be shifted to the appropriate position.
mcp_status_t mcp_write_reg_masked(mcp_info_t *s, uint8_t mask, uint32_t val, int reg_addr) {
    if((mask&val) != val) return MCP_STATUS_BAD_INPUTS;
    uint8_t status = 0xFF;
    uint8_t reg_val = 0xFF;

    // TODO: add status checking
    status = mcp_read_regs(s, &reg_val, 1, reg_addr);
    reg_val &= ~mask;
    reg_val |= val;
    status = mcp_write_regs(s,&reg_val,1,reg_addr);

    return status;
}

// returns 0 for matched, 1 for mismatch, -1 for bad inputs.
int mcp_write_reg_masked_verify(mcp_info_t *s, uint8_t mask, uint32_t val, int reg_addr) {
    mcp_status_t status;
    status = mcp_write_reg_masked(s,mask,val,reg_addr);
    if(status == MCP_STATUS_BAD_INPUTS) return -1;
    uint8_t read_val = 0xFF;
    mcp_read_regs(s,&read_val,1,reg_addr);
    return (read_val & mask) != (val);
}

//
//int mcp_write_config(mcp_info_t *s, mcp_config_t *cfg) {
//    mcp_write_regs(s, cfg_data, 4, MCP_REG_ADDR_CONFIG0);
//        conv_mode | data_format | crc_format | crc_en | offset_cal_en | gain_cal_en
//        boost_current_sel
//        gain_sel
//        zero_mux_en
//        auto_zero_en
//        amclk_prescale
//        osr
//        vref_sel
//        partial_shutdown
//        clk_sel
//        bias_current_sel
//        adc_mode
//        input_mode
//        scan_mode_inputs
//        mux_mode_inputs
//}

mcp_status_t mcp_single_conversion(mcp_info_t *s, uint16_t *dst) {
    // initiate conversion.
    uint8_t tx[3] = {MCP_CMD_DEV_ADDR | MCP_CMD_ADC_CONV_START_FAST, 0x0, 0x0};
    uint8_t rx[3] = {0xFF,0xFF,0xFF};

    gpio_put(s->cs,MCP_CS_SELECT);
    sleep_us(MCP_SLEEPTIME_US);

    spi_write_read_blocking(s->spi, tx, rx, 1);

    sleep_us(MCP_SLEEPTIME_US);
    gpio_put(s->cs,MCP_CS_DESELECT);


    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_DUMMY;
    // wait
    do {
        rx[0] = 0xFF;
        gpio_put(s->cs, MCP_CS_SELECT);
        sleep_us(MCP_SLEEPTIME_US);
        spi_write_read_blocking(s->spi, tx, rx, 1);
        sleep_us(MCP_SLEEPTIME_US);
        gpio_put(s->cs, MCP_CS_DESELECT);
    } while(rx[0]&MCP_STAT_nDR_STATUS_MASK);


    // read out data
    rx[0] = 0xFF;
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_INCR(MCP_REG_ADDR_ADCDATA);

    gpio_put(s->cs,MCP_CS_SELECT);
    sleep_us(MCP_SLEEPTIME_US);

    // length of 3: one status byte, two ADC bytes.
    spi_write_read_blocking(s->spi, tx, rx, 3);

    sleep_us(MCP_SLEEPTIME_US);
    gpio_put(s->cs,MCP_CS_DESELECT);

    *dst = rx[1]<<8 | rx[2];
    return rx[0];
}

mcp_status_t mcp_mux_sel(mcp_info_t *s, mcp_mux_vals_t mux_p, mcp_mux_vals_t mux_n) {
    uint8_t tx[2] = {
            MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_MUX),
            MCP_MUX_N_SEL(mux_n) | MCP_MUX_P_SEL(mux_p)
    };
    uint8_t rx[2] = {0xFF,0xFF};

    gpio_put(s->cs,MCP_CS_SELECT);
    sleep_us(MCP_SLEEPTIME_US);

    spi_write_read_blocking(s->spi, tx, rx, 2);

    sleep_us(MCP_SLEEPTIME_US);
    gpio_put(s->cs,MCP_CS_DESELECT);

    return rx[0];
}

