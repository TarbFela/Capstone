#include "mcp3x6xR.h"

#include <stdint.h>
#include "pico/stdlib.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"

#define MCP_SPI_BAUDRATE 48000
#define MCP_SLEEPTIME_US 500
#define MCP_CS_DESELECT 1
#define MCP_CS_SELECT 0

mcp_status_t mcp_spi_init(mcp_info_t *s, spi_inst_t *spi, int mosi_pin, int miso_pin, int cs_pin, int sck_pin, int nirq_pin) {
    gpio_init(cs_pin);
    gpio_put(cs_pin, MCP_CS_DESELECT);
    gpio_set_dir(cs_pin, GPIO_OUT);

    spi_init(spi, 500*1000); // 500kHz
    // drive device in 0,0 mode
    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
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


mcp_status_t mcp_configure(mcp_info_t *s, uint8_t cfg0, uint8_t cfg1, uint8_t cfg2, uint8_t cfg3) {
    // force no partial shutdown and standby mode.
    s->cfg.cfg[0] = (cfg0 & ~MCP_CFG0_ADC_MODE_BITS) | MCP_CFG0_ADC_MODE_STDBY | MCP_CFG0_ADC_MODE_BITS;
    s->cfg.cfg[1] = cfg1;
    s->cfg.cfg[2] = cfg2;
    s->cfg.cfg[3] = cfg3;

    mcp_status_t status;
    status = mcp_write_regs(s, s->cfg.cfg, 3, MCP_REG_ADDR_CONFIG0);
    if(status == 0x00) return MCP_STATUS_NO_CONNECTION;
    if(status & MCP_STATUS_ERROR_FLAG) return status;

    uint8_t rx[5];
    status = mcp_read_regs(s, rx, 3, MCP_REG_ADDR_CONFIG0);
    if(status == 0x00) return MCP_STATUS_NO_CONNECTION;

    // check that config was written correctly.
    if (    (rx[1] != s->cfg.cfg[0])
        ||  (rx[2] != s->cfg.cfg[1])
        ||  (rx[3] != s->cfg.cfg[2])
        ||  (rx[4] != s->cfg.cfg[3])) {
        return MCP_STATUS_WRITE_FAILED;
    }

    return status;
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


// ============================================================
// Add these functions to mcp3x6xR.c
// ============================================================

// Internal helper: read a 24-bit register into a uint32_t.
// The 3 register bytes are packed as [byte0<<16 | byte1<<8 | byte2].
static mcp_status_t mcp_read_reg24(mcp_info_t *s, uint32_t *dst, int reg_addr) {
    uint8_t buf[3] = {0, 0, 0};
    mcp_status_t status = mcp_read_regs(s, buf, 3, reg_addr);
    *dst = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    return status;
}

// Internal helper: write a uint32_t to a 24-bit register (uses only bits [23:0]).
static mcp_status_t mcp_write_reg24(mcp_info_t *s, uint32_t val, int reg_addr) {
    uint8_t buf[3] = {
            (val >> 16) & 0xFF,
            (val >>  8) & 0xFF,
            (val      ) & 0xFF
    };
    return mcp_write_regs(s, buf, 3, reg_addr);
}

// Internal helper: read-modify-write a 24-bit register.
// mask and val should both be in [23:0], already shifted to position.
static mcp_status_t mcp_write_reg24_masked(mcp_info_t *s, uint32_t mask, uint32_t val, int reg_addr) {
    if ((mask & val) != val) return MCP_STATUS_BAD_INPUTS;
    uint32_t reg_val;
    mcp_status_t status = mcp_read_reg24(s, &reg_val, reg_addr);
    reg_val &= ~mask;
    reg_val |=  val;
    status = mcp_write_reg24(s, reg_val, reg_addr);
    return status;
}

// Set the active SCAN channels via a bitmask of MCP_SCAN_SEL_BIT_* values.
// Enables SCAN mode if any bit is set; passing 0 disables SCAN mode
// and returns the device to MUX mode.
// Does not disturb the DLY[2:0] field.
mcp_status_t mcp_scan_set_channels(mcp_info_t *s, uint16_t channel_mask) {
    return mcp_write_reg24_masked(s,
                                  MCP_SCAN_CHANNEL_MASK,
                                  (uint32_t)channel_mask,
                                  MCP_REG_ADDR_SCAN
    );
}

// Set the inter-conversion delay within a SCAN cycle (DLY[2:0], SCAN[23:21]).
// delay_dly must be one of the MCP_SCAN_DLY_* macros (already shifted).
// Does not disturb the channel selection bits.
mcp_status_t mcp_scan_set_dly(mcp_info_t *s, uint32_t delay_dly) {
    return mcp_write_reg24_masked(s,
                                  MCP_SCAN_DLY_MASK,
                                  delay_dly,
                                  MCP_REG_ADDR_SCAN
    );
}

// Set the delay between consecutive SCAN cycles (TIMER[23:0]).
// timer_dmclk is a raw DMCLK period count: 0 = no delay, max = 0xFFFFFF.
// When timer_dmclk > 256, the ADC enters shutdown between cycles (lower power);
// when <= 256 it enters standby instead.
// Only relevant in continuous conversion mode (CONV_MODE[1:0] = 11).
mcp_status_t mcp_timer_set(mcp_info_t *s, uint32_t timer_dmclk) {
    if (timer_dmclk > 0xFFFFFF) return MCP_STATUS_BAD_INPUTS;
    return mcp_write_reg24(s, timer_dmclk, MCP_REG_ADDR_TIMER);
}

