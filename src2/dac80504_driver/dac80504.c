/*
 * dac80504.c — implementation of TI DAC80504 quad 16-bit SPI DAC driver.
 *
 * See dac80504.h for frame format, SPI mode, and the SPI-bus-sharing note
 * against MCP3462R (modes are incompatible; spi_set_format must be called
 * when switching between DAC and ADC access).
 *
 * HARDWARE NOTE on LDAC# (confirmed 2026-04-18 by Isaiah):
 * On the current MSIF rev, U15 pins 11 (LDAC#) and 12 (CS#) are intentionally
 * shorted — a single magnet-wire bodge from GPIO 42 lands on the shorted pair.
 * This is fine because dac_spi_init() explicitly writes SYNC=0x0000 to put
 * all four channels into asynchronous update mode. In async mode the DAC
 * ignores LDAC# entirely and updates its output on the CS# rising edge at
 * the end of each SPI write.
 *
 * Callers signal the shorted configuration by setting pin_ldac == pin_cs in
 * the dac80504_t struct. The driver then skips configuring the LDAC pin
 * separately (which would otherwise clobber the CS# direction). If a future
 * board rev gives LDAC# its own GPIO, set pin_ldac to that GPIO and
 * dac_latch_all() can be reworked to drive it — see the comment there.
 */

#include "dac80504.h"

#include <string.h>

/* R/nW bit lives in bit 23 of the 24-bit frame — i.e. bit 7 of byte 0. */
#define DAC_CMD_READ    (1u << 7)
#define DAC_CMD_WRITE   (0u << 7)

/* Pack the command byte: R/nW | don't-care[6:4] | addr[3:0]. */
static inline uint8_t dac_cmd_byte(uint8_t rw, uint8_t reg) {
    return (uint8_t)(rw | (reg & 0x0Fu));
}

static inline void dac_cs_low(dac80504_t *d) {
    gpio_put(d->pin_cs, 0);
}

static inline void dac_cs_high(dac80504_t *d) {
    gpio_put(d->pin_cs, 1);
}

void dac_spi_init(dac80504_t *d) {
    /* CS# GPIO: drive HIGH first, then configure as output, so we never
     * clock out a spurious frame at boot. Same pattern as ada4255.c. */
    gpio_init(d->pin_cs);
    gpio_put(d->pin_cs, 1);
    gpio_set_dir(d->pin_cs, GPIO_OUT);

    /* LDAC# GPIO: only configure if it's a distinct pin. When pin_ldac equals
     * pin_cs (the current MSIF bodge case, where LDAC# and CS# are shorted),
     * re-initing the same GPIO here would clobber the OUTPUT direction set
     * above. Leave it alone in that case — GPIO 42 is already driving both
     * the CS# and LDAC# pads simultaneously.
     * If a future rev wants independent GPIO control of LDAC#, point pin_ldac
     * at a different GPIO and this branch configures it high-Z input (safe
     * whether the hardware LDAC is pulled or bodged). Then rework
     * dac_latch_all() to drive it low for a "latch now" pulse. */
    if (d->pin_ldac != d->pin_cs) {
        gpio_init(d->pin_ldac);
        gpio_set_dir(d->pin_ldac, GPIO_IN);
    }

    /* SPI peripheral:
     *   - 8-bit frames (we send/read 3 bytes per DAC frame)
     *   - Mode 1: CPOL=0, CPHA=1 (required by DAC80504)
     *   - MSB first
     *
     * NOTE: this will clobber whatever mode the ADC driver configured.
     * Callers that share this SPI peripheral with MCP3462R must
     * reconfigure with spi_set_format() before ADC access. */
    spi_init(d->spi, d->baud_hz);
    spi_set_format(d->spi, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(d->pin_sck, GPIO_FUNC_SPI);
    gpio_set_function(d->pin_tx,  GPIO_FUNC_SPI);
    gpio_set_function(d->pin_rx,  GPIO_FUNC_SPI);

    /* Force all four channels into asynchronous update mode. Reset default
     * is already 0x0000 (async), but writing it explicitly is cheap insurance:
     * on this board LDAC# is tied to CS# (see file header), so any channel
     * accidentally left in synchronous mode would latch on CS rising edges
     * and never hold a stable buffered value. */
    dac_write_reg(d, DAC80504_REG_SYNC, 0x0000);
}

void dac_write_reg(dac80504_t *d, uint8_t reg, uint16_t val) {
    uint8_t tx[3] = {
        dac_cmd_byte(DAC_CMD_WRITE, reg),
        (uint8_t)(val >> 8),
        (uint8_t)(val & 0xFFu),
    };

    dac_cs_low(d);
    sleep_us(1);
    spi_write_blocking(d->spi, tx, 3);
    sleep_us(1);
    dac_cs_high(d);
}

uint16_t dac_read_reg(dac80504_t *d, uint8_t reg) {
    /* DAC80504 read protocol:
     *   Frame 1: {R/nW=1, addr, 0x00, 0x00} — request the register.
     *            SDO during this frame is the PREVIOUS read response; ignore.
     *   Frame 2: {NOP, 0x00, 0x00}          — clocks out requested register.
     *            SDO[15:0] contains the value we want.
     * CS# must be deasserted between the two frames.
     */
    uint8_t tx1[3] = {
        dac_cmd_byte(DAC_CMD_READ, reg),
        0x00u, 0x00u
    };
    uint8_t tx2[3] = {
        dac_cmd_byte(DAC_CMD_WRITE, DAC80504_REG_NOP),
        0x00u, 0x00u
    };
    uint8_t rx2[3] = { 0, 0, 0 };

    dac_cs_low(d);
    sleep_us(1);
    spi_write_blocking(d->spi, tx1, 3);
    sleep_us(1);
    dac_cs_high(d);

    sleep_us(1);

    dac_cs_low(d);
    sleep_us(1);
    spi_write_read_blocking(d->spi, tx2, rx2, 3);
    sleep_us(1);
    dac_cs_high(d);

    /* Response data is in the low 16 bits (bytes 1..2 of the return frame). */
    return (uint16_t)(((uint16_t)rx2[1] << 8) | (uint16_t)rx2[2]);
}

void dac_soft_reset(dac80504_t *d) {
    /* TRIGGER register, soft-reset field [3:0] = 0b1010.
     * Datasheet specifies < 500 us recovery; 1 ms is conservative. */
    dac_write_reg(d, DAC80504_REG_TRIGGER, DAC80504_TRIGGER_SOFT_RESET);
    sleep_ms(1);
    /* Reset wipes SYNC back to its default (async, 0x0000) — but re-assert
     * it explicitly so there's exactly one place in this driver that owns
     * the "all channels are async" invariant. Keeps the LDAC-tied-to-CS
     * contract intact after any soft reset. */
    dac_write_reg(d, DAC80504_REG_SYNC, 0x0000);
}

void dac_set_channel(dac80504_t *d, uint8_t ch, uint16_t code) {
    if (ch > DAC80504_CH_D) return;
    uint8_t reg = (uint8_t)(DAC80504_REG_DAC_A + ch);
    dac_write_reg(d, reg, code);
}

void dac_latch_all(dac80504_t *d) {
    /* On the current MSIF bodge, LDAC is not independently drivable because
     * it is tied to CS at the DAC package. We therefore treat software LDAC
     * as the only portable "latch now" mechanism for any future synchronous
     * use. In the async mode used by MSIF FMASS control, this write has no
     * effect because DAC_x updates already happen on CS rising edge. */
    dac_write_reg(d, DAC80504_REG_TRIGGER, DAC80504_TRIGGER_LDAC);
}

void dac_set_channel_v(dac80504_t *d, uint8_t ch, float v, float v_ref) {
    /* Clamp to [0, v_ref] — the DAC can't do negative and will saturate at
     * full scale regardless, but explicit clamping keeps the caller honest
     * and makes the 16-bit code math unambiguous. */
    if (v < 0.0f)   v = 0.0f;
    if (v > v_ref) v = v_ref;

    /* 16-bit unipolar: code = v / v_ref * 65535. Round to nearest. */
    float code_f = (v / v_ref) * 65535.0f + 0.5f;
    uint16_t code = (uint16_t)code_f;

    dac_set_channel(d, ch, code);
}
