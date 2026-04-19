/*
 * dac80504.h — TI DAC80504 quad 16-bit SPI DAC driver
 *
 * DAC80504 overview:
 *   - 4 channels (A–D), 16-bit, buffered output
 *   - SPI: 24-bit frames, Mode 1 (CPOL=0, CPHA=1) or Mode 2 (CPOL=1, CPHA=0)
 *   - Frame format: [23]=R/nW, [22:20]=don't-care, [19:16]=addr, [15:0]=data
 *   - CS# active-low; LDAC# active-low pulse to commit buffered values
 *   - Internal 2.5V reference; GAIN register or hardware GAIN pin selects 1x/2x
 *
 * NOTE on SPI bus sharing with MCP3462RT (MSIF ADC):
 *   The DAC and ADC share SCK/SDI/SDO lines on the MSIF board.
 *   DAC80504 requires SPI Mode 1 (CPOL=0, CPHA=1).
 *   MCP3462R requires SPI Mode 0 (CPOL=0, CPHA=0) or Mode 3 (CPOL=1, CPHA=1).
 *   These modes are INCOMPATIBLE on the same bus. When the ADC driver is added,
 *   the SPI peripheral must be reconfigured (spi_set_format) when switching
 *   between DAC and ADC access. CS must be deasserted on both devices during
 *   any mode switch.
 *
 * Reference: TI DAC80504 datasheet (SBAS612B)
 */

#ifndef DAC80504_H
#define DAC80504_H

#include <stdint.h>
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

/* ---- Register addresses ---- */
#define DAC80504_REG_NOP        0x00
#define DAC80504_REG_DEVID      0x01  /* read-only: device ID */
#define DAC80504_REG_SYNC       0x02  /* synchronous update enable */
#define DAC80504_REG_CONFIG     0x03  /* power-down / ref config */
#define DAC80504_REG_GAIN       0x04  /* channel gain (1x or 2x per channel) */
#define DAC80504_REG_TRIGGER    0x05  /* soft reset, LDAC software trigger */
#define DAC80504_REG_BRDCAST    0x06  /* broadcast write to all four channels */
#define DAC80504_REG_STATUS     0x07  /* alarm flags (read-only) */
#define DAC80504_REG_DAC_A      0x08  /* channel A (OUT0) output register */
#define DAC80504_REG_DAC_B      0x09  /* channel B (OUT1) */
#define DAC80504_REG_DAC_C      0x0A  /* channel C (OUT2) */
#define DAC80504_REG_DAC_D      0x0B  /* channel D (OUT3) */

/* ---- TRIGGER register bits ---- */
/* Soft-reset field occupies bits [3:0]. Writing 0b1010 (0x0A) triggers a reset;
 * any other value is ignored. See DAC80504 datasheet table "TRIGGER Register". */
#define DAC80504_TRIGGER_SOFT_RESET  0x000Au
#define DAC80504_TRIGGER_LDAC        (1u << 4)  /* bit 4: software LDAC trigger */

/* ---- Channel index ---- */
#define DAC80504_CH_A  0
#define DAC80504_CH_B  1
#define DAC80504_CH_C  2
#define DAC80504_CH_D  3

/* ---- Driver state ---- */
typedef struct {
    spi_inst_t *spi;
    uint        pin_cs;     /* chip select, active-low GPIO */
    uint        pin_ldac;   /* latch enable, active-low GPIO */
    uint        pin_sck;
    uint        pin_tx;     /* MOSI (SDI on DAC) */
    uint        pin_rx;     /* MISO (SDO on DAC) */
    uint32_t    baud_hz;
} dac80504_t;

/* ---- API ---- */

/*
 * dac_spi_init() — configure SPI peripheral and GPIO pins for the DAC.
 * Call this before any other dac_ function.
 * CS# and LDAC# are driven HIGH (inactive) immediately on entry.
 */
void dac_spi_init(dac80504_t *d);

/*
 * dac_soft_reset() — send software reset sequence, wait 1 ms for recovery.
 * Resets all registers to defaults (all channels output 0, gain 1x).
 */
void dac_soft_reset(dac80504_t *d);

/*
 * dac_write_reg() — write a 16-bit value to a DAC register.
 */
void dac_write_reg(dac80504_t *d, uint8_t reg, uint16_t val);

/*
 * dac_read_reg() — read a 16-bit value from a DAC register.
 */
uint16_t dac_read_reg(dac80504_t *d, uint8_t reg);

/*
 * dac_set_channel() — write a raw 16-bit code to one channel buffer.
 * The output does NOT update immediately; call dac_latch_all() after setting
 * all desired channels to update all outputs simultaneously (glitch-free).
 */
void dac_set_channel(dac80504_t *d, uint8_t ch, uint16_t code);

/*
 * dac_latch_all() — pulse LDAC# low to commit all buffered channel values
 * to the DAC outputs simultaneously.
 */
void dac_latch_all(dac80504_t *d);

/*
 * dac_set_channel_v() — set one channel to a target output voltage.
 * v_ref is the DAC full-scale reference (e.g. MSIF_DAC_V_REF from MSIF_cfg.h).
 * Clamps to [0, v_ref]. Does NOT call dac_latch_all() — caller decides timing.
 *
 * NOTE: v is the voltage at the DAC output pin, NOT at the QDP connector.
 * Apply op-amp gain correction in the board-level layer (msif_analog.c).
 */
void dac_set_channel_v(dac80504_t *d, uint8_t ch, float v, float v_ref);

#endif /* DAC80504_H */
