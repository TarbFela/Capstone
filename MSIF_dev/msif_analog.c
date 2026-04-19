/*
 * msif_analog.c — board-level wrapper for DAC80504 + OPA4197 analog outputs.
 *
 * SPI BUS SHARING: the DAC80504 (U15) and MCP3462RT ADC (U18) share the SPI
 * peripheral, SCK, SDI, and SDO lines. Only the CS# lines are separate.
 * DAC80504 requires SPI mode 1 (CPOL=0, CPHA=1). MCP3462R requires mode 0
 * or mode 3. These modes are INCOMPATIBLE on the same bus — every switch
 * between DAC and ADC access requires calling spi_set_format() with the
 * destination device's mode. This module re-applies the DAC mode inside
 * msif_analog_init() and assumes no ADC traffic happens concurrently.
 * When the ADC driver lands, wrap every msif_set_* call in a "claim the
 * SPI for the DAC" helper that sets the format, or use a mutex.
 *
 * SIGNAL CHAIN (FMASS+):
 *   DAC OUT0 (0..V_REF) -> R52 -> (+) input node -> OPA4197 gain stage
 *                                   ^
 *                                   |
 *                           R56 to FMASS-, which is tied to SGND_IF through
 *                           R77 = 0 ohm on this board rev
 *
 *   The 4.99k/4.99k divider ahead of the op-amp halves the DAC voltage at
 *   the non-inverting input, then the OPA4197 multiplies by about 8, so the
 *   net board-level transfer is approximately:
 *
 *   V_QDP    = AMP_GAIN × V_DAC   where AMP_GAIN ≈ 4 V/V on this schematic
 *   V_DAC    = V_QDP / AMP_GAIN           (what we program into the DAC)
 *   V_QDP(m) = SLOPE × mass_amu + OFFSET   (bench-measured Phase H)
 *
 * Until Phase H is run, SLOPE and OFFSET are zero and msif_set_fmass will
 * command V_DAC = 0 regardless of input. That's safe (DAC at rail-low) but
 * also useless for mass selection — don't expect anything until the sweep
 * constants are filled in.
 */

#include "msif_analog.h"

#include <stdio.h>

#include "hardware/gpio.h"

#include "MSIF_cfg.h"
#include "src2/dac80504_driver/dac80504.h"

/* File-static driver state. One DAC on the MSIF board, one state struct. */
static dac80504_t s_dac;
static bool s_analog_init_done = false;

static float msif_write_qdp_v(uint8_t ch, float v_qdp);
static float msif_fmass_mass_to_v(float mass_amu);

void msif_analog_init(void) {
    /* Park the shared-bus ADC CS# (MCP3462RT / U18) HIGH before we touch the
     * SPI peripheral. There is no ADC driver yet, but GPIO 37's reset state
     * is an input with no pull, and a floating gate could transiently assert
     * ADC CS# low while the DAC is mid-frame — corrupting MISO and spoofing
     * the DAC readback. Owning it here as an output HIGH makes that
     * impossible. When the ADC driver ships, move this to the ADC init and
     * drop it from here. */
    gpio_init(MSIF_ADC_CS_PIN);
    gpio_put(MSIF_ADC_CS_PIN, 1);
    gpio_set_dir(MSIF_ADC_CS_PIN, GPIO_OUT);

    s_dac.spi      = MSIF_DAC_SPI;
    s_dac.pin_cs   = MSIF_DAC_CS_PIN;
    s_dac.pin_ldac = MSIF_DAC_LDAC_PIN;
    s_dac.pin_sck  = MSIF_DAC_SCK_PIN;
    s_dac.pin_tx   = MSIF_DAC_SDI_PIN;
    s_dac.pin_rx   = MSIF_DAC_SDO_PIN;
    s_dac.baud_hz  = MSIF_DAC_BAUD_HZ;

    dac_spi_init(&s_dac);
    dac_soft_reset(&s_dac);

    /* After reset, all four channels are at 0x0000 (output = 0 V at DAC pin)
     * and gain is 1×. V_REF is the internal 2.5 V reference by default.
     * That matches MSIF_DAC_V_REF in the config — don't change it here.
     *
     * On the bodged MSIF board, DAC LDAC is physically tied to CS. Make the
     * intended operating mode explicit by keeping every channel in the DAC's
     * default asynchronous-update mode (SYNC bits = 0), so writes still take
     * effect on CS rising edge and we do not depend on a separate LDAC pin. */
    dac_write_reg(&s_dac, DAC80504_REG_SYNC, 0x0000u);
    s_analog_init_done = true;
}

/* Internal helper: clamp to the realizable QDP range, apply amp gain
 * correction, and program the DAC. */
static float msif_write_qdp_v(uint8_t ch, float v_qdp) {
    float v_qdp_max = MSIF_DAC_V_REF * MSIF_AMP_GAIN;
    if (v_qdp < 0.0f) {
        v_qdp = 0.0f;
    }
    if (v_qdp > v_qdp_max) {
        v_qdp = v_qdp_max;
    }

    float v_dac = v_qdp / MSIF_AMP_GAIN;
    dac_set_channel_v(&s_dac, ch, v_dac, MSIF_DAC_V_REF);
    return v_qdp;
}

static float msif_fmass_mass_to_v(float mass_amu) {
    return MSIF_FMASS_CAL_SLOPE_V_PER_AMU * mass_amu
         + MSIF_FMASS_CAL_OFFSET_V;
}

bool msif_fmass_is_calibrated(void) {
    return MSIF_FMASS_CAL_SLOPE_V_PER_AMU != 0.0f;
}

float msif_set_fmass(float mass_amu) {
    return msif_write_qdp_v(DAC80504_CH_A, msif_fmass_mass_to_v(mass_amu));
}

float msif_set_fmass_v(float v_qdp) {
    return msif_write_qdp_v(DAC80504_CH_A, v_qdp);
}

float msif_set_swidth_v(float v_qdp) {
    return msif_write_qdp_v(DAC80504_CH_B, v_qdp);
}

float msif_set_sem_v(float v_qdp) {
    return msif_write_qdp_v(DAC80504_CH_C, v_qdp);
}

bool msif_dac_loopback(uint16_t pattern, uint16_t *readback) {
    /* SPI-only sanity check: write PATTERN to DAC_A, then immediately read the
     * DAC_A register back through MISO. Useful when the DAC's analog output
     * can't be trusted (e.g. 5V_IF rail missing on USB-only power) — a
     * correct readback still proves every electrical step of the SPI bus
     * (GPIO 42 CS#, SCK, MOSI, MISO, and the bodge wire) is intact. Restores
     * DAC_A to 0x0000 on exit so subsequent analog work starts from a known
     * zero volts. */
    if (!s_analog_init_done || readback == NULL) {
        return false;
    }

    dac_write_reg(&s_dac, DAC80504_REG_DAC_A, pattern);
    *readback = dac_read_reg(&s_dac, DAC80504_REG_DAC_A);
    dac_write_reg(&s_dac, DAC80504_REG_DAC_A, 0x0000u);
    return true;
}

bool msif_dac_snapshot(msif_dac_snapshot_t *snapshot) {
    if (!s_analog_init_done || snapshot == NULL) {
        return false;
    }

    snapshot->devid  = dac_read_reg(&s_dac, DAC80504_REG_DEVID);
    snapshot->sync   = dac_read_reg(&s_dac, DAC80504_REG_SYNC);
    snapshot->config = dac_read_reg(&s_dac, DAC80504_REG_CONFIG);
    snapshot->gain   = dac_read_reg(&s_dac, DAC80504_REG_GAIN);
    snapshot->status = dac_read_reg(&s_dac, DAC80504_REG_STATUS);
    snapshot->dac[0] = dac_read_reg(&s_dac, DAC80504_REG_DAC_A);
    snapshot->dac[1] = dac_read_reg(&s_dac, DAC80504_REG_DAC_B);
    snapshot->dac[2] = dac_read_reg(&s_dac, DAC80504_REG_DAC_C);
    snapshot->dac[3] = dac_read_reg(&s_dac, DAC80504_REG_DAC_D);
    return true;
}
