/*
 * msif_analog.h — board-level wrapper for the MSIF analog output stage.
 *
 * The MSIF analog section drives three QMS-112 command voltages through
 * OPA4197 gain stages. DAC80504 OUT0 → FMASS+, OUT1/2/3 → SWIDTH/SEM/TBD.
 * This module hides the DAC driver, the op-amp gain correction, and the
 * bench-measured voltage↔mass calibration from the rest of the app.
 *
 * Use from main.c:
 *   msif_analog_init();          // in setup, after msif_gpio_init()
 *   msif_set_fmass(28.0f);       // first mass in AMU (CO/N2 ≈ 28)
 *
 * Calibration constants live in MSIF_cfg.h; they come from the Phase H
 * bench-PSU sweep (see sleepy-swinging-volcano.md).
 */

#ifndef MSIF_ANALOG_H
#define MSIF_ANALOG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t devid;
    uint16_t sync;
    uint16_t config;
    uint16_t gain;
    uint16_t status;
    uint16_t dac[4];
} msif_dac_snapshot_t;

/* Initialise SPI + DAC + soft-reset. Must be called after stdio/GPIO init. */
void msif_analog_init(void);

/* Returns true once the FMASS mass->voltage constants in MSIF_cfg.h are
 * populated with a non-zero slope from the bench calibration sweep. */
bool msif_fmass_is_calibrated(void);

/* Set the QMS first-mass target in AMU. Applies bench calibration and the
 * op-amp gain correction, then writes the DAC code. Returns the computed
 * V_QDP target so the caller can sanity-check against a DMM reading. */
float msif_set_fmass(float mass_amu);

/* Drive FMASS+ directly in QDP-pin volts. Useful before Phase H calibration:
 * it bypasses the mass conversion and lets the bench DMM validate the analog
 * path end-to-end. Returns the clamped QDP target actually commanded. */
float msif_set_fmass_v(float v_qdp);

/* Set the QDP SWIDTH (scan width) voltage directly in volts. Channel B. */
float msif_set_swidth_v(float v_qdp);

/* Set the QDP SEM (secondary electron multiplier) voltage directly in volts.
 * Channel C. TBD whether this is actually the right channel — verify from
 * the MSIF Analog schematic before trusting this at the bench. */
float msif_set_sem_v(float v_qdp);

/* Read back key DAC registers over SDO for bench diagnostics. Returns false if
 * the analog layer has not been initialised yet. */
bool msif_dac_snapshot(msif_dac_snapshot_t *snapshot);

/* SPI loopback test: write PATTERN to the DAC_A register, read it back over
 * SDO, store the readback in *readback, and restore DAC_A to 0x0000 before
 * returning. Proves the full MOSI→register→MISO path works even when the DAC
 * analog output can't be confirmed with a DMM (e.g. on USB-only power with
 * 5V_IF missing). Returns false if the analog layer is not initialised or
 * readback is NULL. */
bool msif_dac_loopback(uint16_t pattern, uint16_t *readback);

#endif /* MSIF_ANALOG_H */
