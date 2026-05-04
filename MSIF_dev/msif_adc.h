/*
 * msif_adc.h — board-level wrapper for the MCP3462RT EC-voltage ADC.
 *
 * IMPORTANT: this ADC reads a VOLTAGE, not a current. The QMS-112's internal
 * electrometer (manual sec 9.2.3) does the I->V conversion: ions hit the
 * collector, the high-impedance input-stage resistors do current-to-voltage,
 * the post-amp + RANGE + GAIN scale that to a 0..10 V differential signal at
 * the QDP EC+/EC- pins. By the time the signal reaches MSIF, the unit is V.
 *
 * To recover ion current in amps from v_ec, multiply by the per-RANGE/GAIN
 * transfer factor in QMS-112 manual sec 10.2.1.1 / sec 9.1.2.3 (e.g. RANGE=
 * 10^-7, GAIN=x100 -> 10 V FS = 10^-7 A FS, so 1 V at EC = 10^-8 A). That
 * conversion table is NOT yet codified in firmware — every CSV column and
 * every peak-summary "area" reported is in volt-units, not ampere-units.
 *
 * Signal path on the MSIF board:
 *   QDP EC- -> U19.5 (OPA4197 unity follower) -> INA146 difference amp
 *           -> MCP3462RT differential input.
 *
 * Usage from main.c:
 *   msif_adc_init();                  // after msif_analog_init()
 *   msif_adc_sample_t s;
 *   msif_adc_read_ec(&s);             // one-shot conversion
 *
 * SPI bus is shared with the DAC80504. msif_adc_* public functions re-assert
 * SPI mode 0 before every transaction; msif_analog_* does the same with
 * mode 1. Order of calls between the two doesn't matter at the application
 * layer — the wrappers take care of it.
 *
 * Open items to resolve at the bench:
 *   - which MUX pair is the actual EC- path (assumed CH0+/CH1-)
 *   - INA146 + input-divider gain (MSIF_ADC_INPUT_GAIN in MSIF_cfg.h)
 *   - per-RANGE/GAIN V_EC -> ion-current (amps) transfer table — needs the
 *     QMS-112 manual sec 10.2.1.1 / 9.1.2.3 cross-product table baked in,
 *     or a known-current calibration source for the live system
 */

#ifndef MSIF_ADC_H
#define MSIF_ADC_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t raw_code;     /* signed 16-bit ADC code (differential two's-comp) */
    float   v_adc_diff;   /* differential volts at MCP3462RT VIN+/VIN- */
    float   v_ec;         /* back-projected volts at QDP EC- (after input gain) */
    uint8_t mcp_status;   /* MCP status byte captured during the read */
} msif_adc_sample_t;

typedef struct {
    uint8_t mcp_status;   /* status from the register read */
    uint8_t config[4];    /* CFG0..CFG3 */
    uint8_t irq_reg;      /* IRQ register */
    uint8_t mux_reg;      /* MUX register */
} msif_adc_snapshot_t;

/* Initialise the ADC. Must be called after msif_analog_init(): the DAC init
 * configures the shared SPI peripheral first, and this call layers the
 * ADC-specific mode/config on top. */
void msif_adc_init(void);

/* Trigger a one-shot differential conversion on the configured channel pair,
 * block until DR_STATUS clears, and fill *sample with the raw code and the
 * computed voltages. Returns false if the ADC hasn't been initialised or
 * the sample pointer is NULL. */
bool msif_adc_read_ec(msif_adc_sample_t *sample);

/* Average n_samples one-shot conversions into a single result. raw_code,
 * v_adc_diff, and v_ec are means; mcp_status is the bitwise OR of every
 * sample's status byte (so any error seen across the batch is visible).
 * Returns false if the ADC isn't initialised, sample is NULL, or n_samples
 * is 0. Use when a single reading is too noisy — at OSR=256 each conversion
 * is ~50 µs, so 32 samples adds ~1.6 ms and ~2.5 bits of effective resolution. */
bool msif_adc_read_ec_avg(uint32_t n_samples, msif_adc_sample_t *sample);

/* Read back the ADC's CONFIG0..3, IRQ, and MUX registers for bench
 * diagnostics. Returns false if not initialised. */
bool msif_adc_snapshot(msif_adc_snapshot_t *snap);

#endif /* MSIF_ADC_H */
