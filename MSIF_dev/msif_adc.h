/*
 * msif_adc.h — board-level wrapper for the MCP3462RT ion-current ADC.
 *
 * The QMS electrometer signal enters MSIF at QDP pin `EC-`, gets buffered by
 * U19.5 (OPA4197, unity follower), runs through an INA146 difference-amp
 * stage (U16/U17), and lands at the MCP3462RT's differential inputs. This
 * module hides the mcp3x6xR driver and the input-network gain from the rest
 * of the app.
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
 *   - V_EC -> ion current conversion per QMS RANGE setting (needs service
 *     manual or a known-current calibration source)
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

/* Read back the ADC's CONFIG0..3, IRQ, and MUX registers for bench
 * diagnostics. Returns false if not initialised. */
bool msif_adc_snapshot(msif_adc_snapshot_t *snap);

#endif /* MSIF_ADC_H */
