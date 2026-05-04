/*
 * msif_peak.h — peak-integration primitives for the MSIF firmware.
 *
 * Two modes share the same trapezoidal accumulator:
 *
 *   msif_peak_sweep_mass()   step FMASS across a mass window in N steps,
 *                            settle, average, integrate v_ec(mass) d(mass).
 *                            Reports area in V*AMU and intensity-weighted
 *                            centroid in AMU.
 *   msif_peak_park_time()    hold FMASS at a fixed QDP voltage, sample
 *                            v_ec at MSIF_ADC_PARK_PERIOD_MS cadence for
 *                            duration_ms. Reports area in V*s and mean
 *                            v_ec.
 *
 * Both reuse:
 *   - msif_set_fmass_v()   for DAC writes (raw QDP volts; sweep computes
 *                          AMU->V locally so the stored calibration constants
 *                          are never silently overridden);
 *   - msif_adc_read_ec_avg(MSIF_ADC_AVG_SAMPLES, ...)   for every sample;
 *   - MSIF_FMASS_SETTLE_MS, MSIF_ADC_PARK_PERIOD_MS     for timing.
 *
 * Both functions park FMASS at 0 V on every exit path (success, ADC error,
 * user keypress abort).
 *
 * Calibration: until MSIF_FMASS_CAL_BENCH_VERIFIED is flipped to 1 in
 * MSIF_cfg.h, msif_peak_print_cal_status() prints a loud banner reminding
 * the operator that the slope is a QMS-112 spec default
 * (10 V / MSIF_QMS_MASS_RANGE), not a bench-measured value. Sweep falls
 * back to the spec default if the stored slope is 0.
 */
#ifndef MSIF_PEAK_H
#define MSIF_PEAK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MSIF_PEAK_OK = 0,
    MSIF_PEAK_ERR_ARGS,        /* bad n_steps / duration / window direction */
    MSIF_PEAK_ERR_CAL_ZERO,    /* bench-verified flag set but slope is 0 */
    MSIF_PEAK_ERR_ADC,         /* msif_adc_read_ec_avg() returned false */
    MSIF_PEAK_ERR_ABORTED      /* user keypress mid-run */
} msif_peak_status_t;

typedef struct {
    float    start;            /* AMU (sweep) or 0.0 s (park) */
    float    end;              /* AMU (sweep) or duration_s (park) */
    uint32_t n_samples;        /* may be < requested on abort/error */
    float    area;             /* V*AMU (sweep) or V*s (park) */
    float    centroid;         /* AMU (sweep) or mean v_ec V (park) */
    float    peak_max_v_ec;
    float    peak_at;          /* AMU (sweep) or t_s (park) at peak max */
    uint32_t dwell_ms;         /* total wall time inside the loop */
    uint8_t  mcp_status_or;    /* OR of all MCP status bytes */
} msif_peak_result_t;

/* Mass-domain peak integration. n_steps must satisfy
 * MSIF_PEAK_SWEEP_MIN_STEPS <= n_steps <= MSIF_PEAK_SWEEP_MAX_STEPS, and
 * mass_end_amu must be strictly greater than mass_start_amu. Out-of-band
 * args return MSIF_PEAK_ERR_ARGS without moving the DAC. The DAC is parked
 * at 0 V on every exit path. */
msif_peak_status_t msif_peak_sweep_mass(float mass_start_amu,
                                        float mass_end_amu,
                                        uint32_t n_steps,
                                        msif_peak_result_t *out);

/* Time-domain peak integration. duration_ms must be 1..MSIF_PEAK_PARK_MAX_MS.
 * The DAC is parked at 0 V on every exit path. */
msif_peak_status_t msif_peak_park_time(float v_qdp_park,
                                       uint32_t duration_ms,
                                       msif_peak_result_t *out);

/* Print a calibration-status banner (single line if bench-verified, multi-line
 * loud warning otherwise). Returns true iff the calibration is unverified. */
bool msif_peak_print_cal_status(void);

#endif /* MSIF_PEAK_H */
