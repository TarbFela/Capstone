/*
 * msif_peak.c — peak-integration primitives. See msif_peak.h.
 *
 * Trapezoidal area + discrete weighted-mean centroid, shared between the
 * mass-domain sweep and the time-domain park. Both modes:
 *   1) check args (no DAC writes on bad args)
 *   2) print the calibration-status banner so every invocation reminds the
 *      operator whether the slope is bench-verified or QMS-spec default
 *   3) sweep/park, accumulating into a single msif_peak_acc_t
 *   4) emit a "# PEAK_*_SUMMARY" line and park FMASS at 0 V on every exit
 *
 * CSV columns track the existing 'F'/'P' commands in main.c so downstream
 * MATLAB/python parsers don't need a special case (sweep adds mass_amu as
 * the third column; everything else matches).
 */
#include "msif_peak.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "MSIF_cfg.h"
#include "msif_analog.h"
#include "msif_adc.h"

/* ------------------------------------------------------------------ */
/* Trapezoidal accumulator. Used by both sweep (x = mass_amu) and park */
/* (x = t_s). Area is true trapezoidal sum of y dx; centroid is the    */
/* discrete intensity-weighted mean Sum(x*y) / Sum(y) — appropriate    */
/* for histogram-style centroiding when sampling is fine enough.       */
/* ------------------------------------------------------------------ */
typedef struct {
    bool     first;
    float    prev_x;
    float    prev_y;
    float    area;          /* trapezoidal Sum y dx */
    float    sum_xy;        /* Sum(x*y) for weighted-mean centroid numerator */
    float    sum_y;         /* Sum(y)   for centroid denominator and park mean */
    float    max_y;
    float    x_at_max;
    uint32_t n;
} msif_peak_acc_t;

static inline void msif_peak_acc_init(msif_peak_acc_t *a) {
    a->first = true;
    a->prev_x = 0.0f;
    a->prev_y = 0.0f;
    a->area = 0.0f;
    a->sum_xy = 0.0f;
    a->sum_y = 0.0f;
    a->max_y = 0.0f;
    a->x_at_max = 0.0f;
    a->n = 0u;
}

static inline void msif_peak_acc_push(msif_peak_acc_t *a, float x, float y) {
    if (a->first) {
        a->first = false;
        a->max_y = y;
        a->x_at_max = x;
    } else {
        a->area += 0.5f * (a->prev_y + y) * (x - a->prev_x);
        if (y > a->max_y) {
            a->max_y = y;
            a->x_at_max = x;
        }
    }
    a->sum_xy += x * y;
    a->sum_y  += y;
    a->prev_x = x;
    a->prev_y = y;
    a->n++;
}

/* ------------------------------------------------------------------ */
/* Calibration banner / resolver                                       */
/* ------------------------------------------------------------------ */

bool msif_peak_print_cal_status(void) {
    /* Calibration is whatever's stored in MSIF_cfg.h. Defaults are the
     * QMS-112 spec values (10V/range, 0V); update after bench calibration. */
    printf("# CAL_STATUS: slope=%.6f V/AMU offset=%.6f V\n",
           (double)MSIF_FMASS_CAL_SLOPE_V_PER_AMU,
           (double)MSIF_FMASS_CAL_OFFSET_V);
    return false;
}

/* Resolve the slope/offset to use for AMU->QDP-volts conversion. Always
 * succeeds — slope/offset come straight from MSIF_cfg.h. */
static bool msif_peak_resolve_cal(float *slope_out, float *offset_out) {
    *slope_out  = MSIF_FMASS_CAL_SLOPE_V_PER_AMU;
    *offset_out = MSIF_FMASS_CAL_OFFSET_V;
    return true;
}

/* ------------------------------------------------------------------ */
/* Mass-domain sweep                                                   */
/* ------------------------------------------------------------------ */
msif_peak_status_t msif_peak_sweep_mass(float mass_start_amu,
                                        float mass_end_amu,
                                        uint32_t n_steps,
                                        msif_peak_result_t *out) {
    if (out == NULL) return MSIF_PEAK_ERR_ARGS;
    memset(out, 0, sizeof(*out));
    out->start = mass_start_amu;
    out->end   = mass_end_amu;

    if (n_steps < MSIF_PEAK_SWEEP_MIN_STEPS ||
        n_steps > MSIF_PEAK_SWEEP_MAX_STEPS ||
        !(mass_end_amu > mass_start_amu)) {
        printf("# PEAK_SWEEP: bad args (steps=%u start=%.4f end=%.4f, "
               "need %u..%u steps and end>start)\n",
               (unsigned)n_steps,
               (double)mass_start_amu, (double)mass_end_amu,
               (unsigned)MSIF_PEAK_SWEEP_MIN_STEPS,
               (unsigned)MSIF_PEAK_SWEEP_MAX_STEPS);
        return MSIF_PEAK_ERR_ARGS;
    }

    msif_peak_print_cal_status();

    float slope = 0.0f, offset = 0.0f;
    (void)msif_peak_resolve_cal(&slope, &offset);

    printf("# PEAK_SWEEP mass_start=%.4f mass_end=%.4f steps=%u "
           "samples_per_pt=%u settle_ms=%u slope=%.6f V/AMU offset=%.6f V\n",
           (double)mass_start_amu, (double)mass_end_amu, (unsigned)n_steps,
           (unsigned)MSIF_ADC_AVG_SAMPLES,
           (unsigned)MSIF_FMASS_SETTLE_MS,
           (double)slope, (double)offset);
    printf("step,t_ms,mass_amu,v_fmass,ec_code,v_adc_diff,v_ec,status\n");

    msif_peak_acc_t acc;
    msif_peak_acc_init(&acc);

    msif_peak_status_t status = MSIF_PEAK_OK;
    const float dm = (mass_end_amu - mass_start_amu) / (float)(n_steps - 1u);
    const uint64_t t0_us = time_us_64();
    uint8_t status_or = 0;

    for (uint32_t i = 0; i < n_steps; i++) {
        if (stdio_getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
            status = MSIF_PEAK_ERR_ABORTED;
            break;
        }
        float m = mass_start_amu + dm * (float)i;
        float v = slope * m + offset;
        msif_set_fmass_v(v);
        sleep_ms(MSIF_FMASS_SETTLE_MS);

        msif_adc_sample_t s;
        if (!msif_adc_read_ec_avg(MSIF_ADC_AVG_SAMPLES, &s)) {
            printf("# PEAK_SWEEP: ADC read failed at step %u\n", (unsigned)i);
            status = MSIF_PEAK_ERR_ADC;
            break;
        }
        uint32_t t_ms = (uint32_t)((time_us_64() - t0_us) / 1000u);
        printf("%u,%lu,%.4f,%.4f,%d,%.6f,%.6f,0x%02X\n",
               (unsigned)i, (unsigned long)t_ms,
               (double)m, (double)v,
               s.raw_code, (double)s.v_adc_diff, (double)s.v_ec,
               s.mcp_status);

        msif_peak_acc_push(&acc, m, s.v_ec);
        status_or |= s.mcp_status;
    }

    msif_set_fmass_v(0.0f);
    uint32_t total_ms = (uint32_t)((time_us_64() - t0_us) / 1000u);

    out->n_samples     = acc.n;
    out->area          = acc.area;
    out->centroid      = (acc.sum_y != 0.0f) ? (acc.sum_xy / acc.sum_y) : 0.0f;
    out->peak_max_v_ec = acc.max_y;
    out->peak_at       = acc.x_at_max;
    out->dwell_ms      = total_ms;
    out->mcp_status_or = status_or;

    const char *prefix = (status == MSIF_PEAK_OK) ? "" : "PARTIAL ";
    printf("# PEAK_SWEEP_SUMMARY %sstatus=%d area_V_AMU=%.6e centroid_amu=%.4f "
           "peak_max_v_ec=%.6e peak_at_amu=%.4f n_samples=%u dwell_ms=%lu "
           "mcp_status_or=0x%02X\n",
           prefix, (int)status,
           (double)out->area, (double)out->centroid,
           (double)out->peak_max_v_ec, (double)out->peak_at,
           (unsigned)out->n_samples, (unsigned long)out->dwell_ms,
           (unsigned)out->mcp_status_or);
    printf("# PEAK_SWEEP %s, FMASS parked at 0 V\n",
           (status == MSIF_PEAK_OK) ? "complete" : "stopped");
    return status;
}

/* ------------------------------------------------------------------ */
/* Time-domain park                                                    */
/* ------------------------------------------------------------------ */
msif_peak_status_t msif_peak_park_time(float v_qdp_park,
                                       uint32_t duration_ms,
                                       msif_peak_result_t *out) {
    if (out == NULL) return MSIF_PEAK_ERR_ARGS;
    memset(out, 0, sizeof(*out));
    out->start = 0.0f;
    out->end   = (float)duration_ms / 1000.0f;

    if (duration_ms == 0u || duration_ms > MSIF_PEAK_PARK_MAX_MS) {
        printf("# PEAK_PARK: bad duration_ms=%lu (must be 1..%lu)\n",
               (unsigned long)duration_ms,
               (unsigned long)MSIF_PEAK_PARK_MAX_MS);
        return MSIF_PEAK_ERR_ARGS;
    }

    msif_peak_print_cal_status();

    msif_set_fmass_v(v_qdp_park);
    sleep_ms(MSIF_FMASS_SETTLE_MS);

    printf("# PEAK_PARK v_fmass=%.4f duration_ms=%lu period_ms=%u "
           "samples_per_pt=%u\n",
           (double)v_qdp_park, (unsigned long)duration_ms,
           (unsigned)MSIF_ADC_PARK_PERIOD_MS,
           (unsigned)MSIF_ADC_AVG_SAMPLES);
    printf("t_ms,t_s,ec_code,v_adc_diff,v_ec,status\n");

    msif_peak_acc_t acc;
    msif_peak_acc_init(&acc);

    msif_peak_status_t status = MSIF_PEAK_OK;
    const uint64_t t0_us = time_us_64();
    uint8_t status_or = 0;

    while (1) {
        if (stdio_getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
            status = MSIF_PEAK_ERR_ABORTED;
            break;
        }
        uint32_t t_ms = (uint32_t)((time_us_64() - t0_us) / 1000u);
        if (t_ms >= duration_ms) break;

        msif_adc_sample_t s;
        if (!msif_adc_read_ec_avg(MSIF_ADC_AVG_SAMPLES, &s)) {
            printf("# PEAK_PARK: ADC read failed at t=%lu\n",
                   (unsigned long)t_ms);
            status = MSIF_PEAK_ERR_ADC;
            break;
        }
        float t_s = (float)t_ms / 1000.0f;
        printf("%lu,%.4f,%d,%.6f,%.6f,0x%02X\n",
               (unsigned long)t_ms, (double)t_s,
               s.raw_code, (double)s.v_adc_diff, (double)s.v_ec,
               s.mcp_status);

        msif_peak_acc_push(&acc, t_s, s.v_ec);
        status_or |= s.mcp_status;
        sleep_ms(MSIF_ADC_PARK_PERIOD_MS);
    }

    msif_set_fmass_v(0.0f);
    uint32_t total_ms = (uint32_t)((time_us_64() - t0_us) / 1000u);

    out->n_samples     = acc.n;
    out->area          = acc.area;
    /* Park centroid is the simple mean of v_ec — by design (minimal scope).
     * A weighted-mean time would just be midpoint-ish for a stationary park
     * and isn't useful here. */
    out->centroid      = (acc.n > 0u) ? (acc.sum_y / (float)acc.n) : 0.0f;
    out->peak_max_v_ec = acc.max_y;
    out->peak_at       = acc.x_at_max;
    out->dwell_ms      = total_ms;
    out->mcp_status_or = status_or;

    const char *prefix = (status == MSIF_PEAK_OK) ? "" : "PARTIAL ";
    printf("# PEAK_PARK_SUMMARY %sstatus=%d area_V_s=%.6e mean_v_ec=%.6e "
           "peak_max_v_ec=%.6e peak_at_s=%.4f n_samples=%u dwell_ms=%lu "
           "mcp_status_or=0x%02X\n",
           prefix, (int)status,
           (double)out->area, (double)out->centroid,
           (double)out->peak_max_v_ec, (double)out->peak_at,
           (unsigned)out->n_samples, (unsigned long)out->dwell_ms,
           (unsigned)out->mcp_status_or);
    printf("# PEAK_PARK %s, FMASS parked at 0 V\n",
           (status == MSIF_PEAK_OK) ? "complete" : "stopped");
    return status;
}
