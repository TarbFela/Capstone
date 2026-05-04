/*
 * msif_qms.c — set-by-value wrappers for the QMS-112 remote-control surface.
 * See msif_qms.h for encoding tables and the manual sections each is from.
 */
#include "msif_qms.h"

#include <string.h>
#include <strings.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "MSIF_cfg.h"
#include "msif_analog.h"
#include "msif_gpio.h"

/* Pin tables and write_field live in msif_gpio.{h,c} as the canonical copy. */

/* Filter time constants (us) per QMS-112 manual sec 10.2.2.3. Indexed by
 * msif_qms_speed_t. Index 0 (firmware code 0x0) and 0xA..0xF are never
 * documented in the manual — we report 0 to signal "unknown". */
const uint32_t msif_qms_speed_filter_us[10] = {
    0u,        /* 0x0 — not in manual table */
    25u,       /* 0x1 — 1 ms/u  ->   25 us */
    75u,       /* 0x2 — 3 ms/u  ->   75 us */
    250u,      /* 0x3 — 10 ms/u ->  250 us  (0.25 ms) */
    750u,      /* 0x4 — 30 ms/u ->  750 us  (0.75 ms) */
    2250u,     /* 0x5 — 0.1 s/u -> 2250 us  (2.25 ms) */
    7500u,     /* 0x6 — 0.3 s/u -> 7500 us  (7.5 ms) */
    25500u,    /* 0x7 — 1 s/u   -> 25.5 ms */
    75000u,    /* 0x8 — 3 s/u   -> 75 ms */
    250000u,   /* 0x9 — 10 s/u  -> 0.25 s */
};

bool msif_qms_set_speed(msif_qms_speed_t code) {
    if (code < MSIF_QMS_SPEED_1MS_U || code > MSIF_QMS_SPEED_10S_U) return false;
    msif_gpio_write_field(msif_speed_pins, 4, (uint8_t)code);
    return true;
}

bool msif_qms_set_op_mode(msif_qms_opmode_t mode) {
    /* MODE bits 1 and 2 carry operating mode; MODE bit 0 is scan-mode and
     * we leave it untouched here. So we only write msif_mode_pins[1] and [2].
     *
     * Firmware bit pattern, LSB = MODE1, MSB = MODE2 (0bMODE2_MODE1):
     *   EMISSION_OFF -> 0b00 (both inactive)
     *   INTEGRAL     -> 0b01 (MODE1L=L active)
     *   SPECTRUM     -> 0b10 (MODE2L=L active)
     *   DEGAS        -> 0b11 (both active)
     * TOTAL  is INTEGRAL @ FMASS=8.   HELIUM is SPECTRUM @ FMASS=4. */
    uint8_t bits;
    bool synth_total = false;
    bool synth_helium = false;

    switch (mode) {
        case MSIF_QMS_OPMODE_EMISSION_OFF: bits = 0x0; break;
        case MSIF_QMS_OPMODE_INTEGRAL:     bits = 0x1; break;
        case MSIF_QMS_OPMODE_SPECTRUM:     bits = 0x2; break;
        case MSIF_QMS_OPMODE_DEGAS:        bits = 0x3; break;
        case MSIF_QMS_OPMODE_TOTAL:        bits = 0x1; synth_total = true;  break;
        case MSIF_QMS_OPMODE_HELIUM:       bits = 0x2; synth_helium = true; break;
        default: return false;
    }

    gpio_put(msif_mode_pins[1], (bits     ) & 1u);  /* MODE1L */
    gpio_put(msif_mode_pins[2], (bits >> 1) & 1u);  /* MODE2L */

    if (synth_total)  msif_set_fmass(8.0f);
    if (synth_helium) msif_set_fmass(4.0f);
    return true;
}

void msif_qms_set_scan_mode(msif_qms_scanmode_t mode) {
    /* Enum value already matches the firmware bit (REPEAT=0, SINGLE=1). */
    gpio_put(msif_mode_pins[0], (uint)mode & 1u);
}

bool msif_qms_set_gain(msif_qms_gain_t code) {
    if ((unsigned)code > 0x3u) return false;
    msif_gpio_write_field(msif_gain_pins, 2, (uint8_t)code);
    return true;
}

bool msif_qms_set_range(msif_qms_range_t code) {
    if ((unsigned)code > 0x3u) return false;
    msif_gpio_write_field(msif_range_pins, 2, (uint8_t)code);
    return true;
}

/* ----- string parsers (case-insensitive) -------------------------------- */

#if !defined(strcasecmp)
/* Pico SDK's libc has strcasecmp via strings.h above; fallthrough guard. */
#endif

int msif_qms_parse_speed(const char *s) {
    if (!s) return -1;
    if (!strcasecmp(s, "1ms")   || !strcasecmp(s, "1ms/u"))    return MSIF_QMS_SPEED_1MS_U;
    if (!strcasecmp(s, "3ms")   || !strcasecmp(s, "3ms/u"))    return MSIF_QMS_SPEED_3MS_U;
    if (!strcasecmp(s, "10ms")  || !strcasecmp(s, "10ms/u"))   return MSIF_QMS_SPEED_10MS_U;
    if (!strcasecmp(s, "30ms")  || !strcasecmp(s, "30ms/u"))   return MSIF_QMS_SPEED_30MS_U;
    if (!strcasecmp(s, "100ms") || !strcasecmp(s, "0.1s")
                                || !strcasecmp(s, "0.1s/u"))   return MSIF_QMS_SPEED_100MS_U;
    if (!strcasecmp(s, "300ms") || !strcasecmp(s, "0.3s")
                                || !strcasecmp(s, "0.3s/u"))   return MSIF_QMS_SPEED_300MS_U;
    if (!strcasecmp(s, "1s")    || !strcasecmp(s, "1s/u"))     return MSIF_QMS_SPEED_1S_U;
    if (!strcasecmp(s, "3s")    || !strcasecmp(s, "3s/u"))     return MSIF_QMS_SPEED_3S_U;
    if (!strcasecmp(s, "10s")   || !strcasecmp(s, "10s/u"))    return MSIF_QMS_SPEED_10S_U;
    return -1;
}

int msif_qms_parse_opmode(const char *s) {
    if (!s) return -1;
    if (!strcasecmp(s, "emis_off")  ||
        !strcasecmp(s, "emission_off") ||
        !strcasecmp(s, "off"))                        return MSIF_QMS_OPMODE_EMISSION_OFF;
    if (!strcasecmp(s, "spectrum"))                   return MSIF_QMS_OPMODE_SPECTRUM;
    if (!strcasecmp(s, "integral"))                   return MSIF_QMS_OPMODE_INTEGRAL;
    if (!strcasecmp(s, "degas"))                      return MSIF_QMS_OPMODE_DEGAS;
    if (!strcasecmp(s, "total"))                      return MSIF_QMS_OPMODE_TOTAL;
    if (!strcasecmp(s, "helium")   || !strcasecmp(s, "he")) return MSIF_QMS_OPMODE_HELIUM;
    return -1;
}

int msif_qms_parse_gain(const char *s) {
    if (!s) return -1;
    if (!strcasecmp(s, "x1")   || !strcasecmp(s, "1"))    return MSIF_QMS_GAIN_X1;
    if (!strcasecmp(s, "x10")  || !strcasecmp(s, "10"))   return MSIF_QMS_GAIN_X10;
    if (!strcasecmp(s, "x100") || !strcasecmp(s, "100"))  return MSIF_QMS_GAIN_X100;
    if (!strcasecmp(s, "auto"))                           return MSIF_QMS_GAIN_AUTO;
    return -1;
}

/* ----- analog setters --------------------------------------------------- */

float msif_qms_set_swidth_amu(float width_amu) {
    /* Same calibration logic as msif_peak.c sweep — use stored slope if it
     * has been bench-verified, else fall back to the QMS-spec default
     * 10V / mass_range. SWIDTH has no offset term per the spec equation. */
    float slope;
#if MSIF_FMASS_CAL_BENCH_VERIFIED
    slope = (MSIF_FMASS_CAL_SLOPE_V_PER_AMU != 0.0f)
            ? MSIF_FMASS_CAL_SLOPE_V_PER_AMU
            : (10.0f / (float)MSIF_QMS_MASS_RANGE);
#else
    slope = (MSIF_FMASS_CAL_SLOPE_V_PER_AMU != 0.0f)
            ? MSIF_FMASS_CAL_SLOPE_V_PER_AMU
            : (10.0f / (float)MSIF_QMS_MASS_RANGE);
#endif

    float v_qdp = slope * width_amu;
    if (v_qdp < 0.0f) v_qdp = 0.0f;
    return msif_set_swidth_v(v_qdp);
}

float msif_qms_set_sem_kv(float hv_kv) {
    /* Per QMS-112 manual sec 10.2.4: U_SEM = (HV_kV/4) * 9.762 + 0.238 V.
     * Linear range is 0..3 kV — clamp before applying the formula so the
     * QDP voltage stays in [0.238, 7.559] V. */
    if (hv_kv < 0.0f) hv_kv = 0.0f;
    if (hv_kv > 3.0f) hv_kv = 3.0f;
    float v_qdp = (hv_kv / 4.0f) * 9.762f + 0.238f;
    return msif_set_sem_v(v_qdp);
}
