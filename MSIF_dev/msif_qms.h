/*
 * msif_qms.h — set-by-value wrappers around the QMS-112 remote-control surface.
 *
 * The single-letter CLI in main.c only does increment-by-one cycling for the
 * 4-bit SPEED, 3-bit MODE, 2-bit GAIN, and 2-bit RANGE multi-bit fields. That
 * is fine for bench poking but no good for a machine protocol that wants to
 * say "set GAIN to x10" or "set MODE to SPECTRUM" directly. This module
 * exposes those abstract setters, plus the analog-side transfer functions
 * for SWIDTH (AMU) and SEM (kV) that parallel msif_set_fmass(AMU).
 *
 * All encodings are taken from the QMS-112 service manual chapter 10
 * (Remote Control). The bit ordering in the firmware is LSB-first to match
 * the existing pin tables in main.c (msif_speed_pins[0] = SPEED_0, etc.),
 * which makes the firmware's raw bit pattern the COMPLEMENT of the manual's
 * ACTIVE-LOW convention: firmware bit = 1 means "GPIO HIGH = MOSFET ON =
 * QDP line pulled LOW = active in the manual's L-suffix sense".
 *
 * That's why the encoding constants below carry comments matching them back
 * to the manual's H/L tables — easier to verify against the PDF.
 */
#ifndef MSIF_QMS_H
#define MSIF_QMS_H

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* SCAN SPEED — QMS-112 manual sec 10.2.2.3                           */
/*                                                                    */
/*   Manual table (active-low, MSB on the left):                      */
/*     SPEED3L SPEED2L SPEED1L SPEED0L     scan rate    filter tc     */
/*       H       H       H       L         1   ms/u    25 us         */
/*       H       H       L       H         3   ms/u    75 us         */
/*       H       H       L       L        10   ms/u    0.25 ms       */
/*       H       L       H       H        30   ms/u    0.75 ms       */
/*       H       L       H       L         0.1 s/u     2.25 ms       */
/*       H       L       L       H         0.3 s/u     7.5  ms       */
/*       H       L       L       L         1   s/u     25.5 ms       */
/*       L       H       H       H         3   s/u     75   ms       */
/*       L       H       H       L        10   s/u     0.25 s        */
/*                                                                    */
/* Firmware encoding: each "L" in the manual = bit set in the         */
/* firmware (active = MOSFET on = QDP low). LSB-first.                */
/* ------------------------------------------------------------------ */
typedef enum {
    MSIF_QMS_SPEED_1MS_U   = 0x1,  /* 0001 -> 0L=L, 1..3L=H */
    MSIF_QMS_SPEED_3MS_U   = 0x2,
    MSIF_QMS_SPEED_10MS_U  = 0x3,
    MSIF_QMS_SPEED_30MS_U  = 0x4,
    MSIF_QMS_SPEED_100MS_U = 0x5,
    MSIF_QMS_SPEED_300MS_U = 0x6,
    MSIF_QMS_SPEED_1S_U    = 0x7,
    MSIF_QMS_SPEED_3S_U    = 0x8,
    MSIF_QMS_SPEED_10S_U   = 0x9,
} msif_qms_speed_t;

/* Filter time constants in microseconds, indexed by msif_qms_speed_t. */
extern const uint32_t msif_qms_speed_filter_us[10];

/* ------------------------------------------------------------------ */
/* OPERATING MODE — QMS-112 manual sec 10.2.3                         */
/*                                                                    */
/*   MODE2L  MODE1L                                                   */
/*     H       H     EMISSION OFF                                     */
/*     L       H     SPECTRUM                                         */
/*     H       L     INTEGRAL                                         */
/*     L       L     DEGAS                                            */
/*                                                                    */
/* TOTAL  = INTEGRAL @ FMASS=8                                        */
/* HELIUM = SPECTRUM @ FMASS=4                                        */
/*                                                                    */
/* The op-mode handler also touches FMASS for the synthesized modes;  */
/* the bare-bit helper (msif_qms_set_op_mode_bits) leaves FMASS alone. */
/* ------------------------------------------------------------------ */
typedef enum {
    MSIF_QMS_OPMODE_EMISSION_OFF = 0x0,  /* MODE2L=H, MODE1L=H -> bits 0b00 */
    MSIF_QMS_OPMODE_SPECTRUM     = 0x2,  /* MODE2L=L, MODE1L=H -> bits 0b10 */
    MSIF_QMS_OPMODE_INTEGRAL     = 0x1,  /* MODE2L=H, MODE1L=L -> bits 0b01 */
    MSIF_QMS_OPMODE_DEGAS        = 0x3,  /* MODE2L=L, MODE1L=L -> bits 0b11 */
    MSIF_QMS_OPMODE_TOTAL        = 0xF1, /* synthesized: INTEGRAL @ FMASS=8 */
    MSIF_QMS_OPMODE_HELIUM       = 0xF2, /* synthesized: SPECTRUM @ FMASS=4 */
} msif_qms_opmode_t;

/* ------------------------------------------------------------------ */
/* SCAN MODE (MODE0L) — QMS-112 manual sec 10.2.2.4                   */
/*                                                                    */
/* The third MODE bit is independent: it selects what RESET_SCAN's    */
/* L->H edge does.                                                    */
/*                                                                    */
/*   SCAN     MODE0L     RESET_SCAN                                   */
/*   RESET     X          held L                                      */
/*   SINGLE    L          L->H edge                                   */
/*   REPEAT    H          L->H edge                                   */
/*                                                                    */
/* MODE0L is the LSB of the 3-bit MODE field (msif_mode_pins[0]).     */
/* ------------------------------------------------------------------ */
typedef enum {
    MSIF_QMS_SCANMODE_REPEAT = 0x0, /* MODE0L=H (inactive) -> firmware bit 0 */
    MSIF_QMS_SCANMODE_SINGLE = 0x1, /* MODE0L=L (active)   -> firmware bit 1 */
} msif_qms_scanmode_t;

/* ------------------------------------------------------------------ */
/* GAIN — QMS-112 manual sec 10.2.1.2                                 */
/*                                                                    */
/*   GAIN1L  GAIN0L                                                   */
/*     H       H      x1                                              */
/*     H       L      x10                                             */
/*     L       H      x100                                            */
/*     L       L      AUTO                                            */
/* ------------------------------------------------------------------ */
typedef enum {
    MSIF_QMS_GAIN_X1   = 0x0,  /* both H -> 0b00 */
    MSIF_QMS_GAIN_X10  = 0x1,  /* GAIN0L=L -> 0b01 */
    MSIF_QMS_GAIN_X100 = 0x2,  /* GAIN1L=L -> 0b10 */
    MSIF_QMS_GAIN_AUTO = 0x3,  /* both L -> 0b11 */
} msif_qms_gain_t;

/* ------------------------------------------------------------------ */
/* RANGE — QMS-112 manual sec 10.2.1.1 (cross-table at sec 9.1.2.3)   */
/*                                                                    */
/* RANGE 0..3, two bits, no human-readable enum names — the manual    */
/* documents these indirectly via the front-panel exponent display    */
/* (10^-5..10^-12 A FS depending on RANGE/GAIN combination).          */
/* ------------------------------------------------------------------ */
typedef enum {
    MSIF_QMS_RANGE_0 = 0x0,
    MSIF_QMS_RANGE_1 = 0x1,
    MSIF_QMS_RANGE_2 = 0x2,
    MSIF_QMS_RANGE_3 = 0x3,
} msif_qms_range_t;

/* ------------------------------------------------------------------ */
/* Setters (digital)                                                  */
/* ------------------------------------------------------------------ */

/* Set SPEED to a named value. Returns false on unknown code. */
bool msif_qms_set_speed(msif_qms_speed_t code);

/* Set the operating-mode bits (MODE1L+MODE2L). For TOTAL/HELIUM the
 * helper additionally writes FMASS to 8 or 4 AMU via msif_set_fmass().
 * Returns false on unknown mode. */
bool msif_qms_set_op_mode(msif_qms_opmode_t mode);

/* Set the scan-mode bit (MODE0L). Doesn't trigger the scan — caller
 * still has to pulse RESET_SCAN. */
void msif_qms_set_scan_mode(msif_qms_scanmode_t mode);

/* Set GAIN to a named value. */
bool msif_qms_set_gain(msif_qms_gain_t code);

/* Set RANGE 0..3. Returns false on >= 4. */
bool msif_qms_set_range(msif_qms_range_t code);

/* Lookup helpers — string parse for the protocol layer. Return -1 on no match. */
int msif_qms_parse_speed(const char *s);   /* "1ms"/"3ms"/.../"10s" */
int msif_qms_parse_opmode(const char *s);  /* "emis_off"/"spectrum"/... */
int msif_qms_parse_gain(const char *s);    /* "x1"/"x10"/"x100"/"auto" */

/* ------------------------------------------------------------------ */
/* Setters (analog)                                                   */
/* ------------------------------------------------------------------ */

/* Set SWIDTH+ from a width in AMU. Per QMS-112 manual sec 10.2.2.2:
 *   U_SWIDTH = (WIDTH_AMU / MSIF_QMS_MASS_RANGE) * 10V
 * Re-uses the FMASS slope from MSIF_cfg.h (same V/AMU transfer). Returns
 * the clamped V_QDP target actually commanded. */
float msif_qms_set_swidth_amu(float width_amu);

/* Set SEM+ from a high-voltage value in kV. Per QMS-112 manual sec 10.2.4:
 *   U_SEM = (HV_kV / 4) * 9.762 + 0.238 V
 * The QMS SEM supply is linear only over 0..3 kV, so this function
 * clamps hv_kv to [0.0, 3.0] (-> [0.238, 7.559] V at the QDP pin)
 * before applying the formula. Returns the clamped V_QDP target. */
float msif_qms_set_sem_kv(float hv_kv);

#endif /* MSIF_QMS_H */
