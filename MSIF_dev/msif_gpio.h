/*
 * msif_gpio.h — MSIF digital I/O surface.
 *
 * Single source of truth for the multi-bit field pin tables (SPEED, MODE,
 * GAIN, RANGE) and the bit-pack/unpack helpers that walk them. Other modules
 * (main.c, msif_qms.c, msif_proto.c) include this header and use the same
 * canonical arrays + helpers — no more triplicated copies.
 *
 * Single-bit pins (ON_LINE, RESET_SCAN, CLR_EC, EMIS_OK, SCAN_IN_PROGRESS)
 * are still driven directly via the SDK's gpio_put / gpio_get / gpio_xor_mask
 * with the pin numbers from MSIF_cfg.h — wrapping those would be all noise
 * and no signal. msif_gpio_pulse() is offered for the common "gpio HIGH for
 * N ms then LOW" pattern that several callers need.
 */
#ifndef MSIF_GPIO_H
#define MSIF_GPIO_H

#include <stdint.h>

#include "pico/stdlib.h"

/* Configure all MSIF digital I/O pins. Inputs as inputs (no internal pulls,
 * external 10k on U21 inputs handles idle state). Outputs default LOW
 * (MOSFET off, QMS signal inactive). */
void msif_gpio_init(void);

/* Multi-bit MSIF output field pin tables. LSB-first: index 0 is bit 0,
 * etc. Defined canonically in msif_gpio.c. Use with msif_gpio_read_field
 * and msif_gpio_write_field. */
extern const uint msif_speed_pins[4];
extern const uint msif_mode_pins[3];
extern const uint msif_gain_pins[2];
extern const uint msif_range_pins[2];

/* Read the current value of a multi-bit output field from the latched
 * GPIO_OUT register (NOT the input synchronizer) — safe to call right
 * after a write without worrying about the 2-cycle GPIO_IN race. */
uint8_t msif_gpio_read_field(const uint *pins, int nbits);

/* Write a multi-bit output field. LSB of `value` goes to pins[0]. */
void msif_gpio_write_field(const uint *pins, int nbits, uint8_t value);

/* Drive `pin` HIGH for `ms` milliseconds, then LOW. Blocking. Used for
 * the RESET_SCAN / CLR_EC pulse pattern. Caller is responsible for any
 * surrounding output (e.g. printf logging). */
void msif_gpio_pulse(uint pin, uint32_t ms);

#endif /* MSIF_GPIO_H */
