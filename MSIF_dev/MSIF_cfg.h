#ifndef MSIF_CFG_H
#define MSIF_CFG_H

/*
* This is where you should define pin numbers, SPI channels, etc.
* See `ADPC_cfg.h` in `ADPC_Dev/` for reference
*/

// ==========================================================================
// MSIF pin definitions (from Schematic_MSIF_2026-04-11.pdf)
// ==========================================================================

// Digital inputs from QMS-112, via U21 (74AUP2G14DW inverting Schmitt).
// Because U21 inverts: QMS active-low signals read active-HIGH at the GPIO.
//   gpio_get(MSIF_DI_EMIS_OK_PIN) == 1         --> emission is OK
//   gpio_get(MSIF_DI_SCAN_IN_PROGRESS_PIN) == 1 --> scan in progress
// External 10k pull-ups to 3V3 on U21 inputs (R48, R49) — no internal pulls
// needed. NOTE: input-idle polarity is still unconfirmed at the bench; see
// sleepy-swinging-volcano.md Phase C.
#define MSIF_DI_EMIS_OK_PIN             30
#define MSIF_DI_SCAN_IN_PROGRESS_PIN    31

// Digital outputs driven through BSS138AKS-QX N-channel MOSFETs for
// 3.3V -> QMS-rail level shift. Pull-up rail is on the QMS side of the cable.
//   gpio_put(pin, 1) -> MOSFET ON  -> QMS signal pulled LOW  (active)
//   gpio_put(pin, 0) -> MOSFET OFF -> QMS pull-up holds HIGH (inactive)
// Same "active = 1 at GPIO" convention as the inputs.
#define MSIF_DO_ON_LINE_PIN             16
#define MSIF_DO_SPEED_3_PIN             17
#define MSIF_DO_GAIN_1_PIN              18
#define MSIF_DO_GAIN_0_PIN              19
#define MSIF_DO_MODE_1_PIN              20
#define MSIF_DO_MODE_0_PIN              21
#define MSIF_DO_RANGE_0_PIN             22
#define MSIF_DO_MODE_2_PIN              23
#define MSIF_DO_CLR_EC_PIN              24
#define MSIF_DO_RANGE_1_PIN             25
#define MSIF_DO_RESET_SCAN_PIN          26
#define MSIF_DO_SPEED_0_PIN             27
#define MSIF_DO_SPEED_1_PIN             28
#define MSIF_DO_SPEED_2_PIN             29

#endif
