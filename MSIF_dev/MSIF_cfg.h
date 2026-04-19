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

// ==========================================================================
// DAC80504 (U15) — analog outputs to QMS via OPA4197 gain stage.
// SPI bus SHARED with MCP3462RT ADC (U18). Modes are incompatible:
//   DAC80504  = SPI mode 1 (CPOL=0, CPHA=1)
//   MCP3462R  = SPI mode 0 or 3
// Any code that talks to both must call spi_set_format() on every switch.
//
// HARDWARE BODGE (confirmed 2026-04-18 by Isaiah): the PCB layout omitted
// DAC CS# and LDAC# traces. On the current board rev:
//   - A magnet-wire bodge from GPIO 42 lands on a node that ties DAC pin 12
//     (CS#) and pin 11 (LDAC#) together. The short is intentional.
//   - Every SPI transaction drives both CS# and LDAC# simultaneously.
//   - dac_spi_init() writes SYNC=0x0000 so all four channels are in
//     asynchronous update mode — LDAC# is ignored and outputs update at
//     the CS# rising edge of each SPI write. See src2/dac80504_driver/dac80504.c.
// Because LDAC# and CS# are the same physical GPIO, MSIF_DAC_LDAC_PIN is set
// equal to MSIF_DAC_CS_PIN. The driver detects this and skips its normal
// LDAC pin init (which would otherwise clobber the CS# output direction).
//
// Confirmed from the MSIF MCU schematic + Isaiah (2026-04-18):
//   GPIO35 = shared ADC/DAC SDI (MOSI)   -- RP2350 SPI0 TX
//   GPIO36 = shared ADC/DAC SDO (MISO)   -- RP2350 SPI0 RX
//   GPIO37 = ADC CS (not shared)         -- RP2350 SPI0 CSn slot, driven manually
//   GPIO38 = shared ADC/DAC SCK          -- RP2350 SPI0 SCK
// The intended DAC CS/LDAC nets were never routed, so the live board uses
// the bodged GPIO42 wire for DAC CS instead of the original schematic target.
// ==========================================================================
#define MSIF_DAC_SPI            spi0        // GPIO35/36/37/38 map to SPI0 on RP2350
#define MSIF_DAC_CS_PIN         42          // existing bodge to U15 pins 11/12 (shorted)
#define MSIF_DAC_LDAC_PIN       MSIF_DAC_CS_PIN  // same GPIO 42; driver skips duplicate init
#define MSIF_DAC_SCK_PIN        38          // shared with ADC SCK
#define MSIF_DAC_SDI_PIN        35          // shared with ADC SDI (MOSI)
#define MSIF_DAC_SDO_PIN        36          // shared ADC/DAC SDO (MISO)
#define MSIF_DAC_BAUD_HZ        1000000u    // 1 MHz conservative; DAC80504 tolerates up to 50 MHz

// MCP3462RT ADC (U18) CS# — not shared with the DAC. Keeping this line here
// even though the ADC driver isn't written yet: msif_analog_init() parks this
// pin HIGH so a floating gate can't randomly assert ADC CS# while the DAC is
// mid-transaction on the shared MOSI/MISO/SCK lines. When the ADC driver
// lands, own this pin from its own init and remove the park-high from
// msif_analog.c.
#define MSIF_ADC_CS_PIN         37

// DAC80504 reference and analog chain gain (see MSIF Analog schematic):
//   V_REF = 2.5 V internal reference, GAIN register = 1× after soft reset
//   FMASS/SWIDTH/SEM each feed the op-amp through 4.99k, while the matching
//   "-" input net is tied to SGND_IF through another 4.99k/0Ω path. That
//   halves the DAC voltage at the non-inverting input before the OPA4197's
//   1 + 9.1k/1.3k ≈ 8 closed-loop gain, so the net transfer is ≈ 4 V/V.
//   Verify empirically at the bench once ±12V_IF is present.
#define MSIF_DAC_V_REF          2.5f
#define MSIF_AMP_GAIN           4.0f

// FMASS+ bench calibration (fill after Phase H voltage sweep — see
// sleepy-swinging-volcano.md). V_QDP = SLOPE × mass_amu + OFFSET.
#define MSIF_FMASS_CAL_SLOPE_V_PER_AMU  0.0f
#define MSIF_FMASS_CAL_OFFSET_V         0.0f

#endif
