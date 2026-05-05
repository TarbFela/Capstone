# MSIF Calibration

The FMASS slope/offset and a couple of analog-chain gain values are the
only things that ever need bench measurement. Everything else is wiring
sanity that you check once during bring-up.

---

## FMASS calibration (the actual calibration)

Two numbers in `MSIF_cfg.h`:

```c
#define MSIF_FMASS_CAL_SLOPE_V_PER_AMU  0.05f   // QMS-112 spec, range=200
#define MSIF_FMASS_CAL_OFFSET_V         0.0f
```

Defaults are the QMS-112 manual's spec values (sec 10.2.2.1):
`U_FMASS+ = (mass / mass_range) × 10V`. For the 200-AMU range that's
0.05 V/AMU. **`MSIF_QMS_MASS_RANGE` must match the QMS's physical jumper
setting (J6/J7) — currently 200u in `MSIF_cfg.h`.**

To refine against a real gas peak:

1. With plasma running and a known gas (atmospheric N₂ is fine), pick
   28 AMU (N₂⁺).
2. Run `:SWEEP 26 30 81`.
3. Look at the per-row CSV. The reported `peak_at_amu` in the summary
   line tells you what the firmware currently thinks the peak is at.
4. If the firmware says 27.85 instead of 28.00, scale the slope:
   `new_slope = old_slope × (28.000 / 27.85)`.
5. Update `MSIF_FMASS_CAL_SLOPE_V_PER_AMU` in `MSIF_cfg.h`. Reflash.
6. Optionally repeat with H₂O (18 AMU) or Ar (40 AMU) to verify
   linearity. If centroids are off by a constant fraction of an AMU
   across multiple peaks, set `MSIF_FMASS_CAL_OFFSET_V` to compensate.

That's it.

---

## Analog-chain gain values

Two more numbers in `MSIF_cfg.h` that come from the schematic but should
be verified once at the bench:

```c
#define MSIF_AMP_GAIN           4.0f    // DAC -> OPA4197 -> QDP
#define MSIF_ADC_INPUT_GAIN     1.0f    // QDP -> INA146 -> ADC input
```

To verify `MSIF_AMP_GAIN`:
1. DMM on QDP `FMASS+` pin.
2. Run `:FMASSV 5.0`. DMM should read close to 5 V.
3. If it's off, measured_gain = `V_dmm / V_dac_register`. Update.

To verify `MSIF_ADC_INPUT_GAIN`:
1. Inject a known DC (e.g. +1.000 V) at QDP `EC-` referenced to SIG_GND.
2. Run `:READAVG 256`. Read `v_adc_diff`.
3. measured_gain = `v_adc_diff / 1.000`. Update.

---

## Bring-up sanity checks (one-time)

Not calibration, but worth doing once on a new board:

- `?` over USB → banner reprints. MCU is alive.
- `s` with no QDP cable → both inputs read 0. U21 + pull-ups are OK.
- Each output keystroke (`o`, `r`, `c`, `S`, `M`, `G`, `N`) actually
  moves a MOSFET drain on a scope/DMM. Pin assignments in `MSIF_cfg.h`
  match the schematic.
- `l` (DAC SPI loopback) → wrote `0xBEEF`, read `0xBEEF`. SPI bus and
  the GPIO 42 bodge wire are intact.
- `:A` (ADC register dump) → CFG bytes match what `msif_adc_init()`
  programmed.
- With QMS connected: `:ONLINE 1` lights the REM LED; `:GAIN x10` etc.
  update the QMS front panel.

If any of these fail, fix wiring or pin macros — none of them produce
numbers that go into `MSIF_cfg.h`.
