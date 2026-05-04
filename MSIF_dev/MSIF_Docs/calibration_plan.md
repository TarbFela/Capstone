# MSIF Calibration & Bench-Verification Plan

This document enumerates every constant or assumption in the MSIF firmware
that needs measurement, the hardware required to measure it, and the
order to do it in. Companion to `user_manual.md` (which deliberately
ducks the bench question).

It is structured as **four stages of progressively more equipment**, each
unlocking more of the calibration surface. You can stop at any stage —
later stages depend on earlier ones, but earlier stages are useful by
themselves for code/PCB/firmware bring-up.

> **Reminder on units.** The MSIF ADC reads volts at QDP EC- — the QMS
> does I→V conversion internally (manual §9.2.3). Every "v_ec" the
> firmware reports is in volts; peak-area summaries are V·AMU or V·s,
> not amperes. The per-RANGE/GAIN transfer table that converts v_ec to
> ion current in amps (manual §10.2.1.1) is **not yet codified in the
> firmware**, so this doc focuses on calibrating the voltage chain. The
> amps conversion is a future-firmware item that becomes useful once the
> RANGE/GAIN cross-product table from manual §9.1.2.3 is baked in.

---

## Calibration target table (skim this first)

| # | Constant / assumption | Defined at | Stage to fix | Quick description |
|---|---|---|---|---|
| 1 | DI idle polarity | `MSIF_cfg.h:13` | 1 (USB only) | EMIS_OK / SCAN_IN_PROGRESS read 0 with no QMS |
| 2 | DAC SPI loopback | `msif_analog.c` | 1 (USB only) | `0xBEEF → DAC_A → 0xBEEF` |
| 3 | ADC SPI sanity | `msif_adc.c` | 1 (USB only) | `:A` returns sane CFG bytes |
| 4 | DAC analog chain `MSIF_AMP_GAIN` | `MSIF_cfg.h:117` | 2 (+12VIN_MSIF) | currently 4.0f from schematic math |
| 5 | ADC chain offset (firmware-side zero) | new `:ZERO` cmd (TBD) | 2 (+12VIN_MSIF) | INA146+ADC bias |
| 6 | `MSIF_ADC_INPUT_GAIN` | `MSIF_cfg.h:107` | 2 (+12VIN_MSIF) | currently 1.0f, INA146+divider |
| 7 | ADC MUX channel pair | `msif_adc.c:111` | 2 (+12VIN_MSIF) | currently CH0+/CH1- assumed |
| 8 | EC voltage sign convention | `msif_adc.c:24` | 2 / 4 | first guess on bench, confirm under plasma (sign of injected DC may differ from sign of post-amp output under real ions) |
| 9 | SEM transfer fn end-to-end | `msif_qms.c:135` | 2 (+12VIN_MSIF) | spec eq is in code; verify the chain |
| 10 | SWIDTH transfer fn end-to-end | `msif_qms.c:118` | 2 (+12VIN_MSIF) | same — spec eq in code, chain verify |
| 11 | DAC SEM/SWIDTH channel assignment | `msif_analog.h:53` | 2 (+12VIN_MSIF) | "TBD whether channel C is right" |
| 12 | QMS post-amp ZERO trim | QMS front panel | 3 (QMS connected) | screwdriver job, manual §8.2 |
| 13 | ON_LINE handshake | n/a | 3 (QMS connected) | REM LED responds to firmware |
| 14 | Digital field verification (MODE/GAIN/SPEED/RANGE) | `msif_qms.c` | 3 (QMS connected) | DVM reflects driven values |
| 15 | `MSIF_FMASS_SETTLE_MS` | `MSIF_cfg.h:138` | 3 (QMS connected) | currently 50 ms, verify against actual RF reconfig |
| 16 | `MSIF_FMASS_CAL_SLOPE_V_PER_AMU` | `MSIF_cfg.h:121` | 4 (plasma) | currently 0.0f, spec-default 10/range used as fallback |
| 17 | `MSIF_FMASS_CAL_OFFSET_V` | `MSIF_cfg.h:122` | 4 (plasma) | spec says 0; non-zero = analog-chain error |
| 18 | `MSIF_FMASS_CAL_BENCH_VERIFIED` | `MSIF_cfg.h:165` | 4 (plasma) | flip to 1 once #16/#17 are recorded |

---

## Stage 1 — USB only, no QMS, no ±12V_IF

### What's electrically alive
- The RP2350 (USB VBUS → Pico's onboard regulator → 3V3 → MCU runs).
- Every GPIO that drives a level-shifter MOSFET gate.
- The U21 inverting Schmitt buffer on the digital inputs (powered from 3V3 already on the MCU side).

### What is **not** alive
- The DAC80504, MCP3462RT, INA146s, and OPA4197 stages — all on the
  IF-side rails (3V3_IF, 5V_IF, ±12V_IF) which require **either**
  12VIN_MSIF on PWR1 driving U14/U20, **or** SW3/SW4 closed to bridge
  the MCU rails over to the IF side (breaks isolation, fine for bench).

### Hardware needed
- USB-Micro cable
- Host PC with serial terminal
- Optionally: small flat-blade screwdriver to flip SW3/SW4 if you want
  the IF chips alive on USB power without ±12V_IF.

### Procedure

1. **Boot test.** Plug in USB. Within a few seconds press `?`. The
   banner should reprint over USB CDC. Confirms `pico_stdio_usb` is up
   and the MCU is healthy.
2. **DI idle polarity (#1).** With no QDP cable connected, run `s`.
   Both `EMIS_OK` and `SCAN_IN_PROGRESS` should read **0**. The U21
   inverter + 10 kΩ external pull-ups on the MCU 3V3 rail give you a
   defined idle state. Anything else here is a wiring problem.
3. **DO toggle test.** Run each digital-output command in turn (`o`,
   `r`, `c`, `R`, `C`, `S`, `M`, `G`, `N`) and verify each pin actually
   moves on a scope or DMM at the MOSFET drain. Optional — you'll
   verify them more thoroughly in stage 3 — but it's a good GPIO smoke
   test now.
4. **(Optional, requires SW3/SW4 closed)** SPI loopback (#2). Run `l`.
   Should print `wrote 0xBEEF to DAC_A, read 0xBEEF -> OK`. Confirms the
   GPIO 42 bodge wire and the entire SPI MOSI/MISO/SCK/CS path. **If
   SW3/SW4 are open this will print `0x0000` because the DAC has no
   power.**
5. **(Optional, requires SW3/SW4 closed)** ADC register sanity (#3).
   Run `:A`. Should return non-zero CFG0..CFG3 bytes matching what
   `msif_adc_init()` programmed. Status byte should not be all-ones.

### What gets recorded
- Pass/fail on banner, idle inputs, GPIO toggles.
- If you tried (4)/(5): the readback values for future comparison.

Nothing in `MSIF_cfg.h` actually changes at this stage. It's a "is the
PCB and the firmware skeleton sane" gate.

---

## Stage 2 — Add 12VIN_MSIF on PWR1 (no QMS yet)

### What this unlocks
U14 (CC1R5-1212SR-E isolated DC-DC) generates ±12V_IF, U20
(TPS7A2450DBVR LDO) generates +5V_IF. The OPA4197 op-amps, INA146
difference amps, DAC80504, and MCP3462RT now all have rails. The full
analog chain is alive — but with QDP unplugged the inputs are floating
and outputs go nowhere.

This stage is where most of the constants in `MSIF_cfg.h` get measured.

### Hardware needed (cumulative — keep the USB stuff plus)
- Bench DC supply, ~12 V at a few hundred mA
- DMM
- A pile of QDP-connector-end test points / pin-extender / wire jumpers
- Bench DC supply, low-noise, ~0..5 V for analog-input injection
  (can be the same supply as above if it has two outputs, otherwise a
  second one)
- Function generator (sine/triangle, 0..5 V, mHz..kHz) for the K-command
  shape verification
- Optional: oscilloscope for ramp/edge timing if you care to verify
  settle behavior visually

### Procedure

#### 2.1 — DAC chain gain (#4)
Goal: replace the schematic-math `MSIF_AMP_GAIN = 4.0f` with the
measured ratio.

1. Connect DMM (high-impedance) to QDP pin **FMASS+**, reference
   to **SIG_GND**.
2. Run `:FMASSV 0.0`. DMM should read ~0 V.
3. Run `:FMASSV 5.0`. DMM should read close to 5 V.
4. Run `:FMASSV 10.0`. DMM should read close to 10 V.
5. For each command, also note the **DAC code** that should have been
   programmed: `v_dac = v_qdp / MSIF_AMP_GAIN`. Use `a` to read back the
   DAC register and confirm it.
6. **Real gain** = `V_dmm / v_dac`. If the average across the three
   points is not within ~5 % of 4.0, update `MSIF_AMP_GAIN` in
   `MSIF_cfg.h` to the measured value. Reflash.

#### 2.2 — DAC SEM/SWIDTH channel-assignment check (#11)
Goal: confirm DAC channel B drives SWIDTH+ and channel C drives SEM+.
The header at `msif_analog.h:53` flags this as TBD.

1. DMM on QDP **SWIDTH+**.
2. `:SWIDTHV 5.0`. DMM should read ~5 V × `MSIF_AMP_GAIN` adjustment as
   in 2.1. (After 2.1 has updated `MSIF_AMP_GAIN`, the readback should
   land at 5 V exactly.)
3. DMM on QDP **SEM+**.
4. `:SEMV 5.0`. Same expectation.
5. If either is wrong (e.g. SEM+ moves when you write SWIDTH or vice
   versa), the assignment in `msif_analog.c:130-136` needs the channel
   letters swapped.

#### 2.3 — ADC chain offset (#5, the firmware-side zero)
Goal: measure the bias of the INA146 + MCP3462RT chain so a future
`:ZERO` command can subtract it. (Or so you can record it manually.)

1. With QDP unplugged, jumper **EC+** to **EC-** at the QDP connector
   pins (or the cable end — wherever's accessible).
2. `:READAVG 256`. Record `v_ec`. **This number is the chain offset.**
   It should be on the order of the ADC's LSB (a few hundred µV at most
   if the INA146s are well-balanced).
3. Save it for use as the auto-zero baseline. If you want to bake it
   into firmware, add a `MSIF_ADC_OFFSET_V` constant in `MSIF_cfg.h`
   and subtract in `msif_adc_read_ec*`. (Future-feature; not currently
   in the codebase.)

#### 2.4 — ADC MUX channel pair (#7)
Goal: confirm the EC-path is on CH0+/CH1-.

1. Bench supply set to **+1.000 V**, output across QDP EC- (positive)
   and SIG_GND (negative). Verify with DMM before connecting.
2. `:READAVG 64`. Record the reported `v_ec`.
3. **Expect:** a non-zero value, polarity matching your supply (or
   inverted — that's #8 below, polarity convention).
4. **If you get zero:** the wiring assumption is wrong. Edit
   `msif_adc.c:114` — try `mcp_mux_sel(&s_adc, MCP_MUX_VAL_CH2, MCP_MUX_VAL_CH3)`
   etc. — until injection at EC- shows up at the ADC. Reflash between
   each try.

#### 2.5 — ADC chain gain (#6)
Goal: replace `MSIF_ADC_INPUT_GAIN = 1.0f` with the measured INA146 +
divider gain.

1. Continuing from 2.4, with the +1.000 V injected at EC-:
2. `:READAVG 256`. Record `v_adc_diff`.
3. **Real input gain** = `v_adc_diff / 1.000`. Update
   `MSIF_ADC_INPUT_GAIN` in `MSIF_cfg.h:107`. Reflash.
4. Repeat at +0.500 V and +2.000 V to confirm linearity. Should hold
   within a few percent of the same gain across the range.

#### 2.6 — Ion polarity convention (#8, partial)
Goal: confirm sign convention pre-plasma.

1. With known +1.000 V at EC-, after #2.5 is correct, verify `v_ec`
   reports **+1.000 V** (not -1.000 V).
2. If inverted: swap the MUX argument order in `msif_adc.c:114`
   (`mcp_mux_sel(&s_adc, CH1, CH0)` instead of `(CH0, CH1)`), or
   flip the sign in the v_ec computation. Either is fine.
3. **Final confirmation has to wait for plasma** — the QMS post-amp
   output under real ions may swing the opposite direction from an
   injected bench DC of the same polarity at EC- (the post-amp's I→V
   conversion has its own sign convention). The bench check confirms the
   ADC chain works; the plasma check confirms the QMS+ADC chain is
   reporting "ions present" with the polarity downstream code expects.

#### 2.7 — End-to-end DAC transfer functions (#9, #10)
Goal: confirm `msif_qms_set_swidth_amu` and `msif_qms_set_sem_kv` produce
the right QDP voltages per the QMS-112 manual equations.

For SWIDTH (#10):
1. DMM on SWIDTH+.
2. `:SWIDTH 50`. With `MSIF_QMS_MASS_RANGE = 100`, expect `(50/100)*10 V
   = 5.0 V`. DMM should agree (modulo `MSIF_AMP_GAIN` corrections from
   2.1).
3. `:SWIDTH 100`. Expect 10.0 V.

For SEM (#9):
1. DMM on SEM+.
2. `:SEM 0.0`. Expect 0.238 V (the formula's offset).
3. `:SEM 1.0`. Expect `(1.0/4)*9.762 + 0.238 = 2.679 V`.
4. `:SEM 3.0`. Expect `(3.0/4)*9.762 + 0.238 = 7.560 V`.
5. `:SEM 5.0`. Expect 7.560 V (clamp at 3 kV).

Any disagreement is either an `MSIF_AMP_GAIN` problem (carries through
all DAC channels) or a channel-assignment problem.

#### 2.8 — K command shape test (function-gen exercise)
Goal: prove the trapezoidal accumulator and CSV plumbing work.

1. Function gen: triangle wave, 0..2 V, 10 mHz (period 100 s).
2. Drive triangle into EC- (vs. SIG_GND).
3. `:SWEEP 0 100 21` — 21 mass-domain points across what would be 0..100
   AMU at the spec slope. The sweep takes ~2 s, so it'll catch the
   triangle in a known phase.
4. The per-row CSV `v_ec` should trace the triangle's instantaneous
   value. The summary line's `area_V_AMU` should be close to the
   integrated triangle area over the sweep duration.
5. This is mostly a regression check on the math — once you've done it
   once and the numbers make sense, you don't need to repeat.

### What gets recorded at end of stage 2
| Constant | Old | New (your measurement) |
|---|---|---|
| `MSIF_AMP_GAIN` | 4.0f | … |
| `MSIF_ADC_INPUT_GAIN` | 1.0f | … |
| ADC chain offset | unknown | … V |
| MUX channel pair | CH0+/CH1- | … |
| Ion sign convention | "positive ADC = positive ion" | confirmed / inverted |
| DAC channel assignments | A=FMASS, B=SWIDTH, C=SEM | confirmed / swapped |

Any constant that changed → edit `MSIF_cfg.h` (or the channel arg in
`msif_adc.c`) → reflash → re-verify with one quick spot check.

---

## Stage 3 — Connect QDP to a powered QMS-112 (no plasma yet)

### What this unlocks
- The QMS pull-ups on the digital lines are now real, so DO/DI behaviors
  match the manual rather than relying on dangling wires.
- The QMS responds to ON_LINE_L by going inert except for DVM + ZERO
  trim (manual §10.2).
- You can drive MODE/GAIN/SPEED/RANGE from firmware and watch the QMS's
  own DVM display reflect what you sent.

### Hardware needed (cumulative)
- A QMS-112 in working order, powered up, warmed up at least 5–10 min.
- A QDP cable wired to the MSIF QDP connector. Pin assignments per
  `MSIF_cfg.h:23-41` and the QMS manual §10.1.1.
- DMM (still useful for cross-checking).
- Small flat-blade screwdriver for the ZERO trim pot (#12).

### Procedure

#### 3.1 — QMS post-amp ZERO trim (#12)
Per QMS manual §8.2. Has nothing to do with MSIF.
1. Disconnect the cable to the QME (not the QDP — the OTHER cable, the
   one going to the analyzer head's pre-amp).
2. QMS front panel: DVM → ELECTROM, GAIN → x100.
3. Adjust the ZERO trim pot until DVM reads ≤ ±0.002.
4. Verify at all SPEED settings (display should stay within ±0.003).
5. Reconnect the QME cable.

This is a screwdriver job at the QMS, MSIF doesn't participate. The
ZERO trim pot stays live in remote mode (manual §10.2), so you don't
need to flip the QMS in/out of ON_LINE for this — but you might as
well leave ON_LINE off while you do it.

#### 3.2 — ON_LINE handshake (#13)
1. With QMS in manual, run `s` on MSIF — confirm `ON_LINE` is currently
   reported as 0.
2. Look at QMS front panel — REM LED should be **off**.
3. Run `:ONLINE 1`.
4. REM LED should light up. Front panel knobs/switches should stop
   responding (try turning FIRST MASS — DVM stops reflecting it).
5. Run `:ONLINE 0`. REM LED off, panel responsive again.

If the LED doesn't follow the firmware command, the wiring on the
ON_LINE_L pin (MSIF GPIO 16 → QDP) is wrong.

#### 3.3 — Digital field verification (#14)
With the QMS in remote mode (`:ONLINE 1` from 3.2):

1. Set QMS DVM to **MASS**.
2. `:FMASSV 5.0`. DVM should display "5.00" (or whatever the QMS's mass
   scaling reports — for mass range 100, 5 V at FMASS+ corresponds to
   FIRST_MASS = 50). Compare to QMS manual §10.2.2.1.
3. `:GAIN x10`. DVM should report a different scale label / decimal
   point per manual §10.2.1.2. Cycle through x1, x10, x100, AUTO and
   verify each lights the appropriate LED.
4. `:MODE SPECTRUM`. The QMS's MODE indicator should reflect spectrum
   mode. Cycle through EMISSION_OFF, SPECTRUM, INTEGRAL, DEGAS.
5. `:SPEED 10ms`. The SCAN SPEED indicator should reflect 10 ms/u.
   Cycle through a few representative speeds.
6. `:RANGE 2`. Range indicator should reflect.
7. `:RESET` and `:CLR`. SCAN_LED and CLR_EC behaviors should be visible
   on the panel.

If any digital field doesn't match — re-check the bit ordering / sign
in `msif_qms.c` and the manual tables in `msif_qms.h`. The polarity
gotcha (firmware bit = 1 = active = manual L) is the most likely cause.

#### 3.4 — Settle time validation (#15)
With QMS in remote, run `:SWEEP 0 10 21` while watching SCAN+ on a
scope or DMM. The output should be a clean staircase with each step
fully settled before the firmware moves on. If the QMS's RF
reconfiguration is producing visible glitches that aren't fully
damped at the next sample, increase `MSIF_FMASS_SETTLE_MS` past 50 ms.
If the steps look fully settled in <50 ms, the default is conservative
and you can shorten it.

#### 3.5 — `:CLR`-mediated chain offset (re-do of #5 with QMS support)
With QMS connected, you can now use the QMS's internal short instead
of a wire jumper:

1. `:ONLINE 1` (CLR_EC only operative in remote mode, manual §9.2.3.4).
2. Hold CLR_EC asserted (currently no firmware command does this — see
   "Future feature: `:ZERO`" note at the end of this doc).
3. `:READAVG 256` while held.
4. Compare the result to your stage-2.3 wire-jumper measurement. Should
   agree within noise.

### What gets recorded at end of stage 3
- ZERO trim final position confirmation (just "done").
- Per-field code/symbol mapping verified.
- `MSIF_FMASS_SETTLE_MS` final value if changed from 50.
- Re-confirmed chain offset matches stage 2.3.

Still no FMASS slope changes — that's stage 4.

---

## Stage 4 — Plasma running

### What this unlocks
- Real ions in the source.
- Known mass peaks in the **EC-voltage** vs. FMASS curve (N₂ at 28, H₂O
  at 18, Ar at 40, etc.) — the QMS post-amp converts ion current to
  voltage internally, so the firmware sees a voltage curve, not a
  current curve.
- The actual sign convention of the QMS post-amp output under real ions
  (which polarity comes out of the post-amp at EC+ when the collector is
  hit by ions — may differ from the polarity an injected bench DC of
  the same sign produced).

### Hardware needed (cumulative)
- Working vacuum chamber.
- QME 112 mass filter head connected and at proper bias.
- Filament current to the ionizer.
- A reference gas — N₂ from atmosphere is free, but laboratory-grade Ar
  or a known leaked gas mix is more useful for cross-mass calibration.

This stage doesn't need anything MSIF-specific that you didn't have in
stage 3 — just an actual experiment running.

### Procedure

#### 4.1 — FMASS slope/offset (#16, #17, #18) — the Phase H sweep
Goal: replace the spec-default `MSIF_FMASS_CAL_SLOPE_V_PER_AMU = 0.0f`
(spec fallback `10.0f / MSIF_QMS_MASS_RANGE = 0.1`) with a board- and
QMS-specific slope.

1. With plasma running and a known gas (atmospheric N₂ is fine), pick a
   peak you trust: 28 amu (N₂⁺) is the easy one.
2. `:SWEEP 26 30 81` — 81 points across mass 26..30 with the spec
   default slope.
3. Look at the per-row CSV. Find the row where `v_ec` is maximum.
4. The reported `mass_amu` at the maximum is what the firmware's
   *current* spec-default calibration thinks the peak is at. If the
   centroid in the summary line says `peak_at_amu=27.85` instead of
   28.000, your spec-default slope is **off by 28.000/27.85 ≈ 0.5 %**.
5. Multiply your current slope by that ratio:
   `new_slope = 0.1 V/AMU × (28.000 / 27.85)`. Update
   `MSIF_FMASS_CAL_SLOPE_V_PER_AMU` in `MSIF_cfg.h:121`.
6. Repeat with a second known peak (H₂O at 18 or Ar at 40) to verify
   linearity. If the second peak's centroid lands within a tenth of
   an AMU, slope is good.
7. If there's a residual offset (centroid lands consistently 0.X AMU
   off across multiple peaks), update `MSIF_FMASS_CAL_OFFSET_V`.
   Per QMS-112 manual §10.2.2.1 the spec equation has zero offset, so
   any non-zero is your analog-chain DC error — should be small.
8. Set `MSIF_FMASS_CAL_BENCH_VERIFIED 1` in `MSIF_cfg.h:165`. Reflash.
9. The loud calibration-banner warning that prints at every K/T/SWEEP/
   DWELL invocation collapses to a single one-line BENCH_VERIFIED
   confirmation.

#### 4.2 — Ion polarity final check (#8 confirmation)
With a real ion peak measured:
- `peak_max_v_ec` in the SUMMARY line should be **positive**.
- If it's negative, the polarity convention from stage 2.6 is wrong
  for ions. Either swap MUX args or negate. (Bench DC injection might
  have looked right while the post-amp output naturally swings the
  other way — only plasma tells the truth.)

#### 4.3 — Optional: cross-instrument sanity check
- Run a `:RUN MASS 28; READAVG 256; MASS 32; READAVG 256; MASS 40;
  READAVG 256` sequence. The 28/32/40 amu intensities should roughly
  match the gas composition you know to be in the chamber.
- Useful for catching gross errors (e.g. wrong sign, wrong ratio scale,
  saturation).

### What gets recorded at end of stage 4
| Constant | Old | New |
|---|---|---|
| `MSIF_FMASS_CAL_SLOPE_V_PER_AMU` | 0.0f | … V/AMU (measured) |
| `MSIF_FMASS_CAL_OFFSET_V` | 0.0f | … V (measured, ideally near 0) |
| `MSIF_FMASS_CAL_BENCH_VERIFIED` | 0 | 1 |
| Ion sign convention | tentative from stage 2.6 | confirmed |

After this, the firmware's loud calibration banner goes away and the
peak commands report science-grade numbers.

---

## Future-feature: `:ZERO` command

A natural follow-up that this calibration plan keeps gesturing at but
isn't currently implemented:

```
:ZERO         # asserts CLR_EC, settles, averages, releases, stores offset
:ZERO?        # prints the most recent offset
:ZERO 0.0     # clears the stored offset back to 0
```

Plus an optional `MSIF_ADC_AUTO_OFFSET` flag that makes
`msif_adc_read_ec*` automatically subtract the stored offset from
`v_ec`. ~30 lines in `msif_proto.c` + `msif_adc.c`. File when needed.

---

## Quick reference — what hardware do I need to do which stages?

| Stage | What it covers | Minimum gear |
|---|---|---|
| 1 | MCU+GPIO sanity | USB cable, host PC |
| 2 | Full MSIF analog calibration | Stage 1 + 12 V bench supply, DMM, jumper wires, second bench supply for injection, function gen |
| 3 | QMS integration verification | Stage 2 + powered QMS, QDP cable, screwdriver |
| 4 | FMASS calibration | Stage 3 + working vacuum chamber + plasma + reference gas |

You can do stages 1–2 entirely on a desk away from the QMS. Stage 3
needs the QMS but no chemistry. Only stage 4 demands actual experiment
conditions.

---

## What this doc deliberately doesn't cover

- Schematic capture, PCB rework, or how to populate DNP resistors —
  hardware-side work.
- Code feature additions (e.g. the `:ZERO` command) — those are tracked
  separately from calibration.
- ADPC-side firmware bring-up — when ADPC is built, it gets its own
  calibration plan.
- Long-term drift / re-cal cadence — depends on the experiment.
