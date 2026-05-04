# MSIF Firmware — User Manual

Firmware for the MSIF interface board (RP2350 / Pico 2) that drives a Balzers
QMS-112 quadrupole mass spectrometer over its QDP remote-control connector.
This document covers everything you need to drive the firmware as it ships
right now: connecting, the two command interfaces, the full command list,
peak integration, and how to read the output.

---

## 1. What it does

The MSIF board sits between a controller (a host PC, or the ADPC board) and
the QMS-112's QDP back-panel connector. The firmware:

- Drives the QDP digital outputs (ON_LINE, RANGE, GAIN, MODE, SPEED,
  RESET_SCAN, CLR_EC) through MOSFET level-shifters.
- Reads the QDP digital inputs (EMIS_OK, SCAN_IN_PROGRESS) through an
  inverting Schmitt buffer.
- Writes analog set-points (FMASS+, SWIDTH+, SEM+) via a DAC80504 → OPA4197
  gain stage.
- Reads the QMS electrometer voltage (EC-) via an INA146 difference amp
  and an MCP3462RT 16-bit differential ADC.
- Integrates the EC voltage versus mass or versus time to produce
  peak-area summaries.
- Accepts commands from a USB CDC host **or** from a peer controller across
  an isolated UART link.

The firmware exposes two interfaces concurrently. They share the same
underlying hardware paths and you can mix them freely in one session.

### Note on units: what `v_ec` actually represents

Everywhere the firmware reports `v_ec` or integrates a "peak area," **the
unit is volts, not amperes**. The QMS-112 does I→V conversion internally
(manual 9.2.3): ions hit the collector, the QMS's electrometer + post-amp
+ RANGE/GAIN scaling produces a 0..10 V differential signal at the QDP
EC+/EC- pins. MSIF just digitises that voltage.

To recover ion current in amps from `v_ec`, you have to apply the per-
RANGE/GAIN transfer factor from QMS-112 manual 10.2.1.1 / 9.1.2.3
(e.g. RANGE=10⁻⁷, GAIN=×100 → 10 V FS = 10⁻⁷ A FS, so 1 V at EC ≈ 10⁻⁸ A).
**That conversion table is not yet wired into the firmware** — peak-area
summaries are reported as `area_V_AMU` (mass-domain) or `area_V_s`
(time-domain), and `mean_v_ec` is in volts. Multiply by the appropriate
transfer factor downstream if you need amps or coulombs.

---

## 2. Connecting

### USB

1. Plug a USB-Micro cable into USB on MSIF and into the host PC.
2. Open a terminal
3. Press `?`. The boot banner reprints — this confirms the connection.

The firmware drops printf output silently if no host is attached at the
moment of print, which is why the boot banner is often missed on first
connect. `?` is the reliable way to retrieve it after the host comes up.

### UART (peer / ADPC link)

The same firmware also accepts and emits the same commands over UART0 on
GP12 (TX) / GP13 (RX), through the on-board ISO6721 isolator.

| Setting | Value |
|---|---|
| Pins | GP12 = TX, GP13 = RX (on the MSIF MCU header) |
| Baud | 115200 |
| Frame | 8 data, no parity, 1 stop |
| Flow | none |

Commands flow either direction. `printf` output is mirrored to both USB and
UART simultaneously, so a host watching either channel sees the same
stream.

---

## 3. Two interfaces side by side

| | **Bench CLI** | **Protocol** |
|---|---|---|
| Trigger | single keypress, no ENTER | `:` prefix, then a line + ENTER |
| Audience | a human at a serial terminal | a script or peer controller |
| Example | `M` (cycle MODE field by 1) | `:MODE SPECTRUM` |
| State changes | usually increment-by-one | usually set-by-value |
| Output | free-form printf | structured `# OK CMD ...` / `# ERR ...` |

Use whichever fits. They share underlying functions, so a bench operator
can press `f` to set a mass at the same time a script is sending
`:READAVG 32` from another terminal.

---

## 4. Bench CLI reference

Press the listed key. No ENTER unless the command prompts for an argument.

| Key | Action | Notes |
|---|---|---|
| `?` | Print banner | Use after a late host connect to recover the welcome line |
| `s` | Print digital I/O state | All inputs + all outputs in one line |
| `o` | Toggle ON_LINE | Switches QMS between manual and remote-control modes |
| `r` | Pulse RESET_SCAN ~10 ms | Ends or restarts a scan ramp |
| `c` | Pulse CLR_EC ~10 ms | Shorts EC+ to EC- (zeroes the electrometer) |
| `R` | Pulse RESET_SCAN ~2 s | Slow version, visible on a meter |
| `C` | Pulse CLR_EC ~2 s | Slow version |
| `S` | Increment SPEED (4-bit, wraps) | Cycles all 16 codes; print shows binary |
| `M` | Increment MODE (3-bit, wraps) | Cycles all 8; covers operating + scan modes mixed |
| `G` | Increment GAIN (2-bit, wraps) | x1 / x10 / x100 / AUTO in firmware-bit order |
| `N` | Increment RANGE (2-bit, wraps) | Codes 0..3 |
| `f` | Set FMASS in AMU | Prompts for AMU; uses calibration |
| `v` | Set FMASS+ in raw QDP volts | Prompts for V; bypasses calibration |
| `a` | Print analog (DAC) status | Register snapshot + calibration state |
| `i` | One-shot EC voltage read | Single ADC conversion of v_ec |
| `A` | ADC register snapshot | CFG0..3, IRQ, MUX |
| `d` | Toggle DAC CS# at 1 Hz | Bench probe; press any key to stop |
| `l` | DAC SPI loopback | Writes 0xBEEF, reads it back |
| `F` | FMASS sweep + log | Voltage-domain sweep, CSV stream |
| `P` | FMASS park + stream | Time-domain dwell, CSV stream |
| `K` | **Peak sweep + integrate** | Mass-domain, returns area/centroid |
| `T` | **Peak park + integrate** | Time-domain, returns area/mean |
| `:` | Enter protocol mode | Read a line, dispatch as protocol command |
| `q` | Reboot to BOOTSEL | For flashing new firmware |

Single-letter commands `f`, `v`, `F`, `P`, `K`, `T` interactively prompt
for arguments. You can either pipe arguments inline (e.g. type
`f 28<ENTER>` quickly) or press the key and then type the values when
prompted.

---

## 5. Protocol reference (`:` prefix)

Every protocol command starts with `:` followed by the command name, any
arguments separated by spaces, and ENTER. Names are case-insensitive.
Replies follow a fixed format:

- `# OK <CMD> [key=value ...]` — success.
- `# ERR <reason>` — error. The DAC is left in whatever state it was in
  before the command (no DAC writes happen on argument-parse errors).
- Multi-line CSV streams (from `:SWEEP` / `:DWELL`) match the format the
  bench `F`/`P`/`K`/`T` commands already use.

### Information

| Command | Purpose |
|---|---|
| `:HELP` | Print every command in this section with one-line help |
| `:INFO` | Calibration status + key configuration constants |
| `:STATUS` | Machine-readable digital I/O snapshot (parallel to bench `s`) |

### Analog set-points

| Command | Purpose |
|---|---|
| `:MASS <amu>` | Set FMASS+ from a mass in AMU using the calibration |
| `:FMASSV <v_qdp>` | Set FMASS+ in raw QDP volts (bypasses calibration) |
| `:SWIDTH <amu>` | Set SWIDTH+ from a width in AMU (per spec equation) |
| `:SWIDTHV <v_qdp>` | Set SWIDTH+ in raw QDP volts |
| `:SEM <hv_kv>` | Set SEM HV in kV (clamped to the linear 0..3 kV range) |
| `:SEMV <v_qdp>` | Set SEM in raw QDP volts |

### Digital state

| Command | Purpose |
|---|---|
| `:GAIN <x1\|x10\|x100\|auto>` | Set GAIN field |
| `:MODE <emis_off\|spectrum\|integral\|degas\|total\|helium>` | Set operating mode (TOTAL = INTEGRAL @ FMASS=8; HELIUM = SPECTRUM @ FMASS=4) |
| `:SCANMODE <single\|repeat>` | Set the MODE0L scan-mode bit |
| `:RANGE <0..3>` | Set RANGE field |
| `:SPEED <code>` | Set SCAN SPEED. Codes: `1ms`, `3ms`, `10ms`, `30ms`, `100ms`, `300ms`, `1s`, `3s`, `10s` (suffix `/u` accepted) |
| `:ONLINE <0\|1>` | Drive ON_LINE pin |
| `:RESET` | Pulse RESET_SCAN ~10 ms |
| `:CLR` | Pulse CLR_EC ~10 ms |

### Reads

| Command | Purpose |
|---|---|
| `:READ` | One-shot ion-current reading |
| `:READAVG <n>` | Average `n` consecutive ADC reads |

### Peak integration

| Command | Purpose |
|---|---|
| `:SWEEP <m_start> <m_end> <n_steps>` | Mass-domain peak integration |
| `:DWELL <v_qdp> <duration_ms>` | Time-domain peak integration |

### Sequencer

| Command | Purpose |
|---|---|
| `:WAIT <ms>` | Sleep for `ms` milliseconds. Mainly useful inside `:RUN`. Capped at 600000 (10 min). |
| `:RUN <cmd1>; <cmd2>; ...` | Run a list of protocol commands sequentially. Aborts at the first failed sub-command (e.g. unknown command, bad args, ADC failure, user keypress during a SWEEP/DWELL). |

---

## 6. Peak integration in detail

Two modes share the same trapezoidal accumulator and the same output shape.

### Mass-domain sweep — `K` or `:SWEEP m_start m_end n_steps`

Steps FMASS from `m_start` to `m_end` AMU in `n_steps` linear points. At
each step the firmware:

1. Writes the DAC.
2. Sleeps `MSIF_FMASS_SETTLE_MS` (default 50 ms) for the QMS mass filter.
3. Takes `MSIF_ADC_AVG_SAMPLES` (default 32) averaged ADC reads.
4. Emits a CSV row.
5. Adds the `(mass, v_ec)` point to the trapezoidal area accumulator.

After the last step, FMASS is parked at 0 V and a summary line prints:

```
# PEAK_SWEEP_SUMMARY status=0 area_V_AMU=1.234e-02 centroid_amu=28.012 \
    peak_max_v_ec=4.567e-03 peak_at_amu=28.000 n_samples=41 \
    dwell_ms=2125 mcp_status_or=0x00
```

`area` is in V·AMU (intensity-weighted area under the peak in mass units).
`centroid` is the intensity-weighted mean mass (Σ m·v / Σ v).

`n_steps` must be ≥ 3 and ≤ 2001. `m_end > m_start` is required. The
function rejects on out-of-range arguments without moving the DAC.

### Time-domain park — `T` or `:DWELL v_qdp duration_ms`

Holds FMASS+ at `v_qdp` for `duration_ms`, sampling at
`MSIF_ADC_PARK_PERIOD_MS` cadence (default 100 ms). Emits a CSV row per
sample. After the duration:

```
# PEAK_PARK_SUMMARY status=0 area_V_s=2.345e+01 mean_v_ec=4.690e-03 \
    peak_max_v_ec=4.812e-03 peak_at_s=2.5 n_samples=50 dwell_ms=5050 \
    mcp_status_or=0x00
```

`area` is in V·s (charge-equivalent integral of the ion-current voltage
over time). `mean_v_ec` is the simple mean of all samples.

`duration_ms` must be 1..600000 (10 minute hard cap).

### Aborting

Any keypress mid-run aborts the loop. The firmware:

1. Stops sampling.
2. Parks FMASS at 0 V.
3. Prints a `PARTIAL ` prefix on the summary line.
4. Returns whatever was integrated up to the abort point.

ADC failures mid-run print `# PEAK_*: ADC read failed at step/t=...` and
the same partial-summary mechanism applies.

### Example sessions

```
> :SWEEP 27 29 21
# CAL_STATUS: ... (calibration banner)
# PEAK_SWEEP mass_start=27.0000 mass_end=29.0000 steps=21 ...
step,t_ms,mass_amu,v_fmass,ec_code,v_adc_diff,v_ec,status
0,52,27.000,2.7000,12,0.000183,0.000183,0x00
...
20,1098,29.000,2.9000,8,0.000122,0.000122,0x00
# PEAK_SWEEP_SUMMARY status=0 area_V_AMU=... centroid_amu=... ...
# PEAK_SWEEP complete, FMASS parked at 0 V
```

```
> :DWELL 2.8 3000
# CAL_STATUS: ...
# PEAK_PARK v_fmass=2.8000 duration_ms=3000 period_ms=100 samples_per_pt=32
t_ms,t_s,ec_code,v_adc_diff,v_ec,status
... (30 sample rows)
# PEAK_PARK_SUMMARY status=0 area_V_s=... mean_v_ec=... ...
# PEAK_PARK complete, FMASS parked at 0 V
```

---

## 7. Sequencing — `:RUN`

Executes a list of protocol commands separated by `;`:

```
:RUN MASS 28; WAIT 100; READAVG 64; MASS 32; WAIT 100; READAVG 64
```

The firmware prints a step trace and a final count:

```
# RUN step=0 cmd='MASS 28'
# OK MASS amu=28.0000 v_qdp=...
# RUN step=1 cmd='WAIT 100'
# OK WAIT ms=100
# RUN step=2 cmd='READAVG 64'
# OK READAVG n=64 code=... v_ec=...
...
# OK RUN steps=6
```

Useful for SIM-style monitoring (a list of `MASS ...; WAIT ...; READAVG ...`
trios) or for setup sequences (e.g. `RUN ONLINE 1; MODE SPECTRUM; GAIN x10;
SCANMODE single`). `:WAIT <ms>` is the standard way to insert dwell time
between actions inside a RUN — there's no implicit delay between steps.

RUN aborts on the first sub-command that fails:

```
# RUN step=2 cmd='MASS bogus'
# ERR usage: MASS <amu>
# ERR RUN aborted at step=2
```

A keypress during a long-running `SWEEP`/`DWELL` step bubbles up as a
RUN abort the same way (the peak function returns non-OK status, the
RUN handler stops there). Commands inside RUN that take a long time
(`SWEEP`, `DWELL`, `WAIT`) block the entire sequence — there is no
preemption between steps.

RUN does **not** nest. Putting a `RUN ...` inside another `RUN ...` is
unsupported and will fail at the inner RUN.

---

## 8. Output format reference

All firmware output is plain ASCII. Lines fall into four categories:

| Prefix | Meaning |
|---|---|
| `# OK <CMD> ...` | Successful protocol command, possibly with key=value data |
| `# ERR <reason>` | Failed protocol command. Argument parse errors do not move the DAC |
| `# CAL_STATUS: ...` | FMASS calibration banner, prepended to every K/T/SWEEP/DWELL run |
| `# PEAK_*_SUMMARY ...` | Final integration summary at the end of a peak run |
| `# PEAK_* <verb>...` | Intermediate progress / final state notes from a peak run |
| `# RUN step=...` | Per-step trace inside a `:RUN` sequence |
| `<csv columns>\n<rows>` | Header + per-sample rows from F / P / K / T / SWEEP / DWELL |
| anything else | Bench-CLI free-form text (e.g. `SPEED=0010 (0x2)`) |

The leading `#` on every comment line is intentional — MATLAB's
`importdata` and Python's `numpy.genfromtxt(..., comments='#')` skip those
lines automatically, so you can capture the raw stream and parse the
sample rows directly.

### CSV columns

**Bench `F` (voltage sweep):**
```
step,t_ms,v_fmass,ec_code,v_adc_diff,v_ec,status
```

**Bench `P` (voltage park):**
```
t_ms,ec_code,v_adc_diff,v_ec,status
```

**`K` / `:SWEEP` (mass sweep + integration):**
```
step,t_ms,mass_amu,v_fmass,ec_code,v_adc_diff,v_ec,status
```

**`T` / `:DWELL` (time park + integration):**
```
t_ms,t_s,ec_code,v_adc_diff,v_ec,status
```

### Field meanings

| Column | Meaning |
|---|---|
| `step` | Sample index (0-based) |
| `t_ms` | Wall-clock ms since loop start |
| `t_s` | Same in seconds, present only on park rows |
| `mass_amu` | Target FMASS in AMU at this step |
| `v_fmass` | Target QDP voltage commanded to FMASS+ |
| `ec_code` | Signed 16-bit ADC code (averaged) |
| `v_adc_diff` | Differential volts at the MCP3462RT input |
| `v_ec` | Back-projected volts at QDP EC- (after the input network) |
| `status` | OR'd MCP status byte across the averaging batch |

---

## 9. Boot, reboot, and reflashing

| Trigger | Effect |
|---|---|
| Power-on / USB enumeration | Banner prints; firmware enters CLI loop |
| `?` keypress | Banner reprints |
| `q` keypress | Reboots into BOOTSEL — drag-and-drop a new `.uf2` |
| Reset button on the board | Hard reset, normal boot |
| Power cycle | Same as reset |

`q` is the recommended way to flash a new build: connect, press `q`, the
RP2350 mounts as a USB mass-storage device, copy the `.uf2`, the board
reboots into the new firmware automatically.

---

## 10. Calibration note

The firmware ships with the FMASS slope set to a placeholder value derived
from the QMS-112 manual (`10 V / mass_range`, default `0.1 V/AMU`). Every
peak command (`K`, `T`, `:SWEEP`, `:DWELL`) prints a multi-line warning
banner reminding you of this. The placeholder is functional — the
firmware will move FMASS to a real voltage that's the right order of
magnitude — but is not the same as a measured per-board calibration.

When the board has been calibrated against a known gas peak, the values
in `MSIF_cfg.h` should be updated and `MSIF_FMASS_CAL_BENCH_VERIFIED` set
to 1. After reflashing, the warning banner becomes a single one-line
acknowledgement and the calibrated slope is used in place of the spec
default.

The manual is intentionally silent on how to do that calibration; it
belongs in the bench-test plan, not here.

---

## 11. Quick reference card

```
USB CDC:    any baud, 8N1, no flow.  UART:  GP12 TX / GP13 RX, 115200 8N1.

BENCH CLI (single key, no ENTER unless prompted):
  ?         help banner          q  reboot to BOOTSEL
  s a A     status / dac / adc   l  DAC SPI loopback
  o  R r    ON_LINE / RESET      C c  CLR_EC
  S M G N   incr SPEED/MODE/GAIN/RANGE
  f v       set FMASS by AMU / volts
  i         one-shot EC voltage read
  F P K T   sweep / park / peak-sweep / peak-park
  d         CS# 1 Hz toggle
  :CMD ...  protocol mode  (full list with :HELP)

UNITS: v_ec is VOLTS (QMS already did I->V); apply RANGE/GAIN transfer
       table from QMS-112 manual sec 10.2.1.1 to convert to amps.

PROTOCOL (line-mode, ':' prefix, ENTER terminates):
  :INFO  :HELP  :STATUS
  :MASS amu     :FMASSV v   :SWIDTH amu  :SWIDTHV v   :SEM kv  :SEMV v
  :GAIN sym     :MODE sym   :SCANMODE sym  :RANGE 0..3   :SPEED sym
  :ONLINE 0|1   :RESET      :CLR
  :READ         :READAVG n
  :SWEEP m_start m_end n_steps     :DWELL v_qdp ms
  :WAIT ms      :RUN cmd1; cmd2; cmd3...   (RUN aborts at first sub-error)

OUTPUT:
  # OK CMD k=v ...      success
  # ERR reason          failure (no DAC moved on arg errors)
  # CAL_STATUS: ...     calibration banner before every K/T/SWEEP/DWELL
  # PEAK_*_SUMMARY ...  area / centroid / max / dwell after K/T/SWEEP/DWELL
  # RUN step=...        progress inside :RUN
  <csv header>\n<row>   raw streams (lines without leading '#')

KEYPRESS DURING SWEEP/PARK = abort + park FMASS at 0 V.
```
