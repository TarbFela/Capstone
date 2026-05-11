# ADPC User Guide

## Before You Begin

_AI Generated_

**Startup sequence** — the firmware waits for a single keypress before it begins.
`script.py` handles this automatically. If connecting manually (e.g. with a terminal emulator), send any character to get past the boot prompt.

**`start` must be the first command.** It initializes the ADCs, the PGIA, and the H-bridge outputs. Nothing else will work correctly until this is done.

**The RUN button** on the ADPC hardware can be pressed at any time to force-break a stuck condition (hung stream, blocked loop, etc.) and return to a clean shell prompt.

**`q` reboots the device into USB flash mode** for programming. The firmware is no longer running after this — you must flash new code before the device can be used again.

---

## Serial Shell Reference

Connect via `script.py` (recommended) or any serial terminal at **115200 baud**.

### Initialization

| Command | Description |
|---------|-------------|
| `start` | **Start here.** Initializes ADCs, PGIA, and enables all H-bridge PWM and phase outputs. |
| `init`  | Lower-level initialization only (no PWM/phase enable). Prefer `start`. |

### Output Control

| Command | Description |
|---------|-------------|
| `level <n>` | Set PWM level for all H-bridges. |
| `level`     | Print current level. |
| `phen`      | Enable phase outputs on all H-bridges. |
| `phd`       | Disable phase outputs on all H-bridges. |
| `pwmen`     | Enable PWM on all H-bridges. |
| `pwmd`      | Set all PWM levels to zero. |
| `sdith <val>` | Set output level using spatial dithering across phases (float, e.g. `sdith 0.15`). |

### Current Control Loop

| Command | Description |
|---------|-------------|
| `isp <amps>`    | Set the current setpoint in amps (e.g. `isp 2.5`). Valid range: −10 to 100 A. |
| `ictl`          | Toggle the closed-loop current controller on core 1. Run again to stop it. |
| `coeff i <val>` | Set the integrator coefficient (e.g. `coeff i 0.25`). |
| `coeff p <val>` | Set the proportional coefficient (e.g. `coeff p 0.75`). |

### Current Programs

A *current program* is a sequence of current setpoints stepped through at a fixed timestep.

| Command | Description |
|---------|-------------|
| `iprog`        | Interactively enter a current program. Prompts for timestep (ms) then setpoints one by one. Type `END` to finish. |
| `irun`         | Run the loaded program. Press any key to cancel early. |
| `irun stream`  | Run the loaded program while simultaneously streaming raw ADC data. Stops automatically when the program ends. |

### Data & Diagnostics

| Command | Description |
|---------|-------------|
| `rstream`  | Stream raw ADC data over USB. Send any character to stop. |
| `r0 <n>`   | Print the last `n` raw values from ADC channel 0 (DMA buffer). |
| `r1 <n>`   | Print the last `n` raw values from ADC channel 1 (DMA buffer). |

### System

| Command | Description |
|---------|-------------|
| `q` | **Reboot to USB flash mode.** New firmware must be flashed before the device can be used again. |

---

## Python Scripts

### `script.py` — Interactive Shell

Provides a transparent passthrough to the firmware shell with added support for streaming data capture and CSV program loading.

**Usage:**
```
python3 script.py [/dev/cu.usbmodem<XXXX>]
```
If only one USB modem device is found, it is selected automatically.

**Additional commands available inside `script.py`:**

| Command | Description |
|---------|-------------|
| `rstream` | Stream ADC data. Press **Enter** to stop. Data is saved automatically to `logs/`. |
| `irun stream` | Run the loaded program with ADC streaming. Stops automatically at program end. Data saved to `logs/`. |
| `iprog <file.csv> [timestep_ms]` | Load a current program from a CSV file instead of typing setpoints manually. See CSV format below. |
| `q` | Send reboot command to firmware and exit the script. |

#### CSV File Format for `iprog`

One setpoint per line, in amps. Example:

```
500
1.0
1.5
2.0
1.5
1.0
0.5
END
```

- The first line is used as the **timestep in milliseconds** if no `timestep_ms` argument is provided on the command line. If `timestep_ms` is given explicitly, the first line is treated as a setpoint.
- `END` is optional — the file ends when no more lines remain.
- Non-numeric lines and blank lines are skipped.
- A maximum of 128 setpoints can be loaded (firmware limit).

**Examples:**
```
# Use first line (500 ms) as timestep:
python3 script.py iprog my_program.csv

# Override timestep to 250 ms:
python3 script.py iprog my_program.csv 250
```

---

### `graphing.py` — Log Viewer

Plots saved ADC log files from the `logs/` directory.

**Usage:**
```
python3 graphing.py
```

On launch it lists all available log files and prompts:

| Input | Behavior |
|-------|----------|
| *(blank)* | Plot the most recent log file. |
| A number (e.g. `3`) | Plot the 3 most recent log files. |
| A filename | Plot that specific file. |

Each ADC channel is plotted as a separate subplot with a shared time axis. Time is interpolated linearly between the recorded start and stop timestamps.

---

## Log File Format

Log files are saved to `logs/` by `script.py` and are plain text CSV files readable in any spreadsheet application.

**Filename:** `log [YYYY-MM-DD HH-MM-SS].txt`

**Structure:**
```
# time_start: 2025-01-15 14:32:01
# time_stop:  2025-01-15 14:32:45
CH 0, CH 1, CH 2
-1042, 832, 1205
-1038, 829, 1201
...
```

- Lines 1–2: start and stop timestamps (used by `graphing.py` to reconstruct the time axis).
- Line 3: channel header. Channel IDs correspond to MCP ADC channel numbers.
- Remaining lines: one sample per row, comma-separated. Channels with missing samples for a given row are left blank.

Values are signed 24-bit integers decoded from the raw ADC words.