# Developer Documentation — Capstone TPS System

This documentation set is for developers continuing work on the Temperature Programming Power & Control System after Maxim Laplante's capstone. The intended audience is **scientists with some programming exposure**, not embedded-systems veterans. C-specific concepts and RP2350-specific concepts are explained when first introduced.

If you only have time for one read, read [`01_Architecture.md`](01_Architecture.md) and [`02_Safety_and_Hazards.md`](02_Safety_and_Hazards.md). Then come back for the rest as needed.

---

## What this project is

A power-and-measurement system for **temperature-programmed desorption (TPD) experiments** in the Frederick Lab. Two custom PCBs run the show:

- The **ADPC** ("Analog and Digital Processing and Control") drives heater current and measures sample temperature.
- The **MSIF** ("Mass Spec Interface") talks to a Balzers QMS-112 mass spectrometer.

Both boards host a **PGA2350** daughtercard (an RP2350-based microcontroller from Pimoroni — same chip as the Raspberry Pi Pico 2). Power-stage hardware is on separate **MPHB-002** cards, driven by PWM from the ADPC.

The ADPC firmware is mature; the MSIF firmware is a stub awaiting a developer.

---

## Where to find what

| If you want to…                                          | Read                                                                                        |
| -------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| Understand the system architecture                       | [`01_Architecture.md`](01_Architecture.md)                                                  |
| Avoid letting magic smoke out of the hardware            | [`02_Safety_and_Hazards.md`](02_Safety_and_Hazards.md)                                      |
| Use the existing low-level drivers (ADC, PGIA, H-bridge) | [`03_Driver_Reference.md`](03_Driver_Reference.md)                                          |
| Understand the shell, app state, and command dispatch    | [`04_Application_Layer.md`](04_Application_Layer.md)                                        |
| Understand how high-speed data acquisition works         | [`05_DMA_PIO_Streaming.md`](05_DMA_PIO_Streaming.md)                                        |
| Add UART + DAQ between the ADPC and MSIF                 | [`06_Extending_MSIF_UART.md`](06_Extending_MSIF_UART.md)                                    |
| Just use the device (operator-level)                     | [`HOW_TO_ADPC_Streaming.md`](../ADPC_Dev/ADPC_Docs/HOW_TO_ADPC_Streaming.md)                 |

---

## Directory map

```
Capstone/
├── ADPC_Data_Streaming/       ← Main firmware. Build target: adpc_streaming.uf2
│   ├── adpc_data_streaming.c  ← main() entry
│   ├── adpc_app.[ch]          ← shell, dispatch, app state
│   ├── adpc_app_funcs.[ch]    ← command implementations
│   ├── adpc_core1.[ch]        ← closed-loop current controller (runs on core 1)
│   ├── ada4255_driver/        ← PGIA driver
│   ├── mcp3x6xR_driver/       ← MCP ADC + PIO/DMA streaming driver
│   ├── script.py              ← host-side shell wrapper
│   ├── graphing.py            ← log file viewer
│   └── CMakeLists.txt
│
├── ADPC_Dev/                  ← Shared driver code referenced by the firmware
│   ├── ADPC_ADC.[ch]          ← Configures both MCP ADCs for the ADPC
│   ├── ADPC_cfg.h             ← All pin definitions for the ADPC board
│   ├── adpc_gpio_pwm.[ch]     ← MPHB H-bridge PWM control (the high-power part)
│   ├── adpc_vsns.[ch]         ← Voltage-sense amp gain selection
│   └── ADPC_Docs/             ← Pinout, streaming how-to, etc.
│
├── MSIF_dev/                  ← Skeleton MSIF firmware. NEEDS DEVELOPMENT.
│   ├── main.c                 ← Currently just waits for keypress
│   ├── msif_gpio.[ch]         ← Empty stub
│   ├── MSIF_cfg.h             ← Empty stub for pin definitions
│   └── MSIF_Docs/             ← Schematic PDFs, getting-started doc
│
├── src2/                      ← OLDER drafts. Reference only — do not build from here.
├── MCP3x6xR_Dev/              ← Standalone MCP test project (predates ADPC_Data_Streaming)
├── ADA4255_Dev/               ← Standalone ADA test project
├── W25_dev/                   ← External flash test code (not currently used)
└── Hardware/                  ← KiCad / EasyEDA files, schematics, datasheets
```

`src2/` and the various `*_Dev/` folders are early drafts kept for reference. The active code lives in `ADPC_Data_Streaming/` and `ADPC_Dev/`. New MSIF code goes in `MSIF_dev/`.

---

## A short C / RP2350 vocabulary

| Term | What it means here |
| ---- | ------------------ |
| **GPIO** | A pin on the microcontroller that can be set to high (3.3 V) or low (0 V), or read as an input. The RP2350 has ~48 of them. |
| **PWM** | Pulse-Width Modulation. A square wave whose duty cycle (% of time high) sets an analog-like output. Used to drive the H-bridges. |
| **SPI** | A 4-wire digital communication protocol (clock, two data lines, chip-select). All three precision chips on the ADPC (two ADCs, one PGIA) talk SPI. |
| **UART** | A 2-wire serial protocol (TX, RX). Will be used between the ADPC and MSIF. |
| **DMA** | Direct Memory Access — hardware that copies bytes between peripherals and RAM without involving the CPU. Used to stream ADC samples into memory at high speed. |
| **PIO** | A small programmable state machine block on the RP2350 that can run custom bit-banged protocols. Used here to read the ADCs the moment they signal "data ready." |
| **`volatile`** | A C keyword telling the compiler "this variable may be changed by hardware or another core — don't optimize accesses to it away." Used on the DMA-touched buffers and on `ui_state`. |
| **IRQ / ISR** | Interrupt request / interrupt service routine. A function that runs in response to a hardware event (e.g. "DMA finished a buffer"). Keep them short. |
| **Core 0 / Core 1** | The RP2350 has two CPU cores. Core 0 runs main + the shell. Core 1, when launched, runs the closed-loop current controller. |
| **BOOTSEL / USB flash mode** | The state the RP2350 enters when you want to flash new firmware. The `q` command in the shell forces this. |

---

## Build & flash

From either `ADPC_Data_Streaming/` or `MSIF_dev/`:

```bash
mkdir build && cd build
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -DPICO_BOARD=pico2 ..
make
```

This produces a `.uf2` file. To flash:

1. Hold the **BOOTSEL** button on the PGA2350 while plugging it into USB (or send `q` to a running firmware).
2. The board mounts as a USB drive.
3. Drag the `.uf2` file onto the drive. It will flash and reboot automatically.

The "RUN" button on the ADPC resets the board without entering BOOTSEL — useful for unsticking a hung firmware without reflashing.
