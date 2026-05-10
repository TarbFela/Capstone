# System Architecture

This document describes how the pieces fit together — both the hardware boards and the firmware layers within them. Read this before [`03_Driver_Reference.md`](03_Driver_Reference.md) or any of the deeper documents.

---

## The hardware stack

```
   ┌────────────────────────────┐         ┌────────────────────────────┐
   │          ADPC               │  UART  │           MSIF              │
   │   (control + measurement)   │◄──────►│   (mass spec interface)    │
   │                             │         │                            │
   │     PGA2350 (RP2350)        │         │     PGA2350 (RP2350)       │
   │     + 2× MCP ADCs           │         │     + DAC + ADC frontend   │
   │     + ADA4255 PGIA          │         │                            │
   └─────────────┬───────────────┘         └─────────────┬──────────────┘
                 │                                       │
        PWM ×12, PH_EN ×6                                │
                 │                          QMS-112 remote-control
                 ▼                                       ▼
   ┌─────────────────────┐   ┌─────────────────────┐    ┌──────────────────┐
   │     MPHB-002 #1     │   │    MPHB-002 #2 ...   │    │   Balzers QMS    │
   │  (2 buck converters │   │                      │    │      QMS-112     │
   │   per board, "1A"   │   │                      │    │ mass spectrometer│
   │   and "1B")         │   │                      │    │                  │
   └──────────┬──────────┘   └──────────────────────┘    └──────────────────┘
              │
              ▼
        ┌──────────┐
        │  HEATER  │ (load sits between the two buck converter outputs)
        └──────────┘
```

### The boards

**ADPC** — *Analog and Digital Processing and Control.* Hosts a PGA2350 daughtercard (the actual microcontroller, an RP2350) plus three precision SPI chips:

| Chip | Function | Notes |
|------|----------|-------|
| MCP3462R | 16-bit ADC — current sensing (ISNS) | Faster sample rate; this is the one in the control loop |
| MCP3562R | 24-bit ADC — temperature & voltage sensing (TSNS, VSNS) | Higher precision; scan mode across multiple inputs |
| ADA4255 | Programmable-gain instrumentation amplifier | Front-end for the thermocouple before it hits the MCP3562R |

The ADPC also has an MCP9808 I²C temperature sensor and a TS3A24159 analog switch for voltage-sense amp gain selection.

**MPHB-002** — *Multi-Phase H-Bridge, revision 002.* The power stage. Each MPHB-002 board provides **two** "B-style" channels (e.g. `1B` and `2B`). What we call an "H-bridge" here is actually **two buck converters with the load between them** — this lets us drive the heater both positively and negatively about a common-mode level. Up to three boards can be installed, giving six independent channels (`HB1A`, `HB2A`, `HB3A`, `HB1B`, `HB2B`, `HB3B`).

Each channel exposes:
- **Two PWM inputs** (`A` and `C`, conceptually one for each buck converter half)
- **One phase-enable input** (`PH_EN`) that turns the power stage on/off without changing PWM
- **Current-sense output** wired back to the ADPC's MCP3462R

**MSIF** — *Mass Spec Interface.* Communicates with a Balzers QMS-112 mass spectrometer via that instrument's "QDP" remote-control connector. Hosts its own PGA2350 plus analog I/O front-ends for the QMS-112's analog signals (FMASS+, etc.) and digital handshake lines (EMIS_OK, etc.). Currently a stub — see [`06_Extending_MSIF_UART.md`](06_Extending_MSIF_UART.md).

---

## The firmware stack (ADPC side)

The firmware is organized into roughly four layers, from low-level hardware drivers up to the user-facing shell. Each layer builds on the one below; you can usually do new work at one layer without disturbing the others.

```
  Layer            File(s)                              Purpose
  ─────────────────────────────────────────────────────────────────────────────
   USER SHELL     adpc_data_streaming.c                 main(), boot, reboot
                  adpc_app.[ch]                         line buffer, dispatch
                                                                  
   COMMANDS       adpc_app_funcs.[ch]                   isp, ictl, rstream,
                                                        irun, irun stream,
                                                        adpc_init
                                                                  
   CONTROL LOOP   adpc_core1.[ch]                       runs on core 1:
                                                        PI current controller
                                                                  
   DRIVERS        ADPC_Dev/ADPC_ADC.[ch]                MCP ADC config
                  ADPC_Dev/adpc_gpio_pwm.[ch]           MPHB PWM / PH_EN
                  ADPC_Dev/adpc_vsns.[ch]               VSNS amp gain pin
                  mcp3x6xR_driver/mcp3x6xR.[ch]         raw SPI to ADC
                  mcp3x6xR_driver/mcp_pio.[ch]          PIO + DMA streaming
                  mcp3x6xR_driver/mcp_pio.pio           PIO assembly program
                  ada4255_driver/ada4255.[ch]           raw SPI to PGIA
                                                                  
   PIN MAP        ADPC_Dev/ADPC_cfg.h                   every #define for a pin
```

`ADPC_cfg.h` is the single source of truth for which physical pin does what. Any new feature that touches a pin should reference symbols from this file, not hardcoded GPIO numbers.

---

## What runs where: cores

The RP2350 has two CPU cores. This firmware uses both:

- **Core 0** — runs `main()`. Owns the USB CDC connection (the shell), the DMA interrupt handlers (which deposit data into RAM), and all "slow" command handling. Lives inside `app_shell_task()` in a loop.
- **Core 1** — *only running when closed-loop current control is active.* Runs `core1_ictl()` which polls the DMA write position, computes a moving-average current, runs a PI controller, and updates the PWM output level. Started/stopped from core 0 via `app_cmd_ictl()`.

The two cores communicate through a hardware FIFO. Core 0 sends `MC_FIFO_STOP_FLAG` to ask core 1 to exit cleanly; core 1 responds with an acknowledgement before exiting.

They also share `ui_state` (declared `volatile`) for things like the current setpoint and the most recently computed level. **Race conditions are a real risk here** — see the hazards document.

---

## What runs where: DMA, PIO, interrupts

The ADC sample path is the heart of the system. Here's the chain, end to end:

```
  ADC pulls nIRQ low ──► PIO state machine wakes up
                         (one per ADC: pio0/sm0 and pio1/sm0)
                                  │
                                  │ shifts 32 clock pulses, captures MISO
                                  ▼
                         PIO RX FIFO has one 32-bit word
                                  │
                                  │ DREQ triggers DMA
                                  ▼
                         DMA writes into ring buffer
                         (dma_a or dma_b, ping-pong, chained to each other)
                                  │
                                  │ when buffer half is full:
                                  ▼
                         DMA fires IRQ on the chosen line
                                  │
                                  │ DMA_IRQ_0 → handler_0   (for ADC 0)
                                  │ DMA_IRQ_1 → handler_1   (for ADC 1)
                                  ▼
                         Handler clears the IRQ flag and updates
                         dma0_last_written / dma1_last_written
                                  │
                                  │ Either core can poll these globals
                                  ▼
                         Consumer (core 0 streaming OR core 1 control)
```

A few subtleties worth knowing now and revisiting in [`05_DMA_PIO_Streaming.md`](05_DMA_PIO_Streaming.md):

- Each ADC gets its **own pair of DMA channels** (A and B), wired as a ping-pong. When A fills, it chains to B and vice versa. Both write into a single contiguous buffer of size `2 × DMA_BUFF_SIZE` so it looks like one circular buffer.
- The buffers are **aligned in memory** to a power-of-two boundary so that DMA *ring mode* automatically wraps the write pointer back to the start of each half. This avoids needing software wrap-around.
- The ADC IRQ pin **must** be pulled high externally — the MCPs won't run conversions otherwise. This trips up most people once.
- `dma0_last_written` is `0` if nothing's been written yet, `1` if buffer A finished, `2` if buffer B finished. Whichever is "current" (i.e. the *other* one) can be inferred.

---

## Data flow during normal operation

The firmware can be in one of several modes. Each is initiated by a shell command:

| Mode | Started by | What core 0 does | What core 1 does | Hardware state |
|------|------------|------------------|------------------|----------------|
| Idle | (default after boot) | Shell prompt | not running | PWM off, PH_EN off |
| Initialized | `start` | Shell prompt | not running | PWM running but PH_EN may be off |
| Open-loop level set | `level <n>` | Shell prompt | not running | PWM at fixed differential level |
| Closed-loop current | `ictl` | Shell prompt | PI control on isns | PWM dynamically adjusted |
| Raw data stream | `rstream` | Stream ADC bytes via USB CDC | unchanged | unchanged |
| Program run | `irun` | Step setpoints over time | PI control on isns | PWM dynamically adjusted |
| Program run + stream | `irun stream` | Step setpoints AND stream | PI control on isns | PWM dynamically adjusted |

Note that `rstream` and `irun stream` *do not* alter the control state; they just attach the USB CDC pipe to the DMA buffers and start spitting bytes. The control loop and the streaming output are independent consumers of the same DMA-produced data.

---

## How to think about adding a feature

Pick the layer that matches what you're adding:

| Adding…                                          | Layer | Files to touch |
| ------------------------------------------------ | ----- | -------------- |
| A new shell command                              | USER SHELL → COMMANDS | `adpc_app.c`, `adpc_app_funcs.[ch]` |
| A new mode of using existing peripherals         | COMMANDS | `adpc_app_funcs.[ch]` |
| A change to how the controller computes its output | CONTROL LOOP | `adpc_core1.c` |
| A new chip on an existing bus (SPI, I²C)         | DRIVERS | new driver folder; reference from `adpc_app_funcs.c` |
| A new pin assignment                              | PIN MAP | `ADPC_cfg.h` |
| A new high-speed data source (e.g. mass spec ADC over UART) | DRIVERS → COMMANDS | new driver + new app command; see [`06_Extending_MSIF_UART.md`](06_Extending_MSIF_UART.md) |
