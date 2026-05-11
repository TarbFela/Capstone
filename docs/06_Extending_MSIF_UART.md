# Extending the System: MSIF + UART

The biggest open work item is the **MSIF firmware** — the code that runs on the Mass Spec Interface board and connects to the Balzers QMS-112 mass spectrometer. The MSIF board exists in hardware (see `Hardware/Documentation/ADPC & MSIF Schematics/`), the firmware skeleton is in `MSIF_dev/`, and there's a friendly getting-started document by Maxim for whoever picks this up (`MSIF_dev/MSIF_Docs/isaiah_getting_started.md`). Read that first — it covers basic Pico SDK setup, getting code building, and a starting point for digital I/O.

This document focuses on the **architectural decisions** that come *after* you have basic GPIO working: how to add UART communication between the ADPC and the MSIF, and how to set up DAQ (data acquisition) on the MSIF side that fits the existing patterns.

---

## What the MSIF needs to do

In rough priority order:

1. **Read QMS digital outputs** (`EMIS_OK`, etc.) into RP2350 GPIO inputs. — *Easy. Start here.*
2. **Drive QMS digital inputs** (handshake lines back to the QMS) from RP2350 GPIO outputs. — *Also easy.*
3. **Read QMS analog outputs** (FMASS+, etc.) via an ADC. — *Medium. Looks a lot like the ADPC's TSNS channel.*
4. **Drive QMS analog inputs** with a DAC. — *Medium. New driver work, but conceptually similar to PWM.*
5. **Translate high-level commands** ("set mass to X", "scan from A to B") into the right sequence of analog/digital actions. — *The interesting part.*
6. **Communicate with the ADPC over UART** for coordinated experiments. — *Once 1–5 are working, this glues it all together.*
7. **Stream mass spec data** to the host (or to the ADPC) for logging. — *Once 6 is working.*

The first four items are mostly straightforward hardware-driver work, similar in shape to what's in `ADPC_Dev/`. The architectural decisions are in items 6 and 7.

---

## UART between the ADPC and MSIF

### Pin choice

Looking at `ADPC_Dev/ADPC_Docs/MCU Pinout.md`, GPIO 33 and 34 on the ADPC's PGA2350 are reserved for `UART_RX` and `UART_TX` respectively. These pins are routed through the hardware to the MSIF connector. On the MSIF side, find the matching pair in the MSIF schematic — they should be similarly placed on its PGA2350.

Define them in both boards' config headers:

```c
// in ADPC_Dev/ADPC_cfg.h:
#define ADPC_MSIF_UART       uart0
#define ADPC_MSIF_UART_TX    34
#define ADPC_MSIF_UART_RX    33
#define ADPC_MSIF_UART_BAUD  115200    // start conservative; raise later if needed

// in MSIF_dev/MSIF_cfg.h:
#define MSIF_ADPC_UART       uart0
#define MSIF_ADPC_UART_TX    <pin>
#define MSIF_ADPC_UART_RX    <pin>
#define MSIF_ADPC_UART_BAUD  115200
```

### Driver structure

Mirror the existing driver structure. Create a folder `MSIF_dev/adpc_uart_driver/` (or `ADPC_Dev/adpc_msif_uart.[ch]` for the ADPC side) with:

- A struct that holds the UART instance and any state needed.
- An `_init()` function.
- Read/write functions, at minimum a blocking pair.
- Hazards documented in a `README.md`.

A minimal skeleton:

```c
// adpc_msif_uart.h
typedef struct {
    uart_inst_t *uart;
    uint baud;
    bool initialized;
} adpc_msif_uart_t;

void adpc_msif_uart_init(adpc_msif_uart_t *u, uart_inst_t *uart,
                         int tx_pin, int rx_pin, uint baud);
int  adpc_msif_uart_write(adpc_msif_uart_t *u, const uint8_t *data, size_t len);
int  adpc_msif_uart_read (adpc_msif_uart_t *u,       uint8_t *dst,  size_t len, uint timeout_ms);
```

Initialization is plain Pico SDK:

```c
void adpc_msif_uart_init(adpc_msif_uart_t *u, uart_inst_t *uart,
                         int tx_pin, int rx_pin, uint baud) {
    u->uart = uart;
    u->baud = uart_init(uart, baud);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);     // 8N1
    uart_set_fifo_enabled(uart, true);
    u->initialized = true;
}
```

### Protocol design

Don't try to send floating-point setpoints as raw text — that's slow to parse and error-prone. Use a small framed binary protocol. A sketch:

```
  [STX] [TYPE] [LENGTH] [PAYLOAD...] [CRC16]
   1B    1B     1B        0–252B       2B
```

- `STX` is a fixed byte (e.g. `0xA5`). If the receiver sees anything else, it discards and resyncs.
- `TYPE` indicates the message: setpoint command, status update, data sample, etc.
- `LENGTH` is the payload length, capped at something reasonable.
- `CRC16` checks integrity. A bad CRC means drop the frame and resync to the next `STX`.

Message types you'll want roughly:

| Direction | Type | Payload | Purpose |
|-----------|------|---------|---------|
| ADPC → MSIF | `0x10` SET_MASS | float | Set QMS mass target |
| ADPC → MSIF | `0x11` START_SCAN | float start, float end, float speed | Start a mass scan |
| ADPC → MSIF | `0x12` STOP_SCAN | (none) | Halt the scan |
| MSIF → ADPC | `0x20` SCAN_SAMPLE | float mass, int32 intensity | One data point |
| MSIF → ADPC | `0x21` STATUS | uint8 flags | EMIS_OK, etc. |
| Either | `0x00` PING | (none) | Heartbeat |

The point isn't to lock this design in — it's that **you need to design it before you write either side**. Document it (in a markdown file alongside the driver) so the two ends agree.

### Receive in interrupt or by DMA?

Three patterns:

1. **Polling** — call `uart_is_readable()` in your main loop and read bytes one at a time. Simple, works fine if your main loop is fast and you don't care about losing a few bytes during long operations. Try this first.
2. **UART RX IRQ** — register a handler that fires whenever bytes arrive, accumulate into a ring buffer. Better latency, won't drop bytes during slow operations. More complex.
3. **DMA on UART RX** — point a DMA channel at the UART RX FIFO with `uart_get_dreq()` as the DREQ. Bulletproof, never drops bytes, but only worth it for high-rate data (think: streaming sample data at >1 Mbps).

For the command-channel UART, start with polling. If you have a problem with dropped commands, move to IRQ. DMA on UART is overkill for ~115 kbps command traffic.

For *streaming sample data* over UART (if you decide to send mass spec samples from MSIF to ADPC at high rate), DMA is the right answer — see the section below.

---

## DAQ on the MSIF side

The MSIF will have its own ADC(s) reading QMS analog outputs. The pattern from the ADPC carries over directly: SPI-or-similar ADC, PIO machine to capture conversions, ping-pong DMA into RAM, IRQ updates a `volatile int` that consumers poll.

### If the MSIF uses an MCP3x6xR-family ADC

Lucky you — the entire driver in `mcp3x6xR_driver/` (both `mcp3x6xR.[ch]` and `mcp_pio.[ch]`) can be **copied verbatim** into `MSIF_dev/`. Update the `#include` paths, add the source files to `MSIF_dev/CMakeLists.txt`, and you're 90% of the way there.

The one thing to **double-check** is the PIO state machine count. `mcp_pio_init` is hardcoded to claim `pio0` and `pio1` based on a call counter, with a panic if you try to call it a third time. Since the MSIF is a *separate* RP2350 from the ADPC, it has its own PIO blocks — so the third-call panic doesn't apply across boards. But if you want both an MCP-style ADC *and* some other PIO-based peripheral on the MSIF, you'll need to do the bookkeeping yourself.

### If the MSIF uses a different ADC

You'll need to write a new driver. The structure should mirror `mcp3x6xR_driver/`:

```
MSIF_dev/my_adc_driver/
├── my_adc.h         # register defines, struct types, function prototypes
├── my_adc.c         # SPI-level configuration code
├── my_adc_pio.pio   # PIO program for streaming reads (if applicable)
├── my_adc_pio.h     # PIO state machine setup
├── my_adc_pio.c     # PIO + DMA setup, very similar to mcp_pio.c
└── README.md        # what this driver does, hazards, etc.
```

Use `mcp3x6xR_driver/` as your template. The structure of `mcp_pio.c` in particular is a good guide for any "PIO triggered by an external IRQ pin, results DMA'd into a ring buffer" peripheral.

### Sampling the analog inputs

The QMS-112 analog outputs (FMASS+, etc.) come into the MSIF via op-amp front-ends. The MSIF's ADC reads them on different channels of the same chip — exactly the way the ADPC's 24-bit ADC reads multiple channels in scan mode. Configure the ADC in scan mode and the channel ID will be encoded in the upper bits of each sample word, just like ADC 1 on the ADPC.

```c
// Example: MSIF ADC config for two channels in scan mode
cfg.input_mode = MCP_SCAN_MODE;
cfg.scan_sel = MCP_SCAN_SEL_BIT_DIFF_A | MCP_SCAN_SEL_BIT_DIFF_B;
cfg.cfgs[3] = MCP_CFG3_CONV_MODE_CONTINUOUS | MCP5_CFG3_DATA_FORMAT_32_CHID_SGN4_24;
```

In the consumer, unpack each word with the helpers from the host-side `script.py`:
```python
chid = (word >> 28) & 0xF
val  = sign_extend_24(word & 0xFFFFFF)
```
(C versions of these decode operations belong in a new `MSIF_dev/msif_adc_decode.[ch]` file — the host code is python, that's a python implementation of the same.)

---

## Streaming MSIF data over UART to the ADPC

If you want the MSIF to stream sample data to the ADPC at high rate, the cleanest design is:

```
MSIF                                                    ADPC
────                                                    ────

  ADC samples ─► PIO ─► DMA(ring buf)                     UART RX FIFO
                              │                                  ▲
                              │                                  │
                       IRQ handler updates                       │
                       msif_last_written                         │
                              │                                  │
                              ▼                                  │
                       Main loop:                                │
                       grab newest half-buffer                   │
                       frame it (STX + TYPE + LEN + DATA + CRC) ─┘
                              │
                       uart_write_blocking()
                              or
                       DMA from buffer to UART TX FIFO
                              (using uart_get_dreq for the DREQ)
```

On the ADPC side, a *receive*-DMA pattern complements this:

- Point a DMA channel at the UART RX FIFO with the RX DREQ.
- Use ping-pong, same as for the ADCs.
- IRQ on half-buffer-full sets a flag the main loop polls.

This effectively gives you a "wireless ADC" — the MSIF's ADC, exposed to the ADPC's firmware as if it were a third local ADC. The data ends up in the same DMA-style ring buffer the ADPC firmware already knows how to consume and stream.

If you build this, the ADPC's existing `rstream` command can be extended to stream the MSIF data alongside the ADPC's own data — interleaving the buffers from ADC 0, ADC 1, and "UART ADC" all on the USB output. The host-side decoder already separates channels by their channel-ID bits; you'd add a sentinel byte or a separate channel-ID range for the MSIF data so the decoder knows which board it came from.

---

## A reasonable order of work

For the developer picking up MSIF:

1. **Get basic firmware running.** Build, flash, blink LED, USB CDC echo. *(One day.)*
2. **Wire up the digital I/O.** Read EMIS_OK and friends, drive QMS digital inputs. Add to `msif_gpio.c`. *(A few days.)*
3. **Add UART to the ADPC.** Stub out a frame format. Send PING from the ADPC, blink the LED on receipt. Confirm round-trip. *(A few days.)*
4. **Pick an ADC for the MSIF.** If MCP-family, you're nearly done — copy the driver, instantiate it on the right SPI bus. If not, write a driver following the pattern. *(One to two weeks.)*
5. **Pick a DAC.** Similar story — likely an SPI DAC with its own driver. *(A week.)*
6. **Implement the high-level QMS commands.** Each high-level command becomes a function that orchestrates DAC writes and digital outputs in the right sequence. *(One to two weeks.)*
7. **Streaming.** UART-streaming mass spec samples to the ADPC, then on to the host. *(One week.)*

Total realistic estimate: 4–8 weeks for someone moderately comfortable with C, longer otherwise.

---

## What to put in the existing patterns vs. what to redesign

A note on temptation: when you're picking up a new codebase, there's an urge to clean things up, modularize, rewrite. **Resist this urge for the first month.** The reasons:

- The existing patterns work and have been debugged through real hardware bring-up. Subtle pitfalls (the DMA buffer indexing, the PIO pin ownership transfer, the `is_streaming` flag pattern) have already been found.
- A "cleaner" design that misses one of those pitfalls will cost you weeks of debugging.
- New patterns introduce new pitfalls. The existing ones have known pitfalls — already documented in [`02_Safety_and_Hazards.md`](02_Safety_and_Hazards.md).

After you've got the MSIF working end-to-end *using the existing patterns*, then by all means refactor. The architecture should evolve, just not before you understand it.

A few specific things you might be tempted to "improve" but should leave alone (at first):

- The long `if(strncmp(...))` chain in `app_dispatch`. It's not pretty, but it's debuggable. A function-pointer dispatch table is "cleaner" but harder to trace when a command does the wrong thing.
- The `goto reboot` in `main()`. C's not Java; `goto` for cleanup is fine and idiomatic in embedded code.
- The duplicated coefficient constants in `app_cmd_isp` and `core1_ictl`. Pull them into a `#define` in a header *after* you've confirmed the system works.
- The `dma0_last_written` / `dma1_last_written` globals. They're ugly globals, but they're the single shared word between an ISR and a polling consumer — which is exactly the use case `volatile int` was made for.

The rule of thumb: change *features*, not *architecture*, in your first month. By then you'll have a sense for what's load-bearing and what isn't.
