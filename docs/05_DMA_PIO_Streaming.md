# DMA, PIO & Streaming — A Deep Dive

This document explains how the firmware moves ADC samples from chip to memory and from memory to USB, without bogging down the CPU. **Read this before adding any new high-speed data source** (such as the mass spec interface's ADC).

The same pattern — PIO triggers, DMA into a ping-pong ring buffer, IRQ-updated globals, polling consumers — will work just as well for the MSIF's analog inputs and for any future high-rate UART data. So while the examples here are about the MCP ADCs, treat it as a template.

---

## Why not just read the ADCs in a loop?

The MCP3462R, configured with OSR=2048 and a 15 MHz master clock, produces a new conversion every few microseconds. The 24-bit ADC at OSR=512 is even faster. If we tried to poll-read each conversion from main code:

- We'd need ~5 µs of SPI traffic per sample, all CPU.
- We'd miss samples whenever any other code ran (USB tick, anything else).
- We couldn't simultaneously do anything else at sample rate.

DMA + PIO solves all three: the hardware reads samples on its own, deposits them in RAM, and only interrupts the CPU when an entire batch is done.

---

## The pipeline, end-to-end

```
                    ADPC board                          PGA2350 (RP2350)
                ┌──────────────────┐         ┌──────────────────────────────┐
                │   MCP3462R ADC   │         │                              │
                │                  │  nIRQ   │  PIO state machine           │
   sample ─────►│  (continuous     │ ─────► │  (mcp_pio.pio assembly)      │
                │   conversion)    │         │                              │
                │                  │  SCK    │  shifts 32 bits in           │
                │                  │ ◄────── │                              │
                │                  │  MISO   │                              │
                │                  │ ─────► │                              │
                └──────────────────┘         │              │                │
                                             │              │ RX FIFO        │
                                             │              ▼                │
                                             │       DMA channel A           │
                                             │       (or B, ping-pong)       │
                                             │              │                │
                                             │              │ written to RAM │
                                             │              ▼                │
                                             │   ┌──────────────────────┐   │
                                             │   │  dma_buff_adc_0[]    │   │
                                             │   │   ┌──────┬──────┐    │   │
                                             │   │   │  A   │  B   │    │   │
                                             │   │   └──────┴──────┘    │   │
                                             │   │   ↑ aligned,         │   │
                                             │   │     ring-mode wraps  │   │
                                             │   └──────────────────────┘   │
                                             │              │                │
                                             │              │ IRQ when half  │
                                             │              ▼ is full        │
                                             │  dma_irq_handler_0()          │
                                             │   - clears IRQ flag           │
                                             │   - updates                   │
                                             │     dma0_last_written         │
                                             │              │                │
                                             │              ▼                │
                                             │  Consumer (core 0 streaming   │
                                             │  or core 1 control loop)      │
                                             └──────────────────────────────┘
```

---

## The PIO program

PIO ("Programmable I/O") is a small bit-banged state machine — almost like a tiny coprocessor — that can drive GPIO pins in patterns more complex than dedicated peripherals. The RP2350 has 3 PIO blocks (this firmware uses 2: `pio0` and `pio1`), each with 4 state machines, each running its own little program.

The MCP-read PIO program is in `mcp_pio.pio`:

```
.program mcp_conversions_read
.side_set 1                                ; the SCK pin

.wrap_target
public entry_point:
    set x, 31           side 0             ; we want 32 bits
    wait 1 jmppin       side 0             ; wait for nIRQ to go high (idle)
    wait 0 jmppin       side 0 [1]         ; wait for nIRQ to go low (data ready)
bitloop:
    nop                 side 1 [1]         ; SCK high
    in  pins, 1         side 0             ; SCK low, sample MISO into ISR
    jmp x-- bitloop     side 0
.wrap                                       ; back to entry_point
```

What this does, in human terms:

1. Wait for the nIRQ pin to be high *first* (so we don't trigger on stale low).
2. Wait for nIRQ to go low (the ADC's "data ready" signal).
3. Generate 32 clock cycles on SCK, sampling MISO on each falling edge.
4. The 32 bits get pushed into the PIO's RX FIFO automatically (because of `set_in_shift(..., autopush=true, 32)` set up in `mcp_pio.c`).
5. Loop back to step 1.

The state machine just keeps doing this forever. Each push into the RX FIFO triggers a DREQ ("data request") to the DMA, which then copies the FIFO contents into RAM.

### Notes on PIO

- Each state machine has 4 instruction slots... wait, no, 32 (4×8 = 32 total per PIO block; the program shown above uses 6). State machines on the same PIO block share the instruction memory.
- The `.side_set 1` directive means "one bit of side-set," used here to drive SCK as a side-effect of each instruction.
- `jmppin` and the `wait jmppin` syntax refer to a pin chosen at runtime by `sm_config_set_jmp_pin` — this is how the program knows which physical pin is nIRQ.
- Clock divider: `sm_config_set_clkdiv(&c, 125)` gives a 1 MHz state machine clock on a 125 MHz system clock. With the `[1]` delays in the SCK loop, this produces roughly 250 kHz SCK on the ADC.

---

## Ping-pong DMA

A single DMA channel can be set to do a fixed number of transfers, then stop. If you want continuous capture, you need *two* channels that take turns. This is called *ping-pong* DMA.

In `mcp_pio_init`:

```c
uint dma_a = dma_claim_unused_channel(true);
uint dma_b = dma_claim_unused_channel(true);

// Channel A
channel_config_set_dreq(&cfg_a, pio_get_dreq(pio, sm, false));
channel_config_set_chain_to(&cfg_a, dma_b);      // ← when I finish, start B
channel_config_set_ring(&cfg_a, 1, dma_buff_alignment_bytes);
dma_channel_configure(dma_a, &cfg_a,
    sample_buff,                 // ← write starts at base
    &pio->rxf[sm],               // ← read from the PIO RX FIFO
    DMA_BUFF_SIZE, false);

// Channel B
channel_config_set_chain_to(&cfg_b, dma_a);      // ← when I finish, start A
channel_config_set_ring(&cfg_b, 1, dma_buff_alignment_bytes);
dma_channel_configure(dma_b, &cfg_b,
    sample_buff + DMA_BUFF_SIZE, // ← write starts at base + half
    &pio->rxf[sm],
    DMA_BUFF_SIZE, false);
```

A finishes, automatically triggers B; B finishes, automatically triggers A. The DMA hardware does this without CPU intervention.

**Ring mode** (`channel_config_set_ring`) is the magic that makes this efficient. It tells the DMA "wrap the write address back to a power-of-two-aligned base when its lower bits would overflow." Combined with the `__attribute__((aligned(...)))` on the buffer, this means we don't need to *reset* the write address between transfers — the hardware wraps automatically.

The alignment math:
```c
const uint32_t dma_buff_alignment_bytes = 31 - __builtin_clz(DMA_BUFF_SIZE*sizeof(uint32_t));
```
`__builtin_clz` is "count leading zeros" — for `DMA_BUFF_SIZE = 128` words × 4 bytes = 512 bytes, this returns `9` (since 512 = 2^9). The DMA ring config wants the *log2* of the wrap region. So each DMA channel's write pointer wraps every 512 bytes — i.e. fills its half, then restarts at the half's base.

The buffer declaration:
```c
volatile uint32_t dma_buff_adc_0[DMA_BUFF_SIZE * 2]
    __attribute__((aligned(DMA_BUFF_SIZE * sizeof(uint32_t))));
```
The `aligned` attribute ensures the array starts at a 512-byte boundary, which is necessary for the ring wraparound to work correctly.

---

## The IRQ handler

When a DMA channel finishes a transfer, it raises an IRQ on its assigned line (`DMA_IRQ_0` for ADC 0, `DMA_IRQ_1` for ADC 1). The handler:

```c
void dma_irq_handler_0(void) {
    int culprit_is_a = dma_hw->ints0 & (1u << mpio_0.dma_a);
    if (culprit_is_a) {
        dma_hw->ints0 = 0x1 << (mpio_0.dma_a);   // clear flag for A
        dma0_last_written = 1;
    } else {
        dma_hw->ints0 = 0x1 << (mpio_0.dma_b);   // clear flag for B
        dma0_last_written = 2;
    }
}
```

Three jobs only:

1. Figure out which of the two channels caused the IRQ.
2. Clear the IRQ flag (write `1` to the corresponding bit — yes, you clear by writing `1`).
3. Update `dma0_last_written` so consumers know what's new.

**That's it.** Anything else in this function will hurt the system. There was originally a sample-post-processing loop in the handler (right-shift every sample by 16 bits) — it's now commented out because post-processing belongs in the consumer.

---

## Reading from the DMA buffer

Two patterns are used in the codebase.

### Pattern 1: batch-at-a-time (used by `app_cmd_rstream`)

Wait for a half-buffer to fill, then ship the whole thing:

```c
int dma0_last_printed = dma0_last_written;

while(streaming) {
    // wait for new batch
    while(dma0_last_written == dma0_last_printed) {
        // poll for user input, service USB, etc.
    }

    // ship the half that just got written
    tud_cdc_write(mpio_0.buff + (dma0_last_written - 1) * DMA_BUFF_SIZE,
                  DMA_BUFF_SIZE * sizeof(uint32_t));
    tud_cdc_write_flush();
    dma0_last_printed = dma0_last_written;
}
```

Note the indexing: `dma0_last_written` is `1` if A finished (so we read the lower half, starting at index 0) or `2` if B finished (read the upper half, starting at index `DMA_BUFF_SIZE`). The expression `(dma0_last_written - 1) * DMA_BUFF_SIZE` evaluates to `0` or `DMA_BUFF_SIZE` accordingly.

This pattern is **easy** and **efficient** — you wake up only twice per full buffer cycle, you grab a contiguous block of memory, you ship it. Use it for streaming-style consumers where latency at the half-buffer level is fine.

### Pattern 2: sample-at-a-time (used by `core1_ictl`)

The control loop needs to react as quickly as possible — at the sample level, not at the buffer level. So instead of waiting for a half-buffer, we poll the DMA's *internal* counter:

```c
uint dma_ch_writing = (dma0_last_written == 2) ? mpio_0.dma_a : mpio_0.dma_b;
uint index_old = 0;

while(1) {
    // wait for either the buffer to flip OR for the current write index to advance
    while (lw_current == dma0_last_written &&
           (DMA_BUFF_SIZE - dma_channel_hw_addr(dma_ch_writing)->transfer_count) == index_old) {
        tight_loop_contents();
    }
    
    if(lw_current != dma0_last_written) goto start;   // buffer flipped — re-sync
    
    // process the sample at index_old (with averaging window, etc.)
    int *base = (int *)mpio_0.buff;
    int  current_buf_base = (2 - lw_current) * DMA_BUFF_SIZE;
    int  current_index    = current_buf_base + (DMA_BUFF_SIZE - dma_channel_hw_addr(dma_ch_writing)->transfer_count - 1);
    float avg = 0;
    for(int i = 0; i < 5; i++) {
        avg += base[(current_index - i) & (2*DMA_BUFF_SIZE - 1)];
    }
    avg /= 5;
    
    // ... control law ...
    
    index_old = (index_old + 1) % DMA_BUFF_SIZE;
}
```

A few subtleties:

- `dma_channel_hw_addr(ch)->transfer_count` is the *remaining* count, decremented each time DMA writes. So `DMA_BUFF_SIZE - transfer_count` is how many samples have been *written into the current half*.
- Indexing back through the window uses bitmask wrap: `& (2*DMA_BUFF_SIZE - 1)` works because the total buffer is power-of-two-sized.
- The `goto start` retries the whole setup when the buffer flips. This is the standard escape hatch when the data stops being consistent under your feet.

Use this pattern when you need per-sample latency. Use Pattern 1 otherwise.

---

## USB CDC streaming

When we want to fire ADC bytes at the host, the standard pattern is:

```c
stdio_set_driver_enabled(&stdio_usb, false);   // mute printf

// ... main streaming loop ...
// (per-half-buffer)
bsent += tud_cdc_write(mpio_0.buff + (dma0_last_written - 1) * DMA_BUFF_SIZE,
                       DMA_BUFF_SIZE * sizeof(uint32_t));
tud_cdc_write_flush();
while (tud_cdc_write_available() < CFG_TUD_CDC_TX_BUFSIZE) tud_task();

// ... at end ...
stdio_set_driver_enabled(&stdio_usb, true);    // restore printf
```

Key things to know:

1. **Mute stdio first.** Otherwise random `printf` calls from anywhere (including deep in driver code) will interleave with your binary data and the host parser will get confused.
2. **CFG_TUD_CDC_TX_BUFSIZE** is `512` (set in `CMakeLists.txt`). This is the size of the USB driver's internal buffer.
3. **`tud_cdc_write` may return less than you asked for** if the internal buffer is fuller than what's requested. You must check the return and re-send the remainder, or do what the existing code does: wait for the buffer to drain before continuing.
4. **`tud_task()` must be called periodically** during streaming to actually move bytes out over USB. The `while (... < CFG_TUD_CDC_TX_BUFSIZE) tud_task();` line does this.
5. **Restore stdio** when you're done, or future `printf` calls won't reach the host.

The host side (`script.py`) reads raw bytes and watches for the literal string `"END RAW DATA STREAM"` to know when the stream stops. The firmware emits this string *after* re-enabling stdio. Anything else printed during streaming time is corrupted data.

---

## How to extend this pattern to a new data source

Here's a checklist for adding a new high-speed data source — say, a second ADC over a custom protocol, or the mass spec's data stream coming over UART.

### 1. Decide on the trigger mechanism

You need *something* to indicate "a new sample is ready." Options:

- **A pin going low** (like nIRQ here) — best for SPI/I2C ADCs. Drive a PIO state machine with the pin as the `jmppin`.
- **A UART RX byte arriving** — UART hardware already has a DREQ. Just point a DMA channel at the UART RX FIFO with the UART DREQ as the trigger.
- **A timer expiring** — use a PWM slice with `pwm_get_dreq()` for the DREQ. Cheap and exact.
- **Software-paced** — write to a small register that has a configurable DREQ. Rarely needed.

### 2. Set up the DMA channels

Two ping-pong channels chained to each other, each pointing at a half of an aligned buffer:

```c
#define MY_DMA_BUFF_SIZE 64   // pick a power of 2

volatile uint32_t my_buff[MY_DMA_BUFF_SIZE * 2]
    __attribute__((aligned(MY_DMA_BUFF_SIZE * sizeof(uint32_t))));

uint dma_a = dma_claim_unused_channel(true);
uint dma_b = dma_claim_unused_channel(true);

const uint32_t alignment_bytes = 31 - __builtin_clz(MY_DMA_BUFF_SIZE * sizeof(uint32_t));

dma_channel_config cfg_a = dma_channel_get_default_config(dma_a);
channel_config_set_transfer_data_size(&cfg_a, DMA_SIZE_32);   // or 16 or 8, as appropriate
channel_config_set_read_increment(&cfg_a, false);             // FIFO doesn't advance
channel_config_set_write_increment(&cfg_a, true);             // RAM does
channel_config_set_ring(&cfg_a, 1, alignment_bytes);          // ring-mode wraparound
channel_config_set_dreq(&cfg_a, MY_DREQ);                     // ← your trigger source
channel_config_set_chain_to(&cfg_a, dma_b);
dma_channel_configure(dma_a, &cfg_a, my_buff, MY_FIFO_ADDR, MY_DMA_BUFF_SIZE, false);

// (mirror image for dma_b, pointing at my_buff + MY_DMA_BUFF_SIZE)
```

### 3. Pick an IRQ line

The RP2350 has two DMA IRQ lines (`DMA_IRQ_0` and `DMA_IRQ_1`). In this firmware:
- `DMA_IRQ_0` is used by ADC 0 (16-bit ISNS).
- `DMA_IRQ_1` is used by ADC 1 (24-bit TSNS/VSNS).

If you want to handle your new DMAs separately, you need a third line — which means you'll have to share a line and dispatch in the handler based on which channel raised the flag. The existing handlers in `adpc_app_funcs.c` show the pattern: check `dma_hw->ints0` against each channel's bit position.

### 4. Write your IRQ handler

Three jobs again:

```c
void my_dma_handler(void) {
    int culprit_is_a = dma_hw->ints0 & (1u << my_dma_a);
    if (culprit_is_a) {
        dma_hw->ints0 = 0x1 << my_dma_a;
        my_last_written = 1;
    } else {
        dma_hw->ints0 = 0x1 << my_dma_b;
        my_last_written = 2;
    }
}
```

Keep it short. Don't do post-processing here.

### 5. Hook up the IRQ

```c
irq_set_exclusive_handler(DMA_IRQ_0, my_dma_handler);
dma_channel_set_irq0_enabled(my_dma_a, true);
dma_channel_set_irq0_enabled(my_dma_b, true);
irq_set_enabled(DMA_IRQ_0, true);
```

If you're *sharing* the IRQ line with existing handlers, use `irq_add_shared_handler` instead and accept the small overhead.

### 6. Consume the data

Pick Pattern 1 or 2 from above depending on whether you want batch-at-a-time or sample-at-a-time.

### 7. Add a shell command

Plumb it into `app_dispatch` and `adpc_app_funcs.c` following the conventions in [`04_Application_Layer.md`](04_Application_Layer.md).

---

## Diagnostics

When data acquisition isn't working, the typical questions and how to answer them:

| Question | How to check |
|----------|--------------|
| Is the ADC even running? | `gpio_get(nirq_pin)` should be toggling. If always high, the ADC isn't pulling it low — likely a config problem or missing pull-up. |
| Is the PIO consuming the FIFO? | Look at `pio->fstat` for the relevant SM — should see RX FIFO neither full nor empty most of the time. If consistently full, the DMA isn't draining it. |
| Is the DMA running? | `dma_channel_hw_addr(ch)->transfer_count` should be counting down. If it's stuck, the DMA isn't getting DREQs. |
| Are the IRQs firing? | Toggle a GPIO inside the IRQ handler and look at it on a scope. |
| Is the consumer seeing fresh data? | Print `dma0_last_written` from main and watch it alternate 1 → 2 → 1 → 2. |
| Are the values plausible? | Use `r0` or `r1` shell commands to dump the last few raw samples to the terminal. Decode by hand against the chip datasheet. |

A trick the firmware uses: in `core1_ictl`, the line
```c
sio_hw->gpio_hi_togl |= (1U << (ADPC_PIN_LED - 32));
```
toggles the on-board LED at the sample rate. A square wave on the scope (or on the LED if you can see it flickering predictably) confirms the loop is running at the expected frequency.
