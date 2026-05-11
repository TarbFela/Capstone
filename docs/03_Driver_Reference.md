# Driver Reference

This document covers the **low-level drivers** — the code that talks directly to hardware. There are four:

| Driver | Files | Talks to |
|--------|-------|----------|
| MCP ADC | `mcp3x6xR_driver/mcp3x6xR.[ch]` | Either MCP ADC (16-bit ISNS or 24-bit TSNS/VSNS) over SPI |
| MCP PIO | `mcp3x6xR_driver/mcp_pio.[ch]`, `mcp_pio.pio` | The same ADCs but in *streaming* mode via PIO + DMA |
| ADA4255 PGIA | `ada4255_driver/ada4255.[ch]` | The programmable-gain amplifier |
| MPHB GPIO/PWM | `ADPC_Dev/adpc_gpio_pwm.[ch]` | The MPHB-002 power boards (PWM + PH_EN pins) |

There is also a thin wrapper around the two MCP ADCs in `ADPC_Dev/ADPC_ADC.[ch]` that knows about the *specific* ADPC board (which pins, what configuration). Most app code should call into this wrapper rather than the raw MCP driver.

---

## MCP ADC driver (`mcp3x6xR_driver/`)

The MCP3x6xR family is a series of high-resolution sigma-delta ADCs with an SPI interface and a "data ready" interrupt pin. This driver provides two modes of operation:

1. **One-shot / configuration mode** via the raw SPI driver in `mcp3x6xR.[ch]`. Used during startup to write configuration registers.
2. **Continuous streaming mode** via the PIO+DMA driver in `mcp_pio.[ch]`. Used during data acquisition to capture samples without CPU intervention.

The two are mostly independent. The configuration code uses the RP2350's hardware SPI block; the streaming code uses a PIO state machine that *physically reuses the same pins* but takes them away from the SPI hardware when streaming starts. Pin ownership flips back and forth.

### Key types

```c
typedef struct mcp_info_t {
    spi_inst_t *spi;       // which SPI hardware block (spi0 or spi1)
    int mosi, miso, cs, sck, nirq, mclk;   // pin numbers
    mcp_cfg_t cfg;         // the configuration most recently written
} mcp_info_t;

typedef struct mcp_pio_t {
    bool initialized, running;
    PIO pio;               // pio0 or pio1
    uint sm;               // state machine number (0..3)
    mcp_info_t *mcp_info;  // back-pointer to the SPI-level state
    uint dma_a, dma_b;     // the two ping-pong DMA channels
    uint irqn;             // DMA_IRQ_0 or DMA_IRQ_1
    uint32_t *buff;        // pointer to the 2× DMA_BUFF_SIZE buffer
} mcp_pio_t;
```

You typically don't construct these directly. The ADPC-level wrapper (`ADPC_ADC.c`) instantiates two global pairs (`mcp_0`/`mpio_0` and `mcp_1`/`mpio_1`) and you use those.

### Key functions

**Configuration (raw SPI mode):**

```c
mcp_status_t mcp_spi_init(mcp_info_t *s, spi_inst_t *spi,
                          int mosi, int miso, int cs, int sck, int nirq, int mclk);
```
Sets up the GPIO pins for SPI, initializes the SPI block, prepares the nIRQ and MCLK pins. Does not yet start the ADC.

```c
mcp_status_t mcp_configure(mcp_info_t *s, mcp_cfg_t *cfg);
```
Writes a full set of configuration registers (CONFIG0..3, MUX, SCAN, etc.). Build the `mcp_cfg_t` by OR-ing the field macros — see `ADPC_ADC.c` for example.

```c
mcp_status_t mcp_mclk_init(mcp_info_t *s, uint32_t freq);
```
Generates the master clock for the ADC on a PWM pin. The freq parameter is in Hz; typical is 15 MHz.

**Streaming (PIO+DMA mode):**

```c
void mcp_pio_init(mcp_pio_t *s, mcp_info_t *mcp,
                  uint32_t *sample_buff, void (*dma_handler)(void));
```
Claims a PIO state machine, loads the PIO assembly program, claims two DMA channels, chains them ping-pong style, and registers your `dma_handler` for the DMA's IRQ line. **Can only be called twice** in the lifetime of the firmware — see the safety doc.

```c
void mcp_pio_start(mcp_pio_t *s);
void mcp_pio_stop(mcp_pio_t *s);
```
Hands SCK/MISO/nIRQ pin ownership to the PIO and starts the state machine (`start`) or returns ownership to SPI and stops everything (`stop`). The ADC itself must already be in continuous-conversion mode before `start` (see `adpc_adc_start` for the right sequence).

### Example: configuring the 16-bit ADC

Done once at boot in `ADPC_ADC.c`:

```c
mcp_cfg_t cfg;
gpio_pull_up(ADC_0_PIN_IRQ);          // external pull-up assumed too
mcp_spi_init(&mcp_0, ADC_0_SPI,
             ADC_0_PIN_MOSI, ADC_0_PIN_MISO,
             ADC_0_PIN_CS, ADC_0_PIN_SCK,
             ADC_0_PIN_IRQ, ADC_MCLK_PIN);

mcp_pio_init(&mpio_0, &mcp_0, dma_buff_adc_0, dma_irq_handler_0);

cfg.cfgs[0] = MCP_CFG0_VREF_SEL_INTERNAL
            | MCP_CFG0_NO_PARTIAL_SHUTDOWN
            | MCP_CFG0_CLK_SEL_INTERNAL
            | MCP_CFG0_ADC_MODE_STDBY;
cfg.cfgs[1] = MCP_CFG1_AMCLK_PRESCALE_NONE | MCP_CFG1_OSR_2048;
cfg.cfgs[2] = MCP_CFG2_BIAS_CURRENT_SEL_1 | MCP_CFG2_ADC_GAIN_SEL_1
            | MCP_CFG2_AUTO_ZERO_REF_EN | 0x1;
cfg.cfgs[3] = MCP_CFG3_CONV_MODE_CONTINUOUS | MCP4_CFG3_DATA_FORMAT_32_SGN;
cfg.input_mode = MCP_MUX_MODE;
cfg.mux_sel = MCP_MUX_P_SEL(MCP_MUX_VAL_CH0) | MCP_MUX_N_SEL(MCP_MUX_VAL_CH1);
mcp_configure(&mcp_0, &cfg);
mcp_mclk_init(&mcp_0, 15000000);
```

To start it streaming, call `adpc_adc_start(&mpio_0)` later.

### Hazards

- Don't `mcp_pio_init` more than twice. (See safety doc.)
- Configuration register meanings differ between MCP3462R (16-bit) and MCP3562R (24-bit), particularly the data format field. Use the `MCP4_*` macros for the 16-bit chip and `MCP5_*` for the 24-bit. Mismatch silently produces garbage data.
- After `mcp_pio_stop`, the pins return to GPIO_FUNC_SPI. If you want to do something else with them (e.g. use them for unrelated SPI traffic), set the pin function explicitly.

---

## ADPC ADC wrapper (`ADPC_Dev/ADPC_ADC.[ch]`)

Thin board-aware wrapper that does the right configuration for the *specific ADCs on the ADPC board*. This is what application code should call, not the raw MCP driver.

### Globals

```c
extern mcp_info_t mcp_0;   extern mcp_pio_t mpio_0;   // 16-bit ISNS ADC
extern mcp_info_t mcp_1;   extern mcp_pio_t mpio_1;   // 24-bit TSNS/VSNS ADC
```

### Functions

```c
int adpc_adc_init(void (*dma_handler_1)(void), void (*dma_handler_0)(void));
```
Configures both ADCs. You provide the two DMA interrupt handlers (your code in `adpc_app_funcs.c` does this). Returns `0` on success.

```c
int adpc_adc_start(mcp_pio_t *s);
```
Brings the specified ADC to streaming-ready state: writes the "start conversions" command, waits for the first data-ready pulse, performs the first read, and hands pin ownership to the PIO state machine. After this returns, DMA samples are flowing into RAM.

### Note on the 24-bit ADC's scan mode

ADC 1 is configured in *scan mode* with `MCP_SCAN_SEL_BIT_DIFF_A | MCP_SCAN_SEL_BIT_DIFF_B`. This means the chip cycles through the two differential channel pairs by itself, emitting one sample per channel per cycle. The data format `MCP5_CFG3_DATA_FORMAT_32_CHID_SGN4_24` packs the channel ID into the upper 4 bits of each 32-bit word, so when you read a sample you can tell which channel it came from. The host-side `script.py` decodes this.

---

## ADA4255 PGIA driver (`ada4255_driver/`)

The ADA4255 is a programmable-gain instrumentation amplifier with an SPI configuration interface. It sits on the same SPI bus as the 24-bit ADC (different CS pin) and amplifies the thermocouple signal before it hits the ADC.

The driver is simple — it's almost entirely register macros plus a thin SPI wrapper.

### Key functions

```c
void ada_spi_init(ada_info_t *s, spi_inst_t *spi,
                  int mosi, int miso, int cs, int sck);
```
Configures the SPI for the PGIA. Run **after** you've configured the SPI for ADC 1 (they share the bus) but **before** any ADC streaming — both can't talk on the bus at the same time.

```c
int ada_input_select(ada_info_t *s, uint input);
```
Selects which input pair the PGIA reads from. Values: `ADA_INPUT_1`, `ADA_INPUT_2`, `ADA_INPUT_TEST_MUX`, `ADA_INPUT_SHORT`. Use `ADA_INPUT_SHORT` for offset calibration; use `ADA_INPUT_TEST_MUX` for testing.

```c
int ada_input_gain_select(ada_info_t *s, uint input_gain);
```
Sets the PGIA gain. Values range from `ADA_INPUT_GAIN_DIV_16` through `ADA_INPUT_GAIN_128`. The thermocouple front-end uses `ADA_INPUT_GAIN_32`.

```c
uint8_t ada_read_reg(ada_info_t *s, uint8_t addr);
void ada_write_reg(ada_info_t *s, uint8_t addr, uint8_t val);
```
Direct register access. Useful for reading status bits or for any function not yet wrapped in a helper.

```c
uint8_t ada_check_digital_error(ada_info_t *s);
void ada_clear_digital_error(ada_info_t *s, uint8_t bits);
uint8_t ada_check_analog_error(ada_info_t *s);
void ada_clear_analog_error(ada_info_t *s, uint8_t bits);
```
The PGIA latches digital and analog error conditions in two status registers. These read and clear them. Worth polling periodically if you're tracking unusual fault behavior.

### Example: ADPC initialization

```c
ada_info_t ada;
ada_spi_init(&ada, PGIA_SPI,
             PGIA_PIN_MOSI, PGIA_PIN_MISO,
             PGIA_PIN_CS, PGIA_PIN_SCK);

ada_input_select(&ada, ADA_INPUT_1);
ada_input_gain_select(&ada, ADA_INPUT_GAIN_32);
```

### Hazards

- The PGIA shares its SPI bus with ADC 1. If you call `ada_*` functions while the PIO is streaming ADC 1, the bus contention will corrupt both. **Stop the ADC PIO before reconfiguring the PGIA.**
- Default gain after a soft reset is 1, not 32. Re-apply gain after any reset.

---

## MPHB GPIO/PWM driver (`ADPC_Dev/adpc_gpio_pwm.[ch]`)

This is the highest-stakes driver — it controls the actual high-current power output. Read [`02_Safety_and_Hazards.md`](02_Safety_and_Hazards.md) before using it for anything beyond what's already in the firmware.

The MPHB-002 boards expose three pin types per channel:
- Two PWM pins (e.g. `PWM_B_1_PIN`, `PWM_D_1_PIN` for channel `HB1B`) for the two buck converter halves.
- One `PH_EN` pin that gates the power stage.

The RP2350's PWM peripheral assigns these to *slices* and *channels*. Each slice has two channels (A and B), and one slice drives both PWMs for one MPHB channel.

### Key type

```c
typedef enum {HB1A, HB2A, HB3A, HB1B, HB2B, HB3B} mphb_port_t;
```

Six possible channels. The "A" variants are the *first* H-bridge on each MPHB-002 board and the "B" variants are the *second*.

### Initialization

```c
void mphb_gpio_init(mphb_port_t i);
```
Set up the PWM hardware and the `PH_EN` pin for one channel. The PWM is configured at 100 kHz with a wrap count of `MPHB_PWM_WRAP = 1000` (so "level" values range 0–1000 representing 0–100% duty). PWM is initialized **disabled**, PH_EN is initialized **low**. Call this for every channel you want to use before any of the level/enable functions below.

```c
void mphb_setup_multiphase_masked(uint32_t phases_mask);
```
Assigns equal phase offsets across the channels indicated by the mask. With 3 active channels the phase shifts are 0°, 120°, 240° — this reduces ripple and EMI by interleaving the switching events. Pass a mask like `(1U << HB1B) | (1U << HB2B) | (1U << HB3B)`.

### Level control (output)

```c
void mphb_set_levels(mphb_port_t i, uint level_A, uint level_C);
void mphb_set_levels_all(uint level_A, uint level_C);
```
Set the two PWM levels directly. Each is 0..1000. **Symmetric levels (`level_A == level_C`) produce zero net output.** Asymmetric levels produce a net voltage proportional to their difference.

```c
void mphb_set_dlevel(mphb_port_t i, int dlevel);
void mphb_set_dlevel_all(int dlevel);
```
Set a **differential** level about a common-mode duty cycle (default 50%). `dlevel = 0` → zero output. Positive `dlevel` → drive one way; negative → drive the other. This is the preferred interface — it's symmetric about the common mode and easier to reason about.

```c
void mphb_set_dlevel_all_spatial_dithering(float d);
```
Distributes the requested level *unevenly* across active phases to get sub-integer effective resolution. The input is a *fraction*, not an integer — typical range `-0.3` to `+0.3` (which the closed-loop controller clamps to). This is what `core1_ictl()` uses to push the controller's chatter down below the thermocouple noise floor.

### Enable control

```c
void mphb_set_ph_en(mphb_port_t i, bool enable);
void mphb_set_ph_en_all(bool enable);
```
The PH_EN gate. Toggles the power stage on/off without disturbing the PWM signal.

```c
void mphb_set_pwm_en(mphb_port_t i, bool enable);
void mphb_set_pwm_en_all(bool enable);
```
Enables/disables the PWM peripheral itself. When PWM is disabled, the output pins idle low. When re-enabling, the counter resumes from where it left off (you probably want to also reapply the multiphase setup).

### A typical safe startup sequence

```c
// 1. Initialize each channel you intend to use
mphb_gpio_init(HB1B);
mphb_gpio_init(HB2B);
mphb_gpio_init(HB3B);

// 2. Phase-shift them for multiphase operation
mphb_setup_multiphase_masked((1U << HB1B) | (1U << HB2B) | (1U << HB3B));

// 3. Make sure outputs are at zero before enabling power
mphb_set_dlevel_all(0);

// 4. Enable PWM (output pins now show 50%/50% duty — zero differential)
mphb_set_pwm_en_all(true);

// 5. THEN enable the power stage
mphb_set_ph_en_all(true);

// 6. Now you can change levels safely
mphb_set_dlevel_all(20);    // small positive output
```

To shut down safely, reverse the order: drop levels to zero, disable PH_EN, then disable PWM.

### Hazards

- Re-read [§1 of the safety doc](02_Safety_and_Hazards.md#1-always-disable-ph_en-before-reconfiguring-pwm).
- `mphb_set_pwm_en_all` only operates on `initialized` channels — channels you haven't `mphb_gpio_init`ed are silently skipped. This is usually what you want, but worth knowing.
- The "multiphase" setup is recomputed from scratch each call. If you bring a new channel online mid-run, you must call `mphb_setup_multiphase_masked` again with the full set of active channels.

---

## VSNS gain switch (`ADPC_Dev/adpc_vsns.[ch]`)

A tiny driver for the analog switch that selects between two gain settings on the voltage-sense amplifier.

```c
void adpc_vsns_init();
void adpc_vsns_select_gain(int gain_sel);
```

`gain_sel` is `VSNS_AMP_GAIN_SEL_0V196` or `VSNS_AMP_GAIN_SEL_0V476` (defined in `ADPC_cfg.h`). Silently does nothing if you pass anything else.

That's the whole driver. Useful as an example of the minimal pattern for a single-purpose driver — a `_init()` and one or two helpers.
