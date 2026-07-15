/*
 * msif_adc.c — board-level wrapper for the MCP3462RT EC-voltage ADC.
 *
 * NOTE on units: this reads a VOLTAGE (the QMS post-amp output at QDP
 * EC+/EC-), not a current. The QMS-112 does I->V conversion internally
 * (manual sec 9.2.3); we just digitize the result. v_ec in the sample
 * struct is volts at the QDP EC- pin. To recover ion current in amps,
 * multiply by the per-RANGE/GAIN transfer factor from manual sec 10.2.1.1
 * / sec 9.1.2.3 — that table is not yet in firmware.
 *
 * Signal chain (MS ANALOG IN side of the MSIF Main schematic):
 *   QDP EC-  ->  U19.5 OPA4197 unity buffer  ->  EC-_BUFF
 *   QDP EC- and EC-_BUFF  ->  INA146 diff amp (U16 or U17)  ->  ADC VIN+/VIN-
 *
 * The ADC reads the differential voltage across the INA146 output pair,
 * proportional to the voltage at QDP EC- (up to MSIF_ADC_INPUT_GAIN, which
 * is a bench-measured constant — see msif_adc.h).
 *
 * SPI BUS SHARING: SPI0, SCK/MOSI/MISO shared with the DAC80504. DAC is
 * mode 1 (CPOL=0, CPHA=1), ADC is mode 0 (CPOL=0, CPHA=0). Every public
 * function here starts with claim_spi_for_adc(). msif_analog.c's public
 * functions do the mirror-image claim_spi_for_dac(). That means app code
 * can interleave msif_set_fmass_v() and msif_adc_read_ec() freely.
 *
 * OPEN ITEMS (resolve at the bench):
 *   - MUX channel assignment is a guess (CH2+/CH1- differential). If the
 *     first read returns noise or zero, try other MCP_MUX_VAL_CHn pairings.
 *   - MSIF_ADC_INPUT_GAIN is set to 1.0 until a known DC voltage is injected
 *     at QDP EC- and the ADC-input ratio is measured. Update MSIF_cfg.h
 *     once you have real numbers.
 *   - Sign convention at EC: ions on the collector produce a defined-polarity
 *     swing at the QMS post-amp output. With CH2+/CH1- wired as assumed, a
 *     positive ADC code is intended to mean "the QMS post-amp is showing
 *     ions on the collector," but the actual sign has to be confirmed at
 *     the bench against a known polarity input — and re-confirmed under
 *     real ions, because injected DC and post-amp output may differ in sign.
 */

#include "msif_adc.h"

#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include "MSIF_cfg.h"
#include "src2/mcp3x6xR_driver/mcp3x6xR.h"

/* File-static driver state — one ADC on the MSIF board. */
static mcp_info_t s_adc;
static bool s_adc_init_done = false;

/* Re-assert SPI mode 0 (CPOL=0, CPHA=0) before touching the bus. The DAC
 * driver's equivalent sits in msif_analog.c. Keep the two in lockstep if
 * either side's required mode ever changes. */
static void claim_spi_for_adc(void) {
    spi_set_format(MSIF_ADC_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
}

void msif_adc_init(void) {
    /* nIRQ: the MCP3x6xR will refuse to convert unless nIRQ is pulled high
     * externally (per the driver README). Enable the RP2350 internal pull-up
     * as a defensive measure in case the external pull is a DNP on this
     * board rev. The pin stays a GPIO input — the datasheet's open-drain
     * nIRQ output drives it low when a conversion is ready. */
    gpio_init(MSIF_ADC_IRQ_PIN);
    gpio_set_dir(MSIF_ADC_IRQ_PIN, GPIO_IN);
    gpio_pull_up(MSIF_ADC_IRQ_PIN);

    claim_spi_for_adc();

    /* mcp_spi_init re-runs spi_init at a fixed 500 kHz and re-applies mode 0.
     * It's safe to call after the DAC init because every later DAC access
     * re-asserts mode 1 anyway.
     *
     * Last positional argument is mclk_pin — only used if mcp_mclk_init() is
     * subsequently called to set up an external PWM-driven master clock for
     * the ADC. We use the chip's internal clock (MCP_CFG0_CLK_SEL_INTERNAL
     * below), so we pass -1 and never call mcp_mclk_init(). */
    mcp_spi_init(&s_adc, MSIF_ADC_SPI,
                 MSIF_DAC_SDI_PIN,   /* shared MOSI (GPIO 35) */
                 MSIF_DAC_SDO_PIN,   /* shared MISO (GPIO 36) */
                 MSIF_ADC_CS_PIN,    /* unique ADC CS# (GPIO 37) */
                 MSIF_DAC_SCK_PIN,   /* shared SCK  (GPIO 38) */
                 MSIF_ADC_IRQ_PIN,   /* nIRQ        (GPIO 32) */
                 -1);                /* no external MCLK on MSIF */

    /* Configure CFG0..CFG3 directly. We could call mcp_configure() but this
     * snapshot of the driver only writes 3 of 4 CFG registers and does a
     * broken readback check, so we bypass it and use mcp_write_regs straight
     * into CONFIG0 with n=4 (incremental write walks CFG0->CFG1->CFG2->CFG3).
     *
     *   CFG0 = internal 2.4V VREF, no partial shutdown, internal clock,
     *          no sensor bias current, ADC_MODE = conversion (fast cmds will
     *          toggle this into standby/conv as needed).
     *   CFG1 = no prescale, OSR = 256 (default — ~20 kHz sample rate).
     *   CFG2 = bias current x1, gain x1, auto-zero on (REF + MUX), bit 0 = 1
     *          (the datasheet's "reserved, must be 1" bit).
     *   CFG3 = one-shot conversion ending in standby, DATA_FORMAT = 0x0
     *          (16-bit output — mandatory for the MCP3462R on this board;
     *          leaving it at reset-default 24-bit would silently mis-align
     *          the sample byte order vs. mcp_single_conversion's uint16_t
     *          return).
     */
    uint8_t cfg[4];
    cfg[0] = MCP_CFG0_VREF_SEL_INTERNAL
           | MCP_CFG0_NO_PARTIAL_SHUTDOWN
           | MCP_CFG0_CLK_SEL_INTERNAL
           | MCP_CFG0_ADC_MODE_CONV;
    cfg[1] = MCP_CFG1_AMCLK_PRESCALE_NONE | MCP_CFG1_OSR_256;
    cfg[2] = MCP_CFG2_BIAS_CURRENT_SEL_1
           | MCP_CFG2_ADC_GAIN_SEL_1
           | MCP_CFG2_AUTO_ZERO_REF_EN 
           | MCP_CFG2_AUTO_ZERO_MUX_EN 
           | 0x03u;   /* CFG2 bit 0 reserved, must be 1 per datasheet */
    cfg[3] = MCP_CFG3_CONV_MODE_ONE_SHOT_STDBY;  /* DATA_FORMAT bits = 0x0 */

    mcp_write_regs(&s_adc, cfg, 4, MCP_REG_ADDR_CONFIG0);

    /* MUX: assumed QDP EC- path lands at CH2/CH1 differential. Bench test
     * with a known DC source at EC- will confirm this pairing — update
     * here if the schematic/bench disagree. */
    mcp_mux_sel(&s_adc, MCP_MUX_VAL_CH2, MCP_MUX_VAL_CH0);

    // 7/15 - could still check the different channels with the new wiring setup??? 

    s_adc_init_done = true;
}

bool msif_adc_read_ec(msif_adc_sample_t *sample) {
    if (!s_adc_init_done || sample == NULL) return false;

    claim_spi_for_adc();

    uint16_t raw = 0;
    uint8_t status = mcp_single_conversion(&s_adc, &raw);

    /* MCP3462R 16-bit differential output is two's-complement. Reinterpret
     * the uint16_t as int16_t to get the signed code, then scale to volts
     * against the 2.4 V internal reference. Full-scale = +/- V_REF. */
    int16_t signed_raw = (int16_t)raw;
    float   v_adc_diff = ((float)signed_raw / 32768.0f) * MSIF_ADC_V_REF;
    float   v_ec       = v_adc_diff / MSIF_ADC_INPUT_GAIN;

    sample->raw_code   = signed_raw;
    sample->v_adc_diff = v_adc_diff;
    sample->v_ec       = v_ec;
    sample->mcp_status = status;
    return true;
}

bool msif_adc_read_ec_avg(uint32_t n_samples, msif_adc_sample_t *sample) {
    if (!s_adc_init_done || sample == NULL || n_samples == 0) return false;

    claim_spi_for_adc();

    /* Accumulate signed 16-bit codes into a 32-bit sum. Headroom: int32_t
     * overflow would need > 65k samples; we'll never get anywhere close. */
    int32_t code_accum = 0;
    uint8_t status_or  = 0;
    for (uint32_t i = 0; i < n_samples; i++) {
        uint16_t raw = 0;
        uint8_t s = mcp_single_conversion(&s_adc, &raw);
        code_accum += (int32_t)(int16_t)raw;
        status_or  |= s;
    }

    int16_t mean_code  = (int16_t)(code_accum / (int32_t)n_samples);
    float   v_adc_diff = ((float)mean_code / 32768.0f) * MSIF_ADC_V_REF;
    float   v_ec       = v_adc_diff / MSIF_ADC_INPUT_GAIN;

    sample->raw_code   = mean_code;
    sample->v_adc_diff = v_adc_diff;
    sample->v_ec       = v_ec;
    sample->mcp_status = status_or;
    return true;
}

bool msif_adc_snapshot(msif_adc_snapshot_t *snap) {
    if (!s_adc_init_done || snap == NULL) return false;

    claim_spi_for_adc();

    /* Incremental read starting at CONFIG0 grabs CFG0..CFG3 in one SPI
     * transaction. IRQ and MUX are non-contiguous so they get their own
     * single-byte reads. */
    uint8_t rx[4] = {0, 0, 0, 0};

    snap->mcp_status = mcp_read_regs(&s_adc, rx, 4, MCP_REG_ADDR_CONFIG0);
    snap->config[0] = rx[0];
    snap->config[1] = rx[1];
    snap->config[2] = rx[2];
    snap->config[3] = rx[3];

    uint8_t one = 0;
    mcp_read_regs(&s_adc, &one, 1, MCP_REG_ADDR_IRQ);
    snap->irq_reg = one;

    one = 0;
    mcp_read_regs(&s_adc, &one, 1, MCP_REG_ADDR_MUX);
    snap->mux_reg = one;

    return true;
}
