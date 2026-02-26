#ifndef MCP3X6XR_H
#define MCP3X6XR_H

#include <stdint.h>


/*
 * Commands are either Incremental Write, Incremental Read, Static Read, and Fast command
 * All commands start with a COMMAND BYTE (8 bits)
 * During every MOSI COMMAND byte, a STATUS is read out on MISO. This may be ignored or may be used.
 *
 * COMMAND BYTE:
 * bits 7:6 are the device address; this is hard-coded and may be found according to the package marking.
 * It *seems* like it's always `01`
 *
 */

#define MCP_CMD_DEV_ADDR (0x1<<6)

/*  TABLE 6-2: COMMAND TYPES TABLE
 *  10 10 00 ADC Conversion Start/Restart Fast Command (overwrites ADC_MODE[1:0] = 11)
 *  10 11 00 ADC Standby Mode Fast Command (overwrites ADC_MODE[1:0] = 10)
 *  11 00 00 ADC Shutdown Mode Fast Command (overwrites ADC_MODE[1:0] = 00)
 *  11 01 00 Full Shutdown Mode Fast Command (overwrites CONFIG0[7:0] = 0x00 and places the part in Full shutdown Mode)
 *  11 10 00 Device Full Reset Fast Command (resets the entire register map to default value)
 *  11 11 00 Don’t Care
 *  ADDR 01 Static Read of Register Address, ADDR
 *  ADDR 10 Incremental Write Starting at Register Address, ADDR
 *  ADDR 11 Incremental Read Starting at Register Address, ADDR
 */
#define MCP_CMD_ADC_CONV_START_FAST         0x28
#define MCP_CMD_ADC_STANDBY_FAST            0x2C
#define MCP_CMD_ADC_SHUTDOWN_FAST           0x30
#define MCP_CMD_ADC_FULL_SHUTDOWN_FAST      0x34
#define MCP_CMD_ADC_FULL_RESET_FAST         0x38
#define MCP_CMD_ADC_REG_READ_STAT(addr)     (((addr)<<2)|0x1)
#define MCP_CMD_ADC_REG_READ_INCR(addr)     (((addr)<<2)|0x3)
#define MCP_CMD_ADC_REG_WRITE_INCR(addr)    (((addr)<<2)|0x2)
#define MCP_CMD_DUMMY                       0x3C

/*  TABLE 8-1: INTERNAL REGISTERS SUMMARY
    0x0 ADCDATA 4/24/32 R Latest A/D conversion data output value (24 or 32 bits depending on DATA_FORMAT[1:0]) or modulator output stream (4-bit wide) in MDAT Output mode
    0x1 CONFIG0 8 R/W ADC Operating mode, Master Clock mode and Input Bias Current Source mode
    0x2 CONFIG1 8 R/W Prescale and OSR settings
    0x3 CONFIG2 8 R/W ADC boost and gain settings, auto-zeroing settings for analog multiplexer, voltage reference and ADC
    0x4 CONFIG3 8 R/W Conversion mode, data and CRC format settings; enable for CRC on communications, enable for digital offset and gain error calibrations
    0x5 IRQ 8 R/W IRQ Status bits and IRQ mode settings; enable for Fast commands and for conversion start pulse
    0x6 MUX 8 R/W Analog multiplexer input selection (MUX mode only)
    0x7 SCAN 24 R/W SCAN mode settings
    0x8 TIMER 24 R/W Delay value for TIMER between SCAN cycles
    0x9 OFFSETCAL 24 R/W ADC digital offset calibration value
    0xA GAINCAL 24 R/W ADC digital gain calibration value
    0xB RESERVED 24 R/W Reserved
    0xC RESERVED 8 R/W Reserved
    0xD LOCK 8 R/W Password value for SPI Write mode locking
    0xE RESERVED 16 R/W Reserved
    0xF CRCCFG 16 R CRC checksum for device configuration
 */
#define MCP_REG_ADDR_ADCDATA    0x0
#define MCP_REG_ADDR_CONFIG0    0x1
#define MCP_REG_ADDR_CONFIG1    0x2
#define MCP_REG_ADDR_CONFIG2    0x3
#define MCP_REG_ADDR_CONFIG3    0x4
#define MCP_REG_ADDR_IRQ        0x5
#define MCP_REG_ADDR_MUX        0x6
#define MCP_REG_ADDR_SCAN       0x7
#define MCP_REG_ADDR_TIMER      0x8
#define MCP_REG_ADDR_OFFSETCAL  0x9
#define MCP_REG_ADDR_GAINCAL    0xA
#define MCP_REG_ADDR_LOCK       0xD
#define MCP_REG_ADDR_CRCCFG     0xF

/*
 * STATUS BYTE
 * STAT[5:4] = DEV_ADDR[1:0]
 * STAT[3] = ~DEV_ADDR[0]
 *
 * With a dev_addr of 0x1, we would expect: b010
 *
 * STAT[2] = DR_STATUS (ADC data ready interrupt status)
 * STAT[1] = CRCCFG_STATUS (CRC checksum error on the register map interrupt status)
 * STAT[0] = POR_STATUS (POR interrupt status)
 */

#define MCP_STAT_DR_STATUS_MASK 0x4
#define MCP_STAT_CRCCFG_STATUS_MASK 0x2
#define MCP_STAT_POR_STATUS_MASK 0x1

/*======================================*
 *      CONFIGURATION REGISTERS         *
 *======================================*/


/*------------------------------*
 *            CONFIG 3          *
 *------------------------------*/

/* bit 7-6 CONV_MODE[1:0]: Conversion Mode Selection
    * 11 = Continuous Conversion mode or continuous conversion cycle in SCAN mode.
    * 10 = One-shot conversion or one-shot cycle in SCAN mode. It sets ADC_MODE[1:0] to ‘10’ (standby) at
    * the end of the conversion or at the end of the conversion cycle in SCAN mode.
    * 0x = One-shot conversion or one-shot cycle in SCAN mode. It sets ADC_MODE[1:0] to ‘0x’ (ADC
    * Shutdown) at the end of the conversion or at the end of the conversion cycle in SCAN mode (default).
    */
#define MCP_CFG3_CONV_MODE_CONTINUOUS 0x3<<6
#define MCP_CFG3_CONV_MODE_ONE_SHOT_STDBY 0x2<<6
#define MCP_CFG3_CONV_MODE_ONE_SHOT_SHTDWN 0x0

/* bit 5-4 DATA_FORMAT[1:0]: ADC Output Data Format Selection
    * 11 = 32-bit (25-bit right justified data + Channel ID): CHID[3:0] + SGN extension (4 bits) + 24-bit ADC
    * data. It allows overrange with the SGN extension.
    * 10 = 32-bit (25-bit right justified data): SGN extension (8-bit) + 24-bit ADC data. It allows overrange with
    * the SGN extension.
    * 01 = 32-bit (24-bit left justified data): 24-bit ADC data + 0x00 (8-bit). It does not allow overrange (ADC
    * code locked to 0xFFFFFF or 0x800000).
    * 00 = 24-bit (default ADC coding): 24-bit ADC data. It does not allow overrange (ADC code locked to
    * 0xFFFFFF or 0x800000).
    */
#define MCP_CFG3_DATA_FORMAT_CHID_SGN4_24 0x3<<4
#define MCP_CFG3_DATA_FORMAT_SNG8 0x2<<4
#define MCP_CFG3_DATA_FORMAT_LJ32 0x1<<4
#define MCP_CFG3_DATA_FORMAT_24 0x0

/* bit 3 CRC_FORMAT: CRC Checksum Format Selection on Read Communications
    * (it does not affect CRCCFG coding)
    * 1 = 32-bit wide (CRC-16 followed by 16 zeros).
    * 0 = 16-bit wide (CRC-16 only) (default).
    */
#define MCP_CFG3_CRC_FORMAT_32 0x1<<3
#define MCP_CFG3_CRC_FORMAT_16 0x0

/* bit 2 EN_CRCCOM: CRC Checksum Selection on Read Communications
    * (it does not affect CRCCFG calculations)
    * 1 = CRC on communications enabled.
    * 0 = CRC on communications disabled (default).
    */
#define MCP_CFG3_CRC_EN 0x1<<2
#define MCP_CFG3_CRC_DIS 0x0
/* bit 1 EN_OFFCAL: Enable Digital Offset Calibration
    * 1 = Enabled.
    * 0 = Disabled (default).
    */
#define MCP_CFG3_OFFSET_CAL_EN 0x1<<1
#define MCP_CFG3_OFFSET_CAL_DIS 0x0

/* bit 0 EN_GAINCAL: Enable Digital Gain Calibration
    * 1 = Enabled.
    * 0 = Disabled (default).
*/
#define MCP_CFG3_GAIN_CAL_EN 0x1
#define MCP_CFG3_GAIN_CAL_DIS 0x0

/*------------------------------*
 *            CONFIG 2          *
 *------------------------------*/

/*
bits 7-6 BOOST[1:0]: ADC Bias Current Selection
11 = ADC channel has current x 2
10 = ADC channel has current x 1 (default)
01 = ADC channel has current x 0.66
00 = ADC channel has current x 0.5
 */
#define MCP_CFG2_BIAS_CURRENT_SEL_2     0x3<<6
#define MCP_CFG2_BIAS_CURRENT_SEL_1     0x2<<6
#define MCP_CFG2_BIAS_CURRENT_SEL_066   0x1<<6
#define MCP_CFG2_BIAS_CURRENT_SEL_05    0x0
/*
bits 5-3 GAIN[2:0]: ADC Gain Selection
111 = Gain is x64 (x16 analog, x4 digital)
110 = Gain is x32 (x16 analog, x2 digital)
101 = Gain is x16
100 = Gain is x8
011 = Gain is x4
010 = Gain is x2
001 = Gain is x1 (default)
000 = Gain is x1/3
 */
#define MCP_CFG2_ADC_GAIN_SEL_64    0x7<<3
#define MCP_CFG2_ADC_GAIN_SEL_32    0x6<<3
#define MCP_CFG2_ADC_GAIN_SEL_16    0x5<<3
#define MCP_CFG2_ADC_GAIN_SEL_8     0x4<<3
#define MCP_CFG2_ADC_GAIN_SEL_4     0x3<<3
#define MCP_CFG2_ADC_GAIN_SEL_2     0x2<<3
#define MCP_CFG2_ADC_GAIN_SEL_1     0x1<<3
#define MCP_CFG2_ADC_GAIN_SEL_033   0x0
/*
bit 2 AZ_MUX: Auto-Zeroing MUX Setting
1 = ADC auto-zeroing algorithm is enabled. This setting multiplies the conversion time by two and
does not allow Continuous Conversion mode operation (which is then replaced by a series of
consecutive One-Shot mode conversions).
0 = Analog input multiplexer auto-zeroing algorithm is disabled (default).
 */
#define MCP_CFG2_AUTO_ZERO_MUX_EN    1<<2
#define MCP_CFG2_AUTO_ZERO_MUX_DIS   0x0
/*
bit 1 AZ_REF: Auto-Zeroing Reference Buffer Setting
1 = Internal voltage reference buffer chopping algorithm is enabled. This setting has no effect
when external voltage reference is selected (VREF_SEL = 0) (default).
0 = Internal voltage reference buffer chopping auto-zeroing algorithm is disabled.
 */
#define MCP_CFG2_AUTO_ZERO_REF_EN 0x1<<1
#define MCP_CFG2_AUTO_ZERO_REF_DIS 0x0
/*
bit 0 RESERVED: Should always be equal to ‘1’
 */


/*------------------------------*
 *            CONFIG 1          *
 *------------------------------*/

/*
bits 7-6 PRE[1:0]: Prescaler Value Selection for AMCLK
11 = AMCLK = MCLK/8
10 = AMCLK = MCLK/4
01 = AMCLK = MCLK/2
00 = AMCLK = MCLK (default)
*/
#define MCP_CFG1_AMCLK_PRESCALE_DIV_8   0x3<<6
#define MCP_CFG1_AMCLK_PRESCALE_DIV_4   0x2<<6
#define MCP_CFG1_AMCLK_PRESCALE_DIV_2   0x1<<6
#define MCP_CFG1_AMCLK_PRESCALE_NONE    0x0
/*
bits 5-2 OSR[3:0]: Oversampling Ratio for Delta-Sigma A/D Conversion
1111 = OSR: 98304
1110 = OSR: 81920
1101 = OSR: 49152
1100 = OSR: 40960
1011 = OSR: 24576
1010 = OSR: 20480
1001 = OSR: 16384
1000 = OSR: 8192
0111 = OSR: 4096
0110 = OSR: 2048
0101 = OSR: 1024
0100 = OSR: 512
0011 = OSR: 256 (default)
0010 = OSR: 128
0001 = OSR: 64
0000 = OSR: 32
*/
#define MCP_CFG1_OSR_98304  0xF<<2
#define MCP_CFG1_OSR_81920  0xE<<2
#define MCP_CFG1_OSR_49152  0xD<<2
#define MCP_CFG1_OSR_40960  0xC<<2
#define MCP_CFG1_OSR_24576  0xB<<2
#define MCP_CFG1_OSR_20480  0xA<<2
#define MCP_CFG1_OSR_16384  0x9<<2
#define MCP_CFG1_OSR_8192   0x8<<2
#define MCP_CFG1_OSR_4096   0x7<<2
#define MCP_CFG1_OSR_2048   0x6<<2
#define MCP_CFG1_OSR_1024   0x5<<2
#define MCP_CFG1_OSR_512    0x4<<2
#define MCP_CFG1_OSR_256    0x3<<2
#define MCP_CFG1_OSR_128    0x2<<2
#define MCP_CFG1_OSR_64     0x1<<2
#define MCP_CFG1_OSR_32     0x0
/*
bits 1-0 RESERVED[1:0]: Should always be set to ‘00’
 */


/*------------------------------*
 *            CONFIG 0          *
 *------------------------------*/


/*
bit 7 VREF_SEL: Internal Voltage Reference Bit
1 = Internal voltage reference is selected and buffered internally. REFIN+/OUT pin voltage is set at 2.4V (default).
0 = External Voltage reference is selected and not buffered internally. The internal voltage reference buffer is shut down.
*/
#define MCP_CFG0_VREF_SEL_INTERNAL      0x1<<7
#define MCP_CFG0_VREF_SEL_EXTERNAL      0x0
/*
bit 6 CONFIG0[6]: If CONFIG0 = 0x0, the device goes into Partial Shutdown mode. This bit does not have any other function.
*/
#define MCP_CFG0_PARTIAL_SHUTDOWN       0x0
#define MCP_CFG0_NO_PARTIAL_SHUTDOWN    0x1<<6
/*
bit 5-4 CLK_SEL[1:0]: Clock Selection
11 = Internal clock is selected and AMCLK is present on the analog master clock output pin.
10 = Internal clock is selected and no clock output is present on the CLK pin.
01 = External digital clock
00 = External digital clock (default)
*/
#define MCP_CFG0_CLK_SEL_INTERNAL_AMCLKOUT  0x3<<4
#define MCP_CFG0_CLK_SEL_INTERNAL           0x2<<4
#define MCP_CFG0_CLK_SEL_EXTERNAL           0x0

/*
bit 3-2 CS_SEL[1:0]: Current Source/Sink Selection Bits for Sensor Bias (source on VIN+/Sink on VIN-)
11 = 15 µA is applied to the ADC inputs
10 = 3.7 µA is applied to the ADC inputs
01 = 0.9 µA is applied to the ADC inputs
00 = No current source is applied to the ADC inputs (default)
*/
#define MCP_CFG0_CURRENT_BIAS_SEL_15uA  0x3<<2
#define MCP_CFG0_CURRENT_BIAS_SEL_3u7A  0x2<<2
#define MCP_CFG0_CURRENT_BIAS_SEL_0u9A  0x1<<2
#define MCP_CFG0_CURRENT_BIAS_SEL_0u0A  0x0
/*
bit 1-0 ADC_MODE[1:0]: ADC Operating Mode Selection
11 = ADC Conversion mode
10 = ADC Standby mode
01 = ADC Shutdown mode
00 = ADC Shutdown mode (default)
*/
#define MCP_CFG0_ADC_MODE_CONV      0x3
#define MCP_CFG0_ADC_MODE_STDBY     0x2
#define MCP_CFG0_ADC_MODE_SHTDWN    0x0


/*======================================*
 *          MUX INPUT SELECT            *
 *======================================*/

/*
Bit 7-4 MUX_VIN+[3:0]: Input Selection
Bit 3-0 MUX_VIN-[3:0]: Input Selection
    1111 = Internal VCM
    1110 = Internal Temperature Sensor Diode M (Temp Diode M)(1)
    1101 = Internal Temperature Sensor Diode P (Temp Diode P)(1)
    1100 = REFIN-
    1011 = REFIN+/OUT
    1010 = Reserved (do not use)
    1001 = AVDD
    1000 = AGND
    0111 = CH7
    0110 = CH6
    0101 = CH5
    0100 = CH4
    0011 = CH3
    0010 = CH2
    0001 = CH1 (default for mux-)
    0000 = CH0 (default for mux+)
    */
#define MCP_MUX_P_SEL(mux_val) ((mux_val)<<4)
#define MCP_MUX_N_SEL(mux_val) (mux_val)

#define MCP_MUX_VAL_Int_VCM             0xF
#define MCP_MUX_VAL_Int_Temp_Diode_M    0xE
#define MCP_MUX_VAL_Int_Temp_Diode_P    0xD
#define MCP_MUX_VAL_REFIN_N             0xC
#define MCP_MUX_VAL_REFIN_P             0xB
#define MCP_MUX_VAL_AVDD                0x9
#define MCP_MUX_VAL_AGND                0x8
#define MCP_MUX_VAL_CH7                 0x7
#define MCP_MUX_VAL_CH6                 0x6
#define MCP_MUX_VAL_CH5                 0x5
#define MCP_MUX_VAL_CH4                 0x4
#define MCP_MUX_VAL_CH3                 0x3
#define MCP_MUX_VAL_CH2                 0x2
#define MCP_MUX_VAL_CH1                 0x1
#define MCP_MUX_VAL_CH0                 0x0

typedef uint8_t mcp_status_t;

typedef struct mcp_info {
    int cs;
    int miso;
    int mosi;
    int sck;
} mcp_info_t;

mcp_status_t mcp_spi_init(mcp_info_t *s, int mosi_pin, int miso_pin, int cs_pin, int sck_pin);

mcp_status_t mcp_read_cfgn(mcp_info_t *s, uint8_t *dst, int cfg_n);

mcp_status_t mcp_write_cfgn(mcp_info_t *s, uint8_t val, int cfg_n);

mcp_status_t mcp_singe_conversion(mcp_info_t *s, uint8_t *post_status);

#endif
