#ifndef ADA4255_H
#define ADA4255_H

#include <stdint.h>
#include "hardware/spi.h"

#define ADA_CS_DESELECT 1
#define ADA_CS_SELECT 0

#define ADA_CMD_READ 0x1<<7
#define ADA_CMD_WRITE 0x0

/*
 * REGISTERS
 */

/*
 * Gain Mux Register
 */
#define ADA_ADDR_GAIN_MUX           0x00
#define ADA_GAIN_REG_MUX_OUTPUT_SCALING_1V375 0x1<<7
#define ADA_GAIN_REG_MUX_OUTPUT_SCALING_1   0
#define ADA_GAIN_REG_MUX_INPUT_GAIN_BITS 0x78
#define ADA_GAIN_REG_MUX_INPUT_GAIN_DIV_16  0x0<<3
#define ADA_GAIN_REG_MUX_INPUT_GAIN_DIV_8   0x1<<3
#define ADA_GAIN_REG_MUX_INPUT_GAIN_DIV_4   0x2<<3
#define ADA_GAIN_REG_MUX_INPUT_GAIN_DIV_2   0x3<<3
#define ADA_GAIN_REG_MUX_INPUT_GAIN_1       0x4<<3
#define ADA_GAIN_REG_MUX_INPUT_GAIN_2       0x5<<3
#define ADA_GAIN_REG_MUX_INPUT_GAIN_4       0x6<<3
#define ADA_GAIN_REG_MUX_INPUT_GAIN_8       0x7<<3
#define ADA_GAIN_REG_MUX_INPUT_GAIN_16      0x8<<3
#define ADA_GAIN_REG_MUX_INPUT_GAIN_32      0x9<<3
#define ADA_GAIN_REG_MUX_INPUT_GAIN_64      0xA<<3
#define ADA_GAIN_REG_MUX_INPUT_GAIN_128     0xB<<3
#define ADA_GAIN_REG_MUX_CTL_GP1              0x1<<1
#define ADA_GAIN_REG_MUX_CTL_GP0              0x1


/*
 * Reset (write-only, perform a soft reset)
 */
#define ADA_ADDR_RESET              0x01
#define ADA_RESET_REG_RESET     0x01

/*
 * CLOCK SYNCHRONIZATION CONFIGURATION REGISTER
 */
#define ADA_ADDR_SYNC_CFG           0x02

#define ADA_SYNC_CFG_REG_CLK_CP_SEL_16MHz       0x0
#define ADA_SYNC_CFG_REG_CLK_CP_SEL_8MHz        0x1<<7

// GPIO4 clock frequency (when INT_CLK_OUT = 1)
#define ADA_SYNC_CFG_REG_CLK_OUT_SEL_1MHZ       0x0
#define ADA_SYNC_CFG_REG_CLK_OUT_SEL_125KHZ     0x1<<6

// rising vs falling edge
#define ADA_SYNC_CFG_REG_SYNC_POL_RISING        0x1<<4
#define ADA_SYNC_CFG_REG_SYNC_POL_FALLING       0x0

#define ADA_SYNC_CFG_REG_INT_CLK_DIV_1          0x0
#define ADA_SYNC_CFG_REG_INT_CLK_DIV_2          0x1
#define ADA_SYNC_CFG_REG_INT_CLK_DIV_4          0x2
#define ADA_SYNC_CFG_REG_INT_CLK_DIV_8          0x3
#define ADA_SYNC_CFG_REG_INT_CLK_DIV_16         0x4
#define ADA_SYNC_CFG_REG_INT_CLK_DIV_32         0x5

/*
 * DIGITAL ERROR REGISTER
 */
#define ADA_ADDR_DIGITAL_ERR        0x03

#define ADA_DIGITAL_ERR_REG_CAL_BUSY 0x01<<6
#define ADA_DIGITAL_ERR_REG_SPI_CRC_ERR 0x1<<5
#define ADA_DIGITAL_ERR_REG_SPI_RW_ERR 0x1<<4
#define ADA_DIGITAL_ERR_REG_SPI_SCLK_CNT_ERR 0x1<<3
#define ADA_DIGITAL_ERR_REG_MM_CRC_ERR 0x1<<1
#define ADA_DIGITAL_ERR_REG_ROM_CRC_ERR 0x1

/*
 * ANALOG ERROR REGISTER
 */
#define ADA_ADDR_ANALOG_ERR         0x04

#define ADA_ANALOG_ERR_REG_G_RST    0x1<<7
#define ADA_ANALOG_ERR_REG_POR_HV   0x1<<6
#define ADA_ANALOG_ERR_REG_WB_ERR   0x1<<4
#define ADA_ANALOG_ERR_REG_FAULT_INT    0x1<<3
#define ADA_ANALOG_ERR_REG_OUTPUT_ERR   0x1<<2
#define ADA_ANALOG_ERR_REG_INPUT_ERR    0x1<<1
#define ADA_ANALOG_ERR_REG_MUX_OV_ERR   0x1

/*
 * GPIO DATA REGISTER
 */
#define ADA_ADDR_GPIO_DATA          0x05

#define ADA_GPIO_DATA_REG_GPIO(n)   (0x01<<(n))

/*
 * INTERNAL MUX CONTROL REGISTER
 */
#define ADA_ADDR_INPUT_MUX          0x06

#define ADA_INPUT_MUX_REG_SW_A1     (0x1<<6)
#define ADA_INPUT_MUX_REG_SW_A2     (0x1<<5)
#define ADA_INPUT_MUX_REG_SW_B1     (0x1<<4)
#define ADA_INPUT_MUX_REG_SW_B2     (0x1<<3)
#define ADA_INPUT_MUX_REG_SW_C1     (0x1<<2)
#define ADA_INPUT_MUX_REG_SW_C2     (0x1<<1)
#define ADA_INPUT_MUX_REG_SW_D12    0x1

/*
 *
 */
#define ADA_ADDR_WB_DETECT          0x07

/*
 * GPIO DIRECTIONS REGISTERS
 */
#define ADA_ADDR_GPIO_DIR           0x08
// 1 = output, 0 = input (default for all)
#define ADA_GPIO_DIR_REG_GPIO_OUT(n)   (0x01<<(n))

#define ADA_ADDR_SCS                0x09
#define ADA_ADDR_ANALOG_ERR_DIS     0x0A
#define ADA_ADDR_DIGITAL_ERR_DIS    0x0B

/*
 * SPECIAL FUNCTION CONFIGURATION REGISTER
 */
#define ADA_ADDR_SF_CFG             0x0C

#define ADA_SCS_REG_OSC_OUT         (0x1<<5)
#define ADA_SCS_REG_OSC_EXT_CLK_IN  (0x1<<4)
#define ADA_SCS_REG_FAULT_INT_OUT   (0x1<<3)
#define ADA_SCS_REG_CAL_BUSY_OUT    (0x1<<2)
#define ADA_SCS_REG_EXT_MUX_EN_1    (0x1<<1)
#define ADA_SCS_REG_EXT_MUX_EN_0    0x1

#define ADA_ADDR_ERR_CFG            0x0D
#define ADA_ADDR_TEST_MUX           0x0E
#define ADA_ADDR_EX_CURRENT_CFG     0x0F
#define ADA_ADDR_GAIN_CAL(n)        (0x10+(n))
#define ADA_ADDR_TRIG_CAL           0x2A
#define ADA_ADDR_M_CLK_CNT          0x2E
#define ADA_ADDR_DIE_REV_ID         0x2F
#define ADA_ADDR_PART_ID(n)         (0x64+(n))


typedef struct {
    int nothing;
} ada_config_t;

typedef struct ada_info {
    spi_inst_t *spi;
    int cs_pin;
    ada_config_t cfg;
} ada_info_t;

enum ada_inputs {ADA_INPUT_1, ADA_INPUT_2, ADA_INPUT_TEST_MUX, ADA_INPUT_SHORT};

enum ada_input_gains {ADA_INPUT_GAIN_DIV_16,
    ADA_INPUT_GAIN_DIV_8,
    ADA_INPUT_GAIN_DIV_4,
    ADA_INPUT_GAIN_DIV_2, ADA_INPUT_GAIN_1, ADA_INPUT_GAIN_2, ADA_INPUT_GAIN_4, ADA_INPUT_GAIN_8, ADA_INPUT_GAIN_16, ADA_INPUT_GAIN_32, ADA_INPUT_GAIN_64, ADA_INPUT_GAIN_128};

void ada_spi_init(ada_info_t *s, spi_inst_t *spi, int mosi_pin, int miso_pin, int cs_pin, int sck_pin);
int ada_input_select(ada_info_t *s, uint input);
int ada_input_gain_select(ada_info_t *s, uint input_gain);

uint8_t ada_read_reg(ada_info_t *s, uint8_t addr);
void ada_write_reg(ada_info_t *s, uint8_t addr, uint8_t val);

uint8_t ada_check_digital_error(ada_info_t *s);
void ada_clear_digital_error(ada_info_t *s, uint8_t bits);
uint8_t ada_check_analog_error(ada_info_t *s);
void ada_clear_analog_error(ada_info_t *s, uint8_t bits);

#endif