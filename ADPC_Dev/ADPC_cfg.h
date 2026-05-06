// Pin definitions for the ADPC-001 board (motherboard) using the Pimoroni PGA2350 daughter board

#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"

// MCU LED pin on ADPC
#define ADPC_PIN_LED        32

// 16-bit MCP3462R ADC: performs ISNS
#define ADC_0_PIN_MOSI      3
#define ADC_0_PIN_MISO      0
#define ADC_0_PIN_CS        5
#define ADC_0_PIN_SCK       6
#define ADC_0_SPI           spi0
#define ADC_0_PIN_IRQ       2

// 24-bit MCP3562R ADC: performs TSNS and VSNS functionalitites
#define ADC_1_PIN_MOSI      11
#define ADC_1_PIN_MISO      12
#define ADC_1_PIN_CS        13
#define ADC_1_PIN_SCK       14
#define ADC_1_PIN_IRQ       9
#define ADC_1_SPI           spi1

#define ADC_MCLK_PIN        1


// ADA4255 programmable-gain instrumentation amplifier: TSNS amplifier
// Sits on same SPI bus as ADC 1
#define PGIA_PIN_MOSI       ADC_1_PIN_MOSI
#define PGIA_PIN_MISO       ADC_1_PIN_MISO
#define PGIA_PIN_CS         7
#define PGIA_PIN_SCK        ADC_1_PIN_SCK
#define PGIA_SPI            ADC_1_SPI

#define PGIA_PIN_GPIO3_FAULT 8
#define PGIA_PIN_GPIO2_CALIB 4

// MCP9808 I2C temperature sensor
#define TMP_SNS_PIN_SDA 10
#define TMP_SNS_PIN_SCL 15
#define TMP_SNS_I2C i2c1

// MPHB-002 Signals
// Headers labelled as follows (a carry-over from MPHB-001 which had four bucks per board (ABCD) as opposed to two
//      HB-1A --> PWM_AC_1
//      HB-1B --> PWM_BD_1
//      HB-2A --> PWM_AC_2
//      HB-2B --> PWM_BD_2
//      HB-3A --> PWM_AC_3
//      HB-3B --> PWM_BD_3

#define PWM_A_1_PIN 18
#define PWM_C_1_PIN 19
#define PWM_B_1_PIN 20
#define PWM_D_1_PIN 21
#define PWM_C_2_PIN 22
#define PWM_A_2_PIN 23
#define PWM_D_2_PIN 24
#define PWM_B_2_PIN 25
#define PWM_B_3_PIN 26
#define PWM_D_3_PIN 27
#define PWM_A_3_PIN 28
#define PWM_C_3_PIN 29

#define	PH_EN_A2_PIN 35
#define	PH_EN_B2_PIN 36
#define	PH_EN_B1_PIN 37
#define	PH_EN_A3_PIN 38
#define	PH_EN_A1_PIN 39
#define	PH_EN_B3_PIN 40

#define PWM_A_1_CHAN PWM_CHAN_A
#define PWM_C_1_CHAN PWM_CHAN_B
#define PWM_B_1_CHAN PWM_CHAN_A
#define PWM_D_1_CHAN PWM_CHAN_B
#define PWM_C_2_CHAN PWM_CHAN_A
#define PWM_A_2_CHAN PWM_CHAN_B
#define PWM_D_2_CHAN PWM_CHAN_A
#define PWM_B_2_CHAN PWM_CHAN_B
#define PWM_B_3_CHAN PWM_CHAN_A
#define PWM_D_3_CHAN PWM_CHAN_B
#define PWM_A_3_CHAN PWM_CHAN_A
#define PWM_C_3_CHAN PWM_CHAN_B

#define PWM_AC_1_SLICE 1
#define PWM_BD_1_SLICE 2
#define PWM_AC_2_SLICE 3
#define PWM_BD_2_SLICE 4
#define PWM_BD_3_SLICE 5
#define PWM_AC_3_SLICE 6

