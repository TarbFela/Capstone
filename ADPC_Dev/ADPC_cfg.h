// Pin definitions for the ADPC-001 board (motherboard) using the Pimoroni PGA2350 daughter board

#include "hardware/spi.h"
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


