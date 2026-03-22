// Pin definitions for the breadboard MCP3462R with an RP2040-based RPi Pico W that I have on hand
#include "hardware/spi.h"
#include "hardware/i2c.h"

#define ADC_1_PIN_MOSI      11
#define ADC_1_PIN_MISO      12
#define ADC_1_PIN_CS        13
#define ADC_1_PIN_SCK       10
#define ADC_1_PIN_IRQ       15
#define ADC_1_SPI           spi1

