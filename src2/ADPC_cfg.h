// Pin definitions for the ADPC-001 board (motherboard) using the Pimoroni PGA2350 daughter board

// 24-bit MCP3562R ADC: performs TSNS and VSNS functionalitites
#define ADC_1_PIN_MOSI      11
#define ADC_1_PIN_MISO      12
#define ADC_1_PIN_CS        13
#define ADC_1_PIN_SCK       14
#define ADC_1_SPI           spi1

// ADA4255 programmable-gain instrumentation amplifier: TSNS amplifier
// Sits on same SPI bus as ADC 1
#define PGIA_PIN_MOSI       ADC_1_PIN_MOSI
#define PGIA_PIN_MISO       ADC_1_PIN_MISO
#define PGIA_PIN_CS         7
#define PGIA_PIN_SCK        ADC_1_PIN_SCK
#define PGIA_SPI            ADC_1_SPI
