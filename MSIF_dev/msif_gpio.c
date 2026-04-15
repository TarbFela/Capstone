#include "msif_gpio.h"

#include "hardware/gpio.h"

#include "MSIF_cfg.h"

// Configure all MSIF digital I/O pins.
// Outputs default to 0 -> MOSFET off -> QMS signal inactive.
void msif_gpio_init(void) {
    // Digital inputs (no internal pulls — external 10k on U21 inputs)
    gpio_init(MSIF_DI_EMIS_OK_PIN);
    gpio_set_dir(MSIF_DI_EMIS_OK_PIN, GPIO_IN);

    gpio_init(MSIF_DI_SCAN_IN_PROGRESS_PIN);
    gpio_set_dir(MSIF_DI_SCAN_IN_PROGRESS_PIN, GPIO_IN);

    // Digital outputs — default all to 0 (MOSFET off, QMS signal inactive)
    #define MSIF_INIT_OUT(pin) do {         \
        gpio_init(pin);                     \
        gpio_set_dir(pin, GPIO_OUT);        \
        gpio_put(pin, 0);                   \
    } while (0)

    MSIF_INIT_OUT(MSIF_DO_ON_LINE_PIN);
    MSIF_INIT_OUT(MSIF_DO_RESET_SCAN_PIN);
    MSIF_INIT_OUT(MSIF_DO_CLR_EC_PIN);
    MSIF_INIT_OUT(MSIF_DO_SPEED_0_PIN);
    MSIF_INIT_OUT(MSIF_DO_SPEED_1_PIN);
    MSIF_INIT_OUT(MSIF_DO_SPEED_2_PIN);
    MSIF_INIT_OUT(MSIF_DO_SPEED_3_PIN);
    MSIF_INIT_OUT(MSIF_DO_MODE_0_PIN);
    MSIF_INIT_OUT(MSIF_DO_MODE_1_PIN);
    MSIF_INIT_OUT(MSIF_DO_MODE_2_PIN);
    MSIF_INIT_OUT(MSIF_DO_GAIN_0_PIN);
    MSIF_INIT_OUT(MSIF_DO_GAIN_1_PIN);
    MSIF_INIT_OUT(MSIF_DO_RANGE_0_PIN);
    MSIF_INIT_OUT(MSIF_DO_RANGE_1_PIN);

    #undef MSIF_INIT_OUT
}
