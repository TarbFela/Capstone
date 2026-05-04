#include "msif_gpio.h"

#include "hardware/gpio.h"

#include "MSIF_cfg.h"

/* Canonical MSIF multi-bit output field pin tables. LSB-first.
 * Anyone needing to walk these (main.c bench CLI, msif_qms.c set-by-value
 * wrappers, msif_proto.c STATUS handler) externs them via msif_gpio.h. */
const uint msif_speed_pins[4] = {
    MSIF_DO_SPEED_0_PIN, MSIF_DO_SPEED_1_PIN,
    MSIF_DO_SPEED_2_PIN, MSIF_DO_SPEED_3_PIN
};
const uint msif_mode_pins[3] = {
    MSIF_DO_MODE_0_PIN, MSIF_DO_MODE_1_PIN, MSIF_DO_MODE_2_PIN
};
const uint msif_gain_pins[2] = {
    MSIF_DO_GAIN_0_PIN, MSIF_DO_GAIN_1_PIN
};
const uint msif_range_pins[2] = {
    MSIF_DO_RANGE_0_PIN, MSIF_DO_RANGE_1_PIN
};

uint8_t msif_gpio_read_field(const uint *pins, int nbits) {
    uint8_t v = 0;
    for (int i = 0; i < nbits; i++) {
        v |= (uint8_t)((gpio_get_out_level(pins[i]) & 1u) << i);
    }
    return v;
}

void msif_gpio_write_field(const uint *pins, int nbits, uint8_t value) {
    for (int i = 0; i < nbits; i++) {
        gpio_put(pins[i], (value >> i) & 1u);
    }
}

void msif_gpio_pulse(uint pin, uint32_t ms) {
    gpio_put(pin, 1);
    sleep_ms(ms);
    gpio_put(pin, 0);
}

/* Configure all MSIF digital I/O pins.
 * Outputs default to 0 -> MOSFET off -> QMS signal inactive. */
void msif_gpio_init(void) {
    /* Digital inputs (no internal pulls — external 10k on U21 inputs) */
    gpio_init(MSIF_DI_EMIS_OK_PIN);
    gpio_set_dir(MSIF_DI_EMIS_OK_PIN, GPIO_IN);

    gpio_init(MSIF_DI_SCAN_IN_PROGRESS_PIN);
    gpio_set_dir(MSIF_DI_SCAN_IN_PROGRESS_PIN, GPIO_IN);

    /* Digital outputs — default all to 0 (MOSFET off, QMS signal inactive) */
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
