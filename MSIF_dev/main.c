#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"   // +++ digital I/O
#include "hardware/clocks.h"
#include "pico/multicore.h"

// +++ ======================================================================
// +++ MSIF pin definitions (from Schematic_MSIF_2026-04-11.pdf)
// +++ Digital inputs from QMS-112, via U21 (74AUP2G14DW inverting Schmitt).
// +++ Because U21 inverts: QMS active-low signals read active-HIGH at the GPIO.
// +++   gpio_get(MSIF_EMIS_OK_IN_PIN) == 1  --> emission is OK
// +++   gpio_get(MSIF_SCAN_IN_PROGRESS_IN_PIN) == 1  --> scan in progress
// +++ External 10k pull-ups on U21 inputs (R48, R49) — no internal pulls needed.
// +++ ======================================================================
#define MSIF_EMIS_OK_IN_PIN           30
#define MSIF_SCAN_IN_PROGRESS_IN_PIN  31

// +++ ======================================================================
// +++ MSIF digital outputs (from Schematic_MSIF_2026-04-11.pdf)
// +++ Driven through BSS138AKS-QX N-channel MOSFETs for 3.3V->12V level shift.
// +++   gpio_put(pin, 1) -> MOSFET ON  -> QMS signal pulled LOW  (active)
// +++   gpio_put(pin, 0) -> MOSFET OFF -> QMS pull-up holds HIGH (inactive)
// +++ Same "active = 1 at GPIO" convention as the inputs.
// +++ ======================================================================
#define MSIF_ON_LINE_PIN              16
#define MSIF_SPEED_3_PIN              17
#define MSIF_GAIN_1_PIN               18
#define MSIF_GAIN_0_PIN               19
#define MSIF_MODE_1_PIN               20
#define MSIF_MODE_0_PIN               21
#define MSIF_RANGE_0_PIN              22
#define MSIF_MODE_2_PIN               23
#define MSIF_CLR_EC_PIN               24
#define MSIF_RANGE_1_PIN              25
#define MSIF_RESET_SCAN_PIN           26
#define MSIF_SPEED_0_PIN              27
#define MSIF_SPEED_1_PIN              28
#define MSIF_SPEED_2_PIN              29

// +++ Configure all MSIF digital I/O pins.
static void msif_gpio_init(void) {
    // Digital inputs (no internal pulls — external 10k on U21 inputs)
    gpio_init(MSIF_EMIS_OK_IN_PIN);
    gpio_set_dir(MSIF_EMIS_OK_IN_PIN, GPIO_IN);

    gpio_init(MSIF_SCAN_IN_PROGRESS_IN_PIN);
    gpio_set_dir(MSIF_SCAN_IN_PROGRESS_IN_PIN, GPIO_IN);

    // Digital outputs — default all to 0 (MOSFET off, QMS signal inactive)
    #define INIT_OUT(pin) do {              \
        gpio_init(pin);                     \
        gpio_set_dir(pin, GPIO_OUT);        \
        gpio_put(pin, 0);                   \
    } while (0)

    INIT_OUT(MSIF_ON_LINE_PIN);
    INIT_OUT(MSIF_RESET_SCAN_PIN);
    INIT_OUT(MSIF_CLR_EC_PIN);
    INIT_OUT(MSIF_SPEED_0_PIN);
    INIT_OUT(MSIF_SPEED_1_PIN);
    INIT_OUT(MSIF_SPEED_2_PIN);
    INIT_OUT(MSIF_SPEED_3_PIN);
    INIT_OUT(MSIF_MODE_0_PIN);
    INIT_OUT(MSIF_MODE_1_PIN);
    INIT_OUT(MSIF_MODE_2_PIN);
    INIT_OUT(MSIF_GAIN_0_PIN);
    INIT_OUT(MSIF_GAIN_1_PIN);
    INIT_OUT(MSIF_RANGE_0_PIN);
    INIT_OUT(MSIF_RANGE_1_PIN);

    #undef INIT_OUT
}

// +++ Read all MSIF digital I/O pins and print state over USB serial.
static void msif_print_digital_io(void) {
    // Inputs (via U21 inverter — 1 = active at QMS)
    printf("IN : EMIS_OK=%d  SCAN_IN_PROGRESS=%d\n",
           (int)gpio_get(MSIF_EMIS_OK_IN_PIN),
           (int)gpio_get(MSIF_SCAN_IN_PROGRESS_IN_PIN));

    // Outputs (via BSS138 — 1 = MOSFET on = signal active at QMS)
    printf("OUT: ON_LINE=%d  RESET_SCAN=%d  CLR_EC=%d  "
           "SPEED=%d%d%d%d  MODE=%d%d%d  GAIN=%d%d  RANGE=%d%d\n",
           (int)gpio_get(MSIF_ON_LINE_PIN),
           (int)gpio_get(MSIF_RESET_SCAN_PIN),
           (int)gpio_get(MSIF_CLR_EC_PIN),
           (int)gpio_get(MSIF_SPEED_3_PIN), (int)gpio_get(MSIF_SPEED_2_PIN),
           (int)gpio_get(MSIF_SPEED_1_PIN), (int)gpio_get(MSIF_SPEED_0_PIN),
           (int)gpio_get(MSIF_MODE_2_PIN),  (int)gpio_get(MSIF_MODE_1_PIN),
           (int)gpio_get(MSIF_MODE_0_PIN),
           (int)gpio_get(MSIF_GAIN_1_PIN),  (int)gpio_get(MSIF_GAIN_0_PIN),
           (int)gpio_get(MSIF_RANGE_1_PIN), (int)gpio_get(MSIF_RANGE_0_PIN));
}

int main(void) {
                        /*** INITS ***/
    stdio_init_all();
    sleep_ms(1000);

    msif_gpio_init();   // +++ configure all MSIF digital I/O pins

    printf("Hello Capstone World!\n");
    printf("Commands: s=status, o=toggle ON_LINE, q=BOOTSEL\n");


    while(1) {
        int uii = stdio_getchar_timeout_us(100);
        if(uii != PICO_ERROR_TIMEOUT) {
            char ui = (char)uii;
            // QUIT
            if (ui == 'q') break;
            // +++ STATUS: print all digital I/O state
            else if (ui == 's') msif_print_digital_io();
            // +++ TOGGLE ON_LINE: flip MSIF_ON_LINE_PIN and print new state
            else if (ui == 'o') {
                gpio_xor_mask(1u << MSIF_ON_LINE_PIN);
                printf("ON_LINE=%d\n", (int)gpio_get(MSIF_ON_LINE_PIN));
            }
            else printf("Input recieved! [s=status, o=ON_LINE, q=BOOTSEL]\n");
        }
    }

    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
    return 0;
}