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

// +++ Configure MSIF digital input pins as GPIO inputs.
static void msif_gpio_init(void) {
    gpio_init(MSIF_EMIS_OK_IN_PIN);
    gpio_set_dir(MSIF_EMIS_OK_IN_PIN, GPIO_IN);

    gpio_init(MSIF_SCAN_IN_PROGRESS_IN_PIN);
    gpio_set_dir(MSIF_SCAN_IN_PROGRESS_IN_PIN, GPIO_IN);
}

// +++ Read both MSIF digital inputs and print over USB serial.
static void msif_print_digital_inputs(void) {
    bool emis_ok      = gpio_get(MSIF_EMIS_OK_IN_PIN);
    bool scan_in_prog = gpio_get(MSIF_SCAN_IN_PROGRESS_IN_PIN);
    printf("EMIS_OK=%d  SCAN_IN_PROGRESS=%d\n",
           (int)emis_ok, (int)scan_in_prog);
}

int main(void) {
                        /*** INITS ***/
    stdio_init_all();
    sleep_ms(1000);

    msif_gpio_init();   // +++ configure MSIF digital input pins

    printf("Hello Capstone World!\n");
    printf("Commands: s=status, q=BOOTSEL\n");   // +++ advertise new 's' command


    while(1) {
        int uii = stdio_getchar_timeout_us(100);
        if(uii != PICO_ERROR_TIMEOUT) {
            char ui = (char)uii;
            // QUIT
            if (ui == 'q') break;
            // +++ STATUS: print digital input state
            else if (ui == 's') msif_print_digital_inputs();
            else printf("Input recieved! [s=status, q=BOOTSEL]\n");
        }
    }

    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
    return 0;
}