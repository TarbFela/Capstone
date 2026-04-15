#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"

#include "MSIF_cfg.h"
#include "msif_gpio.h"

// Pin tables for the multi-bit MSIF output fields. LSB-first.
static const uint msif_speed_pins[4] = {
    MSIF_DO_SPEED_0_PIN, MSIF_DO_SPEED_1_PIN, MSIF_DO_SPEED_2_PIN, MSIF_DO_SPEED_3_PIN
};
static const uint msif_mode_pins[3]  = {
    MSIF_DO_MODE_0_PIN, MSIF_DO_MODE_1_PIN, MSIF_DO_MODE_2_PIN
};
static const uint msif_gain_pins[2]  = {
    MSIF_DO_GAIN_0_PIN, MSIF_DO_GAIN_1_PIN
};
static const uint msif_range_pins[2] = {
    MSIF_DO_RANGE_0_PIN, MSIF_DO_RANGE_1_PIN
};

// Read a multi-bit output field from latched GPIO_OUT state (LSB-first).
// Uses gpio_get_out_level() so it's safe to call right after a write
// (avoids the 2-cycle GPIO_IN synchronizer race).
static uint8_t msif_read_field(const uint *pins, int nbits) {
    uint8_t v = 0;
    for (int i = 0; i < nbits; i++) {
        v |= (uint8_t)((gpio_get_out_level(pins[i]) & 1) << i);
    }
    return v;
}

// Write a multi-bit output field (LSB-first).
static void msif_write_field(const uint *pins, int nbits, uint8_t value) {
    for (int i = 0; i < nbits; i++) {
        gpio_put(pins[i], (value >> i) & 1);
    }
}

// Print the hello banner and command help. Called at startup AND on '?'.
// The startup copy is usually dropped because VS Code Serial Monitor
// doesn't assert DTR until after the firmware has already printed and
// the pico_stdio_usb layer silently drops output with no host connected,
// so '?' is the reliable way to retrieve this after a late connect.
static void msif_print_banner(void) {
    printf("Hello Capstone World!\n");
    printf("Commands: s=status, o=ONLINE, r=RESET_SCAN c=CLR_EC, "
           "S=SPEED M=MODE G=GAIN N=RANGE (inc), ?=help, q=BOOTSEL\n");
}

// Read all MSIF digital I/O pins and print state over USB serial.
// Inputs use gpio_get() (real pad state). Outputs use gpio_get_out_level()
// which reads the latched GPIO_OUT register directly — avoids the 2-cycle
// synchronizer race where gpio_get() on an output pin returns stale data
// if called immediately after a write.
static void msif_print_digital_io(void) {
    // Inputs (via U21 inverter — 1 = active at QMS)
    printf("IN : EMIS_OK=%d  SCAN_IN_PROGRESS=%d\n",
           (int)gpio_get(MSIF_DI_EMIS_OK_PIN),
           (int)gpio_get(MSIF_DI_SCAN_IN_PROGRESS_PIN));

    // Outputs (via BSS138 — 1 = MOSFET on = signal active at QMS)
    printf("OUT: ON_LINE=%d  RESET_SCAN=%d  CLR_EC=%d  "
           "SPEED=%d%d%d%d  MODE=%d%d%d  GAIN=%d%d  RANGE=%d%d\n",
           (int)gpio_get_out_level(MSIF_DO_ON_LINE_PIN),
           (int)gpio_get_out_level(MSIF_DO_RESET_SCAN_PIN),
           (int)gpio_get_out_level(MSIF_DO_CLR_EC_PIN),
           (int)gpio_get_out_level(MSIF_DO_SPEED_3_PIN), (int)gpio_get_out_level(MSIF_DO_SPEED_2_PIN),
           (int)gpio_get_out_level(MSIF_DO_SPEED_1_PIN), (int)gpio_get_out_level(MSIF_DO_SPEED_0_PIN),
           (int)gpio_get_out_level(MSIF_DO_MODE_2_PIN),  (int)gpio_get_out_level(MSIF_DO_MODE_1_PIN),
           (int)gpio_get_out_level(MSIF_DO_MODE_0_PIN),
           (int)gpio_get_out_level(MSIF_DO_GAIN_1_PIN),  (int)gpio_get_out_level(MSIF_DO_GAIN_0_PIN),
           (int)gpio_get_out_level(MSIF_DO_RANGE_1_PIN), (int)gpio_get_out_level(MSIF_DO_RANGE_0_PIN));
}

int main(void) {
                        /*** INITS ***/
    stdio_init_all();
    sleep_ms(1000);

    msif_gpio_init();

    msif_print_banner();

    while(1) {
        int uii = stdio_getchar_timeout_us(100);
        if(uii != PICO_ERROR_TIMEOUT) {
            char ui = (char)uii;
            // QUIT
            if (ui == 'q') break;
            // HELP: reprint the banner (useful after a late host connect)
            else if (ui == '?') msif_print_banner();
            // STATUS: print all digital I/O state
            else if (ui == 's') msif_print_digital_io();
            // TOGGLE ON_LINE: flip MSIF_DO_ON_LINE_PIN and print new state
            else if (ui == 'o') {
                gpio_xor_mask(1u << MSIF_DO_ON_LINE_PIN);
                printf("ON_LINE=%d\n", (int)gpio_get_out_level(MSIF_DO_ON_LINE_PIN));
            }
            // PULSE RESET_SCAN (~10ms high, then low)
            else if (ui == 'r') {
                gpio_put(MSIF_DO_RESET_SCAN_PIN, 1);
                sleep_ms(10);
                gpio_put(MSIF_DO_RESET_SCAN_PIN, 0);
                printf("RESET_SCAN pulsed (10ms)\n");
            }
            // PULSE CLR_EC (~10ms high, then low)
            else if (ui == 'c') {
                gpio_put(MSIF_DO_CLR_EC_PIN, 1);
                sleep_ms(10);
                gpio_put(MSIF_DO_CLR_EC_PIN, 0);
                printf("CLR_EC pulsed (10ms)\n");
            }
            // INCREMENT SPEED (4-bit, wraps at 16)
            else if (ui == 'S') {
                uint8_t v = (uint8_t)((msif_read_field(msif_speed_pins, 4) + 1) & 0x0F);
                msif_write_field(msif_speed_pins, 4, v);
                printf("SPEED=%d%d%d%d (0x%X)\n",
                    (v >> 3) & 1, (v >> 2) & 1, (v >> 1) & 1, v & 1, v);
            }
            // INCREMENT MODE (3-bit, wraps at 8)
            else if (ui == 'M') {
                uint8_t v = (uint8_t)((msif_read_field(msif_mode_pins, 3) + 1) & 0x07);
                msif_write_field(msif_mode_pins, 3, v);
                printf("MODE=%d%d%d (0x%X)\n",
                    (v >> 2) & 1, (v >> 1) & 1, v & 1, v);
            }
            // INCREMENT GAIN (2-bit, wraps at 4)
            else if (ui == 'G') {
                uint8_t v = (uint8_t)((msif_read_field(msif_gain_pins, 2) + 1) & 0x03);
                msif_write_field(msif_gain_pins, 2, v);
                printf("GAIN=%d%d (0x%X)\n",
                    (v >> 1) & 1, v & 1, v);
            }
            // INCREMENT RANGE (2-bit, wraps at 4). 'N' for raNge (r is RESET_SCAN).
            else if (ui == 'N') {
                uint8_t v = (uint8_t)((msif_read_field(msif_range_pins, 2) + 1) & 0x03);
                msif_write_field(msif_range_pins, 2, v);
                printf("RANGE=%d%d (0x%X)\n",
                    (v >> 1) & 1, v & 1, v);
            }
            else printf("Input received! [s=status, o=ONLINE, r=RESET c=CLR_EC, "
                        "S=SPEED M=MODE G=GAIN N=RANGE, ?=help, q=BOOTSEL]\n");
        }
    }

    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
    return 0;
}
