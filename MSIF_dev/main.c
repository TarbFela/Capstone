#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>

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
#include "msif_analog.h"

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

#define MSIF_DMM_PULSE_MS 2000

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
           "R=RESET_SCAN_DMM C=CLR_EC_DMM, "
           "S=SPEED M=MODE G=GAIN N=RANGE (inc), f=FMASS, v=FMASS_V, "
           "a=ANALOG, d=CS_TOGGLE l=LOOPBACK, ?=help, q=BOOTSEL\n");
}

// Read a number from the serial port into buf, terminated by CR/LF or timeout.
// Returns the number of characters read (0 if nothing came in). Used by the
// 'f' command to collect a floating-point mass argument.
static int msif_read_line(char *buf, int maxlen, uint32_t timeout_us) {
    int n = 0;
    while (n < maxlen - 1) {
        int c = stdio_getchar_timeout_us(timeout_us);
        if (c == PICO_ERROR_TIMEOUT) break;
        if (c == '\r' || c == '\n') {
            if (n == 0) continue;
            break;
        }
        buf[n++] = (char)c;
    }
    buf[n] = '\0';
    return n;
}

static bool msif_parse_float(const char *buf, float *value) {
    char *endptr;
    float parsed;

    while (isspace((unsigned char)*buf)) buf++;
    if (*buf == '\0') return false;

    parsed = strtof(buf, &endptr);
    if (buf == endptr) return false;

    while (isspace((unsigned char)*endptr)) endptr++;
    if (*endptr != '\0') return false;

    *value = parsed;
    return true;
}

static bool msif_read_float_arg(const char *prompt, float *value) {
    char buf[32];
    int n = msif_read_line(buf, sizeof(buf), 1000);

    if (n == 0) {
        printf("%s", prompt);
        n = msif_read_line(buf, sizeof(buf), 10 * 1000 * 1000);
        if (n == 0) {
            printf("timeout, no value entered\n");
            return false;
        }
    }

    if (!msif_parse_float(buf, value)) {
        printf("invalid number: '%s'\n", buf);
        return false;
    }
    return true;
}

static void msif_pulse_output(uint pin, uint32_t ms, const char *label) {
    gpio_put(pin, 1);
    sleep_ms(ms);
    gpio_put(pin, 0);
    printf("%s pulsed (%lums)\n", label, (unsigned long)ms);
}

static void msif_print_analog_status(void) {
    msif_dac_snapshot_t snap;

    if (!msif_dac_snapshot(&snap)) {
        printf("ANALOG: DAC layer not initialised\n");
        return;
    }

    printf("ANALOG: DEVID=0x%04X SYNC=0x%04X CONFIG=0x%04X "
           "GAIN=0x%04X STATUS=0x%04X\n",
           snap.devid, snap.sync, snap.config, snap.gain, snap.status);
    printf("DAC: A=0x%04X B=0x%04X C=0x%04X D=0x%04X\n",
           snap.dac[0], snap.dac[1], snap.dac[2], snap.dac[3]);
    printf("FMASS calibration: %s (slope=%.6f V/AMU, offset=%.6f V)\n",
           msif_fmass_is_calibrated() ? "loaded" : "NOT LOADED",
           MSIF_FMASS_CAL_SLOPE_V_PER_AMU,
           MSIF_FMASS_CAL_OFFSET_V);
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
    msif_analog_init();

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
                msif_pulse_output(MSIF_DO_RESET_SCAN_PIN, 10, "RESET_SCAN");
            }
            // PULSE CLR_EC (~10ms high, then low)
            else if (ui == 'c') {
                msif_pulse_output(MSIF_DO_CLR_EC_PIN, 10, "CLR_EC");
            }
            // DMM-FRIENDLY RESET_SCAN pulse: hold active long enough to see on a meter.
            else if (ui == 'R') {
                msif_pulse_output(MSIF_DO_RESET_SCAN_PIN, MSIF_DMM_PULSE_MS,
                                  "RESET_SCAN_DMM");
            }
            // DMM-FRIENDLY CLR_EC pulse: hold active long enough to see on a meter.
            else if (ui == 'C') {
                msif_pulse_output(MSIF_DO_CLR_EC_PIN, MSIF_DMM_PULSE_MS,
                                  "CLR_EC_DMM");
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
            // ANALOG STATUS: read back DAC registers for bench diagnostics.
            else if (ui == 'a') {
                msif_print_analog_status();
            }
            // SET FIRST MASS: 'f <mass_amu>' -> DAC OUT0 -> OPA4197 -> QDP FMASS+
            // Accepts either inline ('f 28') or prompted entry ('f' then ENTER).
            else if (ui == 'f') {
                float m;
                if (msif_read_float_arg("FMASS mass (AMU): ", &m)) {
                    float v_qdp = msif_set_fmass(m);
                    if (!msif_fmass_is_calibrated()) {
                        printf("FMASS: calibration slope is still zero in "
                               "MSIF_cfg.h; use 'v' for direct voltage checks\n");
                    }
                    printf("FMASS: mass=%.2f AMU -> target V_QDP=%.4f V "
                           "(verify with DMM at QDP FMASS+ pin)\n", m, v_qdp);
                }
            }
            // SET FMASS VOLTAGE DIRECTLY AT THE QDP CONNECTOR.
            // Useful before the Phase H mass calibration exists.
            else if (ui == 'v') {
                float v_qdp;
                if (msif_read_float_arg("FMASS target V_QDP (V): ", &v_qdp)) {
                    float v_cmd = msif_set_fmass_v(v_qdp);
                    printf("FMASS_V: target V_QDP=%.4f V -> DAC pin ~= %.4f V "
                           "(DMM should read %.4f V at QDP FMASS+)\n",
                           v_cmd, v_cmd / MSIF_AMP_GAIN, v_cmd);
                }
            }
            // SLOW CS# TOGGLE: pulses MSIF_DAC_CS_PIN (GPIO 42) at 1 Hz so a
            // DMM can visibly see the swing at the bodge wire and at U15
            // pins 11/12. Any keypress stops the loop and parks CS# HIGH.
            // Useful when 'a' returns DEVID=0x0000 and you need to confirm
            // the bodge wire itself is conducting.
            else if (ui == 'd') {
                printf("DAC CS# toggle @1Hz on GPIO %d, press any key to stop...\n",
                       MSIF_DAC_CS_PIN);
                bool stop = false;
                while (!stop) {
                    gpio_put(MSIF_DAC_CS_PIN, 0);
                    sleep_ms(500);
                    if (stdio_getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) { stop = true; break; }
                    gpio_put(MSIF_DAC_CS_PIN, 1);
                    sleep_ms(500);
                    if (stdio_getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) { stop = true; break; }
                }
                gpio_put(MSIF_DAC_CS_PIN, 1);
                printf("DAC CS# toggle stopped (CS# parked HIGH)\n");
            }
            // DAC SPI LOOPBACK: write a known pattern to DAC_A, read it back,
            // compare. Proves MOSI->register->MISO traffic independently of
            // whether the DAC analog output is alive.
            else if (ui == 'l') {
                uint16_t pattern = 0xBEEFu;
                uint16_t rb = 0;
                if (!msif_dac_loopback(pattern, &rb)) {
                    printf("LOOPBACK: analog layer not initialised\n");
                } else {
                    printf("LOOPBACK: wrote 0x%04X to DAC_A, read 0x%04X -> %s\n",
                           pattern, rb, (rb == pattern) ? "OK" : "MISMATCH");
                }
            }
            else printf("Input received! [s=status, o=ONLINE, r=RESET c=CLR_EC, "
                        "R=RESET_DMM C=CLR_EC_DMM, "
                        "S=SPEED M=MODE G=GAIN N=RANGE, f=FMASS, v=FMASS_V, "
                        "a=ANALOG, d=CS_TOGGLE l=LOOPBACK, ?=help, q=BOOTSEL]\n");
        }
    }

    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
    return 0;
}
