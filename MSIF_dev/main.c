#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"


#include "hardware/gpio.h"


#include "MSIF_cfg.h"
#include "msif_gpio.h"
#include "msif_analog.h"
#include "msif_adc.h"
#include "msif_peak.h"
#include "msif_proto.h"

// Pin tables (msif_speed_pins / mode / gain / range) and the bit pack/unpack
// helpers (msif_gpio_read_field, msif_gpio_write_field) live canonically in
// msif_gpio.{h,c} — included via "msif_gpio.h" above. main.c just uses them.

#define MSIF_DMM_PULSE_MS 2000 // Allows for a 2ms pulse to be seen on a DMM

// Print the hello banner and command help. Called at startup AND on '?'.
// The startup copy is usually dropped because VS Code Serial Monitor
// doesn't assert DTR until after the firmware has already printed and
// the pico_stdio_usb layer silently drops output with no host connected,
// so '?' is the reliable way to retrieve this after a late connect.
static void msif_print_banner(void) {
    printf("OTHER GUY'S CAPSTONE BIG TWO SIX\n");
    printf("Commands: s=status, o=ONLINE, r=RESET_SCAN c=CLR_EC, "
           "R=RESET_SCAN_DMM C=CLR_EC_DMM, "
           "S=SPEED M=MODE G=GAIN N=RANGE (inc), f=FMASS, v=FMASS_V, "
           "F=SWEEP P=PARK, K=PK_SWEEP T=PK_PARK, "
           "a=ANALOG, d=CS_TOGGLE l=LOOPBACK, "
           "i=EC_READ A=ADC_SNAP, ?=help, q=BOOTSEL\n");
    printf("Protocol: ':<CMD> <args>' (e.g. ':MASS 28.5', ':HELP' for list).\n");
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

// Bench CLI pulse helper: delegates the actual GPIO toggle to msif_gpio_pulse
// (canonical in msif_gpio.c) and adds a printf so the operator sees confirmation.
static void msif_pulse_output(uint pin, uint32_t ms, const char *label) {
    msif_gpio_pulse(pin, ms);
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
    printf("FMASS calibration: slope=%.6f V/AMU, offset=%.6f V\n",
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
    msif_adc_init();

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
                uint8_t v = (uint8_t)((msif_gpio_read_field(msif_speed_pins, 4) + 1) & 0x0F);
                msif_gpio_write_field(msif_speed_pins, 4, v);
                printf("SPEED=%d%d%d%d (0x%X)\n",
                    (v >> 3) & 1, (v >> 2) & 1, (v >> 1) & 1, v & 1, v);
            }
            // INCREMENT MODE (3-bit, wraps at 8)
            else if (ui == 'M') {
                uint8_t v = (uint8_t)((msif_gpio_read_field(msif_mode_pins, 3) + 1) & 0x07);
                msif_gpio_write_field(msif_mode_pins, 3, v);
                printf("MODE=%d%d%d (0x%X)\n",
                    (v >> 2) & 1, (v >> 1) & 1, v & 1, v);
            }
            // INCREMENT GAIN (2-bit, wraps at 4)
            else if (ui == 'G') {
                uint8_t v = (uint8_t)((msif_gpio_read_field(msif_gain_pins, 2) + 1) & 0x03);
                msif_gpio_write_field(msif_gain_pins, 2, v);
                printf("GAIN=%d%d (0x%X)\n",
                    (v >> 1) & 1, v & 1, v);
            }
            // INCREMENT RANGE (2-bit, wraps at 4). 'N' for raNge (r is RESET_SCAN).
            else if (ui == 'N') {
                uint8_t v = (uint8_t)((msif_gpio_read_field(msif_range_pins, 2) + 1) & 0x03);
                msif_gpio_write_field(msif_range_pins, 2, v);
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
                    printf("FMASS: mass=%.2f AMU -> target V_QDP=%.4f V "
                           "(verify with DMM at QDP FMASS+ pin)\n", m, v_qdp);
                }
            }
            // SET FMASS VOLTAGE DIRECTLY AT THE QDP CONNECTOR.
            // Bypasses the AMU calibration; useful for direct voltage checks at the bench.
            else if (ui == 'v') {
                float v_qdp;
                if (msif_read_float_arg("FMASS target V_QDP (V): ", &v_qdp)) {
                    float v_cmd = msif_set_fmass_v(v_qdp);
                    printf("FMASS_V: target V_QDP=%.4f V -> DAC pin ~= %.4f V "
                           "(DMM should read %.4f V at QDP FMASS+)\n",
                           v_cmd, v_cmd / MSIF_AMP_GAIN, v_cmd);
                }
            }
            // FMASS SWEEP WITH EC LOGGING: prompts for v_start, v_end, n_steps,
            // then walks FMASS linearly while averaging ADC reads at each point.
            // Emits CSV to stdout (capture it from the serial monitor), with a
            // header line recording the sweep parameters so a later script can
            // reconstruct the run. Used both for bench calibration and as a
            // TPD-style v_ec-vs-mass primitive (v_ec is proportional to ion
            // current, scaled by the QMS RANGE/GAIN transfer — see msif_adc.h).
            //
            //   Per-point time ≈ MSIF_FMASS_SETTLE_MS + MSIF_ADC_AVG_SAMPLES * ~50µs
            //   Default: 50 ms + 1.6 ms ≈ 52 ms/point, so a 41-point 0->10V
            //   sweep (0.25V steps) takes ~2.1 s.
            //
            // Any keypress aborts mid-sweep. On exit FMASS is parked at 0 V so
            // the QMS doesn't end up stuck at some random filter voltage.
            else if (ui == 'F') {
                float v_start = 0.0f, v_end = 0.0f, f_steps = 0.0f;
                if (!msif_read_float_arg("Sweep start V_QDP (V): ", &v_start)) continue;
                if (!msif_read_float_arg("Sweep end V_QDP (V): ",   &v_end))   continue;
                if (!msif_read_float_arg("Number of steps: ",       &f_steps)) continue;
                int n_steps = (int)f_steps;
                if (n_steps < 2) {
                    printf("SWEEP: need >= 2 steps\n");
                    continue;
                }

                printf("# SWEEP v_start=%.4f v_end=%.4f steps=%d "
                       "samples_per_pt=%u settle_ms=%u\n",
                       v_start, v_end, n_steps,
                       (unsigned)MSIF_ADC_AVG_SAMPLES,
                       (unsigned)MSIF_FMASS_SETTLE_MS);
                printf("step,t_ms,v_fmass,ec_code,v_adc_diff,v_ec,status\n");

                float dv = (v_end - v_start) / (float)(n_steps - 1);
                bool aborted = false;
                uint64_t t0_us = time_us_64();
                for (int i = 0; i < n_steps; i++) {
                    if (stdio_getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
                        aborted = true;
                        break;
                    }
                    float v = v_start + dv * (float)i;
                    msif_set_fmass_v(v);
                    sleep_ms(MSIF_FMASS_SETTLE_MS);

                    msif_adc_sample_t s;
                    if (!msif_adc_read_ec_avg(MSIF_ADC_AVG_SAMPLES, &s)) {
                        printf("# SWEEP: ADC read failed at step %d\n", i);
                        aborted = true;
                        break;
                    }
                    uint32_t t_ms = (uint32_t)((time_us_64() - t0_us) / 1000u);
                    printf("%d,%lu,%.4f,%d,%.6f,%.6f,0x%02X\n",
                           i, (unsigned long)t_ms, v,
                           s.raw_code, s.v_adc_diff, s.v_ec, s.mcp_status);
                }

                msif_set_fmass_v(0.0f);
                printf("# SWEEP %s, FMASS parked at 0 V\n",
                       aborted ? "aborted" : "complete");
            }
            // PARK AND LOG: hold FMASS at a fixed voltage and stream averaged
            // EC- readings at MSIF_ADC_PARK_PERIOD_MS intervals. Prompts for
            // the park voltage and a duration in milliseconds (0 = run until
            // a keypress). Use to check electrometer-signal stability at a
            // single mass (e.g. park at the N2 peak and watch the drift /
            // noise floor before committing to TPD runs).
            else if (ui == 'P') {
                float v_park = 0.0f, dur_ms_f = 0.0f;
                if (!msif_read_float_arg("Park V_QDP (V): ",         &v_park))  continue;
                if (!msif_read_float_arg("Duration ms (0=forever): ",&dur_ms_f))continue;
                uint32_t dur_ms = (dur_ms_f < 0.0f) ? 0u : (uint32_t)dur_ms_f;

                msif_set_fmass_v(v_park);
                sleep_ms(MSIF_FMASS_SETTLE_MS);

                printf("# PARK v_fmass=%.4f duration_ms=%lu period_ms=%u "
                       "samples_per_pt=%u (press any key to stop)\n",
                       v_park, (unsigned long)dur_ms,
                       (unsigned)MSIF_ADC_PARK_PERIOD_MS,
                       (unsigned)MSIF_ADC_AVG_SAMPLES);
                printf("t_ms,ec_code,v_adc_diff,v_ec,status\n");

                uint64_t t0_us = time_us_64();
                while (1) {
                    if (stdio_getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) break;
                    uint32_t t_ms = (uint32_t)((time_us_64() - t0_us) / 1000u);
                    if (dur_ms > 0 && t_ms >= dur_ms) break;

                    msif_adc_sample_t s;
                    if (!msif_adc_read_ec_avg(MSIF_ADC_AVG_SAMPLES, &s)) {
                        printf("# PARK: ADC read failed at t=%lu\n",
                               (unsigned long)t_ms);
                        break;
                    }
                    printf("%lu,%d,%.6f,%.6f,0x%02X\n",
                           (unsigned long)t_ms,
                           s.raw_code, s.v_adc_diff, s.v_ec, s.mcp_status);

                    sleep_ms(MSIF_ADC_PARK_PERIOD_MS);
                }

                msif_set_fmass_v(0.0f);
                printf("# PARK stopped, FMASS parked at 0 V\n");
            }
            // PEAK SWEEP: mass-domain integration. Steps FMASS from
            // mass_start_amu to mass_end_amu in n_steps points (settle +
            // averaged ADC at each), trapezoid-integrates v_ec(mass), and
            // prints area + intensity-weighted centroid + max. Per-step CSV
            // matches 'F' with mass_amu added as the 3rd column. Uses the
            // FMASS slope/offset from MSIF_cfg.h (defaulted to QMS-112 spec
            // values; replace with bench-measured values after calibration).
            // Any keypress aborts; FMASS is parked at 0 V on every exit path.
            else if (ui == 'K') {
                float m_start = 0.0f, m_end = 0.0f, f_steps = 0.0f;
                if (!msif_read_float_arg("Peak mass start (AMU): ", &m_start)) continue;
                if (!msif_read_float_arg("Peak mass end (AMU): ",   &m_end))   continue;
                if (!msif_read_float_arg("Number of steps: ",       &f_steps)) continue;
                if (f_steps < 0.0f) f_steps = 0.0f;
                msif_peak_result_t r;
                (void)msif_peak_sweep_mass(m_start, m_end,
                                           (uint32_t)f_steps, &r);
            }
            // PEAK PARK: time-domain integration. Holds FMASS at v_qdp_park
            // for duration_ms, sampling at MSIF_ADC_PARK_PERIOD_MS cadence.
            // Trapezoid-integrates v_ec(t), reports area in V*s plus mean
            // v_ec. Same CSV columns as 'P' with t_s added. Bounded by
            // MSIF_PEAK_PARK_MAX_MS; any keypress aborts; FMASS parks to 0 V.
            else if (ui == 'T') {
                float v_park = 0.0f, dur_ms_f = 0.0f;
                if (!msif_read_float_arg("Park V_QDP (V): ",         &v_park))   continue;
                if (!msif_read_float_arg("Duration ms (>0 required): ", &dur_ms_f)) continue;
                if (dur_ms_f <= 0.0f) {
                    printf("PK_PARK: duration must be > 0\n");
                    continue;
                }
                msif_peak_result_t r;
                (void)msif_peak_park_time(v_park, (uint32_t)dur_ms_f, &r);
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
            // EC READ: one-shot conversion of the EC- differential channel.
            // Prints raw signed code, differential volts at the ADC input,
            // and back-projected volts at the QDP EC- pin. v_ec is what the
            // QMS post-amp is outputting at this instant — proportional to
            // ion current via the RANGE/GAIN transfer table in QMS-112 manual
            // sec 10.2.1.1, but that conversion is not yet wired into the
            // firmware (so this command reports volts, not amps).
            else if (ui == 'i') {
                msif_adc_sample_t s;
                if (!msif_adc_read_ec(&s)) {
                    printf("EC: ADC layer not initialised\n");
                } else {
                    printf("EC: code=%d  V_adc_diff=%.6f V  V_EC=%.6f V  "
                           "(status=0x%02X)\n",
                           s.raw_code, s.v_adc_diff, s.v_ec, s.mcp_status);
                }
            }
            // ADC REGISTER SNAPSHOT: dump CFG0..3, IRQ, MUX for bench
            // debugging. Use after an unexpected 'i' result to see whether
            // the configuration actually made it into the ADC.
            else if (ui == 'A') {
                msif_adc_snapshot_t snap;
                if (!msif_adc_snapshot(&snap)) {
                    printf("ADC_SNAP: ADC layer not initialised\n");
                } else {
                    printf("ADC_SNAP: status=0x%02X CFG0=0x%02X CFG1=0x%02X "
                           "CFG2=0x%02X CFG3=0x%02X IRQ=0x%02X MUX=0x%02X\n",
                           snap.mcp_status, snap.config[0], snap.config[1],
                           snap.config[2], snap.config[3], snap.irq_reg,
                           snap.mux_reg);
                }
            }
            // PROTOCOL: ':' starts a line-mode machine protocol command. The
            // existing single-letter CLI is unchanged — operators keep
            // pressing single keys, scripts/peer-controllers send full
            // ':CMD args\n' lines instead. Both USB and UART input land
            // here (stdio_uart is enabled in CMakeLists with TX/RX remapped
            // to GP12/GP13 per the MSIF MCU schematic).
            else if (ui == ':') {
                char line[MSIF_PROTO_LINE_MAX];
                int n = msif_read_line(line, sizeof(line), 1000 * 1000);
                if (n > 0) {
                    if (!msif_proto_handle_line(line)) {
                        printf("# ERR protocol command failed\n");
                    }
                } else {
                    printf("# ERR :timeout (no command after colon)\n");
                }
            }
            else printf("Input received! [s=status, o=ONLINE, r=RESET c=CLR_EC, "
                        "R=RESET_DMM C=CLR_EC_DMM, "
                        "S=SPEED M=MODE G=GAIN N=RANGE, f=FMASS, v=FMASS_V, "
                        "F=SWEEP P=PARK, K=PK_SWEEP T=PK_PARK, "
                        "a=ANALOG, d=CS_TOGGLE l=LOOPBACK, "
                        "i=EC_READ A=ADC_SNAP, :CMD=protocol, ?=help, q=BOOTSEL]\n");
        }
    }

    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
    return 0;
}
