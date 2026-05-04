/*
 * msif_proto.c — line-protocol parser + dispatch table + RUN sequencer.
 * See msif_proto.h for the protocol shape.
 *
 * The dispatch table is just an array of {name, min_args, max_args,
 * handler, help}. Lookup is linear — there are ~20 commands so a hash is
 * overkill. Each handler is short and uses printf for output; the
 * stdout drain (USB, UART, both) is configured by CMakeLists.
 */
#include "msif_proto.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "MSIF_cfg.h"
#include "msif_gpio.h"
#include "msif_analog.h"
#include "msif_adc.h"
#include "msif_peak.h"
#include "msif_qms.h"

#define MSIF_PROTO_WAIT_MAX_MS 600000u

/* ------------------------------------------------------------------ */
/* Output helpers                                                      */
/* ------------------------------------------------------------------ */
static bool ok(const char *cmd) {
    printf("# OK %s\n", cmd);
    return true;
}

static bool err(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    printf("# ERR ");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
    return false;
}

/* ------------------------------------------------------------------ */
/* Argument helpers                                                    */
/* ------------------------------------------------------------------ */
static bool parse_float(const char *s, float *out) {
    if (!s || !*s) return false;
    char *endp = NULL;
    float v = strtof(s, &endp);
    if (endp == s) return false;
    while (*endp && isspace((unsigned char)*endp)) endp++;
    if (*endp != '\0') return false;
    *out = v;
    return true;
}

static bool parse_uint(const char *s, uint32_t *out) {
    if (!s || !*s) return false;
    char *endp = NULL;
    long v = strtol(s, &endp, 0);
    if (endp == s || v < 0) return false;
    while (*endp && isspace((unsigned char)*endp)) endp++;
    if (*endp != '\0') return false;
    *out = (uint32_t)v;
    return true;
}

/* Pulse + field helpers + pin tables all live in msif_gpio.h. */

/* ------------------------------------------------------------------ */
/* Command handlers                                                    */
/* ------------------------------------------------------------------ */

/* INFO — board state snapshot. Compact, single line per section. */
static bool h_info(int argc, char **argv) {
    (void)argc; (void)argv;
    msif_peak_print_cal_status();
    printf("# OK INFO mass_range=%u amp_gain=%.3f adc_input_gain=%.3f "
           "settle_ms=%u avg_samples=%u\n",
           (unsigned)MSIF_QMS_MASS_RANGE,
           (double)MSIF_AMP_GAIN,
           (double)MSIF_ADC_INPUT_GAIN,
           (unsigned)MSIF_FMASS_SETTLE_MS,
           (unsigned)MSIF_ADC_AVG_SAMPLES);
    return true;
}

/* HELP — listing of commands. Implemented via msif_proto_print_help. */
static bool h_help(int argc, char **argv) {
    (void)argc; (void)argv;
    msif_proto_print_help();
    return true;
}

/* STATUS — machine-readable digital state snapshot. */
static bool h_status(int argc, char **argv) {
    (void)argc; (void)argv;
    printf("# OK STATUS emis_ok=%d scan_in_progress=%d online=%d reset_scan=%d "
           "clr_ec=%d speed=0x%X mode=0x%X gain=0x%X range=0x%X\n",
           (int)gpio_get(MSIF_DI_EMIS_OK_PIN),
           (int)gpio_get(MSIF_DI_SCAN_IN_PROGRESS_PIN),
           (int)gpio_get_out_level(MSIF_DO_ON_LINE_PIN),
           (int)gpio_get_out_level(MSIF_DO_RESET_SCAN_PIN),
           (int)gpio_get_out_level(MSIF_DO_CLR_EC_PIN),
           (unsigned)msif_gpio_read_field(msif_speed_pins, 4),
           (unsigned)msif_gpio_read_field(msif_mode_pins,  3),
           (unsigned)msif_gpio_read_field(msif_gain_pins,  2),
           (unsigned)msif_gpio_read_field(msif_range_pins, 2));
    return true;
}

/* ----- analog setters ------------------------------------------------- */

static bool h_mass(int argc, char **argv) {
    float m;
    if (argc != 2 || !parse_float(argv[1], &m)) return err("usage: MASS <amu>");
    float v = msif_set_fmass(m);
    printf("# OK MASS amu=%.4f v_qdp=%.4f cal=%s\n",
           (double)m, (double)v,
           msif_fmass_is_calibrated() ? "loaded" : "spec_default");
    return true;
}

static bool h_fmassv(int argc, char **argv) {
    float v;
    if (argc != 2 || !parse_float(argv[1], &v)) return err("usage: FMASSV <v_qdp>");
    float a = msif_set_fmass_v(v);
    printf("# OK FMASSV v_qdp=%.4f\n", (double)a);
    return true;
}

static bool h_swidth(int argc, char **argv) {
    float w;
    if (argc != 2 || !parse_float(argv[1], &w)) return err("usage: SWIDTH <amu>");
    float v = msif_qms_set_swidth_amu(w);
    printf("# OK SWIDTH amu=%.4f v_qdp=%.4f\n", (double)w, (double)v);
    return true;
}

static bool h_swidthv(int argc, char **argv) {
    float v;
    if (argc != 2 || !parse_float(argv[1], &v)) return err("usage: SWIDTHV <v_qdp>");
    float a = msif_set_swidth_v(v);
    printf("# OK SWIDTHV v_qdp=%.4f\n", (double)a);
    return true;
}

static bool h_sem(int argc, char **argv) {
    float kv;
    if (argc != 2 || !parse_float(argv[1], &kv)) return err("usage: SEM <hv_kv>");
    float v = msif_qms_set_sem_kv(kv);
    printf("# OK SEM hv_kv=%.4f v_qdp=%.4f (linear range 0..3 kV)\n",
           (double)kv, (double)v);
    return true;
}

static bool h_semv(int argc, char **argv) {
    float v;
    if (argc != 2 || !parse_float(argv[1], &v)) return err("usage: SEMV <v_qdp>");
    float a = msif_set_sem_v(v);
    printf("# OK SEMV v_qdp=%.4f\n", (double)a);
    return true;
}

/* ----- digital setters ----------------------------------------------- */

static bool h_gain(int argc, char **argv) {
    if (argc != 2) return err("usage: GAIN <x1|x10|x100|auto>");
    int code = msif_qms_parse_gain(argv[1]);
    if (code < 0) return err("GAIN: unknown '%s'", argv[1]);
    if (!msif_qms_set_gain((msif_qms_gain_t)code)) return err("GAIN: write failed");
    printf("# OK GAIN code=%d sym=%s\n", code, argv[1]);
    return true;
}

static bool h_mode(int argc, char **argv) {
    if (argc != 2) return err("usage: MODE <emis_off|spectrum|integral|degas|total|helium>");
    int op = msif_qms_parse_opmode(argv[1]);
    if (op < 0) return err("MODE: unknown '%s'", argv[1]);
    if (!msif_qms_set_op_mode((msif_qms_opmode_t)op)) return err("MODE: write failed");
    printf("# OK MODE sym=%s%s\n",
           argv[1],
           (op == MSIF_QMS_OPMODE_TOTAL)  ? " (synth: INTEGRAL @ FMASS=8)" :
           (op == MSIF_QMS_OPMODE_HELIUM) ? " (synth: SPECTRUM @ FMASS=4)" : "");
    return true;
}

static bool h_scanmode(int argc, char **argv) {
    if (argc != 2) return err("usage: SCANMODE <single|repeat>");
    msif_qms_scanmode_t m;
    if      (!strcasecmp(argv[1], "single")) m = MSIF_QMS_SCANMODE_SINGLE;
    else if (!strcasecmp(argv[1], "repeat")) m = MSIF_QMS_SCANMODE_REPEAT;
    else return err("SCANMODE: unknown '%s'", argv[1]);
    msif_qms_set_scan_mode(m);
    printf("# OK SCANMODE sym=%s\n", argv[1]);
    return true;
}

static bool h_range(int argc, char **argv) {
    uint32_t code;
    if (argc != 2 || !parse_uint(argv[1], &code) || code > 3u) {
        return err("usage: RANGE <0..3>");
    }
    if (!msif_qms_set_range((msif_qms_range_t)code)) return err("RANGE: write failed");
    printf("# OK RANGE code=%u\n", (unsigned)code);
    return true;
}

static bool h_speed(int argc, char **argv) {
    if (argc != 2) return err("usage: SPEED <1ms|3ms|10ms|30ms|100ms|300ms|1s|3s|10s>");
    int code = msif_qms_parse_speed(argv[1]);
    if (code < 0) return err("SPEED: unknown '%s'", argv[1]);
    if (!msif_qms_set_speed((msif_qms_speed_t)code)) return err("SPEED: write failed");
    printf("# OK SPEED sym=%s code=0x%X filter_us=%lu\n",
           argv[1], code,
           (unsigned long)msif_qms_speed_filter_us[code]);
    return true;
}

static bool h_online(int argc, char **argv) {
    uint32_t v;
    if (argc != 2 || !parse_uint(argv[1], &v) || v > 1u) return err("usage: ONLINE <0|1>");
    gpio_put(MSIF_DO_ON_LINE_PIN, (int)v);
    printf("# OK ONLINE state=%u\n", (unsigned)v);
    return true;
}

static bool h_reset(int argc, char **argv) {
    (void)argc; (void)argv;
    msif_gpio_pulse(MSIF_DO_RESET_SCAN_PIN, 10u);
    return ok("RESET");
}

static bool h_clr(int argc, char **argv) {
    (void)argc; (void)argv;
    msif_gpio_pulse(MSIF_DO_CLR_EC_PIN, 10u);
    return ok("CLR");
}

/* ----- ADC ----------------------------------------------------------- */

static bool h_read(int argc, char **argv) {
    (void)argc; (void)argv;
    msif_adc_sample_t s;
    if (!msif_adc_read_ec(&s)) return err("READ: ADC not initialised");
    printf("# OK READ code=%d v_adc_diff=%.6f v_ec=%.6f status=0x%02X\n",
           s.raw_code, (double)s.v_adc_diff, (double)s.v_ec, s.mcp_status);
    return true;
}

static bool h_readavg(int argc, char **argv) {
    uint32_t n;
    if (argc != 2 || !parse_uint(argv[1], &n) || n == 0u) return err("usage: READAVG <n>");
    msif_adc_sample_t s;
    if (!msif_adc_read_ec_avg(n, &s)) return err("READAVG: ADC failed");
    printf("# OK READAVG n=%u code=%d v_adc_diff=%.6f v_ec=%.6f status=0x%02X\n",
           (unsigned)n, s.raw_code,
           (double)s.v_adc_diff, (double)s.v_ec, s.mcp_status);
    return true;
}

/* ----- peak integration --------------------------------------------- */

static bool h_sweep(int argc, char **argv) {
    float m_s, m_e; uint32_t n;
    if (argc != 4 ||
        !parse_float(argv[1], &m_s) ||
        !parse_float(argv[2], &m_e) ||
        !parse_uint(argv[3], &n)) {
        return err("usage: SWEEP <m_start_amu> <m_end_amu> <n_steps>");
    }
    msif_peak_result_t r;
    msif_peak_status_t st = msif_peak_sweep_mass(m_s, m_e, n, &r);
    /* msif_peak_sweep_mass prints its own SUMMARY line. */
    return st == MSIF_PEAK_OK;
}

static bool h_dwell(int argc, char **argv) {
    float v; uint32_t ms;
    if (argc != 3 ||
        !parse_float(argv[1], &v) ||
        !parse_uint(argv[2], &ms)) {
        return err("usage: DWELL <v_qdp> <duration_ms>");
    }
    msif_peak_result_t r;
    msif_peak_status_t st = msif_peak_park_time(v, ms, &r);
    return st == MSIF_PEAK_OK;
}

static bool h_wait(int argc, char **argv) {
    uint32_t ms = 0;
    if (argc != 2 || !parse_uint(argv[1], &ms)) return err("usage: WAIT <ms>");
    if (ms > MSIF_PROTO_WAIT_MAX_MS) {
        return err("WAIT: max is %u ms", (unsigned)MSIF_PROTO_WAIT_MAX_MS);
    }
    sleep_ms(ms);
    printf("# OK WAIT ms=%u\n", (unsigned)ms);
    return true;
}

/* ------------------------------------------------------------------ */
/* Dispatch table                                                      */
/* ------------------------------------------------------------------ */
typedef bool (*msif_proto_handler_t)(int argc, char **argv);

typedef struct {
    const char            *name;
    msif_proto_handler_t   handler;
    const char            *help;
} msif_cmd_t;

/* RUN is special-cased before this table is consulted (it needs the raw
 * line tail to split on ';', not pre-tokenized argv). */
static const msif_cmd_t s_cmds[] = {
    { "INFO",     h_info,     "board calibration + config snapshot" },
    { "HELP",     h_help,     "this list" },
    { "STATUS",   h_status,   "digital IO state snapshot" },
    { "MASS",     h_mass,     "<amu>             FMASS via stored cal" },
    { "FMASSV",   h_fmassv,   "<v_qdp>           FMASS raw QDP volts" },
    { "SWIDTH",   h_swidth,   "<amu>             SWIDTH via spec eq" },
    { "SWIDTHV",  h_swidthv,  "<v_qdp>           SWIDTH raw QDP volts" },
    { "SEM",      h_sem,      "<hv_kv>           SEM via spec eq (0..3 kV)" },
    { "SEMV",     h_semv,     "<v_qdp>           SEM raw QDP volts" },
    { "GAIN",     h_gain,     "<x1|x10|x100|auto>" },
    { "MODE",     h_mode,     "<emis_off|spectrum|integral|degas|total|helium>" },
    { "SCANMODE", h_scanmode, "<single|repeat>" },
    { "RANGE",    h_range,    "<0..3>" },
    { "SPEED",    h_speed,    "<1ms|3ms|10ms|30ms|100ms|300ms|1s|3s|10s>" },
    { "ONLINE",   h_online,   "<0|1>" },
    { "RESET",    h_reset,    "                  pulse RESET_SCAN ~10 ms" },
    { "CLR",      h_clr,      "                  pulse CLR_EC ~10 ms" },
    { "READ",     h_read,     "                  one-shot EC voltage read" },
    { "READAVG",  h_readavg,  "<n>               averaged EC voltage read" },
    { "SWEEP",    h_sweep,    "<m_s> <m_e> <n>   mass-domain peak integration" },
    { "DWELL",    h_dwell,    "<v_qdp> <ms>      time-domain peak integration" },
    { "WAIT",     h_wait,     "<ms>              delay for RUN sequencing" },
    /* RUN is documented in the help banner but handled before dispatch. */
};
static const size_t s_cmd_count = sizeof(s_cmds) / sizeof(s_cmds[0]);

void msif_proto_print_help(void) {
    printf("# Protocol commands (prefix each with ':' from the bench CLI):\n");
    for (size_t i = 0; i < s_cmd_count; i++) {
        printf("#   %-10s %s\n", s_cmds[i].name, s_cmds[i].help);
    }
    printf("#   %-10s %s\n", "RUN", "<cmd>; <cmd>; ... — sequential program (abort on err)");
}

/* ------------------------------------------------------------------ */
/* Tokenizer + dispatcher                                              */
/* ------------------------------------------------------------------ */

#define MSIF_PROTO_MAX_ARGV     8

static const msif_cmd_t *find_cmd(const char *name) {
    for (size_t i = 0; i < s_cmd_count; i++) {
        if (!strcasecmp(name, s_cmds[i].name)) return &s_cmds[i];
    }
    return NULL;
}

/* Reentrant tokenizer — same shape as POSIX strtok_r, locally implemented
 * so we don't depend on _POSIX_C_SOURCE feature-test macro plumbing through
 * newlib. *cursor is advanced past each consumed token + delim run. The
 * input buffer is mutated (delim runs replaced with NULs). */
static char *next_token(char **cursor, const char *delims) {
    if (!cursor || !*cursor) return NULL;
    char *p = *cursor;
    while (*p && strchr(delims, (unsigned char)*p)) p++;
    if (!*p) { *cursor = p; return NULL; }
    char *start = p;
    while (*p && !strchr(delims, (unsigned char)*p)) p++;
    if (*p) { *p = '\0'; p++; }
    *cursor = p;
    return start;
}

bool msif_proto_handle_line(const char *line) {
    if (!line) return false;

    /* Skip leading whitespace */
    while (*line && isspace((unsigned char)*line)) line++;
    if (!*line) return false;

    /* Special-case RUN: split remainder on ';' and recurse on each piece.
     * Has to happen before standard tokenization because semicolons
     * aren't necessarily space-separated (e.g. "RUN MASS 28; READAVG 32"). */
    if ((!strncasecmp(line, "RUN ",  4)) ||
        (!strncasecmp(line, "RUN\t", 4))) {
        const char *rest = line + 4;
        while (*rest && isspace((unsigned char)*rest)) rest++;
        if (!*rest) return err("RUN: empty program");

        char buf[MSIF_PROTO_LINE_MAX];
        strncpy(buf, rest, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        char *cursor = buf;
        int step = 0;
        char *piece;
        while ((piece = next_token(&cursor, ";")) != NULL) {
            while (*piece && isspace((unsigned char)*piece)) piece++;
            if (!*piece) continue;
            printf("# RUN step=%d cmd='%s'\n", step++, piece);
            if (!msif_proto_handle_line(piece)) {
                return err("RUN aborted at step=%d", step - 1);
            }
        }
        printf("# OK RUN steps=%d\n", step);
        return true;
    }

    /* Standard tokenization: copy into stack buffer, split on whitespace. */
    char buf[MSIF_PROTO_LINE_MAX];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *argv[MSIF_PROTO_MAX_ARGV];
    int argc = 0;
    char *cursor = buf;
    while (argc < MSIF_PROTO_MAX_ARGV) {
        char *t = next_token(&cursor, " \t\r\n");
        if (!t) break;
        argv[argc++] = t;
    }
    if (argc == 0) return false;

    const msif_cmd_t *cmd = find_cmd(argv[0]);
    if (!cmd) return err("unknown command '%s' (try HELP)", argv[0]);
    return cmd->handler(argc, argv);
}
