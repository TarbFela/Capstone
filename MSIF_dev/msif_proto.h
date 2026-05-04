/*
 * msif_proto.h — line-based ASCII command protocol.
 *
 * Layered ABOVE the existing single-letter bench CLI in main.c. The bench
 * operator presses single keys (no ENTER) to drive the bench commands;
 * a host or peer controller sends a `:`-prefixed line followed by ENTER
 * to invoke the protocol, e.g.:
 *
 *   :MASS 28.5
 *   :MODE SPECTRUM
 *   :SWEEP 27 29 41
 *   :RUN MASS 28; READAVG 64; MASS 32; READAVG 64
 *
 * Replies follow a comment-marker convention shared with the existing
 * sweep/park CSV streams:
 *
 *   # OK <CMD>                    -- simple success
 *   # OK <CMD> key=value ...      -- success with data
 *   # ERR <reason>                -- failure
 *   <csv stream>                  -- for SWEEP / DWELL: as-is from msif_peak.c
 *
 * The same parser is used for both USB stdio and UART input — the
 * protocol doesn't care which channel a line came in on. Output goes
 * through printf() which (after CMakeLists changes) is mirrored to both
 * channels by the SDK's stdio multiplexer.
 *
 * RUN is meta: it splits its remainder on ';' and recurses for each
 * piece. Useful as a minimal action scheduler. Aborts at the first
 * error.
 */
#ifndef MSIF_PROTO_H
#define MSIF_PROTO_H

#include <stdbool.h>
#include <stdint.h>

/* Maximum line length the protocol parser will accept. Longer lines are
 * silently truncated at the readline layer. */
#define MSIF_PROTO_LINE_MAX     192

/* Handle a single command line. The caller has already stripped the
 * leading ':' (if any) and the trailing CR/LF. Tokenizes the line in a
 * local stack buffer (no globals — reentrant — RUN recurses safely).
 * Prints results / errors via printf.
 * Returns true on success, false on parse/dispatch/handler error. */
bool msif_proto_handle_line(const char *line);

/* Print the supported command list with brief help text. Also emitted
 * when the protocol receives `:HELP`. */
void msif_proto_print_help(void);

#endif /* MSIF_PROTO_H */
