#ifndef OBD_DTC_H
#define OBD_DTC_H

#include <stdint.h>

/* Fault codes (modes 03, 07 and 0A), ISO 15031-6 text form.
 *
 * Read-only by construction: this module only decodes bytes that were
 * already read from the car. It cannot send anything, and mode 04 (clear
 * codes) never appears in this repository. */

typedef struct {
    char code[6];  /* "P0133" and the terminating NUL */
} obd_dtc;

/* Decodes the payload of a 03/07/0A response: the bytes after the
 * service byte, for example { 0x01, 0x33, 0x00, 0x00, 0x00, 0x00 } for
 * "43 01 33 00 00 00 00".
 *
 * Two layouts exist in the wild and the length tells them apart:
 *   - odd length (1 + 2n): the first byte is the count of codes, which
 *     is what a CAN (ISO 15765-4) ECU sends.
 *   - even length (2n): codes start at the first byte; K-line and J1850
 *     ECUs answer like that, padded with zero bytes.
 * Zero pairs are padding and are skipped. The rule is a heuristic on a
 * length, so obd_dtc_parse_ex() exists for a caller that already knows
 * the protocol (ATDPN) and does not want to depend on it. An answer of
 * more than three codes spans several lines: the caller joins the data
 * bytes of every line, drops the service byte once, and then only the
 * count byte tells the two layouts apart.
 *
 * Returns the number of codes written, or -1 on invalid arguments. */
int obd_dtc_parse(const uint8_t *payload, int len, obd_dtc *out, int max);

/* Same, with the layout forced. has_count != 0 means payload[0] is the
 * number of codes. If that count promises more codes than the payload
 * holds, the complete pairs that are there are decoded: they are real,
 * the count is not. */
int obd_dtc_parse_ex(const uint8_t *payload, int len, int has_count,
                     obd_dtc *out, int max);

/* Short English description for a code in the form obd_dtc_parse()
 * produces, or NULL when this table does not know it. Unknown must stay
 * NULL: the screen shows the code alone rather than a made-up meaning. */
const char *obd_dtc_describe(const char *code);

/* PID 01 (four bytes, first one is the interesting one): bit 7 is the
 * MIL, bits 0..6 the number of stored codes.
 *
 * Both outputs are set to -1 when there is no answer to read, so the
 * caller can tell "unknown" (N/D) from "lamp off, zero codes". */
void obd_dtc_mil(const uint8_t *payload, int len, int *mil_on, int *count);

#endif
