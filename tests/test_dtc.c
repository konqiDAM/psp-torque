#include "obd/dtc.h"
#include "test_util.h"

#include <string.h>

/* "P0133": one letter of the four ISO classes and four hexadecimal
 * digits, always the same width. */
static int is_code(const char *code)
{
    static const char letters[] = "PCBU";
    static const char digits[] = "0123456789ABCDEF";

    if (strlen(code) != 5) {
        return 0;
    }
    if (strchr(letters, code[0]) == NULL) {
        return 0;
    }
    for (int i = 1; i < 5; i++) {
        if (strchr(digits, code[i]) == NULL) {
            return 0;
        }
    }
    return 1;
}

void test_dtc(void)
{
    obd_dtc codes[8];
    int n;

    /* K-line answer with headers off: 43 01 33 00 00 00 00. The bytes
     * after the service byte carry no count here: P0133 and padding. */
    const uint8_t kl_one[] = { 0x01, 0x33, 0x00, 0x00, 0x00, 0x00 };
    n = obd_dtc_parse(kl_one, 6, codes, 8);
    CHECK(n == 1);
    CHECK(strcmp(codes[0].code, "P0133") == 0);
    CHECK(is_code(codes[0].code) == 1);

    /* CAN answer: 43 02 01 33 03 00 00 00. The first byte is the count
     * the ECU sends, then P0133 and P0300. */
    const uint8_t can_two[] = { 0x02, 0x01, 0x33, 0x03, 0x00, 0x00,
                                0x00 };
    n = obd_dtc_parse(can_two, 7, codes, 8);
    CHECK(n == 2);
    CHECK(strcmp(codes[0].code, "P0133") == 0);
    CHECK(strcmp(codes[1].code, "P0300") == 0);

    /* The layout is a decision the caller can pin, not a guess hidden in
     * the parser. Read as CAN: count 2, two codes. Read as K-line: the
     * count byte is taken as a code and the answer is wrong on purpose. */
    n = obd_dtc_parse_ex(can_two, 7, 1, codes, 8);
    CHECK(n == 2);
    CHECK(strcmp(codes[0].code, "P0133") == 0);
    CHECK(strcmp(codes[1].code, "P0300") == 0);
    n = obd_dtc_parse_ex(can_two, 7, 0, codes, 8);
    CHECK(n == 2);
    CHECK(strcmp(codes[0].code, "P0201") == 0);
    CHECK(strcmp(codes[1].code, "P3303") == 0);

    /* Letters: byte 1 bits 7..6 pick powertrain, chassis, body or
     * network. 43 00 -> C0300, 81 00 -> B0100, C1 00 -> U0100. */
    const uint8_t chassis[] = { 0x43, 0x00 };
    const uint8_t body[] = { 0x81, 0x00 };
    const uint8_t network[] = { 0xC1, 0x00 };
    n = obd_dtc_parse(chassis, 2, codes, 8);
    CHECK(n == 1);
    CHECK(strcmp(codes[0].code, "C0300") == 0);
    n = obd_dtc_parse(body, 2, codes, 8);
    CHECK(n == 1);
    CHECK(strcmp(codes[0].code, "B0100") == 0);
    n = obd_dtc_parse(network, 2, codes, 8);
    CHECK(n == 1);
    CHECK(strcmp(codes[0].code, "U0100") == 0);

    /* Digits above nine are hexadecimal, not decimal. */
    const uint8_t hex_digits[] = { 0x0A, 0x0A };
    n = obd_dtc_parse(hex_digits, 2, codes, 8);
    CHECK(n == 1);
    CHECK(strcmp(codes[0].code, "P0A0A") == 0);

    /* Padding: zero pairs are skipped wherever they appear. */
    const uint8_t gaps[] = { 0x01, 0x33, 0x00, 0x00, 0x03, 0x00 };
    n = obd_dtc_parse(gaps, 6, codes, 8);
    CHECK(n == 2);
    CHECK(strcmp(codes[0].code, "P0133") == 0);
    CHECK(strcmp(codes[1].code, "P0300") == 0);

    /* Truncated payloads. A count byte with nothing behind it is no
     * codes, not a crash. */
    const uint8_t only_count[] = { 0x01 };
    n = obd_dtc_parse(only_count, 1, codes, 8);
    CHECK(n == 0);

    /* A K-line frame cut in the middle of a pair keeps the whole pairs. */
    const uint8_t cut_pair[] = { 0x01, 0x33, 0x03 };
    n = obd_dtc_parse_ex(cut_pair, 3, 0, codes, 8);
    CHECK(n == 1);
    CHECK(strcmp(codes[0].code, "P0133") == 0);

    /* A count larger than the payload: the codes that are there are
     * real, so they are decoded and the count is not trusted. */
    const uint8_t short_of_count[] = { 0x03, 0x01, 0x33, 0x03 };
    n = obd_dtc_parse_ex(short_of_count, 4, 1, codes, 8);
    CHECK(n == 1);
    CHECK(strcmp(codes[0].code, "P0133") == 0);

    /* No length from 0 to 12 may read outside the payload or produce
     * anything that is not a code. */
    const uint8_t buffer[] = { 0x00, 0x01, 0x33, 0x00, 0x03, 0x00,
                               0x00, 0x42, 0x01, 0xC1, 0x00, 0xFF };
    for (int len = 0; len <= 12; len++) {
        n = obd_dtc_parse(buffer, len, codes, 8);
        CHECK(n >= 0);
        CHECK(n <= len / 2);
        for (int i = 0; i < n; i++) {
            CHECK(is_code(codes[i].code) == 1);
        }
    }

    /* Output buffer: three codes in the payload, room for two. */
    const uint8_t three[] = { 0x01, 0x33, 0x03, 0x00, 0x07, 0x00 };
    n = obd_dtc_parse(three, 6, codes, 2);
    CHECK(n == 2);
    CHECK(strcmp(codes[0].code, "P0133") == 0);
    CHECK(strcmp(codes[1].code, "P0300") == 0);
    CHECK(obd_dtc_parse(three, 6, codes, 0) == 0);

    /* Arguments. */
    CHECK(obd_dtc_parse(NULL, 6, codes, 8) == -1);
    CHECK(obd_dtc_parse(three, -1, codes, 8) == -1);
    CHECK(obd_dtc_parse(three, 6, NULL, 8) == -1);
    CHECK(obd_dtc_parse(three, 6, codes, -1) == -1);
    CHECK(obd_dtc_parse(three, 0, codes, 8) == 0);

    /* Descriptions: the code alone when the table does not know it. */
    CHECK(strcmp(obd_dtc_describe("P0133"),
                 "O2 sensor slow response (bank 1 sensor 1)") == 0);
    CHECK(strcmp(obd_dtc_describe("P0171"),
                 "System too lean (bank 1)") == 0);
    CHECK(strcmp(obd_dtc_describe("P0300"),
                 "Random/multiple cylinder misfire") == 0);
    CHECK(strcmp(obd_dtc_describe("P0420"),
                 "Catalyst efficiency below threshold (bank 1)") == 0);
    CHECK(strcmp(obd_dtc_describe("P0500"),
                 "Vehicle speed sensor circuit") == 0);
    CHECK(strcmp(obd_dtc_describe("P0700"),
                 "Transmission control system (MIL request)") == 0);

    /* Not in the table: a code outside the generic range, a chassis
     * code, a manufacturer code, the wrong width, lowercase, NULL. */
    CHECK(obd_dtc_describe("P0999") == NULL);
    CHECK(obd_dtc_describe("C0300") == NULL);
    CHECK(obd_dtc_describe("P1234") == NULL);
    CHECK(obd_dtc_describe("P013") == NULL);
    CHECK(obd_dtc_describe("p0133") == NULL);
    CHECK(obd_dtc_describe("") == NULL);
    CHECK(obd_dtc_describe(NULL) == NULL);

    /* End to end: a decoded code finds its description. */
    n = obd_dtc_parse(kl_one, 6, codes, 8);
    CHECK(n == 1);
    CHECK(is_code(codes[0].code) == 1);
    if (n == 1) {
        CHECK(obd_dtc_describe(codes[0].code) != NULL);
    }

    /* PID 01: bit 7 is the lamp, bits 0..6 the stored code count. */
    int mil = -9;
    int count = -9;
    const uint8_t lamp_on[] = { 0x81, 0x03, 0x00, 0x00 };
    obd_dtc_mil(lamp_on, 4, &mil, &count);
    CHECK(mil == 1);
    CHECK(count == 1);

    const uint8_t lamp_off[] = { 0x03, 0x00, 0x00, 0x00 };
    obd_dtc_mil(lamp_off, 4, &mil, &count);
    CHECK(mil == 0);
    CHECK(count == 3);

    const uint8_t lamp_on_max[] = { 0xFF, 0x00, 0x00, 0x00 };
    obd_dtc_mil(lamp_on_max, 4, &mil, &count);
    CHECK(mil == 1);
    CHECK(count == 0x7F);

    /* No byte to read: unknown, so the screen can say N/D instead of
     * "lamp off, no codes". */
    obd_dtc_mil(lamp_off, 0, &mil, &count);
    CHECK(mil == -1);
    CHECK(count == -1);
    obd_dtc_mil(NULL, 4, &mil, &count);
    CHECK(mil == -1);
    CHECK(count == -1);

    /* Null outputs are allowed, as is asking for only one of them. */
    obd_dtc_mil(lamp_on, 4, NULL, NULL);
    obd_dtc_mil(lamp_on, 4, &mil, NULL);
    CHECK(mil == 1);
    obd_dtc_mil(lamp_off, 4, NULL, &count);
    CHECK(count == 3);
}
