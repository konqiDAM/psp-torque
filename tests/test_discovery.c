#include "obd/discovery.h"
#include "test_util.h"

#include <string.h>

static void check_line(const char *text, const char *header,
                       int data_len)
{
    obd_disc_line line;
    int ok = obd_disc_parse_line(text, &line);
    CHECK(ok == 1);
    CHECK(strcmp(line.header, header) == 0);
    CHECK(line.data_len == data_len);
}

void test_discovery(void)
{
    obd_disc_line line;
    const uint8_t *payload = NULL;
    int len;

    /* Tramas OBD con cabecera de 11 bits y PCI de single frame. */
    check_line("7E8 06 41 00 BE 3E B8 11", "7E8", 7);
    CHECK(obd_disc_parse_line("7E8 06 41 00 BE 3E B8 11", &line) == 1);
    len = obd_disc_payload(&line, &payload);
    CHECK(len == 6);
    CHECK(payload[0] == 0x41);
    CHECK(payload[1] == 0x00);
    CHECK(obd_disc_mask(payload + 2, len - 2) == 0xBE3EB811u);

    /* Sin PCI, como algunos clones con CAF0. */
    check_line("41 00 BE 3E B8 11", "", 6);
    CHECK(obd_disc_parse_line("41 00 BE 3E B8 11", &line) == 1);
    len = obd_disc_payload(&line, &payload);
    CHECK(len == 6);
    CHECK(obd_disc_mask(payload + 2, len - 2) == 0xBE3EB811u);

    /* Cabecera de 29 bits junta y con espacios. */
    check_line("18DAF110 06 41 00 BE 3E B8 11", "18DAF110", 7);
    check_line("18 DA F1 10 06 41 00 BE 3E B8 11", "18DAF110", 7);

    /* Respuesta positiva UDS a Tester Present: 7E 00. */
    check_line("7E8 02 7E 00", "7E8", 3);
    CHECK(obd_disc_parse_line("7E8 02 7E 00", &line) == 1);
    len = obd_disc_payload(&line, &payload);
    CHECK(len == 2);
    CHECK(payload[0] == 0x7E);
    CHECK(payload[1] == 0x00);

    /* Respuesta negativa UDS: 7F 22 31. */
    check_line("7E8 03 7F 22 31", "7E8", 4);
    CHECK(obd_disc_parse_line("7E8 03 7F 22 31", &line) == 1);
    len = obd_disc_payload(&line, &payload);
    CHECK(len == 3);
    CHECK(payload[0] == 0x7F);
    CHECK(payload[1] == 0x22);
    CHECK(payload[2] == 0x31);

    /* First frame ISO-TP: 10 14 62 F1 90 ... */
    check_line("7E8 10 14 62 F1 90 57 50 30", "7E8", 8);
    CHECK(obd_disc_parse_line("7E8 10 14 62 F1 90 57 50 30", &line) == 1);
    len = obd_disc_payload(&line, &payload);
    CHECK(len == 6);
    CHECK(payload[0] == 0x62);
    CHECK(payload[1] == 0xF1);
    CHECK(payload[2] == 0x90);

    /* Consecutive frame: 21 5A 5A ... */
    check_line("7E8 21 5A 5A 5A 5A 5A 5A 5A", "7E8", 8);
    CHECK(obd_disc_parse_line("7E8 21 5A 5A 5A 5A 5A 5A 5A", &line) == 1);
    len = obd_disc_payload(&line, &payload);
    CHECK(len == 7);
    CHECK(payload[0] == 0x5A);

    /* Un flow control no es datos. */
    check_line("7E8 30 00 00", "7E8", 3);
    CHECK(obd_disc_parse_line("7E8 30 00 00", &line) == 1);
    CHECK(obd_disc_payload(&line, &payload) == 0);

    /* Prefijo "0:" de mensajes largos. */
    check_line("0: 62 F1 87 35 35 32 30 30", "", 8);
    CHECK(obd_disc_parse_line("0: 62 F1 87 35 35 32 30 30", &line) == 1);
    len = obd_disc_payload(&line, &payload);
    CHECK(len == 8);
    CHECK(payload[0] == 0x62);

    /* Conversion ASCII. */
    char text[24];
    const uint8_t vin[] = { 0x5A, 0x41, 0x52, 0x41, 0x53, 0x30, 0x30,
                            0x30, 0x30, 0x00, 0xAA };
    obd_disc_ascii(vin, (int)sizeof(vin), text, sizeof(text));
    CHECK(strcmp(text, "ZARAS0000") == 0);

    /* K-line: la cabecera son TRES bytes. Esta es la linea real que
     * capturo la MX-5 NB2 (docs/COMPATIBILIDAD_MX5_NB2.md); hasta ahora
     * los tres bytes se leian como datos y la mascara salia basura. */
    check_line("48 6B 10 41 00 BE 1F B8 11 AA", "486B10", 7);
    CHECK(obd_disc_parse_line("48 6B 10 41 00 BE 1F B8 11 AA", &line) == 1);
    CHECK(line.data[0] == 0x41);
    len = obd_disc_payload(&line, &payload);
    CHECK(len == 7);
    CHECK(obd_disc_mask(payload + 2, 4) == 0xBE1FB811u);

    /* Y un rechazo tambien trae cabecera. */
    check_line("48 6B 10 7F 01 12", "486B10", 3);

    /* Pero sin cabeceras activadas la misma respuesta NO se puede tomar
     * por una cabecera: el cuarto byte no es una respuesta. */
    check_line("41 00 BE 1F B8 11 AA", "", 7);
    check_line("41 0C 1A F8", "", 4);
    check_line("43 00 00 00 00 00 00", "", 7);

    /* Mascaras reales del MiTo. */
    CHECK(obd_disc_mask((const uint8_t[]){0xBE,0x3E,0xB8,0x11}, 4) ==
          0xBE3EB811u);
    CHECK(obd_disc_mask((const uint8_t[]){0x80,0x02,0x20,0x01}, 4) ==
          0x80022001u);
    CHECK(obd_disc_mask((const uint8_t[]){0xC0,0xC0,0x00,0x00}, 4) ==
          0xC0C00000u);
}
