#include "obd/pid.h"
#include "test_util.h"

#include <string.h>

static void decode_ok(uint8_t pid, const uint8_t *data, int len,
                      float expected, obd_unit unit)
{
    obd_pid_value value;
    int rc = obd_pid_decode(pid, data, len, &value);
    CHECK(rc == 0);
    CHECK(value.valid == 1);
    CHECK(value.unit == unit);
    CHECK_NEAR(value.value, expected, 0.01);
}

void test_pid(void)
{
    obd_pid_value value;

    /* Valores reales medidos en el MiTo (contacto puesto, motor parado). */
    const uint8_t rpm0[] = { 0x00, 0x00 };
    const uint8_t map97[] = { 0x61 };
    const uint8_t ect14[] = { 0x36 };
    const uint8_t iat17[] = { 0x39 };
    decode_ok(0x0C, rpm0, 2, 0.0f, OBD_UNIT_RPM);
    decode_ok(0x0B, map97, 1, 97.0f, OBD_UNIT_KPA);
    decode_ok(0x05, ect14, 1, 14.0f, OBD_UNIT_CELSIUS);
    decode_ok(0x0F, iat17, 1, 17.0f, OBD_UNIT_CELSIUS);

    /* Formulas. */
    const uint8_t rpm1726[] = { 0x1A, 0xF8 };
    const uint8_t speed50[] = { 0x32 };
    const uint8_t load128[] = { 0x80 };
    const uint8_t full[] = { 0xFF };
    const uint8_t zero_deg[] = { 0x80 };
    const uint8_t volts[] = { 0x2E, 0xE0 };
    const uint8_t baro98[] = { 0x62 };
    const uint8_t maf30[] = { 0x0B, 0xB8 };
    const uint8_t lph[] = { 0x00, 0xFA };
    const uint8_t trim0[] = { 0x80 };
    const uint8_t trim_min[] = { 0x00 };
    const uint8_t fuel_press[] = { 0x0A };
    const uint8_t rail_rel[] = { 0x00, 0x64 };
    const uint8_t rail_abs[] = { 0x00, 0x64 };
    const uint8_t seconds[] = { 0x01, 0x00 };
    const uint8_t km[] = { 0x00, 0x64 };
    const uint8_t oil60[] = { 0x64 };

    decode_ok(0x0C, rpm1726, 2, 1726.0f, OBD_UNIT_RPM);
    decode_ok(0x0D, speed50, 1, 50.0f, OBD_UNIT_KPH);
    decode_ok(0x04, load128, 1, 50.196f, OBD_UNIT_PERCENT);
    decode_ok(0x11, full, 1, 100.0f, OBD_UNIT_PERCENT);
    decode_ok(0x0E, zero_deg, 1, 0.0f, OBD_UNIT_DEGREES);
    decode_ok(0x42, volts, 2, 12.0f, OBD_UNIT_VOLT);
    decode_ok(0x33, baro98, 1, 98.0f, OBD_UNIT_KPA);
    decode_ok(0x10, maf30, 2, 30.0f, OBD_UNIT_GRAMS_SEC);
    decode_ok(0x5E, lph, 2, 12.5f, OBD_UNIT_LPH);
    decode_ok(0x2F, load128, 1, 50.196f, OBD_UNIT_PERCENT);
    decode_ok(0x06, trim0, 1, 0.0f, OBD_UNIT_PERCENT);
    decode_ok(0x06, trim_min, 1, -100.0f, OBD_UNIT_PERCENT);
    decode_ok(0x0A, fuel_press, 1, 30.0f, OBD_UNIT_KPA);
    decode_ok(0x22, rail_rel, 2, 7.9f, OBD_UNIT_KPA);
    decode_ok(0x23, rail_abs, 2, 1000.0f, OBD_UNIT_KPA);
    decode_ok(0x1F, seconds, 2, 256.0f, OBD_UNIT_SECONDS);
    decode_ok(0x21, km, 2, 100.0f, OBD_UNIT_KM);
    decode_ok(0x31, km, 2, 100.0f, OBD_UNIT_KM);
    decode_ok(0x49, load128, 1, 50.196f, OBD_UNIT_PERCENT);
    decode_ok(0x4A, load128, 1, 50.196f, OBD_UNIT_PERCENT);
    decode_ok(0x5A, load128, 1, 50.196f, OBD_UNIT_PERCENT);
    decode_ok(0x5C, oil60, 1, 60.0f, OBD_UNIT_CELSIUS);

    /* Sondas O2 de banda estrecha: A/200 V (dato real del MiTo). */
    const uint8_t o2_098[] = { 0xC4, 0x00 };
    decode_ok(0x14, o2_098, 2, 0.98f, OBD_UNIT_VOLT);
    decode_ok(0x15, o2_098, 2, 0.98f, OBD_UNIT_VOLT);

    /* Codificacion no estandar del MiTo, solo si se activa. */
    obd_pid_set_mito_encoding(1);
    const uint8_t mito_trim_zero[] = { 0x00 };
    const uint8_t mito_trim_neg[] = { 0xFB };
    const uint8_t mito_trim_pos[] = { 0x0D };
    const uint8_t mito_adv0[] = { 0x00 };
    const uint8_t mito_adv35[] = { 0x46 };
    const uint8_t mito_sentinel[] = { 0x7F };
    decode_ok(0x06, mito_trim_zero, 1, 0.0f, OBD_UNIT_PERCENT);
    decode_ok(0x07, mito_trim_neg, 1, -3.906f, OBD_UNIT_PERCENT);
    decode_ok(0x06, mito_trim_pos, 1, 10.156f, OBD_UNIT_PERCENT);
    decode_ok(0x0E, mito_adv0, 1, 0.0f, OBD_UNIT_DEGREES);
    decode_ok(0x0E, mito_adv35, 1, 35.0f, OBD_UNIT_DEGREES);
    CHECK(obd_pid_decode(0x0E, mito_sentinel, 1, &value) == -1);
    /* 0x80 con signo daria -100 %: se conserva la formula OBD. */
    decode_ok(0x06, trim0, 1, 0.0f, OBD_UNIT_PERCENT);
    obd_pid_set_mito_encoding(0);
    /* Sin el modo MiTo vuelve el avance estandar. */
    decode_ok(0x0E, mito_adv0, 1, -64.0f, OBD_UNIT_DEGREES);

    /* Longitudes y PID desconocidos. */
    CHECK(obd_pid_decode(0x0C, rpm1726, 1, &value) == -1);
    CHECK(obd_pid_decode(0x10, maf30, 1, &value) == -1);
    CHECK(obd_pid_decode(0x99, maf30, 2, &value) == -1);
    CHECK(obd_pid_decode(0x00, maf30, 4, &value) == -1);
    CHECK(obd_pid_decode(0x0C, NULL, 2, &value) == -1);

    /* Bytes esperados. */
    CHECK(obd_pid_expected_bytes(0x00) == 4);
    CHECK(obd_pid_expected_bytes(0x0C) == 2);
    CHECK(obd_pid_expected_bytes(0x0D) == 1);
    /* 0x13 no tiene decodificacion y por tanto no esta en la lista: mejor
     * fallar en la puerta que pasarla y llegar a un N/D contado como dato. */
    CHECK(obd_pid_expected_bytes(0x13) == 0);
    CHECK(obd_pid_expected_bytes(0x1B) == 1);
    CHECK(obd_pid_expected_bytes(0x14) == 2);
    CHECK(obd_pid_expected_bytes(0x99) == 0);

    /* Mascara real del MiTo: 41 00 BE 3E B8 11. */
    const uint8_t mask_bytes[] = { 0xBE, 0x3E, 0xB8, 0x11 };
    uint32_t mask = obd_pid_mask(mask_bytes, 4);
    CHECK(mask == 0xBE3EB811u);
    CHECK(obd_mask_supports(mask, 0x01) == 1);
    CHECK(obd_mask_supports(mask, 0x03) == 1);
    CHECK(obd_mask_supports(mask, 0x04) == 1);
    CHECK(obd_mask_supports(mask, 0x05) == 1);
    CHECK(obd_mask_supports(mask, 0x06) == 1);
    CHECK(obd_mask_supports(mask, 0x07) == 1);
    CHECK(obd_mask_supports(mask, 0x0B) == 1);
    CHECK(obd_mask_supports(mask, 0x0C) == 1);
    CHECK(obd_mask_supports(mask, 0x0D) == 1);
    CHECK(obd_mask_supports(mask, 0x0E) == 1);
    CHECK(obd_mask_supports(mask, 0x0F) == 1);
    CHECK(obd_mask_supports(mask, 0x11) == 1);
    CHECK(obd_mask_supports(mask, 0x13) == 1);
    CHECK(obd_mask_supports(mask, 0x14) == 1);
    CHECK(obd_mask_supports(mask, 0x15) == 1);
    CHECK(obd_mask_supports(mask, 0x1C) == 1);
    CHECK(obd_mask_supports(mask, 0x20) == 1);
    CHECK(obd_mask_supports(mask, 0x02) == 0);
    CHECK(obd_mask_supports(mask, 0x09) == 0);
    CHECK(obd_mask_supports(mask, 0x0A) == 0);
    CHECK(obd_mask_supports(mask, 0x10) == 0);
    CHECK(obd_mask_supports(mask, 0x12) == 0);
    CHECK(obd_mask_supports(mask, 0x1F) == 0);
    CHECK(obd_mask_supports(mask, 0x21) == 0);
    CHECK(obd_mask_supports(mask, 0x00) == 0);

    /* Mascara real del MiTo: 41 40 C0 C0 00 00. */
    const uint8_t mask40_bytes[] = { 0xC0, 0xC0, 0x00, 0x00 };
    uint32_t mask40 = obd_pid_mask(mask40_bytes, 4);
    CHECK(mask40 == 0xC0C00000u);
    CHECK(obd_mask_supports(mask40, 0x01) == 1);   /* 0x41 */
    CHECK(obd_mask_supports(mask40, 0x02) == 1);   /* 0x42 */
    CHECK(obd_mask_supports(mask40, 0x09) == 1);   /* 0x49 pedal D */
    CHECK(obd_mask_supports(mask40, 0x0A) == 1);   /* 0x4A pedal E */
    CHECK(obd_mask_supports(mask40, 0x1A) == 0);   /* 0x5A no soportado */
    CHECK(obd_mask_supports(mask40, 0x1C) == 0);   /* 0x5C aceite no soportado */
    CHECK(obd_mask_supports(mask40, 0x1E) == 0);   /* 0x5E caudal no soportado */

    /* Simbolos de unidad. */
    CHECK(strcmp(obd_unit_symbol(OBD_UNIT_RPM), "rpm") == 0);
    CHECK(strcmp(obd_unit_symbol(OBD_UNIT_KPA), "kPa") == 0);
    CHECK(strcmp(obd_unit_symbol(OBD_UNIT_LPH), "L/h") == 0);
    CHECK(strcmp(obd_unit_symbol(OBD_UNIT_NONE), "") == 0);
}
