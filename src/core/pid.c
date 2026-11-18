#include "obd/pid.h"

#include <stddef.h>

int obd_pid_expected_bytes(uint8_t pid)
{
    switch (pid) {
    case 0x00:
    case 0x20:
    case 0x40:
    case 0x60:
    case 0x80:
    case 0xA0:
    case 0xC0:
    case 0xE0:
        return 4;

    case 0x04:
    case 0x05:
    case 0x06:
    case 0x07:
    case 0x0A:
    case 0x0B:
    case 0x0D:
    case 0x0E:
    case 0x0F:
    case 0x11:
    case 0x1B:
    case 0x2F:
    case 0x33:
    case 0x49:
    case 0x4A:
    case 0x5A:
    case 0x5C:
        return 1;

    case 0x0C:
    case 0x10:
    case 0x1F:
    case 0x21:
    case 0x22:
    case 0x23:
    case 0x31:
    case 0x42:
    case 0x5E:
        return 2;

    default:
        if (pid >= 0x14 && pid <= 0x1B) {
            return 2;  /* sondas O2 de banda estrecha */
        }
        return 0;
    }
}

static int g_mito_encoding = 0;

void obd_pid_set_mito_encoding(int enabled)
{
    g_mito_encoding = enabled ? 1 : 0;
}

static void set_value(obd_pid_value *out, float value, obd_unit unit)
{
    out->value = value;
    out->unit = unit;
    out->valid = 1;
}

int obd_pid_decode(uint8_t pid, const uint8_t *data, int len,
                   obd_pid_value *out)
{
    if (out == NULL) {
        return -1;
    }
    out->value = 0.0f;
    out->unit = OBD_UNIT_NONE;
    out->valid = 0;

    if (data == NULL || len < 0) {
        return -1;
    }

    int need = obd_pid_expected_bytes(pid);
    if (need == 0 || len < need) {
        return -1;
    }

    switch (pid) {
    case 0x04:
        set_value(out, data[0] * 100.0f / 255.0f, OBD_UNIT_PERCENT);
        break;
    case 0x05:
        set_value(out, (float)data[0] - 40.0f, OBD_UNIT_CELSIUS);
        break;
    case 0x06:
    case 0x07:
        if (g_mito_encoding) {
            /* Byte con signo (0x00 = 0 %, 0xFB = -3.9 %). Si el valor
             * con signo fuese absurdo se mantiene la formula OBD. */
            float alt = (float)(int8_t)data[0] / 1.28f;
            if (alt >= -60.0f && alt <= 60.0f) {
                set_value(out, alt, OBD_UNIT_PERCENT);
            } else {
                set_value(out, (float)data[0] / 1.28f - 100.0f,
                          OBD_UNIT_PERCENT);
            }
        } else {
            set_value(out, (float)data[0] / 1.28f - 100.0f,
                      OBD_UNIT_PERCENT);
        }
        break;
    case 0x0A:
        set_value(out, data[0] * 3.0f, OBD_UNIT_KPA);
        break;
    case 0x0B:
        set_value(out, (float)data[0], OBD_UNIT_KPA);
        break;
    case 0x0C:
        set_value(out, (data[0] * 256.0f + data[1]) / 4.0f,
                  OBD_UNIT_RPM);
        break;
    case 0x0D:
        set_value(out, (float)data[0], OBD_UNIT_KPH);
        break;
    case 0x0E:
        if (g_mito_encoding) {
            /* En esta ECU el byte ya es el avance en medios grados;
             * 0x7F/0xFF se usan como "sin dato". */
            if (data[0] == 0x7F || data[0] == 0xFF) {
                return -1;
            }
            set_value(out, (float)data[0] / 2.0f, OBD_UNIT_DEGREES);
        } else {
            set_value(out, (float)data[0] / 2.0f - 64.0f,
                      OBD_UNIT_DEGREES);
        }
        break;
    case 0x0F:
        set_value(out, (float)data[0] - 40.0f, OBD_UNIT_CELSIUS);
        break;
    case 0x10:
        set_value(out, (data[0] * 256.0f + data[1]) / 100.0f,
                  OBD_UNIT_GRAMS_SEC);
        break;
    case 0x11:
        set_value(out, data[0] * 100.0f / 255.0f, OBD_UNIT_PERCENT);
        break;
    case 0x14:
    case 0x15:
    case 0x16:
    case 0x17:
    case 0x18:
    case 0x19:
    case 0x1A:
    case 0x1B:
        /* Sondas O2 de banda estrecha: A/200 = tension (V). */
        set_value(out, (float)data[0] / 200.0f, OBD_UNIT_VOLT);
        break;
    case 0x1F:
        set_value(out, data[0] * 256.0f + data[1], OBD_UNIT_SECONDS);
        break;
    case 0x21:
        set_value(out, data[0] * 256.0f + data[1], OBD_UNIT_KM);
        break;
    case 0x22:
        set_value(out, (data[0] * 256.0f + data[1]) * 0.079f,
                  OBD_UNIT_KPA);
        break;
    case 0x23:
        set_value(out, (data[0] * 256.0f + data[1]) * 10.0f,
                  OBD_UNIT_KPA);
        break;
    case 0x2F:
        set_value(out, data[0] * 100.0f / 255.0f, OBD_UNIT_PERCENT);
        break;
    case 0x31:
        set_value(out, data[0] * 256.0f + data[1], OBD_UNIT_KM);
        break;
    case 0x33:
        set_value(out, (float)data[0], OBD_UNIT_KPA);
        break;
    case 0x42:
        set_value(out, (data[0] * 256.0f + data[1]) / 1000.0f,
                  OBD_UNIT_VOLT);
        break;
    case 0x49:
    case 0x4A:
        set_value(out, data[0] * 100.0f / 255.0f, OBD_UNIT_PERCENT);
        break;
    case 0x5A:
        set_value(out, data[0] * 100.0f / 255.0f, OBD_UNIT_PERCENT);
        break;
    case 0x5C:
        set_value(out, (float)data[0] - 40.0f, OBD_UNIT_CELSIUS);
        break;
    case 0x5E:
        set_value(out, (data[0] * 256.0f + data[1]) / 20.0f,
                  OBD_UNIT_LPH);
        break;
    default:
        return -1;
    }

    return 0;
}

const char *obd_unit_symbol(obd_unit unit)
{
    switch (unit) {
    case OBD_UNIT_RPM: return "rpm";
    case OBD_UNIT_KPH: return "km/h";
    case OBD_UNIT_CELSIUS: return "C";
    case OBD_UNIT_KPA: return "kPa";
    case OBD_UNIT_PERCENT: return "%";
    case OBD_UNIT_VOLT: return "V";
    case OBD_UNIT_DEGREES: return "deg";
    case OBD_UNIT_GRAMS_SEC: return "g/s";
    case OBD_UNIT_LPH: return "L/h";
    case OBD_UNIT_KM: return "km";
    case OBD_UNIT_SECONDS: return "s";
    default: return "";
    }
}

uint32_t obd_pid_mask(const uint8_t *data, int len)
{
    if (data == NULL || len < 4) {
        return 0;
    }
    return ((uint32_t)data[0] << 24) |
           ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) |
           (uint32_t)data[3];
}

int obd_mask_supports(uint32_t mask, uint8_t pid)
{
    if (pid < 1 || pid > 32) {
        return 0;
    }
    return (int)((mask >> (31 - (pid - 1))) & 1u);
}
