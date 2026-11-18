#include "obd/vehicle.h"

#include <stddef.h>
#include <string.h>

/* Los valores del MiTo son los que ya usaba la app por defecto (7000 de
 * fondo, rojo a 6500): detectarlo no debe cambiarle nada.
 *
 * La MX-5 NB2 llega por ISO 9141-2 con BE1FB811. Su aguja util sigue
 * hasta 7500, asi que el fondo de escala es 8000, la zona roja empieza en
 * 7000 y el corte esta en 7200. No anuncia MAP (0x0B), asi que el gauge
 * central es MAF. */
static const obd_vehicle_profile g_profiles[] = {
    /* MiTo: sin relaciones de cambio verificadas, asi que el indicador de
     * marcha dice N/D en vez de adivinar. */
    { "ALFA_MITO_QV_2010", "Alfa Romeo MiTo QV", OBD_PROTO_CAN,
      0xBE3EB811u, 0x80022001u, 0xC0C00000u,
      7000, 6500, 0, OBD_CENTRE_BOOST, 0, { 0 } },
    /* MX-5 NB 1.8 5 marchas: 3.136, 1.888, 1.330, 1.000, 0.814 y grupo
     * 4.100 con 205/45R16 (1,87 m de circunferencia). km/h por 1000 rpm =
     * 60 * circunferencia / (relacion * grupo):
     *   8.73, 14.50, 20.58, 27.37, 33.62
     * A 3000 rpm en quinta salen 101 km/h, que es lo que marca el coche. */
    { "MAZDA_MX5_NB2", "Mazda MX-5 NB2", OBD_PROTO_KLINE,
      0xBE1FB811u, 0x80000000u, 0x00000000u,
      8000, 7000, 7200, OBD_CENTRE_MAF, 0,
      { 8.73f, 14.50f, 20.58f, 27.37f, 33.62f, 0.0f } },
    { "GENERIC_CAN", "Generic CAN", OBD_PROTO_CAN,
      OBD_VEHICLE_ANY_MASK, OBD_VEHICLE_ANY_MASK, OBD_VEHICLE_ANY_MASK,
      7000, 6500, 0, OBD_CENTRE_FROM_CAPS, 1, { 0 } },
    { "GENERIC_KLINE", "Generic K-line", OBD_PROTO_KLINE,
      OBD_VEHICLE_ANY_MASK, OBD_VEHICLE_ANY_MASK, OBD_VEHICLE_ANY_MASK,
      7000, 6500, 0, OBD_CENTRE_FROM_CAPS, 1, { 0 } }
};

#define PROFILE_COUNT ((int)(sizeof(g_profiles) / sizeof(g_profiles[0])))

int obd_vehicle_proto_class(const char *atdpn)
{
    if (atdpn == NULL || atdpn[0] == '\0') {
        return OBD_PROTO_ANY;
    }
    size_t n = strlen(atdpn);
    switch (atdpn[n - 1]) {
    case '0':
        /* "A0": el adaptador todavia no ha elegido protocolo. */
        return OBD_PROTO_ANY;
    case '3':
    case '4':   /* ISO 14230-4, inicio a 5 baudios */
        return OBD_PROTO_ISO9141;
    case '5':
        return OBD_PROTO_KWP;
    case '6':
    case '8':
    case '7':
    case '9':
    case 'A':
    case 'B':
    case 'C':   /* CAN con las variantes de 29 bits anunciadas */
        return OBD_PROTO_CAN;
    default:
        return OBD_PROTO_ANY;
    }
}

static int proto_matches(int want, int have)
{
    if (want == OBD_PROTO_ANY) {
        return 1;
    }
    if (have == OBD_PROTO_ANY) {
        /* Sin ATDPN no se afirma identidad: como mucho, generico. */
        return 0;
    }
    if (want == OBD_PROTO_KLINE) {
        return have == OBD_PROTO_ISO9141 || have == OBD_PROTO_KWP;
    }
    return want == have;
}

/* El comodin es OBD_VEHICLE_ANY_MASK, no el cero: un perfil puede exigir
 * que un bloque NO exista (por ejemplo el 0140 de la MX-5). */
static int mask_matches(uint32_t want, uint32_t have)
{
    return want == OBD_VEHICLE_ANY_MASK || want == have;
}

const obd_vehicle_profile *obd_vehicle_by_id(const char *id)
{
    if (id == NULL || id[0] == '\0') {
        return NULL;
    }
    for (int i = 0; i < PROFILE_COUNT; i++) {
        if (strcmp(g_profiles[i].id, id) == 0) {
            return &g_profiles[i];
        }
    }
    return NULL;
}

const obd_vehicle_profile *obd_vehicle_match(int proto, uint32_t mask1,
                                             uint32_t mask2, uint32_t mask3,
                                             const char *forced_id,
                                             int *exact)
{
    if (exact != NULL) {
        *exact = 0;
    }

    const obd_vehicle_profile *forced = obd_vehicle_by_id(forced_id);
    if (forced != NULL) {
        return forced;
    }

    /* Identidad: hace falta mascara 0100 (sin ella no hay nada que
     * afirmar) y las tres tienen que casar. */
    if (mask1 != 0) {
        for (int i = 0; i < PROFILE_COUNT; i++) {
            const obd_vehicle_profile *p = &g_profiles[i];
            if (p->generic || p->mask1 == OBD_VEHICLE_ANY_MASK) {
                continue;
            }
            if (proto_matches(p->proto, proto) &&
                mask_matches(p->mask1, mask1) &&
                mask_matches(p->mask2, mask2) &&
                mask_matches(p->mask3, mask3)) {
                if (exact != NULL) {
                    *exact = 1;
                }
                return p;
            }
        }
    }

    /* Capabilidad: generico de la clase. */
    for (int i = 0; i < PROFILE_COUNT; i++) {
        const obd_vehicle_profile *p = &g_profiles[i];
        if (p->generic && proto_matches(p->proto, proto)) {
            return p;
        }
    }
    return &g_profiles[PROFILE_COUNT - 1];
}

int obd_vehicle_gear(const float *gear_kmh, float rpm, float speed_kmh)
{
    if (gear_kmh == NULL || rpm < 500.0f || speed_kmh < 3.0f) {
        return 0;
    }

    /* km/h que deberia marcar en cada marcha a esas rpm. */
    int best = 0;
    float best_err = 0.0f;
    for (int g = 0; g < 6; g++) {
        if (gear_kmh[g] <= 0.0f) {
            break;
        }
        float expected = rpm / 1000.0f * gear_kmh[g];
        if (expected < 1.0f) {
            continue;
        }
        float err = (speed_kmh > expected) ? (speed_kmh - expected) / expected
                                          : (expected - speed_kmh) / expected;
        if (best == 0 || err < best_err) {
            best = g + 1;
            best_err = err;
        }
    }

    /* Un 10% de margen: entre marchas hay mas de un 15% de diferencia, asi
     * que con esto no se confunde una con otra por ruido de redondeo. */
    return (best_err <= 0.10f) ? best : 0;
}

