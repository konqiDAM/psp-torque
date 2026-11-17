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
    { "ALFA_MITO_QV_2010", "Alfa Romeo MiTo QV", OBD_PROTO_CAN,
      0xBE3EB811u, 0x80022001u, 0xC0C00000u,
      7000, 6500, 0, OBD_CENTRE_BOOST, 0 },
    { "MAZDA_MX5_NB2", "Mazda MX-5 NB2", OBD_PROTO_KLINE,
      0xBE1FB811u, 0x80000000u, 0x00000000u,
      8000, 7000, 7200, OBD_CENTRE_MAF, 0 },
    { "GENERIC_CAN", "Generic CAN", OBD_PROTO_CAN,
      0, 0, 0,
      7000, 6500, 0, OBD_CENTRE_FROM_CAPS, 1 },
    { "GENERIC_KLINE", "Generic K-line", OBD_PROTO_KLINE,
      0, 0, 0,
      7000, 6500, 0, OBD_CENTRE_FROM_CAPS, 1 }
};

#define PROFILE_COUNT ((int)(sizeof(g_profiles) / sizeof(g_profiles[0])))

int obd_vehicle_proto_class(const char *atdpn)
{
    if (atdpn == NULL || atdpn[0] == '\0') {
        return OBD_PROTO_ANY;
    }
    size_t n = strlen(atdpn);
    switch (atdpn[n - 1]) {
    case '3':
        return OBD_PROTO_ISO9141;
    case '5':
        return OBD_PROTO_KWP;
    case '6':
    case '8':
    case '7':
    case '9':
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

static int mask_matches(uint32_t want, uint32_t have)
{
    return want == 0 || want == have;
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
            if (p->generic || p->mask1 == 0) {
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
