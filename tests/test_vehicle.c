#include "obd/vehicle.h"
#include "test_util.h"

#include <string.h>

/* FASE U4: el perfil de coche decide tacometro, zona roja, corte y gauge
 * central. Estos tests fijan dos cosas que importan en el coche: que la
 * MX-5 de ivan se reconozca por su evidencia, y que nunca se confunda con
 * el MiTo, porque en esa confusion va el decodificado con signo. */

/* FASE U4b: marcha deducida de rpm y velocidad con las relaciones del
 * perfil. En la MX-5 a 3000 rpm en quinta son 101 km/h; si no cuadra con
 * ninguna marcha, 0 (N/D) en vez de una marcha inventada. */
static void test_gear(void)
{
    const float mx5[6] = { 8.73f, 14.50f, 20.58f, 27.37f, 33.62f, 0.0f };

    CHECK(obd_vehicle_gear(mx5, 3000.0f, 100.8f) == 5);
    CHECK(obd_vehicle_gear(mx5, 3000.0f, 26.2f) == 1);
    CHECK(obd_vehicle_gear(mx5, 3000.0f, 43.5f) == 2);
    CHECK(obd_vehicle_gear(mx5, 3000.0f, 61.7f) == 3);
    CHECK(obd_vehicle_gear(mx5, 3000.0f, 82.1f) == 4);

    /* Parado, sin RPM o sin tabla: no se sabe. */
    CHECK(obd_vehicle_gear(mx5, 800.0f, 0.0f) == 0);
    CHECK(obd_vehicle_gear(mx5, 0.0f, 50.0f) == 0);
    CHECK(obd_vehicle_gear(NULL, 3000.0f, 50.0f) == 0);

    /* Una velocidad que no corresponde a ninguna marcha con esas rpm
     * (embrague pisado, rueda patinando) no devuelve una marcha falsa. */
    CHECK(obd_vehicle_gear(mx5, 6000.0f, 20.0f) == 0);

    /* El perfil del MiTo no trae relaciones: el indicador dice N/D. */
    const obd_vehicle_profile *p =
        obd_vehicle_match(OBD_PROTO_CAN, 0xBE3EB811u, 0x80022001u,
                          0xC0C00000u, NULL, NULL);
    CHECK(p != NULL);
    CHECK(obd_vehicle_gear(p->gear_kmh, 3000.0f, 100.0f) == 0);

    /* Y la MX-5 las trae. */
    p = obd_vehicle_match(OBD_PROTO_ISO9141, 0xBE1FB811u, 0x80000000u, 0u,
                          NULL, NULL);
    CHECK(p != NULL);
    CHECK(obd_vehicle_gear(p->gear_kmh, 3000.0f, 100.8f) == 5);
}

void test_vehicle(void)
{
    int exact = 0;
    const obd_vehicle_profile *p;

    /* Clase de protocolo desde la respuesta de ATDPN. */
    CHECK(obd_vehicle_proto_class("A3") == OBD_PROTO_ISO9141);
    CHECK(obd_vehicle_proto_class("3") == OBD_PROTO_ISO9141);
    CHECK(obd_vehicle_proto_class("A5") == OBD_PROTO_KWP);
    CHECK(obd_vehicle_proto_class("A6") == OBD_PROTO_CAN);
    CHECK(obd_vehicle_proto_class("A9") == OBD_PROTO_CAN);
    CHECK(obd_vehicle_proto_class("?") == OBD_PROTO_ANY);
    CHECK(obd_vehicle_proto_class("") == OBD_PROTO_ANY);
    CHECK(obd_vehicle_proto_class(NULL) == OBD_PROTO_ANY);

    /* La MX-5 de ivan: ISO 9141-2 con BE1FB811. */
    p = obd_vehicle_match(obd_vehicle_proto_class("A3"), 0xBE1FB811u,
                          0x80000000u, 0u, NULL, &exact);
    CHECK(p != NULL);
    CHECK(strcmp(p->id, "MAZDA_MX5_NB2") == 0);
    CHECK(exact == 1);
    CHECK(p->rpm_max == 8000);
    CHECK(p->redline == 7000);
    CHECK(p->fuel_cut == 7200);
    CHECK(p->centre == OBD_CENTRE_MAF);

    /* El MiTo sigue siendo el MiTo, con sus valores de siempre. */
    p = obd_vehicle_match(OBD_PROTO_CAN, 0xBE3EB811u, 0x80022001u,
                          0xC0C00000u, NULL, &exact);
    CHECK(p != NULL);
    CHECK(strcmp(p->id, "ALFA_MITO_QV_2010") == 0);
    CHECK(exact == 1);
    CHECK(p->rpm_max == 7000);
    CHECK(p->redline == 6500);
    CHECK(p->fuel_cut == 0);

    /* Las mascaras del MiTo no dan perfil de identidad en un K-line: es
     * generico, no MiTo. */
    p = obd_vehicle_match(OBD_PROTO_ISO9141, 0xBE3EB811u, 0x80022001u,
                          0xC0C00000u, NULL, &exact);
    CHECK(strcmp(p->id, "ALFA_MITO_QV_2010") != 0);
    CHECK(exact == 0);
    CHECK(p->generic == 1);

    /* Y al reves: mascaras de MX-5 sobre CAN tampoco son MX-5. */
    p = obd_vehicle_match(OBD_PROTO_CAN, 0xBE1FB811u, 0x80000000u, 0u,
                          NULL, &exact);
    CHECK(strcmp(p->id, "MAZDA_MX5_NB2") != 0);
    CHECK(exact == 0);

    /* Mascaras conocidas sin perfil: generico de la clase. */
    p = obd_vehicle_match(OBD_PROTO_ISO9141, 0xBE3FB811u, 0x80000000u, 0u,
                          NULL, &exact);
    CHECK(strcmp(p->id, "GENERIC_KLINE") == 0);
    CHECK(exact == 0);
    p = obd_vehicle_match(OBD_PROTO_CAN, 0x98000000u, 0u, 0u, NULL, &exact);
    CHECK(strcmp(p->id, "GENERIC_CAN") == 0);
    CHECK(exact == 0);

    /* Sin mascara 0100 no se afirma identidad. */
    p = obd_vehicle_match(OBD_PROTO_ISO9141, 0u, 0u, 0u, NULL, &exact);
    CHECK(exact == 0);
    CHECK(p->generic == 1);

    /* Sin protocolo tampoco, y ademas el resultado es generico. */
    p = obd_vehicle_match(OBD_PROTO_ANY, 0xBE1FB811u, 0x80000000u, 0u,
                          NULL, &exact);
    CHECK(exact == 0);
    CHECK(p->generic == 1);

    /* Forzado por id: manda sobre lo detectado. */
    p = obd_vehicle_match(OBD_PROTO_CAN, 0xBE3EB811u, 0x80022001u,
                          0xC0C00000u, "MAZDA_MX5_NB2", &exact);
    CHECK(strcmp(p->id, "MAZDA_MX5_NB2") == 0);
    CHECK(exact == 0);
    p = obd_vehicle_match(OBD_PROTO_CAN, 0u, 0u, 0u, "GENERIC_CAN", &exact);
    CHECK(strcmp(p->id, "GENERIC_CAN") == 0);

    /* El comodin es 0xFFFFFFFF: un mask3 distinto de cero NO es la MX-5,
     * porque el perfil exige que ese bloque no exista. */
    p = obd_vehicle_match(OBD_PROTO_ISO9141, 0xBE1FB811u, 0x80000000u,
                          0xDEADBEEFu, NULL, &exact);
    CHECK(strcmp(p->id, "MAZDA_MX5_NB2") != 0);
    CHECK(exact == 0);

    /* Forzado con un id que no existe: no se fuerza nada y se detecta. */
    p = obd_vehicle_match(OBD_PROTO_ISO9141, 0xBE1FB811u, 0x80000000u, 0u,
                          "NO_EXISTE", &exact);
    CHECK(strcmp(p->id, "MAZDA_MX5_NB2") == 0);
    CHECK(exact == 1);

    /* Clases que el ATDPN puede devolver en otros coches. */
    CHECK(obd_vehicle_proto_class("A4") == OBD_PROTO_ISO9141);
    CHECK(obd_vehicle_proto_class("AC") == OBD_PROTO_CAN);
    CHECK(obd_vehicle_proto_class("AB") == OBD_PROTO_CAN);
    /* "A0": el adaptador aun no ha elegido protocolo, no es un coche. */
    CHECK(obd_vehicle_proto_class("A0") == OBD_PROTO_ANY);
    CHECK(obd_vehicle_proto_class("0") == OBD_PROTO_ANY);

    /* Un id que no existe no fuerza nada. */
    CHECK(obd_vehicle_by_id("NO_EXISTE") == NULL);
    CHECK(obd_vehicle_by_id("") == NULL);
    CHECK(obd_vehicle_by_id(NULL) == NULL);

    test_gear();
}
