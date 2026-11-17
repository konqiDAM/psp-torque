#include "obd/vehicle.h"
#include "test_util.h"

#include <string.h>

/* FASE U4: el perfil de coche decide tacometro, zona roja, corte y gauge
 * central. Estos tests fijan dos cosas que importan en el coche: que la
 * MX-5 de ivan se reconozca por su evidencia, y que nunca se confunda con
 * el MiTo, porque en esa confusion va el decodificado con signo. */
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
}
