#ifndef OBD_VEHICLE_H
#define OBD_VEHICLE_H

#include <stdint.h>

/* Perfiles de coche (FASE U4).
 *
 * Lo que cambia de un coche a otro no es una preferencia: el fondo de
 * escala del tacometro, la zona roja y el corte son hechos del vehiculo.
 * Guardarlos en los ajustes es lo que hacia que la MX-5 heredase el
 * tacometro del MiTo. Aqui van en una tabla const, y el matcher es puro
 * (sin coche, sin ELM, sin estado global) para poder probarlo.
 *
 * La evidencia es la que ya se lee en la conexion: la respuesta de ATDPN
 * y las tres mascaras 0100/0120/0140. Nada nuevo en el bus. */

/* Clase de protocolo, deducida del ultimo caracter de ATDPN. */
#define OBD_PROTO_ANY     (-1)  /* vale cualquier protocolo */
#define OBD_PROTO_KLINE   (-2)  /* ISO 9141-2 o KWP2000 */
#define OBD_PROTO_CAN     0
#define OBD_PROTO_ISO9141 3
#define OBD_PROTO_KWP     5

/* Que gauge va en el centro del dashboard. */
#define OBD_CENTRE_BOOST     0
#define OBD_CENTRE_MAF       1
#define OBD_CENTRE_LOAD      2
#define OBD_CENTRE_FROM_CAPS 3  /* generico: lo decide la mascara */

/* Comodin para una mascara que no importa. El cero NO es comodin: un
 * perfil puede exigir que un bloque no exista (la MX-5 no tiene 0140). */
#define OBD_VEHICLE_ANY_MASK 0xFFFFFFFFu

typedef struct {
    const char *id;      /* "MAZDA_MX5_NB2", el nombre que se escribe en profile= */
    const char *name;    /* "Mazda MX-5 NB2" */
    int proto;           /* OBD_PROTO_* */
    uint32_t mask1;      /* OBD_VEHICLE_ANY_MASK = no importa */
    uint32_t mask2;
    uint32_t mask3;
    int rpm_max;         /* fondo de escala del tacometro */
    int redline;         /* inicio de la zona roja */
    int fuel_cut;        /* corte de inyeccion; 0 = sin dato */
    int centre;          /* OBD_CENTRE_* */
    int generic;         /* 1 = vale para cualquier coche de su clase */
} obd_vehicle_profile;

/* Clase de protocolo a partir de la respuesta de ATDPN ("A3", "3", "A6",
 * "?" si aun no hay respuesta). */
int obd_vehicle_proto_class(const char *atdpn);

/* Perfil por id, o NULL. Para el ajuste manual profile=NAME. */
const obd_vehicle_profile *obd_vehicle_by_id(const char *id);

/* Elige perfil con la evidencia de la conexion.
 *
 * Orden: profile= forzado; si no, un perfil de identidad cuyas tres
 * mascaras casen y cuyo protocolo encaje; si no, el generico de la clase.
 *
 * forced_id puede ser NULL o "". *exact queda a 1 si caso por identidad
 * (mascaras), 0 si es generico o forzado. Nunca devuelve NULL. */
const obd_vehicle_profile *obd_vehicle_match(int proto, uint32_t mask1,
                                             uint32_t mask2, uint32_t mask3,
                                             const char *forced_id,
                                             int *exact);

#endif
