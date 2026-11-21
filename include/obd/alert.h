#ifndef OBD_ALERT_H
#define OBD_ALERT_H

#include <stdint.h>

/* Niveles de aviso, de menor a mayor prioridad. */
enum {
    OBD_ALERT_NONE = 0,
    OBD_ALERT_WARN = 1,
    OBD_ALERT_CRIT = 2
};

/* Que dato ha disparado el aviso. */
enum {
    OBD_ALERT_NONE_K = 0,
    OBD_ALERT_COOLANT = 1,
    OBD_ALERT_INTAKE = 2,
    OBD_ALERT_BATTERY = 3
};

typedef struct {
    uint8_t level;   /* OBD_ALERT_* */
    uint8_t kind;    /* OBD_ALERT_*_K */
    uint8_t more;    /* 1 si hay mas de un aviso activo */
} obd_alert;

/* Evalua los tres sensores. Los flags *_valid son 0 cuando el PID no
 * esta soportado o no hay muestra: en ese caso el sensor no puede
 * disparar un aviso. */
obd_alert obd_alert_eval(float ect_c, int ect_valid,
                         float iat_c, int iat_valid,
                         float volt, int volt_valid);

/* Texto corto para la cabecera: REFRIGERANTE / ADMISION / BATERIA / "". */
const char *obd_alert_text(uint8_t kind);

#endif
