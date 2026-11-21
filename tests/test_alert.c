#include "obd/alert.h"
#include "test_util.h"

#include <string.h>

/* Sin datos validos no hay aviso: un PID no soportado nunca puede
 * disparar una alarma. */
void test_alert_invalid_data(void)
{
    obd_alert a = obd_alert_eval(0, 0, 0, 0, 0, 0);
    CHECK(a.level == OBD_ALERT_NONE);
    CHECK(a.kind == OBD_ALERT_NONE_K);
    CHECK(a.more == 0);
}

/* El umbral es inclusivo y exacto. */
void test_alert_thresholds(void)
{
    CHECK(obd_alert_eval(104.9f, 1, 0, 0, 12.5f, 1).level == OBD_ALERT_NONE);
    CHECK(obd_alert_eval(105.0f, 1, 0, 0, 12.5f, 1).kind == OBD_ALERT_COOLANT);
    CHECK(obd_alert_eval(114.9f, 1, 0, 0, 12.5f, 1).level == OBD_ALERT_WARN);
    CHECK(obd_alert_eval(115.0f, 1, 0, 0, 12.5f, 1).level == OBD_ALERT_CRIT);
    CHECK(obd_alert_eval(0, 0, 70.0f, 1, 12.5f, 1).kind == OBD_ALERT_INTAKE);
    CHECK(obd_alert_eval(0, 0, 69.9f, 1, 12.5f, 1).level == OBD_ALERT_NONE);
    CHECK(obd_alert_eval(0, 0, 0, 0, 11.5f, 1).level == OBD_ALERT_NONE);
    CHECK(obd_alert_eval(0, 0, 0, 0, 11.4f, 1).kind == OBD_ALERT_BATTERY);
}

/* CRIT gana a WARN; 'more' indica que habia otro aviso activo. */
void test_alert_priority(void)
{
    obd_alert a = obd_alert_eval(108.0f, 1, 72.0f, 1, 12.5f, 1);
    CHECK(a.level == OBD_ALERT_WARN);
    CHECK(a.kind == OBD_ALERT_COOLANT);
    CHECK(a.more == 1);

    a = obd_alert_eval(116.0f, 1, 72.0f, 1, 11.0f, 1);
    CHECK(a.level == OBD_ALERT_CRIT);
    CHECK(a.kind == OBD_ALERT_COOLANT);
    CHECK(a.more == 1);

    a = obd_alert_eval(95.0f, 1, 72.0f, 1, 11.0f, 1);
    CHECK(a.kind == OBD_ALERT_INTAKE);
    CHECK(a.more == 1);

    CHECK(strcmp(obd_alert_text(OBD_ALERT_COOLANT), "REFRIGERANTE") == 0);
    CHECK(strcmp(obd_alert_text(OBD_ALERT_NONE_K), "") == 0);
}
