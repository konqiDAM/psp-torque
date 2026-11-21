#include "obd/alert.h"
#include "obd/profile.h"

/* Umbral de bateria: por debajo de 11,5 V. La ECU reporta el voltaje
 * del modulo con el contacto puesto; 11,4 V ya es una bateria descargada. */
#define OBD_BATTERY_WARN_V 11.5f

obd_alert obd_alert_eval(float ect_c, int ect_valid,
                         float iat_c, int iat_valid,
                         float volt, int volt_valid)
{
    obd_alert a;
    a.level = OBD_ALERT_NONE;
    a.kind = OBD_ALERT_NONE_K;
    a.more = 0;

    int coolant_crit = ect_valid && ect_c >= PROFILE_ECT_CRIT_C;
    int coolant_warn = ect_valid && ect_c >= PROFILE_ECT_WARN_C;
    int intake = iat_valid && iat_c >= PROFILE_IAT_WARN_C;
    int battery = volt_valid && volt < OBD_BATTERY_WARN_V;

    int active = (coolant_crit || coolant_warn) + intake + battery;
    if (active > 1) {
        a.more = 1;
    }
    if (active == 0) {
        return a;
    }

    if (coolant_crit) {
        a.level = OBD_ALERT_CRIT;
        a.kind = OBD_ALERT_COOLANT;
    } else if (coolant_warn) {
        a.level = OBD_ALERT_WARN;
        a.kind = OBD_ALERT_COOLANT;
    } else if (intake) {
        a.level = OBD_ALERT_WARN;
        a.kind = OBD_ALERT_INTAKE;
    } else {
        a.level = OBD_ALERT_WARN;
        a.kind = OBD_ALERT_BATTERY;
    }
    return a;
}

const char *obd_alert_text(uint8_t kind)
{
    switch (kind) {
    case OBD_ALERT_COOLANT: return "REFRIGERANTE";
    case OBD_ALERT_INTAKE:  return "ADMISION";
    case OBD_ALERT_BATTERY: return "BATERIA";
    default:                return "";
    }
}
