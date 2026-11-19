#include "obd/profile.h"

#include <stddef.h>

/* FASE F3: periodo base de cada PID (el que usa cuando esta en la
 * pantalla activa) y en que pantallas se muestra.
 *
 * La columna 'screens' dice en que pantallas el PID es visible:
 * bit0 GENERAL, bit1 MOTOR, bit2 DIAG, bit3 TELEM.
 *
 * Fuera de pantalla el periodo se multiplica por PROFILE_SLOW_FACTOR: el
 * ancho de banda del enlace (~12-13 peticiones/s) se reparte entre lo que
 * se esta mirando, no a partes iguales entre los 17 PID. */
static const struct {
    uint8_t pid;
    uint16_t period_ms;
    uint8_t screens;
    uint8_t alt;   /* grupo de alternativas: el coche solo soporta una */
} g_base_periods[] = {
    { 0x0C, 200,   0x0F, 0 },  /* RPM: en todas */
    { 0x0B, 500,   0x03, 0 },  /* MAP: general y motor */
    { 0x10, 1000,  0x03, 0 },  /* MAF: general y motor (si no hay MAP) */
    { 0x0D, 800,   0x0B, 0 },  /* velocidad: general, motor y telem */
    { 0x11, 1000,  0x07, 0 },  /* mariposa: general, motor y telem */
    { 0x49, 1000,  0x03, 1 },  /* pedal (posicion D) */
    { 0x4A, 1000,  0x03, 1 },  /* pedal (posicion E): alternativa del D */
    { 0x5A, 1000,  0x03, 1 },  /* pedal (posicion relativa): alternativa */
    { 0x04, 2000,  0x03, 0 },  /* carga: general y motor */
    { 0x05, 3000,  0x0F, 0 },  /* refrigerante: en todas */
    { 0x0F, 3000,  0x0F, 0 },  /* admision: en todas */
    { 0x06, 1500,  0x04, 0 },  /* STFT: solo DIAG */
    { 0x07, 1500,  0x04, 0 },  /* LTFT: solo DIAG */
    { 0x0E, 1500,  0x04, 0 },  /* avance: solo DIAG */
    { 0x14, 1500,  0x04, 0 },  /* O2-1: solo DIAG */
    { 0x15, 1500,  0x04, 0 },  /* O2-2: solo DIAG */
    { 0x21, 2000,  0x04, 0 },  /* distancia con MIL: solo DIAG */
    { 0x42, 5000,  0x0E, 0 },  /* voltaje: motor, DIAG y telem */
    { 0x5C, 10000, 0x02, 0 },  /* aceite: solo motor */
    { 0x2F, 10000, 0x06, 0 },  /* combustible: motor y DIAG */
    { 0x33, 30000, 0x01, 0 }   /* baro: solo general */
};

#define BASE_PERIOD_COUNT \
    ((int)(sizeof(g_base_periods) / sizeof(g_base_periods[0])))

uint16_t obd_profile_base_period(uint8_t pid)
{
    for (int i = 0; i < BASE_PERIOD_COUNT; i++) {
        if (g_base_periods[i].pid == pid) {
            return g_base_periods[i].period_ms;
        }
    }
    return 0;
}

uint16_t obd_profile_period(uint8_t pid, int visible)
{
    uint16_t base = obd_profile_base_period(pid);
    if (base == 0) {
        return 0;
    }
    if (visible) {
        return base;
    }
    uint32_t slow = (uint32_t)base * PROFILE_SLOW_FACTOR;
    if (slow > PROFILE_MAX_PERIOD_MS) {
        slow = PROFILE_MAX_PERIOD_MS;
    }
    return (uint16_t)slow;
}

int obd_profile_knows_pid(uint8_t pid)
{
    return obd_profile_base_period(pid) != 0;
}

uint8_t obd_profile_visible_mask(uint8_t pid)
{
    for (int i = 0; i < BASE_PERIOD_COUNT; i++) {
        if (g_base_periods[i].pid == pid) {
            return g_base_periods[i].screens;
        }
    }
    return 0;
}

/* Mascaras de pantalla: bit0 GENERAL, bit1 MOTOR, bit2 DIAG, bit3 TELEM.
 * Una mascara sin ningun bit conocido (las pantallas MODULOS, SNIFFER e
 * INFO) usa el conjunto base, que es el de GENERAL. */
#define SCREEN_GENERAL 0x01u
#define SCREEN_KNOWN   0x0Fu

float obd_profile_demand(uint8_t screen_mask)
{
    if ((screen_mask & SCREEN_KNOWN) == 0u) {
        screen_mask = SCREEN_GENERAL;
    }

    /* Los PID de un mismo grupo de alternativas son excluyentes: el coche
     * solo soporta uno de ellos, asi que se cuenta una sola vez. */
    uint32_t counted_alt = 0u;

    float demand = 0.0f;
    for (int i = 0; i < BASE_PERIOD_COUNT; i++) {
        uint8_t alt = g_base_periods[i].alt;
        if (alt != 0u) {
            uint32_t bit = (uint32_t)1u << alt;
            if (counted_alt & bit) {
                continue;
            }
            counted_alt |= bit;
        }
        int visible = (g_base_periods[i].screens & screen_mask) != 0;
        uint16_t period = obd_profile_period(g_base_periods[i].pid, visible);
        if (period > 0) {
            demand += 1000.0f / (float)period;
        }
    }
    return demand;
}

float obd_profile_demand_all(void)
{
    float demand = 0.0f;
    for (int i = 0; i < BASE_PERIOD_COUNT; i++) {
        demand += 1000.0f / (float)g_base_periods[i].period_ms;
    }
    return demand;
}

void obd_forced_set_add(obd_forced_set *set, uint8_t pid)
{
    if (pid <= OBD_PID_MAX) {
        set->pids[pid] = 1;
    }
}

int obd_forced_set_has(const obd_forced_set *set, uint8_t pid)
{
    if (pid > OBD_PID_MAX) {
        return 0;
    }
    return set->pids[pid] != 0;
}

/* FASE E2 + A: los periodos estan calibrados contra la capacidad real
 * del enlace ELM327 WiFi, no contra lo que el coche "deberia" dar.
 *
 * Medido en un viaje real (fase2.40): una peticion completa tarda ~83 ms
 * con el hueco entre comandos, es decir ~12 peticiones/s. La suma de
 * 1000/periodo de todas las tablas no puede pasar de
 * PROFILE_LINK_BUDGET_REQ_S; si lo hace, el planificador se satura y
 * TODOS los PID se refrescan mas tarde de lo pedido.
 *
 * Reparto (FASE A): lo que se mira en el tacometro va rapido, lo que
 * cambia despacio (temperaturas, trims, voltaje, baro, combustible) va
 * lento. El presupuesto se reparte en vez de repartir el ancho de banda
 * a partes iguales, que es lo que hacia que el RPM cayera a 1,3 Hz.
 */

const obd_poll_def PROFILE_QV_POLL[] = {
    { 0x0C, 250, 0 },   /* RPM */
    { 0x0B, 500, 0 },   /* MAP */
    { 0x0D, 800, 1 },   /* velocidad */
    { 0x05, 4000, 2 },  /* refrigerante */
    { 0x0F, 4000, 2 },  /* admision */
    { 0x33, 20000, 3 }  /* baro, si existe */
};

const int PROFILE_QV_POLL_COUNT =
    (int)(sizeof(PROFILE_QV_POLL) / sizeof(PROFILE_QV_POLL[0]));

const obd_poll_def PROFILE_EXTRA_POLL[] = {
    { 0x11, 1000, 1 },  /* mariposa (siempre disponible) */
    { 0x04, 2000, 1 }   /* carga */
};

const int PROFILE_EXTRA_POLL_COUNT =
    (int)(sizeof(PROFILE_EXTRA_POLL) / sizeof(PROFILE_EXTRA_POLL[0]));

const obd_poll_def PROFILE_MOTOR_POLL[] = {
    { 0x06, 12000, 3 }, /* STFT */
    { 0x07, 12000, 3 }, /* LTFT */
    { 0x0E, 12000, 3 }, /* avance */
    { 0x14, 12000, 3 }, /* O2-1 */
    { 0x15, 12000, 3 }, /* O2-2 */
    { 0x21, 12000, 3 }  /* distancia con MIL */
};

const int PROFILE_MOTOR_POLL_COUNT =
    (int)(sizeof(PROFILE_MOTOR_POLL) / sizeof(PROFILE_MOTOR_POLL[0]));

const obd_poll_def PROFILE_ENGINE_POLL[] = {
    { 0x0C, 250, 0 },   /* RPM */
    { 0x0B, 500, 0 },   /* MAP */
    { 0x49, 1000, 1 },  /* pedal del acelerador, si existe */
    { 0x11, 1000, 1 },  /* mariposa */
    { 0x04, 2000, 1 },  /* carga */
    { 0x0D, 800, 2 },   /* velocidad */
    { 0x06, 12000, 2 }, /* STFT */
    { 0x05, 4000, 3 },  /* refrigerante */
    { 0x0F, 4000, 3 },  /* admision */
    { 0x07, 12000, 3 }, /* LTFT */
    { 0x42, 10000, 4 }, /* tension */
    { 0x5C, 10000, 4 }, /* aceite, si existe */
    { 0x33, 20000, 4 }, /* baro */
    { 0x2F, 20000, 4 }  /* nivel combustible, si existe */
};

const int PROFILE_ENGINE_POLL_COUNT =
    (int)(sizeof(PROFILE_ENGINE_POLL) / sizeof(PROFILE_ENGINE_POLL[0]));

/* ------------------------------------------------------------------ */
/* FASE K2: reparto por capacidad del enlace.                         */
/* ------------------------------------------------------------------ */

float obd_profile_link_class(int proto_class)
{
    /* vehicle.h: OBD_PROTO_CAN == 0, ISO 9141-2 == 3, KWP == 5. Todo lo
     * que no sea CAN se planifica como K-line. */
    return (proto_class == 0) ? PROFILE_LINK_CLASS_CAN
                              : PROFILE_LINK_CLASS_KLINE;
}

float obd_capacity_next(float current, float rate, float demand, float ceiling)
{
    if (rate <= 0.0f || demand <= 0.0f) {
        return current;
    }

    /* El enlace no sostuvo lo pedido: la capacidad es lo entregado. Solo
     * recorta, nunca sube, para que un pico no dispare el plan.
     *
     * El umbral es 0,70 y no 0,85 porque el plan ya pide solo el 85% de la
     * capacidad (PROFILE_LINK_UTILIZATION), asi que un enlace sano entrega
     * ~0,85 de lo pedido: con el liston en 0,85 cada ventana recortaba y el
     * plan oscilaba entre 2,20 y 1,40 req/s en la MX-5 (phase1_008.log). */
    if (rate < demand * 0.70f) {
        float next = (rate < current) ? rate : current;
        if (next > ceiling) {
            next = ceiling;
        }
        if (next < PROFILE_LINK_RATE_FLOOR) {
            next = PROFILE_LINK_RATE_FLOOR;
        }
        return next;
    }

    /* Sostuvo lo pedido con holgura: se puede aspirar a mas. Sube un 10%
     * por ventana, con tope en el techo de la clase. */
    if (rate > demand * 0.97f) {
        float next = current * 1.10f;
        if (next > ceiling) {
            next = ceiling;
        }
        return next;
    }

    /* Zona muerta: ni sube ni baja. */
    return current;
}

void obd_plan_init(obd_link_plan *plan, float capacity, uint16_t service_ms,
                   float demand, int max_values)
{
    if (plan == NULL) {
        return;
    }
    if (capacity < PROFILE_LINK_RATE_FLOOR) {
        capacity = PROFILE_LINK_RATE_FLOOR;
    }
    plan->capacity = capacity;
    plan->service_ms = service_ms;
    plan->budget = capacity * PROFILE_LINK_UTILIZATION;
    plan->compressed = (demand > plan->budget) ? 1 : 0;
    if (max_values < 0) {
        max_values = 0;
    }
    if (max_values > 8) {
        max_values = 8;
    }
    plan->max_values = max_values;
}

uint16_t obd_plan_period(const obd_link_plan *plan, uint8_t pid, int visible,
                         int rank, int value_count)
{
    uint16_t base = obd_profile_base_period(pid);
    if (base == 0) {
        return 0;
    }

    /* Sin compresion el plan ES la tabla base. Es lo que deja al MiTo y a
     * cualquier enlace con capacidad de sobra exactamente como estaban. */
    if (plan == NULL || !plan->compressed) {
        return obd_profile_period(pid, visible);
    }

    float service = (float)plan->service_ms;
    float rate;

    /* Cuantos valores de pantalla entran en el cupo de REPARTO y cuantos
     * se quedan fuera pero siguen visibles. Cada uno reserva su suelo (0,2
     * Hz dentro del cupo, 0,1 fuera) y el RPM se lleva el resto: es lo
     * unico que se nota cuando va lento, asi que con pocos valores en
     * pantalla se le puede dar casi todo el enlace. */
    int n = value_count;
    if (n > plan->max_values) {
        n = plan->max_values;
    }
    if (n < 0) {
        n = 0;
    }
    int rest = value_count - n;
    if (rest < 0) {
        rest = 0;
    }
    float reserve = (float)n * PROFILE_PLAN_VALUE_HZ +
                    (float)rest * PROFILE_PLAN_VALUE_SLOW_HZ;
    float rpm_rate = plan->budget - reserve;
    if (rpm_rate < PROFILE_PLAN_RPM_MIN_HZ) {
        /* No cabe el reparto: el RPM se queda su minimo y los valores lo
         * que sobre. El suelo manda sobre el presupuesto, porque el
         * planificador sirve al RPM primero y es lo unico que se nota si
         * se queda lento. */
        rpm_rate = PROFILE_PLAN_RPM_MIN_HZ;
        if (rpm_rate > plan->budget) {
            rpm_rate = plan->budget;
        }
    }
    if (rpm_rate > PROFILE_PLAN_RPM_MAX_HZ) {
        rpm_rate = PROFILE_PLAN_RPM_MAX_HZ;
    }

    if (pid == 0x0C) {
        rate = rpm_rate;
    } else if (visible && rank >= 1 && value_count >= 1 && reserve > 0.0f) {
        /* Lo que queda del presupuesto, repartido en proporcion a lo que
         * pide cada valor: el cupo 0,2 Hz, el resto 0,1 Hz. La suma de
         * todos los periodos con el RPM es el presupuesto exacto, de modo
         * que el plan nunca pide al enlace mas de lo que tiene. */
        float left = plan->budget - rpm_rate;
        float want;
        if (left <= 0.0f) {
            left = 0.0f;
        }
        if (rank <= plan->max_values) {
            want = PROFILE_PLAN_VALUE_HZ;
        } else {
            want = PROFILE_PLAN_VALUE_SLOW_HZ;
        }
        rate = left * want / reserve;
    } else {
        /* Fuera de pantalla: una peticion por minuto, que es lo que
         * aguanta el enlace. */
        return PROFILE_PLAN_BG_PERIOD_MS;
    }

    if (rate <= 0.0f) {
        /* Un valor visible sin presupuesto no puede caer al fondo: aun asi
         * se pide de vez en cuando para que el cuadro siga leyendose. */
        return visible ? PROFILE_PLAN_VALUE_SLOW_MS : PROFILE_PLAN_BG_PERIOD_MS;
    }

    /* El planificador cuenta el periodo desde el fin de la respuesta
     * (obd_sched_end), asi que el tiempo de servicio se cuenta dos veces
     * si no se resta: 1 Hz con 532 ms de servicio no es 1000 ms. */
    float period = 1000.0f / rate - service;
    if (period < 100.0f) {
        period = 100.0f;
    }
    if (period > (float)PROFILE_PLAN_BG_PERIOD_MS) {
        period = (float)PROFILE_PLAN_BG_PERIOD_MS;
    }
    if (period < (float)base) {
        period = (float)base;
    }
    return (uint16_t)(period + 0.5f);
}
