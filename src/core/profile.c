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
     * recorta, nunca sube, para que un pico no dispare el plan. */
    if (rate < demand * 0.85f) {
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
                   float demand)
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

    if (pid == 0x0C) {
        rate = PROFILE_PLAN_RPM_HZ;
        if (rate > plan->budget) {
            rate = plan->budget;
        }
    } else if (visible && rank >= 1 && value_count >= 1 &&
               rank <= PROFILE_PLAN_MAX_VALUES) {
        /* Lo que queda del presupuesto, a partes iguales entre los valores
         * de la pantalla. Si no hay ninguno, no hay nada que repartir. */
        float left = plan->budget - PROFILE_PLAN_RPM_HZ;
        int n = (value_count < PROFILE_PLAN_MAX_VALUES) ? value_count
                                                       : PROFILE_PLAN_MAX_VALUES;
        rate = (left > 0.0f) ? left / (float)n : 0.0f;
    } else {
        /* Fuera de pantalla o por encima del cupo: una peticion por
         * minuto, que es lo que aguanta el enlace. */
        return PROFILE_PLAN_BG_PERIOD_MS;
    }

    if (rate <= 0.0f) {
        return PROFILE_PLAN_BG_PERIOD_MS;
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
