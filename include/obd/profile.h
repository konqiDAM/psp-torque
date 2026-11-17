#ifndef OBD_PROFILE_H
#define OBD_PROFILE_H

#include <stdint.h>

#define PROFILE_ID "ALFA_MITO_QV_2010"
#define PROFILE_DISPLAY_NAME "MiTo QV 2010"

/* Umbrales provisionales: no son limites oficiales del vehiculo. */
#define PROFILE_REDLINE_RPM 6500.0f
#define PROFILE_TACH_MAX_RPM 7000.0f
#define PROFILE_BOOST_MIN_BAR (-1.0f)
#define PROFILE_BOOST_MAX_BAR 1.5f
#define PROFILE_ECT_WARN_C 105.0f
#define PROFILE_ECT_CRIT_C 115.0f
#define PROFILE_IAT_WARN_C 70.0f

/* Fallback de presion barometrica, marcado EST en la UI. */
#define PROFILE_BARO_FALLBACK_KPA 101.3f

typedef struct {
    uint8_t pid;
    uint16_t period_ms;
    uint8_t priority;  /* 0 = mas alta */
} obd_poll_def;

extern const obd_poll_def PROFILE_QV_POLL[];
extern const int PROFILE_QV_POLL_COUNT;

/* FASE F3: periodo base de cada PID, en ms. Es el periodo que se usa
 * cuando el PID esta en la pantalla activa; fuera de pantalla se
 * multiplica por PROFILE_SLOW_FACTOR.
 *
 *  PID  base   que es
 *  0x0C   200  RPM (5 Hz)
 *  0x0B   400  MAP
 *  0x0D   800  velocidad
 *  0x11   800  mariposa
 *  0x49   800  pedal (posicion D)
 *  0x4A   800  pedal (posicion E)
 *  0x5A  1000  pedal (posicion relativa)
 *  0x04  1500  carga
 *  0x05  3000  refrigerante
 *  0x0F  3000  admision
 *  0x06  1000  STFT
 *  0x07  1000  LTFT
 *  0x0E  1000  avance
 *  0x14  1000  O2-1
 *  0x15  1000  O2-2
 *  0x21  2000  distancia con MIL
 *  0x42  5000  voltaje
 *  0x5C 10000  aceite
 *  0x2F 10000  combustible
 *  0x33 30000  baro
 */

/* Multiplicador del periodo cuando el PID no esta en pantalla. */
#define PROFILE_SLOW_FACTOR 4

/* Tope del periodo efectivo. */
#define PROFILE_MAX_PERIOD_MS 30000u

/* Periodo base de un PID, o 0 si no esta en la tabla. */
uint16_t obd_profile_base_period(uint8_t pid);

/* Periodo efectivo: el base si 'visible' es distinto de 0; si no, el
 * base por PROFILE_SLOW_FACTOR, con tope en PROFILE_MAX_PERIOD_MS. */
uint16_t obd_profile_period(uint8_t pid, int visible);

/* 1 si el PID tiene periodo base conocido. */
int obd_profile_knows_pid(uint8_t pid);

/* Mascara de pantallas en las que se muestra el PID: bit0 GENERAL,
 * bit1 MOTOR, bit2 DIAG, bit3 TELEM. 0 si el PID no esta en la tabla. */
uint8_t obd_profile_visible_mask(uint8_t pid);

/* Demanda de peticiones (req/s) de una pantalla. La mascara usa los
 * bits de pantalla de arriba; una mascara sin ninguno cuenta como el
 * conjunto base (MODULOS, SNIFFER e INFO). */
float obd_profile_demand(uint8_t screen_mask);

/* Demanda si todos los PID se pidieran a la vez. Con el reparto por
 * pantalla nunca ocurre, pero sirve de referencia. */
float obd_profile_demand_all(void);

/* PID del bloque 01 que se sondean siempre (mariposa y carga). */
extern const obd_poll_def PROFILE_EXTRA_POLL[];
extern const int PROFILE_EXTRA_POLL_COUNT;

/* PID de diagnostico avanzado (trims, avance, O2, distancia con MIL). */
extern const obd_poll_def PROFILE_MOTOR_POLL[];
extern const int PROFILE_MOTOR_POLL_COUNT;

/* Periodos de los PID que solo se sondean si la mascara los declara
 * soportados. */
#define PROFILE_PID_MAF_PERIOD_MS   1000
#define PROFILE_PID_PEDAL_PERIOD_MS 1000
#define PROFILE_PID_VOLT_PERIOD_MS  10000
#define PROFILE_PID_OIL_PERIOD_MS   10000
#define PROFILE_PID_FUEL_PERIOD_MS  20000

/* FASE E2 + A: presupuesto de peticiones del enlace.
 *
 * Medido sobre un clon ELM327 v1.5 por WiFi: una peticion completa
 * (envio + respuesta del coche + hueco entre comandos) tardaba 83 ms
 * con el hueco de 24 ms que dejaba la version anterior, es decir
 * ~12 peticiones/s. Con el hueco minimo de 8 ms (FASE F1) el ciclo
 * baja a ~60-67 ms y la capacidad sube a ~15 peticiones/s.
 *
 * El presupuesto es 14 req/s: una cifra conservadora para la
 * configuracion objetivo, que el viaje de calibracion confirmara. La
 * demanda de cada pantalla tiene que caber dentro (test
 * `test_demand_by_screen`); la suma de "todo a la vez" no, porque con
 * el reparto por pantalla nunca se pide todo a la vez. */
#define PROFILE_LINK_BUDGET_REQ_S 14.0f

extern const obd_poll_def PROFILE_ENGINE_POLL[];
extern const int PROFILE_ENGINE_POLL_COUNT;

/* FASE F3: conjunto de PIDs que se fuerzan al periodo base porque
 * estan a la vista en un hueco configurable o en una serie de
 * telemetria. Se guarda como array de bytes, no como mascara de bits:
 * PIDs como 0x5C (92) o 0x49 (73) no caben en un desplazamiento de 64
 * bits y `1u << pid` seria comportamiento indefinido. */
#define OBD_PID_MAX 0x7Fu

typedef struct {
    uint8_t pids[OBD_PID_MAX + 1];
} obd_forced_set;

void obd_forced_set_add(obd_forced_set *set, uint8_t pid);
int obd_forced_set_has(const obd_forced_set *set, uint8_t pid);

/* FASE K2: reparto por capacidad del enlace.
 *
 * La tabla de periodos de arriba esta calibrada para un CAN que entrega
 * 12-14 peticiones/s. Un coche de K-line entrega 1,9, y con la misma tabla
 * el RPM se refresca cada 4,4 s (medido en phase1_003). La causa no es
 * solo la falta de ancho de banda: el planificador sirve primero lo que
 * lleva mas tiempo vencido, y un PID de periodo corto casi nunca se atrasa,
 * asi que los de periodo largo le pasan por delante.
 *
 * Aqui va la aritmetica pura (sin PSP, sin ELM, sin estado global) para
 * probarla: capacidad por clase de protocolo, presupuesto util, y el
 * periodo que le toca a cada PID cuando la tabla no cabe. Con capacidad de
 * CAN el plan devuelve exactamente obd_profile_period(), que es lo que deja
 * al MiTo sin cambios. */

/* Porcentaje de la capacidad que se planea usar: el resto es el margen
 * para no tener el enlace saturado y para los comandos del menu. */
#define PROFILE_LINK_UTILIZATION 0.85f

/* Capacidad de cada clase de protocolo, en peticiones/s. CAN es el valor
 * ya medido sobre el MiTo; K-line, el medido sobre la MX-5 (1,9 req/s con
 * 383 ms de ida y vuelta), redondeado hacia arriba para no arrancar por
 * debajo de lo que el coche da. */
#define PROFILE_LINK_CLASS_CAN   16.5f
#define PROFILE_LINK_CLASS_KLINE 2.2f

/* Suelo del estimador: por debajo de esto no se planifica, porque un
 * golpe de timeouts dejaria el plan en un solo PID. */
#define PROFILE_LINK_RATE_FLOOR 0.8f

/* Lo que el plan garantiza: RPM a 1 Hz y hasta tres valores de la pantalla
 * activa con el resto. Cuatro valores a 0,5 Hz mas el RPM son 3 req/s, que
 * este enlace no tiene (1,879 medidos): por eso son tres. */
#define PROFILE_PLAN_RPM_HZ      1.0f
#define PROFILE_PLAN_MAX_VALUES  3

/* Un PID que no esta en pantalla, cuando hay que repartir. */
#define PROFILE_PLAN_BG_PERIOD_MS 60000u

typedef struct {
    float capacity;      /* req/s estimadas del enlace */
    float budget;        /* capacidad por PROFILE_LINK_UTILIZATION */
    uint16_t service_ms; /* tiempo medio por peticion en este enlace */
    int compressed;      /* 1 = hay que repartir; 0 = tabla base tal cual */
} obd_link_plan;

/* Capacidad de partida segun la clase de protocolo (OBD_PROTO_* de
 * vehicle.h: CAN, o cualquier cosa de K-line). */
float obd_profile_link_class(int proto_class);

/* Capacidad nueva a partir de la ventana anterior: 'rate' es lo que el
 * enlace entrego de verdad y 'demand' lo que se le pidio. Baja cuando no
 * llego, sube despacio cuando sostuvo lo pedido, y se queda igual en el
 * medio (zona muerta, para no oscilar). Nunca pasa del techo de la clase
 * ni baja del suelo. */
float obd_capacity_next(float current, float rate, float demand, float ceiling);

/* Prepara el plan: 'demand' es la demanda de la pantalla activa en req/s
 * (obd_profile_demand) para decidir si hace falta repartir. */
void obd_plan_init(obd_link_plan *plan, float capacity, uint16_t service_ms,
                   float demand);

/* Periodo de un PID segun el plan. 'rank' ordena los valores de la
 * pantalla activa (1 = el primero); los que pasan de
 * PROFILE_PLAN_MAX_VALUES van a fondo, igual que los que no se ven. */
uint16_t obd_plan_period(const obd_link_plan *plan, uint8_t pid, int visible,
                         int rank, int value_count);

#endif
