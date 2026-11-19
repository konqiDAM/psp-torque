#ifndef OBD_SCHEDULER_H
#define OBD_SCHEDULER_H

#include <stdint.h>

#define OBD_SCHED_MAX 24

typedef struct {
    uint8_t pid;
    uint16_t period_ms;
    uint8_t priority;      /* 0 = mas alta */
    uint32_t next_due_ms;
    uint8_t no_data_count;
    uint8_t supported;     /* 0 desconocido, 1 confirmado, 2 no soportado */
} obd_poll_item;

typedef struct {
    obd_poll_item items[OBD_SCHED_MAX];
    int count;
    int in_flight;         /* indice o -1 */
    /* FASE K2: con el enlace comprimido se sirve por prioridad y, dentro
     * de la misma, por vencimiento. Sin comprimir se queda en EDF puro,
     * que es lo que usan los coches rapidos y lo que habia antes. */
    int strict;
} obd_scheduler;

void obd_sched_init(obd_scheduler *sched);

int obd_sched_add(obd_scheduler *sched, uint8_t pid,
                  uint16_t period_ms, uint8_t priority);

/* Indice del siguiente PID a pedir, o -1 si no toca o hay uno en vuelo. */
int obd_sched_next(obd_scheduler *sched, uint32_t now_ms);

/* Marca el PID como enviado. */
void obd_sched_begin(obd_scheduler *sched, int idx, uint32_t now_ms);

/* FASE F3: cambia el periodo de un PID en caliente (reparto por
 * pantalla). Con 'due_now' distinto de cero el PID pasa a estar
 * pendiente de inmediato: es lo que hace que un dato que acaba de
 * aparecer en pantalla se pida al instante. */
void obd_sched_set_period(obd_scheduler *sched, int idx, uint16_t period_ms,
                          int due_now);

/* Aplaza un PID que ya esta vencido: le da su proximo turno dentro del
 * segundo tramo de su periodo, con un hueco que depende del PID para que
 * dos aplazados no vuelvan a la vez. Se usa con los PID que no estan en
 * pantalla cuando la tabla se acaba de montar, para que su primer turno
 * no se coma el enlace que necesita el RPM (en phase1_012 siete PID
 * fuera de pantalla se llevaron 19 de las 151 peticiones del primer
 * minuto).
 *
 * No hace nada si el PID ya tiene un turno en el futuro, si esta en
 * vuelo, si esta marcado como no soportado o si su periodo es menor que
 * 2 ms (no hay tramo donde repartirlo). */
void obd_sched_defer(obd_scheduler *sched, int idx, uint32_t now_ms);

/* Cierra la peticion: reprograma y cuenta NO DATA. */
void obd_sched_end(obd_scheduler *sched, int idx, uint32_t now_ms,
                   int got_data);

void obd_sched_set_supported(obd_scheduler *sched, int idx, int supported);

/* Cambia la prioridad de un PID en caliente (0 = mas alta). No toca ni el
 * periodo ni el vencimiento. */
void obd_sched_set_priority(obd_scheduler *sched, int idx, uint8_t priority);

/* 1 = servir por prioridad antes que por vencimiento. */
void obd_sched_set_strict(obd_scheduler *sched, int strict);

/* Marca todos los PID no soportados segun la mascara del bloque.
 * first_pid = 0x01, 0x21, 0x41... Devuelve cuantos ha cambiado. */
int obd_sched_apply_mask(obd_scheduler *sched, uint32_t mask,
                         uint8_t first_pid);

/* Reagenda todo (cambio de modo, reconexion). */
/* Marca pendientes de inmediato todos los PID soportados. La app ya no lo
 * usa (build_scheduler() deja la tabla entera pendiente): marcarlos todos
 * es justo lo que hacia que los PID fuera de pantalla se comieran el
 * primer minuto del enlace. Queda para los tests. */
void obd_sched_reset_due(obd_scheduler *sched, uint32_t now_ms);

#endif
