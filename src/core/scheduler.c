#include "obd/scheduler.h"

#include <string.h>
#include "obd/pid.h"

void obd_sched_init(obd_scheduler *sched)
{
    memset(sched, 0, sizeof(*sched));
    sched->in_flight = -1;
}

int obd_sched_add(obd_scheduler *sched, uint8_t pid,
                  uint16_t period_ms, uint8_t priority)
{
    if (sched->count >= OBD_SCHED_MAX) {
        return -1;
    }

    obd_poll_item *item = &sched->items[sched->count];
    memset(item, 0, sizeof(*item));
    item->pid = pid;
    item->period_ms = (period_ms > 0) ? period_ms : 100;
    item->priority = priority;
    item->next_due_ms = 0;
    item->supported = 0;

    return sched->count++;
}

int obd_sched_next(obd_scheduler *sched, uint32_t now_ms)
{
    if (sched->in_flight >= 0) {
        return -1;
    }

    int best = -1;
    for (int i = 0; i < sched->count; i++) {
        obd_poll_item *item = &sched->items[i];
        if (item->supported == 2) {
            continue;
        }
        if ((int32_t)(now_ms - item->next_due_ms) < 0) {
            continue;
        }
        if (best < 0) {
            best = i;
            continue;
        }

        obd_poll_item *cur = &sched->items[best];
        /* Con el enlace comprimido manda la prioridad. Sin esto, un PID de
         * periodo corto casi nunca se atrasa y los de periodo largo, que
         * siempre estan vencidos, le pasan por delante siempre: por eso el
         * RPM se refrescaba cada 4,4 s en el coche lento. Dentro de la
         * misma prioridad se sigue sirviendo el mas antiguo. */
        if (sched->strict && item->priority != cur->priority) {
            if (item->priority < cur->priority) {
                best = i;
            }
            continue;
        }
        /* EDF: el mas antiguo primero, para no dejar morir a los de
         * baja prioridad cuando el RTT supera sus periodos. */
        if ((int32_t)(item->next_due_ms - cur->next_due_ms) < 0) {
            best = i;
        } else if (item->next_due_ms == cur->next_due_ms) {
            if (item->priority < cur->priority) {
                best = i;
            } else if (item->priority == cur->priority && i < best) {
                best = i;
            }
        }
    }

    return best;
}

void obd_sched_begin(obd_scheduler *sched, int idx, uint32_t now_ms)
{
    (void)now_ms;
    if (idx >= 0 && idx < sched->count) {
        sched->in_flight = idx;
    }
}

void obd_sched_set_period(obd_scheduler *sched, int idx, uint16_t period_ms,
                          int due_now)
{
    if (idx < 0 || idx >= sched->count) {
        return;
    }
    sched->items[idx].period_ms = (period_ms > 0) ? period_ms : 100;
    if (due_now) {
        sched->items[idx].next_due_ms = 0;
    }
}

void obd_sched_defer(obd_scheduler *sched, int idx, uint32_t now_ms)
{
    if (sched == NULL || idx < 0 || idx >= sched->count) {
        return;
    }
    obd_poll_item *item = &sched->items[idx];
    if (item->supported == 2 || sched->in_flight == idx) {
        return;
    }
    uint32_t period = item->period_ms;
    if (period < 2u) {
        /* Sin tramo no hay donde repartir, y period/2 seria cero. */
        return;
    }
    if ((int32_t)(now_ms - item->next_due_ms) < 0) {
        /* Ya tiene turno en el futuro: no se toca. */
        return;
    }
    uint32_t half = period / 2u;
    if (half == 0u) {
        half = 1u;
    }
    /* El hueco sale del PID. Con los medios periodos reales (5 s y 30 s)
     * 977 es coprimo, asi que dos PID distintos no caen en el mismo
     * milisegundo; con otros periodos el reparto sigue siendo valido,
     * solo que dos PID podrian coincidir. */
    item->next_due_ms = now_ms + half + ((uint32_t)item->pid * 977u) % half;
}

void obd_sched_end(obd_scheduler *sched, int idx, uint32_t now_ms,
                   int got_data)
{
    if (idx < 0 || idx >= sched->count) {
        return;
    }
    if (sched->in_flight == idx) {
        sched->in_flight = -1;
    }

    obd_poll_item *item = &sched->items[idx];
    item->next_due_ms = now_ms + item->period_ms;

    if (got_data) {
        item->no_data_count = 0;
        if (item->supported == 0) {
            item->supported = 1;
        }
    } else {
        if (item->no_data_count < 255) {
            item->no_data_count++;
        }
        if (item->no_data_count >= 3) {
            item->supported = 2;
        }
    }
}

void obd_sched_set_supported(obd_scheduler *sched, int idx, int supported)
{
    if (idx < 0 || idx >= sched->count) {
        return;
    }
    sched->items[idx].supported = supported ? 1 : 2;
    sched->items[idx].no_data_count = 0;
}

void obd_sched_set_priority(obd_scheduler *sched, int idx, uint8_t priority)
{
    if (idx < 0 || idx >= sched->count) {
        return;
    }
    sched->items[idx].priority = priority;
}

void obd_sched_set_strict(obd_scheduler *sched, int strict)
{
    sched->strict = strict ? 1 : 0;
}

int obd_sched_apply_mask(obd_scheduler *sched, uint32_t mask,
                         uint8_t first_pid)
{
    int changed = 0;

    for (int i = 0; i < sched->count; i++) {
        obd_poll_item *item = &sched->items[i];
        int offset = (int)item->pid - (int)first_pid;
        if (offset < 0 || offset > 31) {
            continue;
        }

        int supported = obd_mask_supports(mask, (uint8_t)(offset + 1));
        uint8_t next = (uint8_t)(supported ? 1 : 2);
        if (item->supported != next) {
            changed++;
            item->supported = next;
            item->no_data_count = 0;
        }
    }

    return changed;
}

void obd_sched_reset_due(obd_scheduler *sched, uint32_t now_ms)
{
    for (int i = 0; i < sched->count; i++) {
        if (sched->items[i].supported != 2) {
            sched->items[i].next_due_ms = now_ms;
        }
    }
    sched->in_flight = -1;
}
