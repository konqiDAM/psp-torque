#include "obd/smooth.h"

#include <math.h>

float obd_smooth_alpha(uint32_t dt_ms)
{
    if (dt_ms == 0u) {
        /* Sin intervalo conocido se asume que el dato es nuevo: se
         * toma tal cual para no quedarse con un valor viejo. */
        return 1.0f;
    }
    float t = (float)dt_ms / 1000.0f;
    return 1.0f - expf(-t / (OBD_SMOOTH_TAU_MS / 1000.0f));
}

void obd_smooth_reset(obd_smooth *s)
{
    s->value = 0.0f;
    s->last_ts = 0;
    s->initialized = 0;
}

float obd_smooth_step(obd_smooth *s, float target, uint32_t now,
                      uint32_t tau_ms)
{
    if (!s->initialized) {
        s->value = target;
        s->initialized = 1;
        s->last_ts = now;
        return s->value;
    }

    uint32_t dt = now - s->last_ts;
    s->last_ts = now;
    if (dt == 0u || tau_ms == 0u) {
        return s->value;
    }

    float a = 1.0f - expf(-(float)dt / (float)tau_ms);
    s->value += (target - s->value) * a;

    /* Cerca del objetivo se pega: sin esto la cola asintotica deja el
     * valor a un pelo del real durante minutos, y en un display eso se ve
     * como un decimal que nunca cierra. */
    float eps = fabsf(target) * 0.002f + 0.01f;
    if (fabsf(target - s->value) <= eps) {
        s->value = target;
    }
    return s->value;
}

float obd_smooth_apply(obd_smooth *s, float v, uint32_t ts)
{
    if (!s->initialized) {
        s->value = v;
        s->initialized = 1;
    } else {
        uint32_t dt = ts - s->last_ts;
        float a = obd_smooth_alpha(dt);
        s->value = s->value + (v - s->value) * a;
    }
    s->last_ts = ts;
    return s->value;
}
