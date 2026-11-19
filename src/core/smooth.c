#include "obd/smooth.h"

#include <math.h>

float obd_ease_alpha(uint32_t dt_ms, float tau_ms)
{
    if (tau_ms <= 0.0f) {
        /* Sin suavizado: el valor medido, tal cual. */
        return 1.0f;
    }
    if (dt_ms == 0u) {
        /* Dos lecturas en el mismo milisegundo: la segunda no mueve
         * nada, para que pintar el mismo cuadro dos veces no haga saltar
         * la aguja. */
        return 0.0f;
    }
    float t = (float)dt_ms / 1000.0f;
    return 1.0f - expf(-t / (tau_ms / 1000.0f));
}

void obd_smooth_reset(obd_smooth *s)
{
    s->value = 0.0f;
    s->last_ts = 0;
    s->initialized = 0;
}

float obd_ease_apply(obd_smooth *s, float target, uint32_t now, float tau_ms)
{
    if (!s->initialized) {
        s->value = target;
        s->last_ts = now;
        s->initialized = 1;
        return s->value;
    }
    uint32_t dt = now - s->last_ts;
    float a = obd_ease_alpha(dt, tau_ms);
    if (a >= 1.0f) {
        s->value = target;
    } else if (a > 0.0f) {
        s->value = s->value + (target - s->value) * a;
    }
    s->last_ts = now;
    return s->value;
}
