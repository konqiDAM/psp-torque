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

void obd_ramp_reset(obd_ramp *r)
{
    r->from = 0.0f;
    r->to = 0.0f;
    r->cur = 0.0f;
    r->t0 = 0;
    r->span_ms = 0;
    r->sample_ts = 0;
    r->initialized = 0;
}

float obd_ramp_step(obd_ramp *r, float target, uint32_t sample_ts,
                    uint32_t now, uint32_t span_ms)
{
    if (!r->initialized) {
        r->from = target;
        r->to = target;
        r->cur = target;
        r->t0 = now;
        r->span_ms = span_ms;
        r->sample_ts = sample_ts;
        r->initialized = 1;
        return r->cur;
    }

    if (sample_ts != r->sample_ts) {
        /* Muestra nueva: el tramo arranca donde esta la aguja ahora, para
         * que llegue a mitad de camino no se traduzca en un salto. */
        r->from = r->cur;
        r->to = target;
        r->t0 = now;
        r->span_ms = span_ms;
        r->sample_ts = sample_ts;
    } else if (r->to != target) {
        r->from = r->cur;
        r->to = target;
        r->t0 = now;
    }

    if (r->span_ms == 0u) {
        r->cur = r->to;
        return r->cur;
    }

    uint32_t elapsed = now - r->t0;
    float frac = (elapsed >= r->span_ms)
                     ? 1.0f
                     : (float)elapsed / (float)r->span_ms;
    r->cur = r->from + (r->to - r->from) * frac;
    return r->cur;
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
