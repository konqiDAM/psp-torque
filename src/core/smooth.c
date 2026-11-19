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

void obd_predict_reset(obd_predict *p)
{
    p->last = 0.0f;
    p->vel = 0.0f;
    p->last_ts = 0;
    p->samples = 0;
}

float obd_predict_step(obd_predict *p, float value, uint32_t sample_ts,
                       uint32_t now, uint32_t span_ms, float max_step,
                       float deadband)
{
    if (p->samples == 0) {
        p->last = value;
        p->vel = 0.0f;
        p->last_ts = sample_ts;
        p->samples = 1;
        return value;
    }

    if (sample_ts != p->last_ts) {
        uint32_t dt = sample_ts - p->last_ts;
        if (dt > 0u && dt <= 3000u) {
            float d = value - p->last;
            if (d > -deadband && d < deadband) {
                p->vel = 0.0f;
            } else {
                p->vel = d * 1000.0f / (float)dt;
                /* La velocidad se limita a lo que la aguja puede recorrer
                 * en un tramo: un salto entre dos muestras no puede
                 * dispararla a cualquier sitio. */
                float vmax = 0.0f;
                if (span_ms > 0u) {
                    vmax = max_step * 1000.0f / (float)span_ms;
                }
                if (p->vel > vmax) {
                    p->vel = vmax;
                }
                if (p->vel < -vmax) {
                    p->vel = -vmax;
                }
            }
        } else {
            p->vel = 0.0f;
        }
        p->last = value;
        p->last_ts = sample_ts;
        p->samples++;
        /* En el instante de la muestra se ensena la medida, no una
         * prediccion. */
        return value;
    }

    if (span_ms == 0u) {
        return p->last;
    }

    /* Si la muestra se retrasa, la aguja se queda donde llego, no vuelve
     * atras: devolver `last` al agotarse el tramo daba un salto hacia atras
     * de hasta el tope cada vez que el hueco real pasaba de la media (37 %
     * de los huecos de la MX-5). */
    uint32_t age = now - p->last_ts;
    if (age > span_ms) {
        age = span_ms;
    }

    float pred = p->last + p->vel * (float)age / 1000.0f;
    /* Tope de posicion: con la velocidad ya limitada no puede pasarse, pero
     * se deja como red por si alguien toca el limite de velocidad. */
    float hi = p->last + max_step;
    float lo = p->last - max_step;
    if (pred > hi) {
        pred = hi;
    }
    if (pred < lo) {
        pred = lo;
    }
    if (pred < 0.0f) {
        pred = 0.0f;
    }
    return pred;
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
