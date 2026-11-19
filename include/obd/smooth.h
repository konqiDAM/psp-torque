#ifndef OBD_SMOOTH_H
#define OBD_SMOOTH_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Presentation ease: one time constant, whatever the frame rate.      */
/* ------------------------------------------------------------------ */

/* This is the only smoothing left. It works on what is drawn, not on
 * what is stored: the store, the log, the peaks and the history ring all
 * see the measurement exactly as it arrived.
 *
 * Why a time constant and not a fixed mix factor: with a fixed factor
 * the lag is "0.3 of a sample", which at the 1.3 Hz of a K-line rpm is
 * more than a second. With tau fixed, the lag is the same at 60 fps, at
 * 30 fps and with a stalled frame. */

typedef struct {
    float value;
    uint32_t last_ts;
    int initialized;
} obd_smooth;

/* Mix factor for a frame lasting dt_ms with time constant tau_ms:
 * alpha = 1 - exp(-dt/tau).
 *
 *   - tau_ms <= 0 means no filtering at all: the value is taken as is.
 *   - dt_ms == 0 moves nothing, so drawing the same gauge twice in one
 *     millisecond does not make it jump to the target. */
float obd_ease_alpha(uint32_t dt_ms, float tau_ms);

/* Reset the filter: the next value is taken as it is. */
void obd_smooth_reset(obd_smooth *s);

/* Move the drawn value toward `target` for a frame that ends at `now`.
 *
 * The first call after a reset takes the target as it is, so a gauge
 * never glides up from zero when a value appears. The filter is driven
 * by the frame clock, not by the sample clock: the needle keeps moving
 * toward the last measurement between two samples, and the moment a new
 * one arrives the target changes and the movement continues from where
 * the needle was. */
float obd_ease_apply(obd_smooth *s, float target, uint32_t now, float tau_ms);

#endif
