/*
 * splash_mx5.h - Mazda MX-5 NB2 start-up animation for PSP Torque.
 *
 * Second boot animation, selectable in settings. Like draw_splash_frame()
 * in main.c, the draw call owns the whole frame: it opens and closes it
 * with gfx_frame_begin()/gfx_frame_end() and repaints all 480x272 pixels,
 * so the caller only passes the elapsed time.
 *
 * The drawing is a pure function of t_ms: no state, no allocation, no
 * file or bus access. That is what lets the geometry be tested on the
 * host build, where there is no GU.
 */

#ifndef PSP_SPLASH_MX5_H
#define PSP_SPLASH_MX5_H

#include <stdint.h>

/* Length of the animation in milliseconds. draw_splash_frame() runs for
 * 1600 ms, so this one does too; the helper below exists so the caller
 * never has to repeat the literal. */
#define SPLASH_MX5_TOTAL_MS 1600u

/* Second text line. draw_splash_frame() in main.c paints the car name,
 * not the build tag (BUILD_TAG only goes to the log), so this file does
 * the same. Override it at build time to show any other string. */
#ifndef SPLASH_MX5_SUBTITLE
#define SPLASH_MX5_SUBTITLE "ROADSTER NB 2001"
#endif

/* One horizontal slice of the nose silhouette, in frame pixels. */
typedef struct {
    float x;
    float y;
    float w;
} splash_mx5_span;

/*
 * Paints one complete frame at t_ms milliseconds after the animation
 * started (clamped to 0..SPLASH_MX5_TOTAL_MS).
 */
void splash_mx5_draw(uint32_t t_ms);

/* Length of the animation, in milliseconds. */
uint32_t splash_mx5_total_ms(void);

/*
 * Pure geometry, exposed so the host test can check it without a GU:
 * these are the parts that can be wrong. None of them paints.
 */

/* Fills out[] with the rows of the front silhouette, top to bottom, and
 * returns how many were written (at most max). Coordinates are at rest:
 * the caller offsets them by splash_mx5_body_rise(). */
int splash_mx5_body_spans(splash_mx5_span *out, int max);

/* Newest row of splash_mx5_body_spans(), in pixels: the row height. */
float splash_mx5_body_step(void);

/* How far the body is still below its final place, in pixels. */
float splash_mx5_body_rise(uint32_t t_ms);

/* Pop-up lid opening, in degrees: 0 closed, 90 fully raised. */
float splash_mx5_lid_angle_deg(uint32_t t_ms);

#endif
