/*
 * splash_mx5.c - Mazda MX-5 NB2 start-up animation for PSP Torque.
 *
 * Front view of the roadster, painted with the primitives of gfx.h and
 * the active theme, as draw_splash_frame() does for the Alfa Romeo.
 *
 * Timeline:
 *   0..420    the body rises into place from below
 *   380..760  the two pop-up lids swing open
 *   720..980  the lamps light up
 *   1000..    word mark, rule and model line
 *
 * Everything is a pure function of t_ms, so a frame can be reproduced
 * from its timestamp alone.
 *
 * Read-only, like the rest of the UI: this file only paints.
 */

#include <math.h>
#include <stddef.h>

#include "gfx.h"
#include "splash_mx5.h"

/* Timeline, in milliseconds. */
#define RISE_END_MS 420u
#define LID_START_MS 380u
#define LID_END_MS 760u
#define LAMP_START_MS 720u
#define LAMP_END_MS 980u
#define MARK_MS 1000u
#define MARK_FADE_MS 180u
#define SUB_FADE_MS 300u

/* Nose silhouette, in frame pixels. The car is 300 px wide and sits on
 * the floor line, which leaves the lower quarter of the screen for the
 * word mark. */
#define BODY_CX 240.0f
#define BODY_TOP 84.0f
#define BODY_DEPTH 110.0f
#define BODY_BOTTOM (BODY_TOP + BODY_DEPTH)
#define BODY_STEP 2.0f
#define BODY_ROWS 64
#define RISE_PX 56.0f
#define FLOOR_Y BODY_BOTTOM

#define DEG_TO_RAD 0.017453292519943295f

static float clamp_f(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static float phase(uint32_t t_ms, uint32_t from, uint32_t to)
{
    if (t_ms <= from) {
        return 0.0f;
    }
    if (t_ms >= to) {
        return 1.0f;
    }
    return (float)(t_ms - from) / (float)(to - from);
}

/* Decelerating ramp: fast at the start, arriving softly. */
static float ease_out(float p)
{
    float q = 1.0f - clamp_f(p, 0.0f, 1.0f);
    return 1.0f - q * q;
}

/* Half width of the nose at a given depth below BODY_TOP. The widest
 * point is the fender line, then the bumper tapers back in: that bulge
 * is what makes the silhouette read as an NB and not as a box. */
static const float k_body_profile[][2] = {
    { 0.0f, 138.0f },
    { 10.0f, 148.0f },
    { 28.0f, 152.0f },
    { 52.0f, 150.0f },
    { 78.0f, 142.0f },
    { 96.0f, 134.0f },
    { 110.0f, 130.0f }
};

#define PROFILE_ROWS \
    ((int)(sizeof(k_body_profile) / sizeof(k_body_profile[0])))

static float body_half_width(float depth)
{
    if (depth <= k_body_profile[0][0]) {
        return k_body_profile[0][1];
    }

    for (int i = 1; i < PROFILE_ROWS; i++) {
        if (depth <= k_body_profile[i][0]) {
            float d0 = k_body_profile[i - 1][0];
            float d1 = k_body_profile[i][0];
            float w0 = k_body_profile[i - 1][1];
            float w1 = k_body_profile[i][1];
            float k = (depth - d0) / (d1 - d0);
            return w0 + (w1 - w0) * k;
        }
    }

    return k_body_profile[PROFILE_ROWS - 1][1];
}

int splash_mx5_body_spans(splash_mx5_span *out, int max)
{
    int n = 0;

    if (out == NULL || max <= 0) {
        return 0;
    }

    for (float depth = 0.0f; depth < BODY_DEPTH && n < max;
         depth += BODY_STEP) {
        float hw = body_half_width(depth);
        out[n].x = BODY_CX - hw;
        out[n].y = BODY_TOP + depth;
        out[n].w = hw * 2.0f;
        n++;
    }

    return n;
}

float splash_mx5_body_step(void)
{
    return BODY_STEP;
}

/* The body starts low and slides up into place. It never leaves the
 * screen: at its lowest the bumper is still above the bottom edge. */
float splash_mx5_body_rise(uint32_t t_ms)
{
    return RISE_PX * (1.0f - ease_out(phase(t_ms, 0u, RISE_END_MS)));
}

float splash_mx5_lid_angle_deg(uint32_t t_ms)
{
    return 90.0f * ease_out(phase(t_ms, LID_START_MS, LID_END_MS));
}

uint32_t splash_mx5_total_ms(void)
{
    return SPLASH_MX5_TOTAL_MS;
}

/* Full-screen wash, floor line and ground. Painted first so nothing of
 * the previous frame can show through. */
static void draw_backdrop(void)
{
    gfx_rect(0.0f, 0.0f, (float)GFX_W, (float)GFX_H, QV_BG);
    gfx_rect(0.0f, FLOOR_Y, (float)GFX_W, (float)GFX_H - FLOOR_Y,
             gfx_shade(QV_PANEL, 55));
    gfx_rect(0.0f, FLOOR_Y, (float)GFX_W, 1.0f, gfx_shade(QV_DIM, 45));
}

/* Windscreen header and the two pillars over the bonnet: the car is a
 * roadster with the top down, so this is all the "roof" there is. */
static void draw_windscreen(float dy)
{
    float top = BODY_TOP - 30.0f + dy;
    uint32_t c = gfx_shade(QV_DIM, 65);

    gfx_line(BODY_CX - 56.0f, top, BODY_CX + 56.0f, top, 3.0f, c);
    gfx_line(BODY_CX - 56.0f, top, BODY_CX - 84.0f, BODY_TOP + dy,
             2.5f, c);
    gfx_line(BODY_CX + 56.0f, top, BODY_CX + 84.0f, BODY_TOP + dy,
             2.5f, c);
}

/* Bodywork: one rect per silhouette row, in two tones so the bonnet edge
 * reads as a highlight and the bumper stays in shadow. */
static void draw_body(float dy)
{
    splash_mx5_span spans[BODY_ROWS];
    int n = splash_mx5_body_spans(spans, BODY_ROWS);
    float step = splash_mx5_body_step();

    for (int i = 0; i < n; i++) {
        float depth = spans[i].y - BODY_TOP;
        uint32_t c = (depth < 46.0f) ? gfx_shade(QV_RED, 74)
                                     : gfx_shade(QV_RED, 52);
        gfx_rect(spans[i].x, spans[i].y + dy, spans[i].w, step, c);
    }

    if (n > 0) {
        gfx_rect(spans[0].x, spans[0].y + dy, spans[0].w, 2.0f, QV_RED);
    }
}

/* Front tyres, peeking out from behind the bumper. */
static void draw_tyres(float dy)
{
    uint32_t c = gfx_shade(QV_PANEL2, 40);
    float y = BODY_BOTTOM - 14.0f + dy;

    gfx_rect(BODY_CX - 152.0f, y, 26.0f, 16.0f, c);
    gfx_rect(BODY_CX + 126.0f, y, 26.0f, 16.0f, c);
}

/* The wide low intake of the NB, rounded by stacking narrower rows. */
static void draw_grille(float dy)
{
    float y = BODY_TOP + 60.0f + dy;
    uint32_t rim = gfx_shade(QV_DIM, 70);

    gfx_rect(BODY_CX - 52.0f, y, 104.0f, 6.0f, QV_PANEL2);
    gfx_rect(BODY_CX - 62.0f, y + 6.0f, 124.0f, 16.0f, QV_PANEL2);
    gfx_rect(BODY_CX - 66.0f, y + 22.0f, 132.0f, 16.0f, QV_PANEL2);
    gfx_rect(BODY_CX - 56.0f, y + 38.0f, 112.0f, 6.0f, QV_PANEL2);

    gfx_rect(BODY_CX - 58.0f, y + 14.0f, 116.0f, 2.0f, QV_BG);
    gfx_rect(BODY_CX - 60.0f, y + 30.0f, 120.0f, 2.0f, QV_BG);

    gfx_rect(BODY_CX - 52.0f, y - 1.0f, 104.0f, 1.0f, rim);
}

/* Mazda badge on the nose: ring plus the wing inside it. */
static void draw_badge(float dy)
{
    float by = BODY_TOP + 36.0f + dy;
    float r = 13.0f;

    gfx_arc(BODY_CX, by, r, 2.0f, 0.0f, 360.0f, 24, QV_DIM);
    gfx_line(BODY_CX, by - 6.0f, BODY_CX - 7.0f, by + 3.0f, 1.5f,
             QV_TEXT);
    gfx_line(BODY_CX, by - 6.0f, BODY_CX + 7.0f, by + 3.0f, 1.5f,
             QV_TEXT);
    gfx_line(BODY_CX - 7.0f, by + 3.0f, BODY_CX + 7.0f, by + 3.0f,
             1.5f, QV_TEXT);
}

/* One pop-up headlight. side is -1 for the left lamp and +1 for the
 * right one. open is the sine of the lid angle: 0 is a closed pod flush
 * with the bonnet, 1 a lamp standing over the fender. glow is the
 * lighting ramp, 0 while the lamp is off. */
static void draw_headlight(int side, float dy, float angle_deg, float glow)
{
    float hx = BODY_CX + (float)side * 108.0f;
    float base = BODY_TOP + 8.0f + dy;
    float open = sinf(angle_deg * DEG_TO_RAD);
    float pod_h = 2.0f + 26.0f * open;
    float top = base - pod_h;
    float lens = 2.0f + 6.0f * open;
    float cy = top + pod_h * 0.5f;

    gfx_rect(hx - 22.0f, top, 44.0f, pod_h, QV_BG);

    if (glow > 0.0f) {
        gfx_circle(hx, cy, lens + 2.0f + 2.0f * glow,
                   gfx_shade(QV_AMBER, 30));
        gfx_circle(hx, cy, lens + 1.0f,
                   gfx_shade(QV_AMBER, 60 + (int)(35.0f * glow)));
    }

    gfx_circle(hx, cy, lens, glow > 0.0f ? QV_TEXT : QV_DIM);

    gfx_rect(hx - 23.0f, top - 2.0f, 46.0f, 2.0f,
             gfx_shade(QV_RED, 90));
}

/* Word mark, rule and model line, revealed from 1000 ms like the Alfa
 * splash, so both animations hand over to the dashboard at the same
 * point. */
static void draw_marks(uint32_t t_ms)
{
    if (t_ms < MARK_MS) {
        return;
    }

    float fade = ease_out(phase(t_ms, MARK_MS, MARK_MS + MARK_FADE_MS));
    float sub = ease_out(phase(t_ms, MARK_MS,
                               MARK_MS + SUB_FADE_MS));
    float w = 100.0f + 200.0f *
              phase(t_ms, MARK_MS, SPLASH_MX5_TOTAL_MS);

    w = clamp_f(w, 0.0f, 300.0f);

    gfx_text_center(GFX_W / 2, 204, "MAZDA MX-5",
                    gfx_shade(QV_TEXT, 45 + (int)(55.0f * fade)), 3);

    gfx_rect(BODY_CX - w * 0.5f, 232.0f, w * 0.5f, 2.0f, QV_RED);
    gfx_rect(BODY_CX, 232.0f, w * 0.5f, 2.0f, QV_TEXT);

    gfx_text_center(GFX_W / 2, 240, SPLASH_MX5_SUBTITLE,
                    gfx_shade(QV_DIM, 40 + (int)(60.0f * sub)), 1);
}

void splash_mx5_draw(uint32_t t_ms)
{
    float t_rise;
    float angle;
    float glow;

    if (t_ms > SPLASH_MX5_TOTAL_MS) {
        t_ms = SPLASH_MX5_TOTAL_MS;
    }

    t_rise = splash_mx5_body_rise(t_ms);
    angle = splash_mx5_lid_angle_deg(t_ms);
    glow = ease_out(phase(t_ms, LAMP_START_MS, LAMP_END_MS));

    gfx_frame_begin();

    draw_backdrop();
    draw_windscreen(t_rise);
    draw_tyres(t_rise);
    draw_body(t_rise);
    draw_grille(t_rise);
    draw_badge(t_rise);
    draw_headlight(-1, t_rise, angle, glow);
    draw_headlight(1, t_rise, angle, glow);
    draw_marks(t_ms);

    gfx_frame_end();
}
