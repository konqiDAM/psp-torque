/*
 * test_splash_mx5.c - host tests of the MX-5 boot animation.
 *
 * The look cannot be checked without a screen, so this test checks what
 * can be wrong in a silent way:
 *   - the silhouette is symmetric, inside the screen and bulge-shaped
 *   - the timeline (rise, lid opening) is monotonic and ends where the
 *     design says it does
 *   - every frame is a complete, bounded repaint: one frame_begin/end,
 *     the backdrop first, every coordinate on screen, and a vertex
 *     estimate under the gfx vertex buffer
 *
 * gfx is replaced by counting stubs: no GU, no display.
 */

#include "../src/psp/gfx.h"
#include "../src/psp/splash_mx5.h"
#include "test_util.h"

#include <string.h>

/* --- gfx stubs: they count, they do not paint ---------------------- */

gfx_theme g_active_theme;
int g_gfx_dim = 100;

typedef struct {
    int frames_begin;
    int frames_end;
    int rects;
    int lines;
    int circles;
    int arcs;
    int texts;
    long verts;
    float min_x, min_y, max_x, max_y;
    float first_x, first_y, first_w, first_h;
} record;

static record g_rec;

static void rec_reset(void)
{
    memset(&g_rec, 0, sizeof(g_rec));
    g_rec.min_x = 1e9f;
    g_rec.min_y = 1e9f;
    g_rec.max_x = -1e9f;
    g_rec.max_y = -1e9f;
}

static void rec_box(float x, float y, float w, float h)
{
    if (x < g_rec.min_x) {
        g_rec.min_x = x;
    }
    if (y < g_rec.min_y) {
        g_rec.min_y = y;
    }
    if (x + w > g_rec.max_x) {
        g_rec.max_x = x + w;
    }
    if (y + h > g_rec.max_y) {
        g_rec.max_y = y + h;
    }
}

static int clamp_byte(int v)
{
    return v < 0 ? 0 : (v > 255 ? 255 : v);
}

uint32_t gfx_shade(uint32_t c, int percent)
{
    int r = clamp_byte((int)(c & 0xFF) * percent / 100);
    int g = clamp_byte((int)((c >> 8) & 0xFF) * percent / 100);
    int b = clamp_byte((int)((c >> 16) & 0xFF) * percent / 100);

    return GFX_RGBA(r, g, b, (int)((c >> 24) & 0xFF));
}

uint32_t gfx_dimmed(uint32_t c)
{
    return c;
}

void gfx_frame_begin(void)
{
    g_rec.frames_begin++;
}

void gfx_frame_end(void)
{
    g_rec.frames_end++;
}

void gfx_rect(float x, float y, float w, float h, uint32_t color)
{
    (void)color;
    if (w <= 0.0f || h <= 0.0f) {
        return;
    }
    g_rec.rects++;
    g_rec.verts += 6;
    if (g_rec.rects == 1) {
        g_rec.first_x = x;
        g_rec.first_y = y;
        g_rec.first_w = w;
        g_rec.first_h = h;
    }
    rec_box(x, y, w, h);
}

void gfx_line(float x1, float y1, float x2, float y2, float t,
              uint32_t color)
{
    (void)color;
    if (t <= 0.0f) {
        return;
    }
    g_rec.lines++;
    g_rec.verts += 6;
    rec_box(x1, y1, 0.0f, 0.0f);
    rec_box(x2, y2, 0.0f, 0.0f);
}

/* gfx_circle paints one 1 px row per row of the disc. */
void gfx_circle(float cx, float cy, float r, uint32_t color)
{
    (void)color;
    if (r <= 0.0f) {
        return;
    }
    g_rec.circles++;
    g_rec.verts += 6L * (long)(2.0f * r + 1.0f);
    rec_box(cx - r, cy - r, 2.0f * r, 2.0f * r);
}

/* gfx_arc chains segments with lines. */
void gfx_arc(float cx, float cy, float r, float t, float start_deg,
             float end_deg, int segments, uint32_t color)
{
    (void)start_deg;
    (void)end_deg;
    (void)color;
    if (segments < 1) {
        segments = 1;
    }
    g_rec.arcs++;
    g_rec.verts += 6L * (long)segments;
    rec_box(cx - r - t, cy - r - t, 2.0f * (r + t), 2.0f * (r + t));
}

/* Text is not rasterised here. It is charged as the known worst case of
 * text_pass (8 rows x 8 runs per glyph) so the vertex bound holds
 * whatever the glyph contents are. */
int gfx_text(int x, int y, const char *s, uint32_t color, int scale)
{
    (void)color;
    size_t len;

    if (s == NULL) {
        return x;
    }
    if (scale < 1) {
        scale = 1;
    }
    len = strlen(s);
    g_rec.texts++;
    g_rec.verts += 6L * 64L * (long)len;
    rec_box((float)x, (float)y, (float)(8 * scale * (int)len),
            (float)(8 * scale));
    return x + 8 * scale * (int)len;
}

void gfx_text_center(int cx, int y, const char *s, uint32_t color,
                     int scale)
{
    if (s == NULL) {
        return;
    }
    if (scale < 1) {
        scale = 1;
    }
    gfx_text(cx - 8 * scale * (int)strlen(s) / 2, y, s, color, scale);
}

/* --- Silhouette ---------------------------------------------------- */

static void test_body_spans(void)
{
    splash_mx5_span spans[64];
    splash_mx5_span small[4];
    int n;
    int i;
    float step;
    float widest = 0.0f;
    int widest_row = 0;

    n = splash_mx5_body_spans(spans, 64);
    CHECK(n >= 16);
    CHECK(n <= 64);
    CHECK(splash_mx5_body_spans(small, 4) == 4);
    CHECK(splash_mx5_body_spans(NULL, 8) == 0);
    CHECK(splash_mx5_body_spans(spans, 0) == 0);

    step = splash_mx5_body_step();
    CHECK(step > 0.0f);

    for (i = 0; i < n; i++) {
        float centre = spans[i].x + spans[i].w * 0.5f;

        /* Symmetric about the centre of the screen. */
        CHECK_NEAR(centre, GFX_W * 0.5f, 0.01);

        /* Inside the screen, last row included. */
        CHECK(spans[i].x >= 0.0f);
        CHECK(spans[i].x + spans[i].w <= (float)GFX_W);
        CHECK(spans[i].y >= 0.0f);
        CHECK(spans[i].y + step <= (float)GFX_H);

        /* Contiguous rows, top to bottom. */
        if (i > 0) {
            CHECK_NEAR(spans[i].y - spans[i - 1].y, step, 0.01);
        }

        if (spans[i].w > widest) {
            widest = spans[i].w;
            widest_row = i;
        }
    }

    /* Roadster nose: the widest point is the fender line, not the top
     * edge and not the bumper. */
    CHECK(widest > 280.0f);
    CHECK(widest < 340.0f);
    CHECK(widest_row > 0);
    CHECK(widest_row < n - 1);
    CHECK(spans[0].w < widest);
    CHECK(spans[n - 1].w < widest);
}

/* --- Timeline ------------------------------------------------------ */

static void test_timeline(void)
{
    uint32_t t;
    float prev_rise;
    float prev_lid;

    CHECK(splash_mx5_total_ms() == SPLASH_MX5_TOTAL_MS);
    CHECK(splash_mx5_total_ms() == 1600u);

    /* The body starts low and ends in place. The travel is smaller than
     * the gap to the bottom edge, so it never leaves the screen. */
    CHECK(splash_mx5_body_rise(0u) > 0.0f);
    CHECK(splash_mx5_body_rise(0u) <= 80.0f);
    CHECK(splash_mx5_body_rise(420u) == 0.0f);
    CHECK(splash_mx5_body_rise(1600u) == 0.0f);

    prev_rise = splash_mx5_body_rise(0u);
    for (t = 0; t <= SPLASH_MX5_TOTAL_MS; t += 5u) {
        float rise = splash_mx5_body_rise(t);
        CHECK(rise <= prev_rise + 0.001f);
        CHECK(rise >= 0.0f);
        prev_rise = rise;
    }

    /* Pop-ups: closed at the start, open between 380 and 760. */
    CHECK(splash_mx5_lid_angle_deg(0u) == 0.0f);
    CHECK(splash_mx5_lid_angle_deg(380u) == 0.0f);
    CHECK(splash_mx5_lid_angle_deg(760u) == 90.0f);
    CHECK(splash_mx5_lid_angle_deg(1600u) == 90.0f);

    prev_lid = 0.0f;
    for (t = 0; t <= SPLASH_MX5_TOTAL_MS; t += 5u) {
        float lid = splash_mx5_lid_angle_deg(t);
        CHECK(lid >= prev_lid - 0.001f);
        CHECK(lid >= 0.0f);
        CHECK(lid <= 90.0f);
        prev_lid = lid;
    }

    /* They open before the text shows: the final frame is what a user
     * with the animation disabled sees. */
    CHECK(splash_mx5_lid_angle_deg(1000u) == 90.0f);

    /* Even at its lowest the body stays on screen. */
    {
        splash_mx5_span spans[64];
        int n = splash_mx5_body_spans(spans, 64);
        float rise = splash_mx5_body_rise(0u);
        CHECK(n > 0);
        CHECK(spans[0].y + rise > 0.0f);
        CHECK(spans[n - 1].y + rise + splash_mx5_body_step() <=
              (float)GFX_H);
    }
}

/* --- Frames -------------------------------------------------------- */

#define MAX_FRAME_VERTS 32768L

static void check_frame(uint32_t t)
{
    rec_reset();
    splash_mx5_draw(t);

    CHECK(g_rec.frames_begin == 1);
    CHECK(g_rec.frames_end == 1);

    /* Opaque full-screen backdrop as the first stroke. */
    CHECK_NEAR(g_rec.first_x, 0.0f, 0.01);
    CHECK_NEAR(g_rec.first_y, 0.0f, 0.01);
    CHECK_NEAR(g_rec.first_w, (float)GFX_W, 0.01);
    CHECK_NEAR(g_rec.first_h, (float)GFX_H, 0.01);

    /* Everything on screen (2 px of slack for the line thickness). */
    CHECK(g_rec.min_x >= -2.0f);
    CHECK(g_rec.min_y >= -2.0f);
    CHECK(g_rec.max_x <= (float)GFX_W + 2.0f);
    CHECK(g_rec.max_y <= (float)GFX_H + 2.0f);

    /* Under the vertex and quad budget of gfx.c. */
    CHECK(g_rec.verts < MAX_FRAME_VERTS);
    CHECK(g_rec.rects <= MAX_FRAME_VERTS / 6);
    CHECK(g_rec.lines < 200);
    CHECK(g_rec.circles < 40);
    CHECK(g_rec.arcs < 8);
}

static void test_frames(void)
{
    uint32_t t;
    int rects_final;
    int lines_final;
    long verts_final;

    for (t = 0; t <= SPLASH_MX5_TOTAL_MS; t += 20u) {
        check_frame(t);
    }
    check_frame(SPLASH_MX5_TOTAL_MS);

    /* No text before 1000 ms, and from there on both lines are drawn. */
    rec_reset();
    splash_mx5_draw(999u);
    CHECK(g_rec.texts == 0);

    rec_reset();
    splash_mx5_draw(1000u);
    CHECK(g_rec.texts == 2);

    /* The whole body is painted in every frame, the first one too. */
    rec_reset();
    splash_mx5_draw(0u);
    CHECK(g_rec.rects > 60);
    CHECK(g_rec.lines >= 4);

    /* Drawing is a pure function of t: same t, same frame; and a t past
     * the end clamps to the last frame. */
    rec_reset();
    splash_mx5_draw(SPLASH_MX5_TOTAL_MS);
    rects_final = g_rec.rects;
    lines_final = g_rec.lines;
    verts_final = g_rec.verts;

    rec_reset();
    splash_mx5_draw(SPLASH_MX5_TOTAL_MS);
    CHECK(g_rec.rects == rects_final);
    CHECK(g_rec.lines == lines_final);
    CHECK(g_rec.verts == verts_final);

    rec_reset();
    splash_mx5_draw(999999u);
    CHECK(g_rec.rects == rects_final);
    CHECK(g_rec.lines == lines_final);
    CHECK(g_rec.verts == verts_final);
}

void test_splash_mx5(void)
{
    g_active_theme.bg = GFX_RGB(0x05, 0x05, 0x05);
    g_active_theme.panel = GFX_RGB(0x10, 0x12, 0x14);
    g_active_theme.panel2 = GFX_RGB(0x1A, 0x1D, 0x20);
    g_active_theme.track = GFX_RGB(0x2A, 0x2E, 0x32);
    g_active_theme.text = GFX_RGB(0xF2, 0xF0, 0xEB);
    g_active_theme.dim = GFX_RGB(0x8C, 0x91, 0x96);
    g_active_theme.red = GFX_RGB(0xC8, 0x10, 0x2E);
    g_active_theme.green = GFX_RGB(0x00, 0x8C, 0x45);
    g_active_theme.amber = GFX_RGB(0xFF, 0xB0, 0x00);
    g_active_theme.crit = GFX_RGB(0xFF, 0x2A, 0x2A);
    g_active_theme.ok = GFX_RGB(0x2E, 0xCC, 0x71);
    g_active_theme.cyan = GFX_RGB(0x00, 0xC2, 0xD1);

    test_body_spans();
    test_timeline();
    test_frames();
}
