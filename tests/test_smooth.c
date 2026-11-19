#include "obd/smooth.h"
#include "test_util.h"

/* El unico suavizado que queda es el de presentacion: mueve lo que se
 * pinta hacia la ultima medida con una constante de tiempo fija. Estos
 * tests fijan lo que se nota en el panel: el retardo en tiempo, no en
 * numero de cuadros, y que pintar dos veces el mismo cuadro no mueva la
 * aguja. */

#define TAU_NEEDLE_MS 150.0f
#define TAU_SOFT_MS   400.0f

/* Con un intervalo igual a tau, alpha debe ser 1 - 1/e. */
static void test_alpha_tau(void)
{
    CHECK_NEAR(obd_ease_alpha((uint32_t)TAU_NEEDLE_MS, TAU_NEEDLE_MS),
               1.0 - 1.0 / 2.718281828459045, 0.001);
}

/* Sin tiempo transcurrido no se mueve nada; sin constante de tiempo se
 * pinta el valor medido tal cual; con un intervalo muy corto apenas se
 * suaviza y con uno largo se pasa casi todo. */
static void test_alpha_extremos(void)
{
    CHECK_NEAR(obd_ease_alpha(0u, TAU_NEEDLE_MS), 0.0, 0.0001);
    CHECK_NEAR(obd_ease_alpha(16u, 0.0f), 1.0, 0.0001);
    CHECK(obd_ease_alpha(1u, TAU_NEEDLE_MS) < 0.02f);
    CHECK(obd_ease_alpha(50u, TAU_NEEDLE_MS) < 0.30f);
    CHECK(obd_ease_alpha(5000u, TAU_NEEDLE_MS) > 0.99f);
}

/* La primera muestra se toma tal cual: al aparecer un dato la aguja no
 * puede subir desde cero. */
static void test_primera_muestra(void)
{
    obd_smooth s;
    obd_smooth_reset(&s);
    CHECK_NEAR(obd_ease_apply(&s, 2500.0f, 1000u, TAU_NEEDLE_MS), 2500.0,
               0.001);
}

/* Con un objetivo constante, el 63 % del recorrido se alcanza en tau,
 * con independencia de lo que dure el cuadro. */
static void test_retraso_es_tau(void)
{
    uint32_t cuadros[] = { 16u, 33u, 100u };
    const float objetivo = 0.6321205588f;   /* 1 - 1/e */

    for (unsigned c = 0; c < sizeof(cuadros) / sizeof(cuadros[0]); c++) {
        obd_smooth s;
        obd_smooth_reset(&s);

        uint32_t now = 0u;
        obd_ease_apply(&s, 0.0f, now, TAU_NEEDLE_MS);

        float v = 0.0f;
        uint32_t elapsed = 0u;
        uint32_t dt = cuadros[c];

        while (v < objetivo && elapsed < 10000u) {
            now += dt;
            elapsed += dt;
            v = obd_ease_apply(&s, 1.0f, now, TAU_NEEDLE_MS);
        }

        CHECK(elapsed >= (uint32_t)(TAU_NEEDLE_MS * 0.5f));
        CHECK((float)elapsed <= TAU_NEEDLE_MS + (float)dt + 1.0f);
    }
}

/* Un intervalo enorme (por ejemplo tras una reconexion) deja pasar el
 * valor nuevo en vez de arrastrar el viejo. */
static void test_salto_grande(void)
{
    obd_smooth s;
    obd_smooth_reset(&s);
    obd_ease_apply(&s, 100.0f, 1000u, TAU_NEEDLE_MS);
    float v = obd_ease_apply(&s, 5000.0f, 60000u, TAU_NEEDLE_MS);
    CHECK(v > 4900.0f);
}

/* Pintar el mismo cuadro dos veces en el mismo milisegundo no puede
 * mover la aguja: el numero, el pico y el grafico leen el mismo PID
 * varias veces por fotograma. */
static void test_mismo_milisegundo(void)
{
    obd_smooth s;
    obd_smooth_reset(&s);
    obd_ease_apply(&s, 1000.0f, 1000u, TAU_NEEDLE_MS);
    float a = obd_ease_apply(&s, 5000.0f, 1016u, TAU_NEEDLE_MS);
    float b = obd_ease_apply(&s, 5000.0f, 1016u, TAU_NEEDLE_MS);
    CHECK_NEAR(a, b, 0.0001);
    CHECK(a > 1000.0f && a < 5000.0f);
}

/* AGUJA 1 y AGUJA 2 se diferencian en esto: con el mismo cuadro, la
 * constante blanda llega mas tarde. */
static void test_tau_blanda_mas_lenta(void)
{
    obd_smooth duro;
    obd_smooth blando;
    obd_smooth_reset(&duro);
    obd_smooth_reset(&blando);
    obd_ease_apply(&duro, 0.0f, 0u, TAU_NEEDLE_MS);
    obd_ease_apply(&blando, 0.0f, 0u, TAU_SOFT_MS);

    float v_duro = 0.0f;
    float v_blando = 0.0f;
    for (uint32_t now = 16u; now <= 160u; now += 16u) {
        v_duro = obd_ease_apply(&duro, 100.0f, now, TAU_NEEDLE_MS);
        v_blando = obd_ease_apply(&blando, 100.0f, now, TAU_SOFT_MS);
    }
    CHECK(v_duro > v_blando);
    CHECK(v_blando < 40.0f);
}

void test_smooth(void)
{
    test_alpha_tau();
    test_alpha_extremos();
    test_primera_muestra();
    test_retraso_es_tau();
    test_salto_grande();
    test_mismo_milisegundo();
    test_tau_blanda_mas_lenta();
}
