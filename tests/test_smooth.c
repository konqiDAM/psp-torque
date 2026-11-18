#include "obd/smooth.h"
#include "test_util.h"

/* FASE E3: el suavizado es la parte de la cadena que mas se nota en el
 * panel. Estos tests fijan su comportamiento en el tiempo, que es lo
 * que importa: el retardo debe ser tau, no "un numero de muestras". */

/* El factor de mezcla debe seguir la constante de tiempo: con un
 * intervalo igual a tau, alpha debe ser 1 - 1/e. */
static void test_alpha_tau(void)
{
    CHECK_NEAR(obd_smooth_alpha((uint32_t)OBD_SMOOTH_TAU_MS),
                1.0 - 1.0 / 2.718281828459045, 0.001);
}

/* Con un intervalo muy corto apenas se suaviza; con uno largo se pasa
 * casi todo: ese es el comportamiento de un filtro de tau fijo. */
static void test_alpha_extremos(void)
{
    CHECK(obd_smooth_alpha(1u) < 0.02f);
    CHECK(obd_smooth_alpha(50u) < 0.30f);
    CHECK(obd_smooth_alpha(5000u) > 0.99f);
}

/* La primera muestra se toma tal cual. */
static void test_primera_muestra(void)
{
    obd_smooth s;
    obd_smooth_reset(&s);
    CHECK_NEAR(obd_smooth_apply(&s, 2500.0f, 1000u), 2500.0, 0.001);
}

/* Con un intervalo igual a tau, tras una serie de muestras constantes
 * el filtro debe converger al valor real: es la propiedad que
 * guarantee que el retardo es tau y no el numero de muestras. */
static void test_convergencia_con_tau(void)
{
    obd_smooth s;
    obd_smooth_reset(&s);

    uint32_t ts = 0u;
    obd_smooth_apply(&s, 0.0f, ts);          /* valor inicial 0 */

    /* Un salto a 100 con tau = 150 ms y muestreo cada 150 ms. */
    for (int i = 1; i <= 10; i++) {
        ts += (uint32_t)OBD_SMOOTH_TAU_MS;
        obd_smooth_apply(&s, 100.0f, ts);
    }
    CHECK_NEAR(s.value, 100.0, 0.5);
}

/* El retardo real: el tiempo que tarda el filtro en llegar al 63% del
 * valor debe ser tau, con independencia de la cadencia de muestreo.
 *
 * Con un factor fijo de 0,3 (el de antes de la FASE E) y muestreo a
 * 1,3 Hz, el 63% tardaba 2,1 s. Aqui, con cualquier cadencia, el 63%
 * se alcanza en tau. */
static void test_retraso_independiente_de_la_cadencia(void)
{
    uint32_t cadencias[] = { 100u, 350u, 700u };
    const float objetivo = 0.6321205588f;   /* 1 - 1/e */

    for (unsigned c = 0; c < sizeof(cadencias) / sizeof(cadencias[0]); c++) {
        obd_smooth s;
        obd_smooth_reset(&s);

        uint32_t ts = 0u;
        obd_smooth_apply(&s, 0.0f, ts);

        float v = 0.0f;
        uint32_t elapsed = 0u;
        uint32_t dt = cadencias[c];

        while (v < objetivo && elapsed < 10000u) {
            ts += dt;
            elapsed += dt;
            v = obd_smooth_apply(&s, 100.0f, ts);
        }

        /* El tiempo al 63% debe quedarse cerca de tau. Con un margen
         * de una muestra, porque el filtro solo se evalua al llegar
         * una muestra nueva. */
        float limite = OBD_SMOOTH_TAU_MS + (float)dt;
        CHECK(elapsed >= (uint32_t)(OBD_SMOOTH_TAU_MS * 0.5f));
        CHECK((float)elapsed <= limite + 1.0f);
    }
}

/* Un intervalo enorme (por ejemplo tras una reconexion) debe dejar
 * pasar el valor nuevo sin quedarse con el viejo. */
static void test_salto_grande(void)
{
    obd_smooth s;
    obd_smooth_reset(&s);
    obd_smooth_apply(&s, 100.0f, 1000u);
    float v = obd_smooth_apply(&s, 5000.0f, 60000u);
    CHECK(v > 4900.0f);
}

/* La rampa de presentacion: el escalon se recorre entero durante el
 * intervalo de muestreo, no casi todo al principio. El precio es un
 * intervalo de retraso, que es lo que compra el movimiento continuo. */
static void test_rampa(void)
{
    obd_ramp r;
    obd_ramp_reset(&r);

    /* Primera muestra: se coloca, no arranca de cero. */
    CHECK_NEAR(obd_ramp_step(&r, 1000.0f, 1u, 0u, 1000u), 1000.0, 0.001);

    /* Llega la segunda muestra en t=500: la aguja sigue donde estaba y el
     * tramo empieza ahi (continuidad, sin salto). */
    CHECK_NEAR(obd_ramp_step(&r, 2000.0f, 2u, 500u, 1000u), 1000.0, 0.001);
    /* A mitad del tramo, mitad de camino. */
    CHECK_NEAR(obd_ramp_step(&r, 2000.0f, 2u, 1000u, 1000u), 1500.0, 0.001);
    /* Y al final del tramo, en el objetivo. */
    CHECK_NEAR(obd_ramp_step(&r, 2000.0f, 2u, 1500u, 1000u), 2000.0, 0.001);

    /* Si la muestra tarda mas que el tramo, se queda en el objetivo: no se
     * extrapola nada. */
    CHECK_NEAR(obd_ramp_step(&r, 2000.0f, 2u, 9000u, 1000u), 2000.0, 0.001);

    /* Muestra nueva a mitad de camino: el tramo arranca desde donde esta la
     * aguja, no desde el valor viejo. */
    obd_ramp_reset(&r);
    obd_ramp_step(&r, 0.0f, 1u, 0u, 1000u);
    CHECK_NEAR(obd_ramp_step(&r, 100.0f, 2u, 500u, 1000u), 0.0, 0.001);
    CHECK_NEAR(obd_ramp_step(&r, 100.0f, 2u, 1000u, 1000u), 50.0, 0.001);
    float hold = obd_ramp_step(&r, 200.0f, 3u, 1000u, 1000u);
    CHECK_NEAR(hold, 50.0, 0.001);
    CHECK_NEAR(obd_ramp_step(&r, 200.0f, 3u, 1500u, 1000u), 125.0, 0.001);

    /* Mismo instante de muestra y otro objetivo: el tramo se rehace desde
     * la aguja y adopta el tramo nuevo. */
    obd_ramp_reset(&r);
    obd_ramp_step(&r, 0.0f, 9u, 0u, 1000u);
    CHECK_NEAR(obd_ramp_step(&r, 100.0f, 10u, 0u, 1000u), 0.0, 0.001);
    CHECK_NEAR(obd_ramp_step(&r, 100.0f, 10u, 500u, 1000u), 50.0, 0.001);
    /* Mismo ts y mismo objetivo, pero tramo mas largo: tambien se rehace. */
    CHECK_NEAR(obd_ramp_step(&r, 100.0f, 10u, 500u, 2000u), 50.0, 0.001);
    CHECK_NEAR(obd_ramp_step(&r, 100.0f, 10u, 1500u, 2000u), 75.0, 0.001);

    /* Dos lecturas en el mismo fotograma no mueven nada la segunda vez. */
    obd_ramp_reset(&r);
    obd_ramp_step(&r, 0.0f, 20u, 0u, 1000u);
    obd_ramp_step(&r, 100.0f, 21u, 0u, 1000u);
    float once = obd_ramp_step(&r, 100.0f, 21u, 250u, 1000u);
    float twice = obd_ramp_step(&r, 100.0f, 21u, 250u, 1000u);
    CHECK_NEAR(once, twice, 0.0001);

    /* Desbordamiento del contador de milisegundos. */
    obd_ramp_reset(&r);
    obd_ramp_step(&r, 0.0f, 30u, 0xFFFFFE00u, 1000u);
    obd_ramp_step(&r, 100.0f, 31u, 0xFFFFFE00u, 1000u);
    float wrap = obd_ramp_step(&r, 100.0f, 31u, 0x00000100u, 1000u);
    CHECK(wrap > 0.0f);
    CHECK(wrap < 100.0f);

    /* Un tramo de duracion cero coloca el valor de golpe. */
    CHECK_NEAR(obd_ramp_step(&r, 900.0f, 4u, 600u, 0u), 900.0, 0.001);

    /* Nunca se sale del intervalo entre el valor viejo y el nuevo. */
    obd_ramp_reset(&r);
    obd_ramp_step(&r, 0.0f, 1u, 0u, 1000u);
    obd_ramp_step(&r, 100.0f, 2u, 0u, 1000u);
    for (uint32_t t = 0; t <= 2000u; t += 16u) {
        float v = obd_ramp_step(&r, 100.0f, 2u, t, 1000u);
        CHECK(v >= -0.001f);
        CHECK(v <= 100.001f);
    }
}

void test_smooth(void)
{
    test_alpha_tau();
    test_alpha_extremos();
    test_primera_muestra();
    test_convergencia_con_tau();
    test_retraso_independiente_de_la_cadencia();
    test_salto_grande();
    test_rampa();
}
