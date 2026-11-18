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

/* El paso de presentacion mueve el valor en el tiempo: es lo que hace
 * que la aguja no salte cuando llega la muestra. */
static void test_paso_de_presentacion(void)
{
    obd_smooth s;
    obd_smooth_reset(&s);

    /* Primer paso: se coloca en el objetivo, no arranca desde cero. */
    CHECK_NEAR(obd_smooth_step(&s, 3000.0f, 1000u, 300u), 3000.0, 0.001);

    /* Un fotograma de 16 ms con tau 300 avanza poco. */
    float v = obd_smooth_step(&s, 4000.0f, 1016u, 300u);
    CHECK(v > 3000.0f);
    CHECK(v < 3100.0f);

    /* En un tau recorre el 63% del camino. */
    v = obd_smooth_step(&s, 4000.0f, 1016u + 300u, 300u);
    CHECK(v > 3500.0f);
    CHECK(v < 3700.0f);

    /* Y al final llega y se queda clavado: sin cola asintotica. */
    uint32_t t = 1016u + 300u;
    for (int i = 0; i < 200; i++) {
        t += 16u;
        v = obd_smooth_step(&s, 4000.0f, t, 300u);
    }
    CHECK_NEAR(v, 4000.0, 0.001);

    /* Dos lecturas en el mismo fotograma no avanzan el estado. */
    float a = obd_smooth_step(&s, 5000.0f, t + 16u, 300u);
    float b = obd_smooth_step(&s, 5000.0f, t + 16u, 300u);
    CHECK_NEAR(a, b, 0.0001);
}

void test_smooth(void)
{
    test_alpha_tau();
    test_alpha_extremos();
    test_primera_muestra();
    test_convergencia_con_tau();
    test_retraso_independiente_de_la_cadencia();
    test_salto_grande();
    test_paso_de_presentacion();
}
