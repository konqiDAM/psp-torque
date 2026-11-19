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

/* La prediccion: la aguja se adelanta con la velocidad que traia, y en el
 * instante de la muestra se ensena siempre la medida. */
static void test_prediccion(void)
{
    obd_predict p;
    obd_predict_reset(&p);

    /* Primera muestra: se coloca y no predice nada. */
    CHECK_NEAR(obd_predict_step(&p, 1000.0f, 1000u, 1000u, 1000u,
                                600.0f, 25.0f), 1000.0, 0.001);

    /* Sube 500 rpm en un segundo: la velocidad es 500 rpm/s. A mitad del
     * intervalo la aguja debe ir por delante de la ultima medida. */
    obd_predict_step(&p, 1500.0f, 2000u, 2000u, 1000u, 600.0f, 25.0f);
    float mid = obd_predict_step(&p, 1500.0f, 2000u, 2500u, 1000u,
                                 600.0f, 25.0f);
    CHECK(mid > 1500.0f);
    CHECK(mid <= 2100.0f);

    /* Pasado el intervalo, se queda en la medida: no sigue subiendo. */
    CHECK_NEAR(obd_predict_step(&p, 1500.0f, 2000u, 4000u, 1000u,
                                600.0f, 25.0f), 1500.0, 0.001);

    /* Muestra nueva: se ensena la medida exacta, sin mezcla. */
    CHECK_NEAR(obd_predict_step(&p, 2000.0f, 3000u, 3000u, 1000u,
                                600.0f, 25.0f), 2000.0, 0.001);

    /* Bajando, igual pero al reves. */
    obd_predict_step(&p, 1000.0f, 4000u, 4000u, 1000u, 600.0f, 25.0f);
    float down = obd_predict_step(&p, 1000.0f, 4000u, 4500u, 1000u,
                                  600.0f, 25.0f);
    CHECK(down < 1000.0f);

    /* Y nunca por debajo de cero. */
    obd_predict_reset(&p);
    obd_predict_step(&p, 50.0f, 1000u, 1000u, 1000u, 600.0f, 0.0f);
    obd_predict_step(&p, 0.0f, 2000u, 2000u, 1000u, 600.0f, 0.0f);
    float low = obd_predict_step(&p, 0.0f, 2000u, 2500u, 1000u,
                                 600.0f, 0.0f);
    CHECK(low >= 0.0f);
}

/* El ralenti no mueve la aguja: por debajo de la banda muerta la velocidad
 * es cero y no hay adelanto. */
static void test_prediccion_ruido(void)
{
    obd_predict p;
    obd_predict_reset(&p);
    obd_predict_step(&p, 900.0f, 1000u, 1000u, 1000u, 600.0f, 25.0f);
    obd_predict_step(&p, 915.0f, 2000u, 2000u, 1000u, 600.0f, 25.0f);
    CHECK_NEAR(obd_predict_step(&p, 915.0f, 2000u, 2500u, 1000u,
                                600.0f, 25.0f), 915.0, 0.001);
}

void test_smooth(void)
{
    test_alpha_tau();
    test_alpha_extremos();
    test_primera_muestra();
    test_convergencia_con_tau();
    test_retraso_independiente_de_la_cadencia();
    test_salto_grande();
    test_prediccion();
    test_prediccion_ruido();
}
