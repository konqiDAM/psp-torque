#include "obd/oil.h"
#include "test_util.h"

/* La presion de aceite de la MX-5 es calculada: la curva es lo unico que
 * hay, asi que se fija aqui con los numeros que pidio el dueno. */

static void test_oil_frio(void)
{
    /* Frio: 3,5 al ralenti y 4,6 desde 2000 vueltas. */
    CHECK_NEAR(obd_oil_bar(0.0f, 20.0f, 1), 3.5, 0.001);
    CHECK_NEAR(obd_oil_bar(800.0f, 20.0f, 1), 3.5, 0.001);
    CHECK_NEAR(obd_oil_bar(2000.0f, 20.0f, 1), 4.6, 0.001);
    CHECK_NEAR(obd_oil_bar(6500.0f, 20.0f, 1), 4.6, 0.001);
    /* Y en medio, interpolado. */
    CHECK_NEAR(obd_oil_bar(1400.0f, 20.0f, 1), 4.05, 0.02);
}

static void test_oil_caliente(void)
{
    /* Caliente: 1,0 al ralenti y 4,5 a 5000. */
    CHECK_NEAR(obd_oil_bar(800.0f, 95.0f, 1), 1.0, 0.001);
    CHECK_NEAR(obd_oil_bar(5000.0f, 95.0f, 1), 4.5, 0.001);
    CHECK_NEAR(obd_oil_bar(7000.0f, 95.0f, 1), 4.5, 0.001);
    /* 2000 vueltas en caliente: 1,0 + (1200/4200)*3,5 = 2,0. */
    CHECK_NEAR(obd_oil_bar(2000.0f, 95.0f, 1), 2.0, 0.02);
}

/* Entre 80 y 90 grados se mezclan las dos curvas: en el umbral exacto no
 * puede haber un salto de 2,5 bar. */
static void test_oil_transicion(void)
{
    float a = obd_oil_bar(2000.0f, 80.0f, 1);
    float b = obd_oil_bar(2000.0f, 85.0f, 1);
    float c = obd_oil_bar(2000.0f, 90.0f, 1);
    CHECK_NEAR(a, 4.6, 0.001);          /* frio */
    CHECK_NEAR(c, 2.0, 0.02);           /* caliente */
    CHECK(b < a && b > c);              /* y en medio, entre las dos */
    CHECK_NEAR(b, (a + c) / 2.0f, 0.02);
    /* Nunca salta: dos decimas de grado no mueven mas de 0,1 bar. */
    float d1 = obd_oil_bar(2000.0f, 79.9f, 1);
    float d2 = obd_oil_bar(2000.0f, 80.1f, 1);
    CHECK(d1 - d2 < 0.1f);
}

/* Sin agua todavia manda la curva fria, que es la lectura conservadora. */
static void test_oil_sin_agua(void)
{
    CHECK_NEAR(obd_oil_bar(800.0f, 0.0f, 0), 3.5, 0.001);
    CHECK_NEAR(obd_oil_bar(2000.0f, 0.0f, 0), 4.6, 0.001);
}

void test_oil(void)
{
    test_oil_frio();
    test_oil_caliente();
    test_oil_transicion();
    test_oil_sin_agua();
}
