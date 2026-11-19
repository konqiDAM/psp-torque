#include "obd/profile.h"
#include "obd/vehicle.h"
#include "test_util.h"

/* FASE K2: el reparto por capacidad del enlace.
 *
 * Las cifras que se comprueban aqui no son inventadas: 1,879 req/s y
 * 532 ms de ciclo son las medidas de la MX-5 en phase1_003 (13 PID
 * soportados), y 16,5 y 14,0 son las del MiTo por CAN. La condicion que
 * mas importa es la de identidad: con capacidad de sobra el plan tiene que
 * devolver EXACTAMENTE la tabla base, porque el MiTo es el coche de
 * regresion. */

static void test_plan_class(void)
{
    CHECK_NEAR(obd_profile_link_class(OBD_PROTO_CAN), 16.5, 0.01);
    /* 2,5: la MX-5 entrego 2,20-2,40 sostenidos y el techo fisico con
     * 383 ms por peticion es 2,61. */
    CHECK_NEAR(obd_profile_link_class(OBD_PROTO_ISO9141), 2.5, 0.01);
    CHECK_NEAR(obd_profile_link_class(OBD_PROTO_KWP), 2.5, 0.01);
    CHECK_NEAR(obd_profile_link_class(OBD_PROTO_ANY), 2.5, 0.01);
}

/* Con capacidad de CAN el plan es la tabla base, PID a PID y pantalla a
 * pantalla. Si esto falla, el MiTo cambia de comportamiento. */
static void test_plan_can_identity(void)
{
    obd_link_plan plan;
    float demand = obd_profile_demand(0x01);   /* GENERAL */

    obd_plan_init(&plan, PROFILE_LINK_CLASS_CAN, 67, demand, 4);
    CHECK(plan.compressed == 0);
    CHECK_NEAR(plan.budget, 14.02, 0.05);

    const uint8_t pids[] = { 0x0C, 0x0B, 0x0D, 0x11, 0x04, 0x05, 0x0F,
                             0x06, 0x14, 0x21, 0x42, 0x5C, 0x33, 0x2F };
    for (int visible = 0; visible <= 1; visible++) {
        for (int i = 0; i < (int)(sizeof(pids) / sizeof(pids[0])); i++) {
            CHECK(obd_plan_period(&plan, pids[i], visible, 0, 0) ==
                  obd_profile_period(pids[i], visible));
        }
    }
}

/* La aritmetica de la MX-5 con las cifras medidas. */
static void test_plan_kline_arithmetic(void)
{
    obd_link_plan plan;
    float demand = obd_profile_demand(0x01);

    obd_plan_init(&plan, 1.879f, 532, demand, 4);
    CHECK(plan.compressed == 1);
    CHECK_NEAR(plan.budget, 1.597, 0.01);

    /* Tres valores reservan 0.6 Hz y el RESTANTE para el RPM sale por
     * debajo de su minimo (1.597 - 0.6 < 1.0), asi que el RPM se queda el
     * minimo: 1000/1.0 - 532 = 468 ms entre muestras. */
    uint16_t rpm = obd_plan_period(&plan, 0x0C, 1, 0, 3);
    CHECK(rpm == 468);

    /* Cuatro valores se reparten lo que deja el RPM en proporcion a su
     * suelo: (1.597 - 1.0) * 0.2/0.8 = 0.149 Hz, o sea 1000/0.149 - 532 =
     * 6170 ms. El margen cubre el redondeo. */
    uint16_t v = obd_plan_period(&plan, 0x0D, 1, 1, 4);
    CHECK(v > 6000 && v < 6300);

    /* El quinto valor ya no entra en el cupo (0.2 Hz de suelo) pero se
     * pinta: se queda con la mitad de peso, o sea unos 14,5 s, no con el
     * periodo de fondo. */
    uint16_t slow = obd_plan_period(&plan, 0x11, 1, 5, 5);
    CHECK(slow > 13000 && slow < 16000);

    /* Y lo que no se ve, tambien a fondo. */
    CHECK(obd_plan_period(&plan, 0x14, 0, 0, 3) == PROFILE_PLAN_BG_PERIOD_MS);

    /* Con la capacidad fisica (2,611) el reparto nuevo si cabe: tres
     * valores reservan 0.6 Hz y el RPM se queda 2.219 - 0.6 = 1.619 Hz,
     * o sea 1000/1.619 - 383 = 234,7 -> 235 ms entre muestras. */
    obd_plan_init(&plan, 2.611f, 383, demand, 4);
    uint16_t v2 = obd_plan_period(&plan, 0x0D, 1, 1, 3);
    CHECK(v2 < v);
    CHECK(obd_plan_period(&plan, 0x0C, 1, 0, 3) == 235);
    /* Y los valores, su suelo: (2.219 - 1.619) / 3 = 0.2 Hz -> 4617 ms. */
    CHECK(v2 > 4500 && v2 < 4700);

    /* Ningun periodo puede bajar del base ni pasar del tope. */
    for (int i = 0; i < 3; i++) {
        uint8_t pid = (i == 0) ? 0x0C : (i == 1 ? 0x0D : 0x05);
        uint16_t p = obd_plan_period(&plan, pid, 1, i + 1, 3);
        CHECK(p >= obd_profile_base_period(pid));
        CHECK(p <= PROFILE_PLAN_BG_PERIOD_MS);
    }
}

/* La condicion de K2 tal como estaba escrita -RPM a 1 Hz mas cuatro
 * valores a 0,5 Hz- son 3 req/s: este enlace no los tiene. El test lo deja
 * escrito para que nadie lo "arregle" subiendo la cifra sin medir. */
static void test_plan_target_is_bounded(void)
{
    const float needed = 1.0f + 4.0f * 0.5f;
    CHECK(needed > 1.879f);
    CHECK(needed > 1000.0f / 383.0f);

    /* Lo que si cabe: el plan comprimido no pide mas del presupuesto. */
    obd_link_plan plan;
    obd_plan_init(&plan, 2.5f, 532, obd_profile_demand(0x01), 4);
    float total = 0.0f;
    total += 1000.0f / (float)(obd_plan_period(&plan, 0x0C, 1, 0, 3) + 532);
    for (int r = 1; r <= 3; r++) {
        total += 1000.0f / (float)(obd_plan_period(&plan, 0x0D, 1, r, 3) + 532);
    }
    CHECK(total <= plan.budget + 0.01f);
    CHECK(total > 1.3f);   /* y que el reparto no se queda corto */
}

/* Ritmo real de un PID: el planificador cuenta el periodo desde el fin de
 * la respuesta, asi que una muestra tarda periodo + servicio. */
static float plan_rate(const obd_link_plan *plan, uint8_t pid, int visible,
                       int rank, int value_count)
{
    uint16_t p = obd_plan_period(plan, pid, visible, rank, value_count);
    return 1000.0f / (float)(p + plan->service_ms);
}

/* La condicion que de verdad importa con REPARTO: el plan no pide al
 * enlace mas de lo que tiene, contando el RPM, los valores del cupo y los
 * que se quedan fuera del cupo. Cuando los lentos se pagaban aparte esta
 * cuenta salia hasta un 70 % por encima del presupuesto.
 *
 * En el rango normal la condicion es exacta. Por debajo de ~2 req/s queda
 * el caso del suelo: el RPM se queda el presupuesto entero y los valores
 * visibles bajan al suelo absoluto de 10 s (o al tope de 60 s si les toca
 * una fraccion ridicula), asi que la suma se pasa un poco a proposito. */
static void test_plan_never_oversubscribes(void)
{
    for (int c = 12; c <= 25; c++) {
        float cap = (float)c / 10.0f;
        for (int vc = 1; vc <= 10; vc++) {
            for (int mv = 0; mv <= 4; mv++) {
                obd_link_plan plan;
                obd_plan_init(&plan, cap, 383, obd_profile_demand(0x01), mv);
                float total = plan_rate(&plan, 0x0C, 1, 0, vc);
                for (int r = 1; r <= vc; r++) {
                    total += plan_rate(&plan, 0x0D, 1, r, vc);
                }
                if (cap >= 2.0f) {
                    /* Exacto: el reparto pide justo el presupuesto. */
                    CHECK(total <= plan.budget + 0.02f);
                } else {
                    /* Suelo: el RPM se queda el presupuesto y cada valor
                     * visible que acaba en el tope de 60 s anade su
                     * 1/60,3 req/s. */
                    CHECK(total <= plan.budget + 0.02f +
                                   (float)vc * (1000.0f / 60383.0f));
                }
            }
        }
    }

    /* El caso del suelo, escrito: con el enlace en 0,8 req/s el RPM se
     * queda el presupuesto (0,68) y los valores visibles sobreviven a
     * 10 s. El planificador sirve al RPM primero, asi que el que cede es
     * el cuadro, no la aguja. */
    obd_link_plan floor;
    obd_plan_init(&floor, PROFILE_LINK_RATE_FLOOR, 383,
                  obd_profile_demand(0x01), 3);
    CHECK(obd_plan_period(&floor, 0x0D, 1, 1, 4) == PROFILE_PLAN_VALUE_SLOW_MS);
    CHECK(obd_plan_period(&floor, 0x0C, 1, 0, 4) >= 1000);
}

/* El estimador de capacidad: recorta cuando el enlace no llega, sube
 * despacio cuando sobra, y no oscila en el medio. */
static void test_capacity_estimator(void)
{
    float c = PROFILE_LINK_CLASS_KLINE;

    /* El plan pide el 85% de la capacidad, asi que entregar menos de eso
     * es lo normal y no un enlace caido: con 1,4 de 1,87 (75%) no se
     * recorta. Es el caso que hacia oscilar el plan en la MX-5. */
    c = obd_capacity_next(c, 1.4f, 1.87f, 2.2f);
    CHECK_NEAR(c, PROFILE_LINK_CLASS_KLINE, 0.01);

    /* Por debajo del 70% de lo pedido el enlace si falla: la capacidad es
     * lo entregado. */
    c = obd_capacity_next(c, 1.2f, 1.87f, 2.2f);
    CHECK_NEAR(c, 1.2, 0.01);

    /* Sostuvo lo pedido: sube un 10%, sin pasar del techo. */
    c = obd_capacity_next(1.4f, 1.4f, 1.4f, 2.2f);
    CHECK_NEAR(c, 1.54, 0.01);
    for (int i = 0; i < 20; i++) {
        c = obd_capacity_next(c, c, c, 2.2f);
    }
    CHECK(c <= 2.2f + 0.001f);

    /* Zona muerta: entrego entre el 70% y el 97% de lo pedido, ni sube ni
     * baja. */
    float before = 1.5f;
    CHECK_NEAR(obd_capacity_next(before, 1.35f, 1.5f, 2.2f), before, 0.001);
    CHECK_NEAR(obd_capacity_next(before, 1.10f, 1.5f, 2.2f), before, 0.001);

    /* Sin datos, se queda como esta. */
    CHECK_NEAR(obd_capacity_next(1.5f, 0.0f, 1.5f, 2.2f), 1.5f, 0.001);
    CHECK_NEAR(obd_capacity_next(1.5f, 1.5f, 0.0f, 2.2f), 1.5f, 0.001);

    /* Y nunca por debajo del suelo, ni por encima del techo. */
    CHECK(obd_capacity_next(1.5f, 0.1f, 10.0f, 2.2f) >= PROFILE_LINK_RATE_FLOOR);
    CHECK(obd_capacity_next(2.2f, 5.0f, 1.0f, 2.2f) <= 2.2f + 0.001f);
}

/* Idempotencia y degenerados: el editor de huecos llama al plan en cada
 * tecla, y un plan sin valores o con todos los PID retirados no puede
 * dividir por cero. */
static void test_plan_degenerate(void)
{
    obd_link_plan plan;
    obd_plan_init(&plan, 1.879f, 532, obd_profile_demand(0x01), 4);
    uint16_t a = obd_plan_period(&plan, 0x0D, 1, 1, 3);
    uint16_t b = obd_plan_period(&plan, 0x0D, 1, 1, 3);
    CHECK(a == b);

    /* Sin valores en pantalla: el RPM se queda todo el presupuesto y
     * ningun otro PID entra en el reparto. El resultado lo limita el
     * periodo base del RPM (200 ms), no el presupuesto: 1000/1.597 - 532
     * sale negativo. */
    CHECK(obd_plan_period(&plan, 0x0C, 1, 0, 0) == 200);
    /* Con value_count 0 no hay trozo que repartir y el PID no entra en el
     * reparto: periodo de fondo. */
    CHECK(obd_plan_period(&plan, 0x0D, 1, 1, 0) == PROFILE_PLAN_BG_PERIOD_MS);

    /* Un plan sin inicializar no puede devolver basura. */
    obd_link_plan zero;
    obd_plan_init(&zero, 0.0f, 0, 0.0f, 4);
    CHECK(zero.capacity >= PROFILE_LINK_RATE_FLOOR);
    CHECK(zero.compressed == 0);
    CHECK(obd_plan_period(&zero, 0x0D, 1, 1, 3) ==
          obd_profile_period(0x0D, 1));

    /* Y un PID que no esta en la tabla sigue devolviendo 0. */
    CHECK(obd_plan_period(&plan, 0x99, 1, 1, 3) == 0);
}

void test_plan(void)
{
    test_plan_class();
    test_plan_can_identity();
    test_plan_kline_arithmetic();
    test_plan_target_is_bounded();
    test_plan_never_oversubscribes();
    test_capacity_estimator();
    test_plan_degenerate();
}
