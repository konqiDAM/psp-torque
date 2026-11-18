#include "obd/profile.h"
#include "obd/samples.h"
#include "test_util.h"

#include <stdio.h>
#include <string.h>

void test_parser(void);
void test_pid(void);
void test_boost(void);
void test_scheduler(void);
void test_samples(void);
void test_discovery(void);
void test_smooth(void);
void test_set_period(void);
void test_forced_set(void);
void test_vehicle(void);
void test_scheduler_strict(void);
void test_plan(void);
void test_dtc(void);
void test_splash_mx5(void);

int g_failures = 0;

static void test_profile(void)
{
    CHECK(PROFILE_QV_POLL_COUNT >= 5);
    CHECK(PROFILE_ENGINE_POLL_COUNT >= 10);
    CHECK(PROFILE_QV_POLL[0].pid == 0x0C);
    CHECK(PROFILE_QV_POLL[1].pid == 0x0B);
    CHECK(PROFILE_QV_POLL[0].period_ms == 250);
    CHECK(PROFILE_QV_POLL[1].period_ms == 500);
    CHECK(PROFILE_BARO_FALLBACK_KPA > 50.0f);
    CHECK(PROFILE_BARO_FALLBACK_KPA < 110.0f);
}

/* FASE E2: el presupuesto de peticiones del planificador.
 *
 * El enlace ELM327 WiFi da ~12 peticiones/s con el hueco minimo de 8 ms
 * (unos 83 ms por peticion completa: envio, respuesta del coche y hueco
 * entre peticiones). Si la suma de 1000/periodo de todos los PID que la
 * app puede llegar a sondear supera ese techo, el planificador se
 * satura y TODOS los PID se refrescan mas tarde de lo pedido.
 *
 * Con el reparto por pantalla (FASE F3) el presupuesto se comprueba
 * pantalla a pantalla: es `test_demand_by_screen` quien lo vigila. */

/* La edad global del panel es la de la muestra mas reciente de
 * CUALQUIER PID: si el RPM se retrasa pero el resto fluye, el panel
 * esta vivo y no debe avisar de datos viejos. */
static void test_data_age(void)
{
    obd_sample_store st;
    obd_store_init(&st);

    /* Sin muestras: edad desconocida. */
    CHECK(obd_store_newest_ts(&st) == 0u);

    obd_store_set(&st, 0x0C, 1000.0f, OBD_UNIT_RPM, 1000u, OBD_SRC_REAL);
    obd_store_set(&st, 0x0B, 30.0f, OBD_UNIT_KPA, 1500u, OBD_SRC_REAL);

    /* La mas reciente manda, no la del RPM. */
    CHECK(obd_store_newest_ts(&st) == 1500u);

    /* Una muestra invalidada no cuenta. */
    obd_store_invalidate(&st, 0x0B, 1600u);
    CHECK(obd_store_newest_ts(&st) == 1000u);
}

/* FASE F3: los periodos base son los de "el PID esta en pantalla". Fuera
 * de pantalla se multiplican por 4 (tope 30 s) para no gastar ancho de
 * banda en lo que no se mira. */
static void test_profile_periods(void)
{
    /* El RPM es el dato que se mira en el tacometro: 200 ms (5 Hz). */
    CHECK(obd_profile_base_period(0x0C) == 200);
    CHECK(obd_profile_period(0x0C, 1) == 200);
    CHECK(obd_profile_period(0x0C, 0) == 800);

    /* MAP: 500 ms en pantalla, 2000 fuera. */
    CHECK(obd_profile_base_period(0x0B) == 500);
    CHECK(obd_profile_period(0x0B, 0) == 2000);

    /* Tope de 30 s para los muy lentos. */
    CHECK(obd_profile_period(0x33, 0) == 30000);

    /* Los PID de diagnostico en pantalla son rapidos (1,5 s): compiten por
     * el enlace solo cuando no se estan viendo. */
    CHECK(obd_profile_base_period(0x06) == 1500);
    CHECK(obd_profile_base_period(0x21) == 2000);

    /* PID desconocido: 0. */
    CHECK(obd_profile_base_period(0x77) == 0);
    CHECK(!obd_profile_knows_pid(0x77));
}

/* La demanda de cada pantalla tiene que caber en la capacidad del
 * enlace. El reparto por pantalla es lo que permite subir el RPM a
 * 5 Hz sin que el resto se quede sin ancho de banda. */
static void test_demand_by_screen(void)
{
    float d_general = obd_profile_demand(0x01);
    float d_motor   = obd_profile_demand(0x02);
    float d_diag    = obd_profile_demand(0x04);
    float d_telem   = obd_profile_demand(0x08);

    printf("  demanda: general %.2f  motor %.2f  diag %.2f  telem %.2f"
           "  (todo %.2f) req/s\n",
           d_general, d_motor, d_diag, d_telem, obd_profile_demand_all());

    CHECK(d_general <= PROFILE_LINK_BUDGET_REQ_S);
    CHECK(d_motor   <= PROFILE_LINK_BUDGET_REQ_S);
    CHECK(d_diag    <= PROFILE_LINK_BUDGET_REQ_S);
    CHECK(d_telem   <= PROFILE_LINK_BUDGET_REQ_S);

    /* Una pantalla sin PIDs propios (MODULOS, SNIFFER, INFO) no puede
     * quedarse sin nada que pedir: cuenta con el conjunto base. */
    CHECK(obd_profile_demand(0x10) > 0.0f);
    CHECK(obd_profile_demand(0x10) <= PROFILE_LINK_BUDGET_REQ_S);

    /* El reparto es lo que lo hace caber: cada pantalla pide menos que
     * pedirlo todo a la vez. */
    CHECK(d_general < obd_profile_demand_all());

    /* El RPM tiene que ser el mas rapido de todos. */
    CHECK(obd_profile_base_period(0x0C) <=
          obd_profile_base_period(0x0B));
}

int main(void)
{
    test_parser();
    test_pid();
    test_boost();
    test_scheduler();
    test_scheduler_strict();
    test_samples();
    test_discovery();
    test_profile();
    test_profile_periods();
    test_data_age();
    test_demand_by_screen();
    test_smooth();
    test_set_period();
    test_forced_set();
    test_vehicle();
    test_plan();
    test_dtc();
    test_splash_mx5();

    if (g_failures != 0) {
        printf("%d comprobaciones fallidas\n", g_failures);
        return 1;
    }

    printf("OK: todas las pruebas pasan\n");
    return 0;
}
