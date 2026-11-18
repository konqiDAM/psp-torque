#include "obd/scheduler.h"
#include "obd/pid.h"
#include "test_util.h"

void test_scheduler(void)
{
    obd_scheduler sched;

    obd_sched_init(&sched);
    CHECK(sched.count == 0);
    CHECK(sched.in_flight == -1);

    int rpm = obd_sched_add(&sched, 0x0C, 100, 0);
    int map = obd_sched_add(&sched, 0x0B, 100, 0);
    int speed = obd_sched_add(&sched, 0x0D, 200, 1);
    CHECK(rpm == 0);
    CHECK(map == 1);
    CHECK(speed == 2);

    /* Empate de prioridad: el primero registrado. */
    CHECK(obd_sched_next(&sched, 0) == 0);

    /* Mientras hay uno en vuelo no se encola otro. */
    obd_sched_begin(&sched, 0, 0);
    CHECK(obd_sched_next(&sched, 0) == -1);
    obd_sched_end(&sched, 0, 50, 1);
    CHECK(sched.in_flight == -1);
    CHECK(sched.items[0].next_due_ms == 150);

    /* A los 100 ms el PID 0 sigue sin vencer; toca MAP. */
    CHECK(obd_sched_next(&sched, 100) == 1);
    obd_sched_begin(&sched, 1, 100);
    obd_sched_end(&sched, 1, 100, 1);
    CHECK(sched.items[1].next_due_ms == 200);

    /* A los 150 ms: RPM vence, pero velocidad aun no se ha sondeado y su
     * deadline (0) es mas antigua, asi que EDF la atiende primero. */
    CHECK(obd_sched_next(&sched, 150) == 2);
    obd_sched_begin(&sched, 2, 150);
    obd_sched_end(&sched, 2, 150, 1);
    CHECK(obd_sched_next(&sched, 150) == 0);
    obd_sched_begin(&sched, 0, 150);
    obd_sched_end(&sched, 0, 150, 1);

    /* A los 200 ms solo MAP esta vencido. */
    CHECK(obd_sched_next(&sched, 200) == 1);
    obd_sched_begin(&sched, 1, 200);
    obd_sched_end(&sched, 1, 200, 1);

    /* Tres NO DATA seguidos desactivan el PID. */
    obd_scheduler nodata;
    obd_sched_init(&nodata);
    CHECK(obd_sched_add(&nodata, 0x33, 5000, 3) == 0);

    uint32_t now = 0;
    for (int i = 0; i < 2; i++) {
        CHECK(obd_sched_next(&nodata, now) == 0);
        obd_sched_begin(&nodata, 0, now);
        obd_sched_end(&nodata, 0, now, 0);
        now += 5000;
    }
    CHECK(nodata.items[0].supported == 0);
    CHECK(nodata.items[0].no_data_count == 2);

    CHECK(obd_sched_next(&nodata, now) == 0);
    obd_sched_begin(&nodata, 0, now);
    obd_sched_end(&nodata, 0, now, 0);
    CHECK(nodata.items[0].supported == 2);
    CHECK(obd_sched_next(&nodata, now + 5000) == -1);

    /* Volver a habilitarlo reinicia el contador. */
    obd_sched_set_supported(&nodata, 0, 1);
    CHECK(nodata.items[0].supported == 1);
    CHECK(nodata.items[0].no_data_count == 0);
    CHECK(obd_sched_next(&nodata, now + 5000) == 0);

    /* Mascara real del MiTo sobre una lista QV. */
    obd_scheduler masked;
    obd_sched_init(&masked);
    obd_sched_add(&masked, 0x0B, 100, 0);
    obd_sched_add(&masked, 0x0C, 100, 0);
    obd_sched_add(&masked, 0x0D, 200, 1);
    obd_sched_add(&masked, 0x10, 250, 1);
    obd_sched_add(&masked, 0x1F, 1000, 2);

    const uint8_t mask_bytes[] = { 0xBE, 0x3E, 0xB8, 0x11 };
    uint32_t mask = obd_pid_mask(mask_bytes, 4);
    CHECK(obd_sched_apply_mask(&masked, mask, 0x01) == 5);
    CHECK(masked.items[0].supported == 1);
    CHECK(masked.items[1].supported == 1);
    CHECK(masked.items[2].supported == 1);
    CHECK(masked.items[3].supported == 2);
    CHECK(masked.items[4].supported == 2);
    CHECK(obd_sched_next(&masked, 0) == 0);

    /* Bloque 0x21-0x40: solo el primer bit. */
    obd_scheduler block2;
    obd_sched_init(&block2);
    obd_sched_add(&block2, 0x21, 1000, 2);
    obd_sched_add(&block2, 0x2F, 5000, 3);
    obd_sched_add(&block2, 0x33, 5000, 3);
    CHECK(obd_sched_apply_mask(&block2, 0x80000000u, 0x21) == 3);
    CHECK(block2.items[0].supported == 1);
    CHECK(block2.items[1].supported == 2);
    CHECK(block2.items[2].supported == 2);

    /* Reagendar. */
    obd_sched_reset_due(&block2, 1234);
    CHECK(block2.items[0].next_due_ms == 1234);
    CHECK(block2.items[1].next_due_ms == 0);
    CHECK(block2.in_flight == -1);

    /* Limite de la tabla. */
    obd_scheduler full;
    obd_sched_init(&full);
    for (int i = 0; i < OBD_SCHED_MAX; i++) {
        CHECK(obd_sched_add(&full, (uint8_t)(0x20 + i), 1000, 3) == i);
    }
    CHECK(obd_sched_add(&full, 0x0C, 100, 0) == -1);
}

/* FASE F3: cambiar el periodo de un PID en caliente, para repartir el
 * ancho de banda segun la pantalla activa. */
void test_set_period(void)
{
    obd_scheduler s;
    obd_sched_init(&s);
    obd_sched_add(&s, 0x0C, 200, 0);

    /* Un PID recien anadido ya esta pendiente. */
    CHECK(obd_sched_next(&s, 0u) == 0);

    obd_sched_begin(&s, 0, 100u);
    obd_sched_end(&s, 0, 100u, 1);       /* next_due = 100 + 200 = 300 */
    CHECK(s.items[0].next_due_ms == 300);

    /* El periodo nuevo se aplica al siguiente ciclo; el pendiente
     * actual no se toca (no se cancela una peticion ya vencida). */
    obd_sched_set_period(&s, 0, 800, 0);
    CHECK(s.items[0].period_ms == 800);
    CHECK(obd_sched_next(&s, 300u) == 0);   /* sigue pendiente (vencia en 300) */

    obd_sched_begin(&s, 0, 300u);
    obd_sched_end(&s, 0, 300u, 1);          /* next_due = 300 + 800 = 1100 */
    CHECK(obd_sched_next(&s, 500u) < 0);    /* ya no toca hasta 1100 */
    CHECK(obd_sched_next(&s, 1100u) == 0);

    /* Con due_now el PID pasa a estar pendiente de inmediato: es lo
     * que hace que un dato que acaba de aparecer se pida al instante. */
    obd_sched_set_period(&s, 0, 800, 1);
    CHECK(obd_sched_next(&s, 500u) == 0);

    /* Reprogramar con una peticion EN VUELO no deja el planificador
     * colgado: cuando la peticion en vuelo termina, sigue sirviendo. */
    obd_sched_begin(&s, 0, 1000u);
    obd_sched_set_period(&s, 0, 300, 1);
    CHECK(obd_sched_next(&s, 1000u) < 0);   /* hay una en vuelo */
    obd_sched_end(&s, 0, 1000u, 1);        /* next_due = 1300 */
    CHECK(obd_sched_next(&s, 1000u) < 0);   /* todavia no vence */
    CHECK(obd_sched_next(&s, 1300u) == 0);  /* sigue sirviendo */

    /* Un periodo de 0 no vale: se usa el minimo. */
    obd_sched_set_period(&s, 0, 0, 0);
    CHECK(s.items[0].period_ms == 100);

    /* Un indice fuera de rango no hace nada. */
    obd_sched_set_period(&s, 99, 500, 1);
    obd_sched_set_period(&s, -1, 500, 1);
    CHECK(s.items[0].period_ms == 100);
}

/* FASE K2: con el enlace comprimido manda la prioridad. Es lo que hace
 * que el RPM no se quede detras de los PIDs de periodo largo, que siempre
 * estan vencidos. Sin comprimir, EDF como siempre. */
void test_scheduler_strict(void)
{
    obd_scheduler sched;

    obd_sched_init(&sched);
    /* RPM con periodo largo y prioridad 0; un PID lento con periodo corto
     * y prioridad baja. Asi los dos acaban vencidos y el mas antiguo es el
     * de prioridad baja: es exactamente el caso que rompia el RPM. */
    int rpm = obd_sched_add(&sched, 0x0C, 400, 0);
    int slow = obd_sched_add(&sched, 0x14, 100, 2);
    CHECK(rpm == 0);
    CHECK(slow == 1);

    obd_sched_begin(&sched, rpm, 0);
    obd_sched_end(&sched, rpm, 0, 1);
    CHECK(sched.items[rpm].next_due_ms == 400);
    obd_sched_begin(&sched, slow, 0);
    obd_sched_end(&sched, slow, 0, 1);
    CHECK(sched.items[slow].next_due_ms == 100);

    /* A los 450 los dos estan vencidos y el lento tiene el plazo mas
     * antiguo: EDF lo sirve a el. */
    CHECK(obd_sched_next(&sched, 450) == slow);

    /* Con el enlace comprimido manda la prioridad: primero el RPM. */
    obd_sched_set_strict(&sched, 1);
    CHECK(sched.strict == 1);
    CHECK(obd_sched_next(&sched, 450) == rpm);

    /* Y en cuanto se sirve, el lento no se queda sin turno. */
    obd_sched_begin(&sched, rpm, 450);
    obd_sched_end(&sched, rpm, 450, 1);
    CHECK(obd_sched_next(&sched, 450) == slow);

    /* Aflojar el enlace devuelve el comportamiento de siempre. */
    obd_sched_set_strict(&sched, 0);
    CHECK(sched.strict == 0);

    /* Cambiar una prioridad no toca periodo ni vencimiento. */
    uint16_t period = sched.items[slow].period_ms;
    uint32_t due = sched.items[slow].next_due_ms;
    obd_sched_set_priority(&sched, slow, 1);
    CHECK(sched.items[slow].priority == 1);
    CHECK(sched.items[slow].period_ms == period);
    CHECK(sched.items[slow].next_due_ms == due);

    /* Y un indice fuera de rango se ignora. */
    obd_sched_set_priority(&sched, 99, 3);
    obd_sched_set_strict(&sched, 1);
}
