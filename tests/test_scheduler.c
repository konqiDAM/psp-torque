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

/* Aplazar un PID que no esta en pantalla: su primer turno se reparte dentro
 * de su periodo en vez de salir todas las peticiones en el primer segundo.
 * Es lo que en phase1_012 gasto 19 de 151 peticiones en datos que nadie
 * veia. */
void test_sched_defer(void)
{
    const uint16_t periods[] = { 100, 2000, 60000 };
    const uint8_t pids[] = { 0x04, 0x0C, 0x21 };

    /* El turno nuevo cae en el segundo tramo del periodo. */
    for (unsigned p = 0; p < sizeof(periods) / sizeof(periods[0]); p++) {
        for (unsigned k = 0; k < sizeof(pids) / sizeof(pids[0]); k++) {
            obd_scheduler s;
            obd_sched_init(&s);
            int idx = obd_sched_add(&s, pids[k], periods[p], 2);
            obd_sched_begin(&s, idx, 0);
            obd_sched_end(&s, idx, 0, 1);
            s.items[idx].next_due_ms = 0;      /* vencido */

            obd_sched_defer(&s, idx, 1000u);
            uint32_t delta = s.items[idx].next_due_ms - 1000u;
            CHECK(delta >= periods[p] / 2u);
            CHECK(delta < periods[p]);
            CHECK(s.items[idx].period_ms == periods[p]);
            CHECK(s.items[idx].priority == 2);
        }
    }

    /* Un PID con turno en el futuro no se toca. */
    obd_scheduler s;
    obd_sched_init(&s);
    int a = obd_sched_add(&s, 0x0C, 1000, 0);
    s.items[a].next_due_ms = 6000u;
    obd_sched_defer(&s, a, 1000u);
    CHECK(s.items[a].next_due_ms == 6000u);

    /* Vencido justo ahora o hace rato: siempre hacia adelante. */
    s.items[a].next_due_ms = 1000u;
    obd_sched_defer(&s, a, 1000u);
    CHECK((int32_t)(s.items[a].next_due_ms - 1000u) > 0);
    s.items[a].next_due_ms = 1000u - 5000u;
    obd_sched_defer(&s, a, 1000u);
    CHECK((int32_t)(s.items[a].next_due_ms - 1000u) > 0);

    /* Los PID fuera de pantalla de la MX-5 no caen a la vez, y cada uno
     * queda dentro del segundo tramo de su periodo. */
    obd_scheduler seven;
    obd_sched_init(&seven);
    const uint8_t bg[] = { 0x04, 0x05, 0x06, 0x07, 0x11, 0x14, 0x15, 0x21 };
    for (unsigned i = 0; i < sizeof(bg) / sizeof(bg[0]); i++) {
        obd_sched_add(&seven, bg[i], 60000, 2);
    }
    for (int i = 0; i < seven.count; i++) {
        obd_sched_defer(&seven, i, 1000u);
    }
    for (int i = 0; i < seven.count; i++) {
        uint32_t delta = seven.items[i].next_due_ms - 1000u;
        CHECK(delta >= 30000u && delta < 60000u);
        for (int j = i + 1; j < seven.count; j++) {
            CHECK(seven.items[i].next_due_ms != seven.items[j].next_due_ms);
        }
    }

    /* Un PID sin confirmar (supported 0) si se aplaza: es el caso normal
     * al arrancar, cuando la mascara aun no ha dicho nada. */
    obd_scheduler unknown;
    obd_sched_init(&unknown);
    int u = obd_sched_add(&unknown, 0x33, 60000, 2);
    CHECK(unknown.items[u].supported == 0);
    obd_sched_defer(&unknown, u, 1000u);
    CHECK(unknown.items[u].next_due_ms > 1000u);

    /* El estado del PID no cambia, y el que esta en vuelo tampoco. */
    obd_scheduler st;
    obd_sched_init(&st);
    int b = obd_sched_add(&st, 0x0F, 5000, 1);
    st.items[b].no_data_count = 2;
    st.items[b].supported = 1;
    obd_sched_begin(&st, b, 100u);
    obd_sched_defer(&st, b, 1000u);
    CHECK(st.in_flight == b);
    CHECK(st.items[b].no_data_count == 2);
    CHECK(st.items[b].supported == 1);

    /* Con el PID en vuelo no se aplaza nada. */
    uint32_t before = st.items[b].next_due_ms;
    CHECK(before == 0);
    obd_sched_defer(&st, b, 1000u);
    CHECK(st.items[b].next_due_ms == 0);

    /* Cerrar la peticion manda: end reprograma sobre lo aplazado. */
    obd_sched_end(&st, b, 2000u, 1);
    CHECK(st.items[b].next_due_ms == 7000u);

    /* Un PID no soportado no se toca. */
    obd_scheduler park;
    obd_sched_init(&park);
    int c = obd_sched_add(&park, 0x14, 60000, 2);
    park.items[c].supported = 2;
    obd_sched_defer(&park, c, 1000u);
    CHECK(park.items[c].next_due_ms == 0);

    /* Periodo minimo: no hay tramo donde repartir y no puede dividir por
     * cero. Se toca el campo a mano porque add/set_period lo fuerzan. */
    obd_scheduler tiny;
    obd_sched_init(&tiny);
    int d = obd_sched_add(&tiny, 0x0D, 100, 1);
    tiny.items[d].period_ms = 1;
    obd_sched_defer(&tiny, d, 1000u);
    CHECK(tiny.items[d].next_due_ms == 0);

    /* Indices fuera de la tabla. */
    obd_sched_defer(&tiny, -1, 1000u);
    obd_sched_defer(&tiny, tiny.count, 1000u);

    /* El reloj de 32 bits da la vuelta: se compara con signo. */
    obd_scheduler wrap;
    obd_sched_init(&wrap);
    int e = obd_sched_add(&wrap, 0x0C, 60000, 0);
    wrap.items[e].next_due_ms = 0xFFFFFFF0u;
    obd_sched_defer(&wrap, e, 0xFFFFFFF0u);
    /* El hueco se mide hacia adelante y sigue cayendo en el segundo tramo. */
    uint32_t fwd = wrap.items[e].next_due_ms - 0xFFFFFFF0u;
    CHECK(fwd >= 30000u && fwd < 60000u);
    /* Y desde el reloj actual todavia no toca (comparacion con signo). */
    CHECK((int32_t)(0xFFFFFFF0u - wrap.items[e].next_due_ms) < 0);
}
