#include "obd/boost.h"

#include <string.h>

static int plausible_baro(float kpa)
{
    return kpa >= 50.0f && kpa <= 110.0f;
}

void boost_init(boost_state *state, float fallback_kpa)
{
    memset(state, 0, sizeof(*state));

    if (!plausible_baro(fallback_kpa)) {
        fallback_kpa = 101.3f;
    }
    state->baro_valid = 1;
    state->baro_kpa = fallback_kpa;
    state->baro_source = BARO_CONFIG;
    state->boost_valid = 0;
}

void boost_set_map(boost_state *state, float kpa, uint32_t ts_ms)
{
    /* El MAP absoluto no puede estar fuera de la atmosfera: un byte
     * corrupto en plena aceleracion pintaba 1,5 bar como lectura valida y
     * ademas se aprendia como pico. */
    if (kpa < 20.0f || kpa > 350.0f) {
        return;
    }
    state->map_valid = 1;
    state->map_kpa = kpa;
    state->map_ts_ms = ts_ms;
    boost_recompute(state);
}

void boost_set_baro_pid(boost_state *state, float kpa, uint32_t ts_ms)
{
    if (!plausible_baro(kpa)) {
        return;
    }
    state->baro_valid = 1;
    state->baro_kpa = kpa;
    state->baro_source = BARO_PID;
    state->baro_ts_ms = ts_ms;
    boost_recompute(state);
}

int boost_capture_key_on(boost_state *state,
                         float map_kpa,
                         int rpm_valid, float rpm,
                         int speed_valid, float speed_kph,
                         uint32_t ts_ms, uint32_t session_ms)
{
    if (!plausible_baro(map_kpa)) {
        return 0;
    }
    if (state->baro_source == BARO_PID ||
        state->baro_source == BARO_KEY_ON) {
        return 0;
    }
    if (!rpm_valid || rpm >= 50.0f) {
        return 0;
    }
    if (speed_valid && speed_kph >= 1.0f) {
        return 0;
    }
    /* Solo al principio de la sesion, antes de que el motor arranque. */
    if (session_ms > 60000u) {
        return 0;
    }

    state->baro_valid = 1;
    state->baro_kpa = map_kpa;
    state->baro_source = BARO_KEY_ON;
    state->baro_ts_ms = ts_ms;
    boost_recompute(state);
    return 1;
}

void boost_recompute(boost_state *state)
{
    if (state->map_valid && state->baro_valid) {
        state->boost_bar = (state->map_kpa - state->baro_kpa) / 100.0f;
        state->boost_valid = 1;
        state->boost_is_estimate = (state->baro_source == BARO_CONFIG);
    } else {
        state->boost_bar = 0.0f;
        state->boost_valid = 0;
        state->boost_is_estimate = 0;
    }
}
