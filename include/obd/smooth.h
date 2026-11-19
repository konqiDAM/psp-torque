#ifndef OBD_SMOOTH_H
#define OBD_SMOOTH_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Suavizado exponencial con constante de tiempo fija.                */
/* ------------------------------------------------------------------ */

/* Constante de tiempo del filtro, en milisegundos.
 *
 * Es el retardo que introduce: el filtro alcanza el 63% del valor real
 * pasado un tau, sea cual sea la cadencia de muestreo.
 *
 * Con un factor de mezcla fijo (0,3) el retardo dependia de la
 * cadencia: 2,8 muestras hasta el 63%, es decir 2,1 s cuando el RPM
 * solo se refrescaba a 1,3 Hz. Ese era el retardo que se notaba en el
 * tacometro cuando se aceleraba. */
#define OBD_SMOOTH_TAU_MS 150.0f

typedef struct {
    float value;
    uint32_t last_ts;
    int initialized;
} obd_smooth;

/* Factor de mezcla para un intervalo dt entre dos muestras:
 * alpha = 1 - exp(-dt/tau). */
float obd_smooth_alpha(uint32_t dt_ms);

/* Reinicia el filtro: la siguiente muestra se toma tal cual. */
void obd_smooth_reset(obd_smooth *s);

/* Aplica el filtro a una muestra y devuelve el valor suavizado. */
float obd_smooth_apply(obd_smooth *s, float v, uint32_t ts);

/* ------------------------------------------------------------------ */
/* Prediccion de presentacion.                                        */
/* ------------------------------------------------------------------ */

/* La aguja no interpola hacia atras: se adelanta. Con la ultima medida y
 * la velocidad que traia, el valor pintado sigue la misma direccion hasta
 * que llega la muestra siguiente, y en ese instante vuelve a ser
 * exactamente el valor medido. Es lo que hace que un tacometro muestreado
 * a 1 Hz no parezca con un segundo de retraso al acelerar.
 *
 * Reglas:
 *   - En el instante de la muestra devuelve el valor medido, sin tocar.
 *   - Entre muestras extrapola con la velocidad estimada, con tope de
 *     `max_step` unidades y solo durante `span_ms`; despues se queda
 *     quieto en la ultima medida (no se inventa una rampa infinita).
 *   - Si el cambio entre muestras es menor que `deadband`, la velocidad se
 *     toma como cero: el ruido de un ralenti no mueve la aguja.
 *   - Con una sola muestra no hay velocidad y no se extrapola nada.
 */
typedef struct {
    float last;          /* ultima medida */
    float vel;           /* unidades por segundo */
    uint32_t last_ts;
    int samples;
} obd_predict;

void obd_predict_reset(obd_predict *p);

float obd_predict_step(obd_predict *p, float value, uint32_t sample_ts,
                       uint32_t now, uint32_t span_ms, float max_step,
                       float deadband);

#endif
