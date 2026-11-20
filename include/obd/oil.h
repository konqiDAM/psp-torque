#ifndef OBD_OIL_H
#define OBD_OIL_H

/* Presion de aceite calculada.
 *
 * La MX-5 NB2 no tiene sensor de presion de aceite: el cuadro de fabrica
 * lleva un manometro "de mentira" que la ECU mueve por rpm. Esto replica
 * esa idea con la curva que pidio el dueno, y quien lo pinte tiene que
 * marcarlo como EST: es una estimacion, nunca una medida.
 *
 *   agua <= 80 C (frio):    3,5 bar al ralenti, 4,6 bar desde 2000 rpm
 *   agua >= 90 C (caliente): 1,0 bar al ralenti, 4,5 bar a 5000 rpm
 *
 * Lineal en medio y entre las dos curvas se mezcla de 80 a 90 C, para que
 * el numero no salte 2,5 bar justo en el umbral del termosato. */

/* bar para unas rpm y una temperatura de agua. water_known = 0 usa la
 * curva fria: es la lectura que no puede hacer pensar que el motor esta
 * mejor de lo que esta. */
float obd_oil_bar(float rpm, float water_c, int water_known);

#endif
