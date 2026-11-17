# Compatibilidad OBD-II - Mazda MX-5 NB2

Segundo coche de referencia, despues del Alfa Romeo MiTo QV 2010. El MiTo es
CAN 11 bits y es el coche sobre el que se calibro todo; el MX-5 es **K-line
ISO 9141-2**, que es otro mundo en tiempos y en PIDs disponibles. Este
documento recoge lo medido, no lo esperado.

## 1. Vehiculo y enlace

| Componente | Detalle |
|---|---|
| Coche | Mazda MX-5 NB2 (segunda generacion, 2001-2005), gasolina atmosferico |
| Consola | PSP-2000/3000 con CFW ARK4 |
| Adaptador | ELM327 **v2.3** por WiFi (clon), red abierta `V-LINK` |
| Direccion | `192.168.0.10:35000` (la de fabrica de la app) |
| Alimentacion | `ATRV` 13.6-13.8 V con el contacto puesto |

Pendiente de rellenar por el propietario: version exacta, motor y VIN del
coche concreto. No se inventa.

## 2. Protocolo

Medido, no supuesto:

```
ATDPN  -> A3            (3 = ISO 9141-2)
ATDP   -> AUTO, ISO 9141-2
```

Y la cabecera real de la respuesta a `0100`, capturada con `ATH1`+`ATS1`:

```
48 6B 10 41 00 BE 1F B8 11 AA
```

Esa linea es la evidencia del fallo de descubrimiento de modulos en K-line:
la cabecera son **tres bytes** (`48 6B 10`), no un ID CAN de 3 o 8 digitos
hexadecimales, y `obd_disc_parse_line` no la reconoce. Ver ROADMAP, fase K4.

### 2.1 Tiempos del enlace

| Magnitud | MiTo (CAN) | MX-5 (ISO 9141-2) |
|---|---|---|
| Ida y vuelta de un PID | 50-67 ms | **383 ms** (500-516 tras una caida) |
| Busqueda de protocolo (`0100` tras `ATSP0`) | despreciable | **4315 ms** |
| Peticiones por segundo reales | 12-14 | **1,9** |
| Peticiones por segundo maximas | ~18 | **~2,6** (1/0,383) |

La busqueda de protocolo es la razon por la que la app no funcionaba: el
timeout de 3 s la abortaba 1,3 s antes de que el adaptador respondiera, y
dejaba al ELM327 atascado en `STOPPED` durante el resto de la sesion (876
respuestas `STOPPED` en el primer intento). Corregido en fase K1.

## 3. PIDs

Mascaras declaradas por la ECU:

```
0100 -> BE 1F B8 11
0120 -> 80 00 00 00
0140 -> no soportado
```

PIDs declarados: `01 03 04 05 06 07 0C 0D 0E 0F 10 11 13 14 15 1C 20 21`.

Resultado de la sonda `SONDA DEL COCHE` (18 PIDs probados uno a uno): **13
responden, 5 no existen**. Valores de la sesion `phase1_003` con el motor al
ralenti:

| PID | Dato | Estado | Valor medido |
|---|---|---|---|
| 0C | RPM | si | 801-820 |
| 0D | velocidad | si | 0 |
| 05 | refrigerante | si | 82-86 C |
| 0F | admision | si | 53 C |
| 10 | MAF | si | 2,20-2,28 g/s |
| 11 | mariposa | si | 0 % |
| 04 | carga | si | 18 % |
| 06 | STFT | si | +0,00 % |
| 07 | LTFT | si | +3,13 % |
| 0E | avance | si | +7,5 grados |
| 14 | O2 banco 1 sonda 1 | si | 0,11 V |
| 15 | O2 banco 1 sonda 2 | si | 0,11 V |
| 21 | distancia con MIL | si | 0 km |
| 0B | MAP | **no** | - |
| 33 | baro | **no** | - |
| 42 | tension de modulo | **no** | - |
| 5C | temperatura de aceite | **no** | - |
| 2F | nivel de combustible | **no** | - |

Ademas: `09` (informacion del vehiculo) **no esta declarado**, asi que el
VIN, CalID, CVN y nombre de ECU no se pueden leer por modo 09 en este coche.
Que la sonda los devuelva vacios es lo correcto, no un fallo.

Observacion a vigilar: 53 C de admision al ralenti es alto (calor acumulado
en el vano). Conviene comprobar que baja al circular antes de tratarlo como
medida rara.

## 4. Consecuencias de diseno

1. **BOOST no puede existir en este coche.** `BOOST = MAP - baro` y no hay
   ni `0B` ni `33`. El indicador central de la pantalla GENERAL esta
   condenado a `N/D`. Lo que si hay es MAF (g/s) y carga (%), que es donde
   debe ir el centro del panel en un atmosferico.
2. **El voltimetro tampoco.** Sin `42` no hay tension de bateria por OBD; la
   unica tension disponible es la del propio adaptador (`ATRV`), que no es
   la del coche.
3. **La tabla de periodos no sirve.** Esta calibrada para 12-14 req/s contra
   un enlace de 1,9 req/s: el planificador va saturado unas 7 veces y todos
   los PID se refrescan mucho mas tarde de lo pedido. Medido: RPM cada
   4,4 s (36 peticiones en 157 s) en vez de cada 200 ms.
4. **El sondeo debe priorizar poco y rapido.** Con 2,6 req/s de techo, cuatro
   valores vivos son un refresco cada 1,5 s. Todo lo demas (trims, sondas,
   avance) sobra en marcha.
5. **Las pantallas de MODULOS y SNIFFER no pueden funcionar aqui** tal como
   estan: el escaneo UDS es solo CAN y el parser de cabeceras no entiende
   `48 6B 10`.

## 5. Caidas del enlace

En `phase1_003` el enlace se cayo dos veces: en `t=157` y en `t=233`. En la
primera, el `rtt` subio de 383 a 500 ms, se acumularon 9 timeouts y la
reconexion automatica recupero al primer intento. En la segunda fallaron
los tres intentos y la sesion quedo `offline` hasta el final.

No se sabe aun si es el adaptador (pila TCP del clon v2.3), el ahorro de
energia del WiFi de la PSP o el coche. Ver PRUEBAS_PENDIENTES, T4.

## 6. Como repetir la sonda

1. Copiar `EBOOT.PBP` a `ms0:/PSP/GAME/HOMEBREW/psp-torque/` (o donde se
   tenga la app).
2. Adaptador encendido, contacto puesto, motor parado o al ralenti.
3. Lanzar la app y esperar a que conecte. La primera conexion puede tardar
   hasta 20 s: esta esperando a que el adaptador encuentre el protocolo.
4. `L` hasta la pestana `SONDA`, pulsar `X`. Tarda 30-60 s.
5. Salir con `START > SALIR > X` y leer
   `ms0:/PSP/SAVEDATA/PSPTORQUE/phase1_*.log`.

## 7. Limites del adaptador

- El ELM327 v2.3 es un clon. Su ida y vuelta de 383 ms es mas del triple de
  lo que da un ELM327 autentico en K-line (100-150 ms tipicos). Con otro
  adaptador el techo de 2,6 req/s subiria y el problema de cadencia se
  reduciria, pero no desaparece.
- No se puede pedir mas de un PID por comando en modo 01. No hay atajo.
- El coche solo habla ISO 9141-2: no hay CAN que escuchar.
