# Arquitectura

## 1. Principios

- **Read-only**: la app solo envía peticiones de lectura OBD-II (modo 01) y, más
  adelante, modo 22 documentado. Nunca envía modo 04 (borrar DTC), 2E/2F/31
  (escritura/control) ni comandos de actuadores.
- **Núcleo portable**: todo lo que no sea PSP (protocolo, PID, cálculo, stats,
  CSV) se compila y testea en PC.
- **Tiempo real razonable, no determinista**: el ELM327 es lento y con latencia
  variable. La UI muestra un dato con su timestamp; si está obsoleto, lo indica.
- **Sin datos falsos**: `N/D`, `—` o `STALE` son estados de primera clase. El
  modo simulador se marca `SIM` en pantalla y en el CSV.

## 2. Estructura de fuentes prevista

```
include/
  obd/
    pid.h              definiciones y fórmulas de PID estándar
    elm327.h           transporte + parser de líneas
    scheduler.h        planificación de sondeos
    samples.h          almacén de muestras con timestamps
    boost.h            MAP/BOOST/baro
    stats.h            mínimos, máximos, medias
    profile.h          perfil de vehículo
    csv_log.h          registro
src/
  core/                implementación portable (C99)
  psp/
    main.c             arranque, bucle principal, GU
    ui_qv.c            pantalla QV
    ui_engine.c        pantalla motor
    ui_telemetry.c     pantalla telemetría
    net_psp.c          sockets + sceNetApctl
    storage_psp.c      Memory Stick
    font/              fuente bitmap y assets
  host/
    sim_main.c         simulador de escritorio (opcional)
tools/
  elm327_sim.py        servidor ELM327 sintético para desarrollo
tests/
  test_pid.c
  test_parser.c
  test_boost.c
  test_stats.c
```

La estructura exacta puede ajustarse al implementar, pero la separación
`core` / `psp` / `host` no se rompe.

## 3. Flujo de datos

```
Scheduler ──► cmd_queue (SPSC) ──► Net thread
                                      │  write "010C\r"
                                      │  read líneas
                                      ▼
                                 ELM parser
                                      │  respuesta cruda
                                      ▼
                                 PID decoder
                                      │  Sample {pid, value, unit, ts, valid, source}
                                      ▼
                         sample_store (doble buffer)
                          │                    │
                          ▼                    ▼
                         UI                 CSV / stats
```

- El **scheduler** decide qué PID toca según el modo y su periodo.
- La **cola de comandos** es de productor único (UI/scheduler) y consumidor
  único (hilo de red). Tamaño fijo (p. ej. 32 entradas).
- El **hilo de red** es el único que escribe en el socket. Usa `select()` con
  timeout de 100 ms y socket no bloqueante para poder cerrar limpio.
- El **sample_store** publica cada muestra con `seq` (seqlock) o doble buffer
  por PID. La UI lee sin bloquear y sin ver un valor a medias.
- El **CSV** se escribe desde el hilo de red o desde un hilo de logging con cola
  propia; nunca desde el render.

## 4. Protocolo ELM327

### 4.1 Secuencia de inicialización

```
ATZ          reset (esperar "ELM327 v...")
ATE0         echo off
ATL0         linefeeds off
ATS0         spaces off (respuesta compacta)
ATH0         headers off (modo estándar)
ATSP0        protocolo automático
0100         comprobar enlace y máscara de PID soportados
```

Notas:

- `ATZ` puede tardar ~1 s. No hacerlo en cada reconexión agresiva.
- `ATSP0` deja que el adaptador elija; para diagnóstico se puede fijar
  `ATSP6` (CAN 11/500) y comprobar con `ATDPN`.
- `ATH1` solo se activa temporalmente para diagnóstico o para modo 22, porque
  añade bytes de cabecera y complica el parser.
- `ATST`/`ATAT` solo si hay problemas de timeout; no abusar.
- La app no debe asumir que el adaptador es un ELM327 genuino: muchos clones
  responden distinto. El parser tolera ruido, mayúsculas/minúsculas y líneas
  vacías.

### 4.2 Parser de líneas

Buffer de recepción de 256 bytes. Casos:

| Entrada | Acción |
|---|---|
| `OK`, `ELM327 v1.5`, `ATDPN`... | Respuesta de comando AT, no es dato. |
| `SEARCHING...` | Información; seguir leyendo. |
| `BUS INIT: OK` | Información; seguir leyendo. |
| `41 0C 1A F8` | Respuesta válida al PID 0x0C. |
| `NO DATA` | PID no soportado o ECU no responde. Marcar `N/D`. |
| `?` | Comando no reconocido. Error de adaptador. |
| `UNABLE TO CONNECT` | Sin enlace con el vehículo. Reconectar. |
| `STOPPED` / `ERROR` | Error; descartar comando y continuar. |
| Línea con `0:`, `1:`... | Tramas CAN/ISO-TP; el modo 01 normalmente no las usa con `ATH0`. |

Reglas:

- Eliminar `\r`, `\n` y espacios sobrantes.
- Ignorar el eco si por alguna razón `ATE0` no se aplicó.
- Validar que la respuesta empieza por `41` + PID solicitado antes de decodificar.
- Validar el número de bytes de datos esperado por PID. Si faltan, `N/D`.
- Guardar la respuesta cruda (hex) para diagnóstico y para el CSV de depuración.

### 4.3 Timeouts y reconexión

- Timeout de lectura por comando: 250–500 ms (configurable). En CAN suele
  bastar; en K-line puede necesitar más.
- Si hay 3 timeouts consecutivos: cerrar socket, volver a `CONNECTING`,
  reinit ELM327 y reintentar con backoff (1 s, 2 s, 5 s, 10 s).
- Si el adaptador responde `UNABLE TO CONNECT`, no insistir en bucle: mostrar
  `SIN CONTACTO` y permitir reintento manual.
- Al cambiar de modo (QV/ENGINE/TELEMETRY), no reiniciar el ELM327; solo
  cambiar la lista de PID sondeados.

## 5. Scheduler y presupuesto de tiempo

El ELM327 WiFi no es rápido: el cuello de botella real es el enlace serie
interno ELM327↔ECU y la ECU. No se puede pedir 20 PID a 20 Hz.

### 5.1 Lo que mide el enlace (FASE E)

Medido sobre un clon ELM327 v1.5 por WiFi en un viaje real (`phase1_067`,
log limpio de 124 s):

| Magnitud | Valor |
|---|---|
| Ida y vuelta de un comando `AT` (sin CAN) | 9–20 ms |
| Ida y vuelta de una petición de PID | 50–67 ms (modo ~61 ms) |
| Coste por ciclo completo (envío + respuesta + hueco) | ~72 ms |
| Peticiones por segundo sustainable | ~12,6 |

De esos ~72 ms, **~26 ms eran culpa nuestra** (FASE E1): la respuesta solo se
podía detectar en el siguiente frame (16,7 ms de cuantización) y el comando
siguiente salía en la iteración posterior (16,7 ms de hueco muerto). El suelo
del hardware son los ~46 ms restantes.

Con el sondeo desacoplado del render (E1) el ciclo baja a ~55 ms y el enlace
da ~18 peticiones/s.

### 5.2 La demanda tiene que caber en la capacidad

Si la suma de `1000/periodo` de todos los PID que la app puede sondear supera
la capacidad del enlace, **el planificador se satura y todos los PID se
refrescan más tarde de lo pedido**, aunque cada uno pida un periodo corto.
Antes de la FASE E la app pedía 48,9 req/s contra 12,6 disponibles: el RPM
pasaba de pedir 100 ms a refrescarse cada **765 ms**.

El presupuesto vive en `PROFILE_LINK_BUDGET_REQ_S` y hay un test del núcleo
(`test_link_budget`) que calcula la demanda en el caso peor y falla si se
supera. Es la red de seguridad de esta sección.

### 5.3 Planificación vigente

| Parámetro | PID | Periodo | Prioridad |
|---|---|---|---|
| RPM | 0x0C | 350 ms | alta |
| MAP | 0x0B | 350 ms | alta |
| Velocidad | 0x0D | 700 ms | media |
| Mariposa | 0x11 | 700 ms | media |
| Pedal | 0x49 / 0x4A | 700 ms | media |
| Carga | 0x04 | 1000 ms | media |
| ECT | 0x05 | 2000 ms | baja |
| IAT | 0x0F | 2000 ms | baja |
| STFT / LTFT / avance / O2 / distancia | 0x06, 0x07, 0x0E, 0x14, 0x15, 0x21 | 3000 ms | baja |
| Tensión | 0x42 | 5000 ms | baja (si existe) |
| Aceite | 0x5C | 5000 ms | baja (si existe) |
| Baro | 0x33 | 10000 ms | baja (si existe) |
| Combustible | 0x2F | 10000 ms | baja (si existe) |

`BOOST` no se pide: se calcula a partir de MAP y baro.

Estrategia:

- En cada tick, el scheduler encola como máximo un comando.
- Si hay un comando en vuelo, no se encola otro.
- Se salta PID cuyo periodo no ha vencido.
- Al entrar en ENGINE, se reparte el ancho de banda entre más PID (periodos
  mayores) y se da prioridad a MAP/boost.
- Si un PID devuelve `NO DATA` 3 veces seguidas, se marca no soportado y se
  deja de pedir hasta la próxima sesión (o reintento manual).
- Medir el tiempo real de ida y vuelta y adaptar los periodos si el bus va
  lento; no bloquear la UI.

### 5.4 El sondeo no depende del frame (FASE E1)

El bucle principal hace las dos cosas (dibujar y sondear). Antes de la FASE E
el sondeo solo avanzaba una vez por frame, lo que añadía ~26 ms a cada
petición. Ahora:

- Mientras hay una petición en vuelo (o acaba de completarse) la espera es
  de **2 ms** en vez de un VBlank completo (16,7 ms).
- La cadencia de **dibujo** no cambia: sigue governed por la cuenta de
  VBlanks, así que los FPS no se resienten.

### 5.5 El suavizado es un filtro de tiempo, no de muestras (FASE E3)

Un filtro exponencial con factor de mezcla fijo tiene un retardo que depende
de la cadencia de muestreo: con factor 0,3 y RPM a 1,3 Hz, el indicador
tardaba **2,1 s** en llegar al 63% del valor real. Ese era el retardo visible.

El filtro del núcleo (`obd_smooth`) usa constante de tiempo fija
(`tau = 150 ms`): `alpha = 1 - exp(-dt/tau)`. El retardo es siempre `tau`,
venga la cadencia como venga.

### 5.5b Interpolación de presentación (fase2.55)

El filtro anterior se aplica **al guardar la muestra**, así que entre dos
muestras el valor está quieto: en CAN no se nota (un PID vuelve cada 330 ms),
pero en el K-line de la MX-5 cada valor vuelve cada 1-10 s y la aguja daba un
salto y se quedaba parada. Eso es lo que parece "lento" aunque el dato sea
correcto.

`sample_value()` aplica ahora `obd_smooth_step()` en cada fotograma: el valor
pintado se mueve hacia la última medida con `tau = periodo/3` (120-800 ms), así
que el RPM llega en ~300 ms y el MAF cruza un escalón de 10 s en unos dos
segundos. Propiedades que mantiene:

- Solo se acerca a medidas que han llegado: no extrapola.
- La antigüedad y el `N/D` se miden con la marca de la muestra, no con esto.
- `sample_value_raw()` es la lectura exacta, y es la que usan el log de estado y
  los máximos: la evidencia no puede ser la animación.
- Se reinicia al conectar y al cambiar el ajuste, para no arrastrar medio valor.
- Se apaga con el ajuste `SUAVIZADO`.

### 5.6 Reparto del ancho de banda por pantalla (FASE F3)

El enlace da ~12-14 peticiones/s, el planificador tiene 19 PID y **no todos
se están mirando a la vez**. Los periodos base (tabla `g_base_periods` en
`src/core/profile.c`) son los que se usan cuando el PID está en la pantalla
activa; fuera de ella se multiplican por 4 con tope de 30 s.

| PID | base en pantalla | fuera de pantalla |
|---|---|---|
| 0x0C RPM | 200 ms (5 Hz) | 800 ms |
| 0x0B MAP | 500 ms | 2000 ms |
| 0x0D / 0x11 / 0x49 | 800-1000 ms | ×4 |
| 0x05 / 0x0F temperatura | 3000 ms | ×4 |
| 0x06 / 0x07 / 0x0E / 0x14 / 0x15 | 1500 ms | 6000 ms |
| 0x21 distancia MIL | 2000 ms | 8000 ms |
| 0x42 / 0x5C / 0x2F / 0x33 | 5-30 s | tope 30 s |

`apply_screen_schedule()` (capa PSP) recalcula los periodos al cambiar de
pantalla y deja pendientes de inmediato los PID que acaban de aparecer.

### 5.6b Enlace lento: reparto y estimación de capacidad (FASE K2, fase2.53)

Esa tabla está dimensionada para CAN. En K-line el techo real son **2,5
peticiones/s** (383 ms por petición medidas en la MX-5), así que quince PID no
caben: el plan se comprime y reparte el presupuesto por niveles — RPM, valores
visibles en la pantalla activa, y fondo.

Hay dos bucles de realimentación y conviene no confundirlos:

1. **Reparto** (`obd_plan_init` + `obd_plan_period`): divide el presupuesto
   entre lo que pide la pantalla activa. Se recalcula al cambiar de pantalla y
   cada 10 s.
2. **Capacidad** (`obd_capacity_next`): compara lo entregado con lo pedido y
   sube un 10 % por ventana si sobra, o recorta si falta. El recorte está en el
   70 % de lo pedido, no en el 85 %, porque el plan ya pide solo el 85 % de la
   capacidad: con el listón en el 85 % un enlace sano se recortaba en cada
   ventana y el plan oscilaba entre 2,20 y 1,40 req/s (visto en
   `phase1_008.log`).
Los huecos del GENERAL y las series de TELEMETRIA son configurables:
cualquier PID puede acabar en pantalla, así que se fuerza su periodo base
con un conjunto de bytes (`obd_forced_set`) que cubre todo el rango de
OBD-II: una máscara de 64 bits daría comportamiento indefinido con PIDs
como 0x5C (92).

Demanda por pantalla: GENERAL 13,53 · MOTOR 13,81 · DIAG 12,30 · TELEM
9,55 req/s, todas por debajo del presupuesto de 14.

### 5.7 Hueco mínimo entre comandos (FASE F1)

Medido sobre el clon ELM327 v1.5: si el siguiente comando llega apenas
termina la respuesta (hueco 0), el adaptador retrasa su respuesta un tick de
16,7 ms (el prompt pasa de 50 a 66,7 ms) y la capacidad cae de ~12 a 9,25
peticiones/s. **El hueco no es tiempo perdido: es un requisito del
adaptador.**

El ajuste `HUECO ELM` del menú (0/2/4/6/8/12/17 ms, defecto 8) permite
calibrar el umbral real de cada adaptador. El log registra `gap=` (hueco
medido) y `gapcfg=` (configurado) para que un solo viaje dé la curva.

## 6. Modelo de muestra

```c
typedef enum {
    SRC_REAL = 0,
    SRC_ESTIMATED,   /* p. ej. baro key-on, boost calculado */
    SRC_SIMULATED,   /* solo simulador */
    SRC_UNAVAILABLE
} sample_source;

typedef struct {
    uint16_t pid;
    float value;
    uint16_t unit;
    uint32_t timestamp_ms;   /* monotónico desde arranque */
    uint32_t seq;
    uint8_t valid;
    uint8_t source;
} sample_t;
```

- `valid=0` + `source=UNAVAILABLE` → la UI muestra `N/D`.
- Un dato con más de `2 × periodo` sin actualizarse se marca `STALE` en la UI.
- El boost calculado hereda `ESTIMATED` si el baro es estimado; si MAP es real
  y baro es real, el boost es derivado pero fiable (se indica `CALC`).

## 7. Boost y baro

```
BOOST_bar = (MAP_kPa − BARO_kPa) / 100
```

- `MAP` es absoluta (PID 0x0B, kPa).
- `BARO` en orden de preferencia:
  1. PID 0x33 (presión barométrica) si está soportado y el valor es plausible
     (50–110 kPa).
  2. Captura de MAP con el motor parado y el contacto puesto (key-on). Se
     guarda como baro de sesión y se marca `KEY-ON`.
  3. Valor configurable por el usuario (por defecto 1013 hPa, ajustable
     900–1050 hPa). Se marca `EST`.
- Si MAP < BARO, el resultado es negativo (vacío). **No se recorta a cero.**
- Si MAP no está disponible, BOOST no se inventa: `N/D`.

## 8. Persistencia

- Perfil compilado por defecto: `ALFA_MITO_QV_2010`.
- Configuración de usuario en `ms0:/PSP/GAME/PSPTorqueOBD/config.ini`
  (clave=valor, parser simple, sin dependencias).
- Logs en `ms0:/PSP/GAME/PSPTorqueOBD/logs/`.
- Escritura con `sceIoOpen` en modo append; `sceIoSync`/flush periódico
  (cada 1 s o 32 filas). Si la Memory Stick se extrae, cerrar y avisar; no
  bloquear la UI.
- Nunca guardar datos en la NAND flash interna de la PSP.

## 9. Memoria y rendimiento

- PSP-1000: 32 MB; PSP-2000/3000: 64 MB. Presupuesto de la app: < 8 MB.
- Sample store: ~32 PID × 32 B = 1 KB.
- Ring de gráficas: 600 muestras × 4 B × 8 series ≈ 19 KB.
- Fuente bitmap: p. ej. 96 glifos × 16×24 × 1 B ≈ 37 KB.
- Texturas: mantener pequeñas; el trébol puede ser vectorial o una textura
  de 128×128.
- Render a 60 fps solo si el coste lo permite; 30 fps es suficiente para un
  dashboard. El hilo de red no debe competir por CPU con el render.
- Evitar `malloc/free` y `printf` en el bucle de render.

## 10. Diagnóstico integrado

Pantalla o menú de diagnóstico (no visible en conducción) con:

- Estado del socket y de `sceNetApctl`.
- `ATI`, `ATDPN`, `ATRV` (tensión del adaptador).
- Máscaras de PID soportados (`0100`, `0120`, ...).
- Últimas respuestas crudas y contadores de timeout/`NO DATA`.
- Versión de firmware y perfil activo.

Esto es clave para distinguir “el coche no lo soporta” de “la app lo pide mal”.

## 11. Fuera de alcance (por ahora)

- Escritura en ECU, codificación, borrado de DTC.
- PID propietarios sin documentación validada.
- Conexión simultánea con Torque/AlfaOBD (el ELM327 WiFi suele admitir un
  cliente; además la ECU no multiplexa bien dos maestros de diagnóstico).
- Bluetooth o USB: la PSP no los soporta de forma práctica para este uso.
