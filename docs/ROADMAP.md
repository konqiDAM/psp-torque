# Roadmap y criterios de aceptación

## Fase 0 — Viabilidad técnica (bloqueante)

Objetivo: demostrar que la PSP puede hablar con el ELM327 WiFi y que el
toolchain compila un EBOOT.

Tareas:

1. Instalar `pspdev` (Docker o nativo) y compilar un “hello world” para PSP.
2. Probar en la PSP real: arranque, pantalla 480×272, lectura de botones,
   acceso a Memory Stick.
3. Configurar la red WiFi del adaptador y comprobar el modo de seguridad.
4. Implementar un cliente TCP mínimo en PSP (`sceNetApctl` + `sceNetInet`) y
   conectarlo al simulador ELM327 del PC.
5. Enviar `ATI`, `ATE0`, `ATSP0`, `0100`, `010C` y mostrar la respuesta cruda
   en pantalla.

Criterios de aceptación:

- [x] `EBOOT.PBP` arranca en la PSP objetivo.
- [x] La PSP obtiene IP (DHCP o estática) y abre TCP contra `192.168.0.10:35000`
      (o contra el simulador).
- [x] Se ve en pantalla una respuesta real de `0100` y `010C`.
- [x] Se puede desconectar y reconectar sin reiniciar la PSP.
- [x] Documentado el modo WiFi real del adaptador y el modelo/CFW de la PSP.

Resultado (05/11/2026): cerrada. PSP-2000/3000 con ARK4, red WiFi propia
abierta, IP `192.168.0.11`, `ATI`=`ELM327 v1.5`, `0100`=`4100BE3EB811`,
`RESULTADO: OK`. Ver [`FASE0.md`](FASE0.md).

Riesgos: WPA2 no soportado, adaptador sin DHCP, AP que no acepta el cliente PSP,
toolchain no disponible.

## Fase 1 — Núcleo OBD + datos en vivo

Estado: **cerrada**. Núcleo y app de texto implementados y validados en coche;
el dashboard grafico de la fase 2 sustituyo a la pantalla de texto.

Tareas:

1. [x] Núcleo portable: parser ELM327, PID estándar, muestras, scheduler,
   boost/baro.
2. [x] Tests de unidad del núcleo en PC (fórmulas, respuestas válidas, `NO DATA`,
   timeouts, líneas con ruido).
3. [x] Perfil `ALFA_MITO_QV_2010` compilado.
4. [x] UI QV gráfica (tacómetro, velocidad, boost, ECT, IAT, MAP, conexión).
5. [x] Animación de arranque del trébol.
6. [x] Simulador ELM327 con datos sintéticos marcados `SIM`.

Criterios de aceptación:

- [x] En el coche, RPM, velocidad, MAP, ECT e IAT se actualizan a una cadencia
      útil (objetivo ≥ 2 Hz para RPM/MAP; ≥ 1 Hz para temperaturas).
- [x] BOOST se calcula correctamente y muestra signo; `EST` visible si el baro
      es estimado.
- [x] Un PID no soportado muestra `N/D`, nunca `0` (garantizado por el núcleo).
- [x] La UI sigue respondiendo si se desenchufa el adaptador.
- [x] Reconexión automática con backoff.
- [x] Sin escrituras a la ECU.

La app de texto de esta fase y el detalle del núcleo están en
[`FASE1.md`](FASE1.md).

## Fase 2 — ENGINE MODE + TELEMETRÍA

Estado: **cerrada**. Dashboard grafico completo, con las siete pantallas, diez
temas, panel de telemetria editable y descubrimiento de centralitas. La
exportacion en formato CSV largo queda pendiente (el log de sesion actual es
un log de texto con una linea por muestreo).

Tareas:

1. Segunda pantalla de motor con presiones, temperaturas, aire, combustible y
   encendido.
2. STFT/LTFT, lambda/O2 solo si están disponibles.
3. Estadísticas de sesión (mín/máx/media/muestras).
4. CSV long-format con flush periódico y cierre seguro.
5. Gráficas con ring buffer y huecos.
6. Diagnóstico integrado (máscaras de PID, `ATDPN`, `ATRV`, respuestas crudas).

Criterios de aceptación:

- [ ] CSV abrible en Excel/LibreOffice, con timestamps y sin filas corruptas
      tras apagar la PSP. *Pendiente: hoy el log es texto plano.*
- [x] Las estadísticas coinciden con los valores vistos en pantalla.
- [x] Las gráficas no conectan huecos largos.
- [x] El diagnóstico distingue “PID no soportado” de “timeout”.
- [x] La sesión completa no supera el presupuesto de memoria.

## Fase 2.5 — Robustez de la capa OBD (FASE D)

Objetivo: que la sesion sobreviva a un adaptador que se apaga, a un bus que
se cae y a una reconexion, sin congelar la interfaz y sin mostrar datos
falsos.

Estado: **D1, D2 y D3 implementados** (`fase2.39`); pendiente de validacion
final en hardware.

1. [x] **D1 — Lectura OBD no bloqueante.** Una sola peticion en vuelo; el
   estado avanza una vez por iteracion del bucle. Nunca se envia un comando
   nuevo mientras hay otro pendiente y la GUI no se bloquea esperando.
2. [x] **D2 — Handshake validado.** Drenado del socket antes de cada
   secuencia AT, `ATE0` previo a `ATI` (para que el eco no contamine el
   nombre del adaptador) y reintento del handshake si `0100` devuelve la
   mascara `00000000`.
3. [x] **D3 — Reconexion no bloqueante.** La espera entre reintentos ocurre
   en el bucle principal (la GUI sigue viva); `SELECT` cancela una conexion
   en curso; si hay IP pero ningun ELM responde, se reconecta el AP y se
   reintenta el escaneo. El resumen de sesion es acumulativo: cuenta cortes
   de enlace y reconexiones, y ya no se reinicia al reconectar.
4. [ ] Validacion en hardware de los tres puntos con el vehiculo real.

Criterios de aceptacion:

- [x] Ninguna escritura a la ECU en ningun camino de recuperacion.
- [x] Un corte de enlace no congela la interfaz.
- [x] El resumen final refleja la sesion completa, no solo el ultimo tramo.
- [ ] Sin cortes de bus prolongados ni desincronizacion de respuestas tras
      reconectar (medido sobre el log de una sesion real en coche).

## Fase 2.6 — Latencia de la lectura (FASE E)

Objetivo: que el panel refleje el coche sin retardo perceptible. Antes de
esta fase, acelerar tardaba ~1,5 s en verse en el tacometro.

Estado: **E1, E2 y E3 implementados** (`fase2.40`); pendiente de validacion
en hardware.

Diagnostico (sobre `phase1_067`, el log limpio de un viaje real): el RPM se
refrescaba cada **765 ms** en vez de cada 100 ms, y el suavizado anadia otros
**2,1 s**. Tres causas encadenadas:

1. [x] **E1 — El sondeo estaba acoplado al frame.** La respuesta solo se
   detectaba en el siguiente frame (16,7 ms) y el comando siguiente salia en
   la iteracion posterior (16,7 ms): ~26 ms de cada ciclo eran nuestros. Con
   una espera de 2 ms mientras hay peticion en vuelo, el ciclo baja de ~72 a
   ~55 ms.
2. [x] **E2 — El planificador estaba saturado x3,9.** Se pedian 48,9
   peticiones/s contra 12,6 disponibles, asi que todos los PID se degradaban
   a la vez. Los periodos se recalibraron a la capacidad real del enlace
   (presupuesto `PROFILE_LINK_BUDGET_REQ_S`, con test que lo vigila).
3. [x] **E3 — El suavizado estaba calibrado para 10 Hz y recibia 1,3 Hz.**
   Con factor de mezcla fijo (0,3) el retardo dependia de la cadencia: 2,1 s
   hasta el 63% del valor real. Ahora es un filtro de constante de tiempo
   fija (`tau = 150 ms`), con tests propios en el nucleo.

Criterios de aceptacion:

- [x] La demanda total del planificador cabe en la capacidad del enlace
      (test `test_link_budget`).
- [x] El suavizado tiene retardo constante, independiente de la cadencia
      (test `test_smooth`).
- [x] Ninguna escritura a la ECU.
- [ ] El RPM se refresca a >= 2,5 Hz en el coche (era 1,31 Hz).
- [ ] El retardo percibido al acelerar es < 0,5 s (era ~1,5 s).

## Fase 2.7 — Suavidad del indicador (FASE C + A)

Objetivo: que la aguja no de saltos. Con la FASE E el retardo habia
desaparecido, pero el RPM se refrescaba solo 2,14 veces por segundo: la
aguja se dibujaba a 30 fps sobre un dato que cambiaba 2,14 veces/s, asi que
daba escalones de 1400 rpm.

Diagnostico sobre `phase1_071` (fase2.40, 27 min, sin STOPPED ni timeouts):

| Magnitud | Valor |
|---|---|
| Ciclo por peticion | 83 ms (50 del ELM327 + ~24 de hueco + ~9 de lo demas) |
| Capacidad del enlace | 12,05 peticiones/s |
| Demanda del planificador | 16,03 req/s (saturado un 33%) |
| Cadencia del RPM | 467 ms (2,14 Hz) |
| Suavizado EMA | alpha = 0,955 a 467 ms: ya era transparente |

Tareas:

1. [x] **C — Envio inmediato.** El comando siguiente sale en cuanto se
   procesa la respuesta, sin esperar al frame ni al hueco de espera. Se
   anade la metrica `gap` al log para medir el hueco real. Si el ELM327
   no tolera comandos seguidos apareceran STOPPED y habra que buscar el
   minimo seguro.
2. [x] **A — Reparto del presupuesto.** Los periodos se reparten en vez de
   repartir el ancho de banda a partes iguales: RPM a 250 ms (4 Hz), MAP a
   500 ms, y lo que cambia despacio (trims, O2, avance, distancia,
   voltaje, baro, combustible) a 10-20 s.
   La demanda baja de 16,03 a 12,05 req/s y cabe en el presupuesto
   (`PROFILE_LINK_BUDGET_REQ_S` = 13, la cifra conservadora).

Criterios de aceptacion:

- [x] La demanda del planificador cabe en la capacidad del enlace
      (test `test_link_budget`: 12,05 req/s < 13,0).
- [ ] El RPM se refresca a >= 3,5 Hz en el coche (era 2,14 Hz).
- [ ] No aparecen STOPPED al enviar comandos seguidos (si aparecen, hay
      que fijar un hueco minimo).
- [ ] El retardo percibido al acelerar es < 0,3 s.

## Fase 2.8 — Hueco del adaptador y reparto por pantalla (FASE F1 + F3)

Objetivo: que la aguja deje de dar escalones grandes. Con la FASE E ya no
habia retardo, pero el RPM se refrescaba 2,14 veces por segundo.

- [x] **F3 — Reparto por pantalla.** El ancho de banda del enlace (~12-14
  peticiones/s) se reparte segun lo que se esta mirando, no a partes iguales
  entre los 19 PID. Lo que esta en pantalla va a su periodo base y lo demas
  se ralentiza. Los PID de diagnostico pasan de 12 s a 1,5 s cuando se mira
  la pantalla DIAG. El RPM puede ir a 200 ms (5 Hz) sin que el resto se
  quede sin ancho de banda.
- [x] **F1 — Hueco minimo al ELM327.** Medido: con hueco 0 el clon v1.5
  retrasa la respuesta un tick de 16,7 ms y la capacidad cae de 12 a 9,25
  peticiones/s. El hueco no era tiempo perdido, era un requisito del
  adaptador. Ajuste `HUECO ELM` en el menu (0-17 ms, defecto 8) y metricas
  `gap=` y `gapcfg=` en el log para calibrar en un solo viaje.
- [x] **F2.5 — Edad del dato global robusta.** La edad del panel es la de la
  muestra mas reciente de cualquier PID: si el RPM se retrasa pero el resto
  fluye, el panel no da una falsa alarma de "datos viejos".

Criterios de aceptacion:

- [x] La demanda de cada pantalla cabe en el presupuesto del enlace
      (test `test_demand_by_screen`: 13,53 / 13,81 / 12,30 / 9,55 req/s).
- [x] Reprogramar el planificador con una peticion en vuelo no lo cuelga
      (test `test_set_period`).
- [ ] El RPM se refresca a >= 4,5 Hz en el coche y no quedan STOPPED con
      el hueco calibrado.

## Phase 2.9 - K-line support (PHASE K)

Goal: make the app useful in a car that is not the reference one. The second car is a
Mazda MX-5 NB2, ISO 9141-2, with an ELM327 v2.3: 383 ms round trip and ~2 req/s, against
the MiTo's 50-67 ms and 12-14 req/s. What was measured is in
`docs/COMPATIBILIDAD_MX5_NB2.md` and `docs/ANALISIS_LOGS.md`.

- [x] **K0 - Vehicle probe (`SONDA DEL COCHE`).** Eighth tab, read-only. Reads adapter
  and ECU identity, the masks with the list of declared PIDs, the raw header line, and
  then requests every PID once to show which ones the car answers. Everything to the log.
- [x] **K1 - Wait for the protocol search.** While the adapter says `SEARCHING` the app
  now waits up to 20 s. It used to abort at 3 s, which left the ELM327 stuck in `STOPPED`
  for the rest of the session (876 `STOPPED` and zero `NO DATA` in `phase1_001`).
  `protocol.txt` additionally allows forcing the protocol (3 = ISO 9141-2, 5 = KWP fast
  init, 6/7 = CAN).
- [x] **K1b - A log that survives.** Written to the card every second, filesystem sync
  every 2 s. Before, leaving the app without the menu left the log with the boot lines
  only (235 bytes of an entire session).
- [ ] **K2 - Period split by measured capacity.** The base periods assume 12-14 req/s;
  K-line gives 1.9. Measured: RPM every 4.4 s. The scheduler must start from the link's
  capacity (measured, or by protocol class) and split: few PIDs, fast, with the slow ones
  off-screen. Criterion: at idle and while driving, RPM at >= 1 Hz and the four values of
  the active screen at >= 0.5 Hz, without saturating the link.
- [ ] **K3 - GENERAL panel without MAP.** On a naturally aspirated car with no `0B` and
  no `33` the centre boost gauge is a permanent `N/D`. Replace it with MAF (g/s) and load
  (%), and disable `TURBO AUTO` learning when there is no MAP. Criterion: the GENERAL
  screen shows no permanently missing value.
- [ ] **K4 - K-line module discovery.** `obd_disc_parse_line` only recognises CAN headers
  (3 or 8 hex digits). This car's real header is `48 6B 10` (three bytes, six digits).
  Criterion: a unit test with the line `48 6B 10 41 00 BE 1F B8 11 AA` returning mask
  `BE1FB811`, and the MODULOS screen finding the ECU over K-line.
- [ ] **K5 - Link stability.** Two drops in five minutes in `phase1_003` (at 157 s and
  233 s), the second one never recovered. Separate adapter, PSP WiFi and car, and decide
  whether reconnecting needs more than the current three attempts.

## Fase 3 — Parámetros específicos Alfa Romeo (investigación)

Tareas:

1. Leer VIN, CalID, CVN y nombre de ECU.
2. Recopilar documentación/exportaciones verificables.
3. Implementar modo 22 con cabecera explícita y ensamblado ISO-TP si hace falta.
4. Añadir PID extendidos solo con validación en el coche, marcados
   `experimental`.
5. Visor de peticiones/respuestas para validar antes de usar en el dashboard.

Criterios de aceptación:

- [ ] Cada PID extendido tiene fuente, petición, fórmula y fecha.
- [ ] Se ha validado en ralentí, carga y deceleración.
- [ ] Ningún PID inventado ni copiado sin contrastar.
- [ ] Los parámetros no validados siguen apareciendo como `N/D`.

## Fase 4 — Pulido

- Perfil guardable desde la PSP (config.ini).
- Ajuste de umbrales, redline, escala de boost y unidades.
- Gestión de energía y aviso de batería baja.
- Empaquetado (`EBOOT.PBP`, icono, manual).
- Pruebas de uso real: montaje, legibilidad con sol, guantes, acompañante.

## Phase 5 - On-board diagnostics (FALLOS screen)

Goal: have the app say why the check-engine lamp came on. Today it only shows live data
and PID `21` (distance with MIL); it never reads codes.

- [ ] Read PID `01` (MIL state and number of stored codes).
- [ ] Mode `03`: stored codes, decoded to P0xxx with a local description table.
- [ ] Mode `07`: pending codes. Mode `0A`: permanent codes.
- [ ] Mode `02`: freeze frame for the first code (a snapshot of the data at the fault).
- [ ] Mode `06`: on-board monitor results (pass/fail).
- [ ] Read-only throughout. Mode `04` (clear) and mode `08` are never sent.

Criterion: on a car with the lamp on, the screen shows the same code as a commercial
reader, and with no codes it says so explicitly instead of showing an empty list.

## Phase 6 - UI, preferences and vehicle profiles

Goal: stop navigating eight tabs with two triggers, make SELECT carry its own job, keep
the layout per car, and let the app work out which car it is in. Full design in
`docs/superpowers/specs/2026-11-17-ui-screens-preferences-and-vehicle-profiles-design.md`.

- [x] **U1 - Screen picker on SELECT.** Done in `fase2.44`. SELECT opens a full-screen
  list of all eight screens with a one-line purpose each; UP/DOWN moves, X or START opens
  it, SELECT or CIRCLE closes it. START keeps the settings menu. The current screen is
  marked ACTUAL and the selection skips screens this car cannot use.
- [x] **U2 - Two-tier bar.** Done in `fase2.44`. GENERAL, MOTOR, DIAG and TELEM are the
  driving tier: the only screens L/R cycles and the only ones drawn in the bar. MODULOS,
  SNIFFER, SONDA and INFO are the tools tier, reached through the picker, and the bar then
  shows the screen name plus `HERRAMIENTAS  L/R: VOLVER`. MODULOS and SNIFFER are greyed
  with a reason on a K-line car (`scan_can()` reads the ATDPN reply, no bus traffic), and
  `last=` in `settings.txt` reopens the app on the last driving screen, written once per
  change and on exit.
- [ ] **U3 - Preferences v2.** `version=2` line, `start` (a screen name or `last`), `last`,
  `hidden`, and per-vehicle keys `v.<id>.<key>` for layout and tacho. Existing keys keep
  working as global defaults.
- [ ] **U4 - Vehicle profiles.** A table in `src/core/vehicle.c` with the MiTo and the
  MX-5 plus two generic fallbacks, matched on protocol and masks, overridable by hand, and
  logged as one line per session with the evidence. The centre gauge follows capability:
  BOOST, MAF or LOAD.
- [ ] **U5 - New screens in both languages.** Route the SONDA labels (and anything added
  from here) through `tr()` instead of literals, so the screen follows the language setting like the rest.

Criterion: on the MX-5 the bar shows four driving screens, SNIFFER is not offered, the
layout is the MX-5's own, the app opens on the preferred screen, and the log names the car
and the masks it was recognised by. On the MiTo nothing changes except the bar.

## Fuera de alcance

- Escritura/codificación en la ECU.
- Borrado de DTC.
- Conexión simultánea con Torque/AlfaOBD.
- Bluetooth/USB en PSP.
- Android/iOS/PC como producto final (el simulador de PC es solo herramienta).

## Riesgos principales

| Risk | Impact | Mitigation |
|---|---|---|
| PSP without native WPA2 | High | Open/WEP/WPA-TKIP adapter, ARK4 plugin, or a bridge |
| Adapter without DHCP | Medium | Configurable static IP |
| PIDs not available | High | Masks + `N/D`, per-session profile |
| Slow or unstable ELM327 clone | Medium | Timeouts, backoff, diagnostics |
| Low bandwidth | Medium | Priority scheduler, never poll everything |
| PSP memory/CPU | Medium | Fixed buffers, 30 fps, font atlas |
| Slow or removed Memory Stick | Low | Periodic flush, clean close |
| Misreading boost/Torque | High | Explicit MAP/baro, `EST`, never invent |
| Fatigue/safety while driving | High | Mounting, passenger, read-only |
| K-line link of ~2 req/s | High | Capacity-based split (K2), few PIDs on screen |
| Three-byte headers on K-line | Medium | Per-protocol discovery parser (K4) |
| ELM327 v2.3 clone at 383 ms | Medium | Document the ceiling; recommend a better adapter |

## Decisiones pendientes

1. Real ELM327 security mode and exact PSP model/CFW.
2. Toolchain: Docker `pspdev` or a native install.
3. Confirm whether the car exposes `0x0B` (MAP), `0x33` (baro), `0x5C` (oil) and
   `0x5E` (fuel rate). **Answered for the MX-5 in `COMPATIBILIDAD_MX5_NB2.md`: none of
   them exists.**
4. Confirm the real redline and the thresholds the owner wants.
5. Whether the ARK4 WPA2 plugin will be used or a bridge.
6. K2: fixed periods per protocol class, or measured live from `rtt`.
7. FALLOS: its own tab, or inside DIAG.
8. MODULOS and SNIFFER on a K-line car: hide them, warn, or adapt them.
9. GENERAL panel: adapt automatically to the car (no MAP means MAF/load), or make the
   layout a selectable profile.
10. Whether K-line support is generic (any car) or MX-5 only.
