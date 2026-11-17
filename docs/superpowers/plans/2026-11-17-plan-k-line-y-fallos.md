# Plan: soporte K-line y pantalla de fallos

- **Fecha:** 2026-11-17
- **Estado:** plan de trabajo, sin empezar los paquetes K2 en adelante
- **Rama:** `feat/sonda-diagnostico` sobre `09159e1` (fork `konqiDAM/psp-torque`)
- **Base medida:** `docs/COMPATIBILIDAD_MX5_NB2.md`, `docs/ANALISIS_LOGS.md`
  (sesiones `phase1_001` a `phase1_003`), `docs/PRUEBAS_PENDIENTES.md`

## 0. Resumen

El segundo coche es un Mazda MX-5 NB2 en ISO 9141-2 con un ELM327 v2.3. La
app ya conecta y lee 13 PIDs, pero el planificador esta dimensionado para un
enlace 7 veces mas rapido, el panel central ensena un boost que ese coche no
puede dar, las pantallas de MODULOS y SNIFFER no funcionan en K-line, y la app
no sabe decir por que se enciende el testigo de averia. Este plan cierra esas
cuatro cosas sin romper el MiTo.

Lo que ya esta hecho y no se vuelve a tocar: la sonda `SONDA DEL COCHE`, la
espera de la busqueda de protocolo, el `protocol.txt`, el log que se escribe
cada segundo y la carpeta `ms0:/PSP/SAVEDATA/PSPTORQUE`.

## 1. Decisiones tomadas

1. **Nada de datos inventados.** Se mantiene la regla: sin lectura, `N/D`. No
   se interpola la aguja ni se estima un boost que no existe.
2. **Solo lectura.** Modo `01`, `02`, `03`, `06`, `07`, `09`, `0A` y comandos
   `AT`. Nunca `04` (borrar), ni `2E`/`2F`/`31`, ni actuadores.
3. **Los periodos se deciden por capacidad de enlace**, no por lo que el coche
   "deberia" dar. El presupuesto constante actual (`PROFILE_LINK_BUDGET_REQ_S`
   = 14) solo vale para CAN.
4. **El panel se adapta al coche por los PIDs que existen**, no por un perfil
   escrito a mano: si no hay MAP, el centro es MAF y carga.
5. **Las pantallas que no aplican avisan**, no se quedan en blanco.
6. **El MiTo es la red de seguridad.** Cada cambio del nucleo se valida contra
   el (T10 en `docs/PRUEBAS_PENDIENTES.md`).

## 2. Paquetes de trabajo

### K2 - Reparto de periodos por capacidad (primero)

**Por que primero:** sin esto la app es un panel de datos que se refresca cada
4,4 s; todo lo demas que se anada se vera mal.

**Datos de partida:** 383 ms por peticion, 1,9 req/s medidos, 15 PID activos,
RPM cada 4,4 s. Con 2,6 req/s de techo, cuatro valores vivos son un refresco
cada 1,5 s.

**Archivos:** `src/core/profile.c` (tabla `g_base_periods`, funcion
`obd_profile_period`), `include/obd/profile.h` (presupuesto), `src/psp/main.c`
(deteccion del protocolo en `scan_refresh_proto` y aplicacion en
`apply_screen_schedule`), `tests/test_main.c`.

**Diseno propuesto:**
- Un presupuesto por clase de enlace: CAN (14 req/s, lo actual) y K-line
  (~2 req/s). La clase sale de `ATDPN`: `1-5` es J1850/K-line, `6-9` es CAN.
- En K-line, los periodos base se sustituyen por una tabla corta: RPM 700 ms,
  MAF/carga 1000 ms, refrigerante 3000 ms, y **fuera de pantalla todo lo
  demas** (trims, sondas, avance) a 30 s o desactivado.
- Test nuevo `test_demand_by_link`: para cada clase, la demanda de la pantalla
  activa tiene que caber en el presupuesto, y el RPM tiene que ser siempre el
  mas rapido.

**Criterios de cierre:**
- Test del nucleo en verde con las dos clases.
- En el coche: RPM a >= 1 Hz y los cuatro valores de la pantalla a >= 0,5 Hz,
  sin `STOPPED`.
- En el MiTo: nada cambia (T10).

**Riesgo:** que el usuario quiera ver trims en un K-line. Se resuelve dando
prioridad a la pantalla activa: en DIAG, los trims suben y el resto baja.

### K3 - Panel GENERAL sin MAP

**Por que:** `BOOST` necesita `0B` y `33`; este coche no tiene ninguno. El
indicador central esta muerto y el voltimetro tambien (`42` no existe).

**Archivos:** `src/psp/main.c` (`draw_qv`, `draw_tacho`, `widget_available`,
`g_slot_item` por defecto, `boost_learn_update`), `src/core/boost.c` (que ya
marca `N/D`; tocar solo si hace falta), `RUN.txt`, `README.md`.

**Diseno propuesto:**
- Si el coche no declara `0B`, el hueco central por defecto pasa a MAF (barra)
  y el segundo a carga (%). El fondo de escala de MAF es aprendido, como ya se
  hace con el boost.
- `TURBO AUTO` se desactiva y no se guarda un maximo de boost inexistente.
- El voltimetro se oculta si no hay `42`; en su lugar, la tension del adaptador
  (`ATRV`) queda en la pantalla INFO, marcada como tal.

**Criterios de cierre:** en marcha no hay ningun dato fijo en `N/D` en GENERAL
y los valores se mueven con el acelerador (T6).

### K4 - Descubrimiento de modulos en K-line

**Por que:** `obd_disc_parse_line` solo reconoce cabeceras CAN (3 u 8 digitos
hex). La cabecera K-line son **tres bytes** y ya esta capturada:
`48 6B 10 41 00 BE 1F B8 11 AA`.

**Archivos:** `src/core/discovery.c` (parser de linea), `include/obd/discovery.h`,
`tests/test_discovery.c`, `src/psp/main.c` (`scan_obd`, `draw_modules`).

**Diseno propuesto:**
- Reconocer una cabecera de 3 bytes cuando el protocolo no es CAN: los tres
  primeros tokens de 2 digitos hex, sin pasar de un numero razonable.
- Si no hay cabecera reconocible, **buscar el byte `0x41`** en la trama y
  tomar los 4 bytes siguientes como mascara (que es lo que ya hace la sonda).
  Esto vale para cualquier protocolo y es lo que hace que la sonda funcione
  hoy.
- Test unitario con la linea real del MX-5 que devuelve `BE1FB811`, y otro con
  la linea CAN de siempre (`7E8 06 41 00 BE 3E B8 11`) para no romper el MiTo.
- La pantalla MODULOS dice en que protocolo esta y que el escaneo UDS solo
  aplica a CAN, en vez de mostrar una lista vacia.

**Criterios de cierre:** el test pasa con las dos lineas y la pantalla MODULOS
encuentra la centralita en el MX-5 (T9).

### K5 - Estabilidad del enlace

**Por que:** dos caidas en cinco minutos, la segunda sin recuperar. Es lo que
mas va a molestar en un uso real y no se arregla con mas tiempo de espera.

**Archivos:** `src/psp/main.c` (`elm_rx_step`, el bloque de reconexion de
`run_dashboard`, `connect_network_and_elm`).

**Trabajo:** antes de tocar codigo, ejecutar T4 con 15 minutos de sesion y
mirar el patron: si el `rtt` sube antes de caer, si el socket muere o solo se
queda mudo, y si `ATI` responde tras la caida. Con eso se decide: mas
reintentos, reabrir el AP, o reiniciar el ELM327 con `ATZ`.

**Criterios de cierre:** 15 minutos sin caidas, o caidas recuperadas en el
primer reintento, con el log diciendo cual fue la causa.

### F5 - Pantalla FALLOS (diagnostico a bordo)

**Por que:** es la funcion que de verdad se usa en un coche viejo. Hoy la app
no sabe decir por que esta encendido el testigo.

**Archivos:** `src/psp/main.c` (pantalla nueva o dentro de DIAG), un modulo
nuevo en `src/core/` para decodificar codigos (`dtc.c`/`dtc.h`), su test y la
tabla de descripciones. `RUN.txt` y `README.md`.

**Trabajo, en orden:**
1. PID `01`: estado del testigo y numero de codigos. Una peticion.
2. Modo `03`: codigos guardados, `43 01 XX YY ...`, traducidos a `P0xxx`.
3. Modo `07` y `0A`: pendientes y permanentes.
4. Modo `02`: freeze frame del primer codigo.
5. Modo `06`: monitores a bordo, solo si sale barato en K-line.

**Criterios de cierre:** con el testigo apagado dice "sin codigos"; con codigos,
coincide con un lector comercial (T7). Ninguna ruta envia modo `04`.

## 3. Orden y dependencias

```
K2 (periodos)  ->  K3 (panel)
     |                  |
     +----> K5 (enlace) +----> F5 (fallos) -> K4 (modulos)
```

K2 va primero porque cambia los tiempos de todo lo demas. K4 es independiente
y se puede hacer en cualquier momento. F5 no depende de nada mas que de tener
una pantalla donde ponerlo.

## 4. Convenciones de trabajo

- Codigo y comentarios en espanol, sin acentos en el codigo, como el resto del
  repositorio. Documentacion tambien en espanol.
- Un cambio = una rama = un commit con mensaje en espanol. Nada de mezclar
  pantallas y nucleo en el mismo commit.
- Cero avisos con `-Wall -Wextra` (ver `docs/DESARROLLO.md`).
- Cada cambio de `src/core/` lleva su prueba en `tests/`.
- Antes de instalar en la PSP, comprobar la pantalla en PPSSPP.
- Actualizar `RUN.txt` (manual de usuario) y `README.md`/`README_EN.md`
  (funciones) cuando cambie lo que ve el usuario. Mediciones a
  `docs/ANALISIS_LOGS.md`; comportamiento por coche a
  `docs/COMPATIBILIDAD_*.md`; criterios a `docs/ROADMAP.md`.
- El log es la prueba: cada criterio de cierre de este plan se demuestra con un
  log, no con "parece que va".

## 5. Lo que no se hace en esta fase

- Escritura en la ECU, borrado de codigos, actuadores. Nada.
- Interpolar la aguja o estimar un boost que el coche no da.
- PIDs de fabricante (modo 22) sin documentacion verificable.
- Escaneo UDS en K-line.
- Cambiar el adaptador: el ELM327 v2.3 es el que hay. Se documenta su techo.
