# PSP Torque

**Panel de telemetría OBD‑II de solo lectura para PSP.**

Lee datos reales del coche a través de un adaptador **ELM327 WiFi** y los
representa en un dashboard gráfico de 480×272 con temas de vídeo, medidores
de turbo, telemetría y diagnóstico.

Desarrollado y validado sobre un **Alfa Romeo MiTo Quadrifoglio Verde 2010**
(1.4 Turbo MultiAir, 170 CV) con PSP‑2000/3000 y CFW ARK4, pero el núcleo es
**universal**: usa solo OBD‑II estándar (SAE J1979) y se adapta a las
capacidades reales de cualquier ECU.

```
┌──────────────────────── PSP 480×272 ────────────────────────┐
│  GENERAL   MOTOR   DIAG   TELEM   MODULOS   SNIFFER   INFO  │
│  ┌────────┐   ┌──────────┐   ┌────────┐   ┌────────┐        │
│  │  RPM   │   │ BOOST    │   │  ECT   │   │  MAP   │        │
│  │  3 240 │   │  +0.82   │   │  87 °C │   │ 112 kPa│        │
│  │  km/h  │   │   bar    │   │        │   │        │        │
│  └────────┘   └──────────┘   └────────┘   └────────┘        │
│                      ONLINE ·  87 km/h                       │
└──────────────────────────────────────────────────────────────┘
```

---

## Características

- **Solo lectura, siempre.** No escribe en la ECU, no borra códigos de fallo
  (DTC), no codifica y no manda órdenes a actuadores.
- **Universal.** Descubre qué PIDs existen leyendo las máscaras `0100`/`0120`/
  `0140` reales de la centralita. Si un PID no existe, se muestra `N/D` o
  `NO SOP.` — nunca un `0` inventado.
- **Boost correcto.** `BOOST = MAP − presión barométrica`. La presión de
  referencia se toma del PID `0x33`, de una captura KEY‑ON con el motor parado
  o, en último caso, de un valor estimado marcado como `EST`.
- **Modo universal sin PIDs de fabricante.** No usa modo 22 ni DIDs propietarios:
  no son universales y no se inventan.
- **7 pantallas:** `GENERAL`, `MOTOR`, `DIAG`, `TELEM`, `MODULOS`, `SNIFFER`,
  `INFO`.
- **10 temas**, y los de juego cambian la estructura del reloj, la aguja y los
  indicadores, no solo los colores.
- **Panel de telemetría** con 5 series simultáneas, colores sin colisiones y
  máximo/mínimo por serie.
- **Dashboard editable:** cada hueco puede ser `RELOJ`, `BARRA`, `TEXTO` u
  `OCULTO`, y se elige qué dato muestra.
- **Descubrimiento de centralitas** (OBD‑II y UDS de identificación) y
  **sniffer de bus CAN** para capturar el tráfico completo de la sesión.
- **Sonda del coche** (`SONDA DEL COCHE`): pantalla de diagnóstico que lee
  la identidad del adaptador y de la ECU, las máscaras de PID, la cabecera
  cruda y prueba cada PID uno a uno. Sirve para saber qué puede dar un coche
  que no es el de referencia, sin tocar el planificador a ciegas.
- **Modo demo** con un perfil de conducción simulado de ~130 s, para probar la
  interfaz sin coche.
- **Log por sesión** en la Memory Stick, con métricas de rendimiento
  (CPU, espera, VBlanks, vértices) y resumen final de la sesión.
- **Latencia baja:** lectura OBD no bloqueante, una sola petición en vuelo y
  cadencia de dibujo sincronizada con VBlank. El sondeo **no** depende del
  frame: la respuesta se detecta en ~2 ms en vez de esperar al siguiente
  frame, y la cadencia de cada PID está calibrada contra la capacidad real
  del enlace, no contra lo que el coche "debería" dar.

## Requisitos

| Componente | Detalle |
|---|---|
| PSP | PSP‑2000 / PSP‑3000 / PSP‑E1000 con WiFi. Requiere CFW (ARK4) para la configuración de red. |
| Adaptador | ELM327 **WiFi** (clone `v1.5` validado). Debe abrir un punto de acceso propio. |
| Red | La PSP se conecta a la red del adaptador y habla **TCP** contra él. |
| Coche | Cualquier vehículo con OBD‑II de lectura. |
| Puerto | `35000` por defecto (también `23` y `2000` en otros clones). |

> **Limitación de hardware:** la PSP original no soporta WPA2. El adaptador
> debe usar una red abierta, WEP o WPA‑PSK/TKIP. Con CFW y el plugin
> `wpa2psp` se puede usar WPA2‑AES, o se puede interponer un router/ESP32
> que hable con el adaptador. Ver
> [`docs/COMPATIBILIDAD.md`](docs/COMPATIBILIDAD.md).

### Configurar el destino del adaptador

La app lee `ms0:/PSP/SAVEDATA/PSPTORQUE/host.txt`. Si el archivo no existe,
usa `192.168.0.10:35000`.

```
# Formato: IP PUERTO
192.168.0.10 35000
```

También puedes cambiar IP y puerto desde el propio menú de la app.

## Instalación

1. Copia la carpeta `PSPTorqueOBD` a `ms0:/PSP/GAME/PSPTorqueOBD/` en tu
   Memory Stick. Debe quedar:

   ```
   ms0:/PSP/GAME/PSPTorqueOBD/EBOOT.PBP
   ms0:/PSP/SAVEDATA/PSPTORQUE/              (datos: la app la crea)
   ms0:/PSP/SAVEDATA/PSPTORQUE/host.txt      (opcional)
   ```

2. Enciende el ELM327, pon el **contacto** con el motor **parado**.
3. En la PSP: **Juego → Memory Stick → PSP Torque**.
4. La app abre el dashboard al instante y, por detrás, se conecta sola al
   perfil WiFi guardado. Verás `CONECTANDO WIFI` y `BUSCANDO ELM327` sobre el
   propio dashboard.
5. Si no conecta: `TRIANGLE` reintenta con el perfil guardado, `SQUARE` abre
   el diálogo WiFi de la PSP.
6. Arranca el motor y deja que esté a ralentí 30–60 s.

### Binario precompilado

Hay una **Release** adjunta con `EBOOT.PBP` ya compilado, pero es de
`fase2.39`: no tiene ni el perfil del MX-5 ni la salida limpia ni la pantalla
FALLOS que se describen aquí. Para lo que cuenta esta página hay que compilar, o
pedir el `EBOOT` de la última sesión.

## Controles

| Botón | Acción |
|---|---|
| `L` / `R` / `◀` / `▶` | Cambiar de pantalla de conducción (GENERAL, MOTOR, DIAG, TELEM) |
| `SELECT` | Abrir / cerrar el selector de pantallas |
| `START` | Abrir / cerrar el menú de ajustes |
| `START` (mantener 1,2 s) | Salir de la aplicación desde cualquier pantalla, incluidas las que corren su propio bucle (sniffer, FALLOS, escaneo). Un escaneo en curso se cancela con `CIRCLE` |
| `TRIANGLE` | Reconectar con el perfil WiFi guardado |
| `SQUARE` | Abrir el diálogo WiFi de la PSP y reconectar |
| `X` o `START` | Saltar la animación de arranque · confirmar · salir del modo demo |
| `L` + `R` (1,5 s) | Activar / desactivar el **modo demo** |
| `CIRCLE` | Cancelar un escaneo en curso · borrar el máximo de turbo aprendido |

Para salir de la aplicación: mantener `START` pulsado 1,2 s, o `START` → bajar
hasta `SALIR` → `X`. Las dos salidas son limpias: guardan los ajustes y escriben
el resumen de la sesión al final del log. Con el botón HOME de la PSP no se
ejecuta nada de eso y el log termina sin resumen.

Desde una pantalla de herramientas, `L`/`R` vuelve a GENERAL.

## El selector de pantallas

`SELECT` abre la lista completa de las nueve pantallas con una línea de para qué sirve
cada una. Arriba y abajo mueven, `X` o `START` abre la elegida, `SELECT` o `CIRCLE`
cierra la lista. Las pantallas que el coche no puede usar salen en gris con el motivo:
en un coche anterior a CAN, MODULOS y SNIFFER necesitan CAN y no se ofrecen.

La barra y `L`/`R` llevan solo las cuatro pantallas de conducción. Desde una herramienta
la barra dice el nombre de la pantalla y `L/R: VOLVER`, así que la vuelta está siempre a
la vista.

## Dónde arranca

Se abre en la pantalla de conducción en la que se quedó. La elección se guarda en
`ms0:/PSP/SAVEDATA/PSPTORQUE/settings.txt` como `last=`, una vez por cambio y otra al
salir. Si se quedó en una herramienta, abre GENERAL.

## Las pantallas

| Pantalla | Contenido |
|---|---|
| **GENERAL** | Tacómetro, velocímetro, boost y tiles de ECT / IAT / MAP. En un coche sin MAP, el coche cambia los tres huecos que quedarían vacíos: MAF al centro, MARCHA a un lado y AVANCE al otro. |
| **MOTOR** | Rejilla de tiles: MAP (o MAF), BOOST, ECT, IAT, acelerador, mariposa/load, tensión, aceite y combustible. Las cajas no disponibles se compactan: no dejan huecos. |
| **DIAG** | Diagnóstico avanzado de solo lectura: STFT, LTFT, avance, tensiones O₂, combustible, voltaje, distancia con MIL y voltaje de batería. |
| **TELEM** | Gráfica de 5 series a la vez (por defecto RPM, boost, avance y mariposa) + máximos/mínimos de sesión. |
| **FALLOS** | Códigos de avería: estado de la lámpara, guardados (03), pendientes (07) y permanentes (0A) con su significado, decodificados en la PSP, más la foto del fallo (02) si la ECU la da. `X` lee, `CIRCLE` limpia la pantalla — nunca la centralita. |
| **MODULOS** | Descubrimiento de centralitas OBD‑II y UDS. `X` lanza el escaneo. Funciona también en coches de K-line, donde lee las máscaras OBD‑II y se salta el sondeo que es solo CAN. |
| **SNIFFER** | Captura de todo el tráfico CAN del bus. `X` inicia o detiene. |
| **INFO** | Qué es cada parámetro, con su PID estándar de origen. |
| **SONDA** | Lee los PID uno a uno y dice cuáles responde la ECU, cuáles dan `NO DATA` y cuáles no existen. Tarda 30–60 s y se cancela con `CIRCLE`. |

### Ajustes

`START` abre el menú: `TEMA`, `COLOR`, `IDIOMA` (auto/español/inglés),
`CONTRASTE`, `ANIMACION`, `MUESTREO`, `EDITAR VISTA`, `PICO MAX`, `SHIFT RPM`,
`RPM MAX`, `UNIDADES` (métrico/imperial), `TURBO AUTO`, `BOOST MAX`, `HUECO ELM`,
`PANTALLA INICIAL`, `DEBUG`, `LOG`, `MODO SHOW`, `AGUJA`, `REPARTO` y `SALIR`.

`AGUJA` elige cuánto se suavizan las agujas del tacómetro y del velocímetro
(`DIRECTA`, `SUAVE` de 150 ms o `MUY SUAVE` de 400 ms); los números y las barras
van siempre con la medida tal cual. `REPARTO` (0 a 4) decide cuántos valores de
la pantalla entran en el reparto rápido del enlace: más bajo, aguja más rápida.
En un coche de CAN el reparto no se usa (la tabla base cabe entera), así que
`REPARTO` no cambia nada en el MiTo.

`LOG` decide qué se escribe en la tarjeta: `NO` no abre archivo, `SESION`
guarda el arranque, las líneas de estado y el resumen final, y `COMPLETO` (lo
que viene de fábrica) guarda además cada petición y cada respuesta. Si el
archivo está apagado a propósito no sale el aviso `SIN LOG`: apagarlo es una
decisión, no un fallo.

`MODO SHOW` arranca el cuadro sin red y sin adaptador: entra en el modo demo,
cambia de pantalla cada 12 s y de tema cada 30 s, y no escribe ni el log ni los
ajustes. Una pulsación cualquiera deja el cuadro quieto un minuto y luego sigue
girando. `SELECT` y `START` mantenidos dos segundos cierran la app y devuelven
el arranque normal a la tarjeta; los gatillos, `TRIANGLE` y `SQUARE` no hacen
nada en ese modo, para que nadie en una feria deje el cuadro en un menú o
pidiendo datos. Se enciende en el menú de ajustes y se aplica al siguiente
encendido.

Se guardan en `ms0:/PSP/SAVEDATA/PSPTORQUE/settings.txt` y se aplican al
instante.

## Temas

`NORMAL` (con color a elegir: rojo, verde, ámbar, azul o monocromo) más nueve
temas de juego: **FORZA**, **NFS**, **GRAN TURISMO**, **INITIAL D**,
**CYBERPUNK**, **RETRO 80**, **NOTHING**, **BACK FUTURE** y **TODO GAS**.

Los temas de juego no son un simple cambio de paleta: cambian la estructura del
reloj (aros dobles, escala por bloques, aro técnico), la forma de la aguja
(triangular, doble con punta de flecha, roja con contrapeso), la tipografía
(cursiva con halo, negrita, retro) y los indicadores (barras troceadas,
franjas rosas, barras neón segmentadas). Cada uno añade además su efecto
propio:

| Tema | Efecto |
|---|---|
| INITIAL D | Campaneo de aviso a 100 km/h (fijo) y `AVISO` parpadeante. |
| NFS | Tinte azul, estelas laterales y barra de nitró que sigue la aceleración. |
| FORZA | Marcador de `SKILL` con puntos que suben mientras aceleras fuerte. |
| GRAN TURISMO | Medidor de G calculado y aviso rojo en la zona roja. |

> **Tema `NOTHING`:** fondo oscuro, sin adornos.

## Modo demo

Mantén `L` + `R` durante 1,5 segundos. La cabecera muestra `DEMO` y el panel
presenta un coche simulado (ralenti, aceleración hasta 130 km/h, crucero,
estirada hasta 185 km/h y frenada). Los valores se marcan como `SIM` en el log.

No necesita coche ni adaptador: sirve para probar temas, escalas, unidades y
el aprendizaje del turbo. Se desactiva con el mismo combo.

## Compilar desde el código

Requiere Docker (con la imagen `pspdev/pspdev`) o el PSPSDK instalado de
forma nativa.

```bash
# Linux / macOS / Git Bash
MSYS_NO_PATHCONV=1 docker run --rm \
  -v "D:/ruta/al/proyecto:/src" -w /src \
  pspdev/pspdev \
  bash -lc 'export PATH=/usr/local/pspdev/bin:$PATH; make'
```

```powershell
# Windows
powershell -ExecutionPolicy Bypass -File tools\build.ps1
```

Genera `EBOOT.PBP` en la raíz y lo copia junto a `host.txt` y `RUN.txt` en
`dist\PSPTorqueOBD\`, listo para copiar a la Memory Stick.

#### Alternativa sin Docker: WSL

Si Docker no está disponible (o se rompe), el toolchain también funciona
dentro de una distro WSL de Linux:

```bash
# 1. Extraer la imagen pspdev/pspdev del registro de Docker Hub
#    (216 MB; no necesita el motor de Docker) a un directorio, por
#    ejemplo D:/pspdev-root
# 2. Copiar el toolchain a la ruta que espera:
cp -a /mnt/d/pspdev-root/usr/local/pspdev /usr/local/
# 3. El loader de musl busca las librerias en su fichero de ruta:
echo /mnt/d/pspdev-root/usr/lib > /etc/ld-musl-x86_64.path
echo /mnt/d/pspdev-root/lib     >> /etc/ld-musl-x86_64.path
# 4. Compilar:
sh /mnt/d/psp_torqueOBD/tools/build_wsl.sh
```

`tools/build_wsl.sh` hace la limpieza, compila, copia a `dist/` y saca el
SHA-256.

### Pruebas del núcleo

El núcleo portable no depende de la PSP y se prueba en PC con `gcc`:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_core.ps1
```

220 comprobaciones sobre el parser ELM327, el decodificador de PIDs, el
cálculo de boost/baro, el scheduler, el almacén de muestras y el
descubrimiento de módulos.

### Simulador

`tools/elm327_sim.ps1` levanta en el PC un servidor TCP que habla el
protocolo ELM327 con datos sintéticos. Sirve para desarrollar la interfaz sin
coche:

```powershell
powershell -ExecutionPolicy Bypass -File tools\elm327_sim.ps1
```

## Estructura del proyecto

```
.
├── include/obd/     Cabeceras del núcleo portable (C99, sin dependencias de PSP)
├── src/core/        Parser ELM327, PIDs, boost/baro, scheduler, muestras, discovery, suavizado
├── src/psp/         Capa PSP: GU, fuentes, entrada, audio, red y bucle principal
├── tests/           Pruebas del núcleo en PC (gcc nativo)
├── tools/           Build en Docker, simulador ELM327 y lector de logs
├── profiles/        Perfil de vehículo con sus valores por defecto
├── docs/            Documentación técnica de diseño
├── assets/          Icono del EBOOT
├── host.txt         Destino IP/puerto del ELM327
└── RUN.txt          Manual de uso que se copia a la Memory Stick
```

## Arquitectura

```
┌──────────────────────────── PSP ────────────────────────────┐
│  GUI (GU, 480×272, 30/60 fps)                               │
│      ▲ muestras                                              │
│  Scheduler de PID ──► 1 petición en vuelo ──► ELM327 (TCP)  │
│      │                                   ▲                   │
│  Almacén de muestras ◄── decodificadores ◄─┘                 │
│      │                                                      │
│  Log de sesión ◄─── Parser ELM327 ◄─── socket no bloqueante  │
└──────────────────────────────────────────────────────────────┘
```

- **Núcleo portable en C99** (`src/core/`): parser del protocolo ELM327,
  decodificación de PIDs estándar, cálculo de boost y barométrica,
  scheduler de sondeo, almacén de muestras y descubrimiento de módulos.
  Sin una sola dependencia de la PSP, y por eso se prueba en el PC.
- **Capa PSP** (`src/psp/`): pipeline GU, fuente 8×8 propia, entrada de
  mando, audio sintetizado, sockets `sceNetInet`/`sceNetApctl` y bucle
  principal.
- **Lectura no bloqueante:** el núcleo emite una sola petición OBD en vuelo
  y avanza su estado una vez por iteración del bucle. Nunca se envía un
  comando nuevo mientras hay otro pendiente, y la interfaz nunca se bloquea
  esperando una respuesta.
- **Sondeo desacoplado del render:** mientras hay una petición en vuelo, el
  bucle espera 2 ms en vez de un VBlank completo (16,7 ms). Antes, esa espera
  añadía ~26 ms a cada petición y limitaba el enlace a 12,6 peticiones/s;
  ahora da ~18. La cadencia de dibujo no cambia: sigue contando VBlanks.
- **Cadencia adaptativa, por dos bucles.** El primero reparte el presupuesto
  del enlace entre lo que pide la pantalla activa y lo que no, y se recalcula
  al cambiar de pantalla o de capacidad. El segundo mide lo que el adaptador
  entrega de verdad y sube o baja ese presupuesto cada 10 s. En CAN se planifica
  a 16,5 peticiones/s; en el K-line de la MX-5, a 2,5 (383 ms por petición son
  2,6 físicos). Un test del núcleo calcula la demanda y falla si se pasa.
- **Aguja suavizada, sin inventar:** las agujas del tacómetro y del velocímetro
  se acercan a la última medida en cada fotograma y ahí se quedan: nunca
  extrapolan por delante de la última lectura ni se inventan un valor. Los
  números, las barras y la gráfica van siempre con la medida tal cual.
  `AGUJA` en el menú elige `DIRECTA`, `SUAVE` (150 ms) o `MUY SUAVE` (400 ms).
- **Panel adaptado al coche:** en la MX-5, que no tiene MAP, el GENERAL
  cambia el hueco del centro por un bloque con el aire (MAF), la presión de
  aceite **calculada** (marcada `EST`: ese coche no lleva sensor) y el agua; y
  el hueco del MAP pasa a ser el avance. Un BOOST o un MAP en un coche sin ese
  sensor se sustituyen también con la vista guardada; el bloque y la marcha
  llegan con la vista de fábrica o con la migración de una tarjeta vieja.
- **Reparto del enlace (`REPARTO`):** cuántos valores de pantalla entran en el
  reparto rápido, de 0 a 4. Cada valor visible reserva su suelo y el RPM se lleva
  el resto, así que con `REPARTO` bajo la aguja va más rápida; los valores que se
  pintan y no entran en el cupo vuelven cada ~10 s en vez de apagarse a `N/D`.
  Solo cuenta lo que está en pantalla, no todo lo que el perfil marca.
- **El almacén guarda la medida tal cual llega:** el log, los máximos y el
  histórico de la gráfica son evidencia, no presentación. El suavizado vive solo
  en lo que se pinta.
- **Cadencia por VBlank:** el dibujo se sincroniza con `sceDisplayGetVcount()`
  en lugar de con temporizadores, y el número de VBlanks por fotograma se
  ajusta solo según la carga de CPU.
- **Memoria:** buffers y tablas estáticas; nada de `malloc` en el bucle de
  render.

Detalle completo en [`docs/ARQUITECTURA.md`](docs/ARQUITECTURA.md).

## Reglas del proyecto

Estas reglas son la razón de ser del proyecto y no se negocian:

1. **Solo lectura.** Nunca se envían comandos de escritura a la ECU: nada de
   borrar DTC, codificar, actuar sobre actuadores ni cambiar parámetros.
2. **No se inventan lecturas.** Un PID no disponible se muestra como `N/D`
   (no disponible), nunca como `0`.
3. **No se inventan PIDs propietarios, direcciones CAN ni comandos de
   fabricante.** Los parámetros extendidos solo se incorporan con
   documentación contrastada y validación en el vehículo real.
4. **Los datos simulados se marcan siempre como `SIM`.** El simulador es una
   herramienta de desarrollo, no una fuente de datos presentada como real.
5. **`MAP` y `BOOST` son parámetros distintos.** `MAP` es presión absoluta
   (kPa/bar absolutos); `BOOST` es `MAP − barométrica`. Si la barométrica no
   es fiable, se marca como `EST`.

## Limitaciones conocidas

- La PSP original no soporta WPA2 (ver más arriba).
- Los datos de suspensión, presión de turbo objetivo, DPF, EGR y otros
  parámetros de fabricante **no son universales** y no se muestran. El
  descubridor de módulos puede detectar esas centralitas y leer su
  identificación estándar, pero no sus datos en vivo sin conocer los DIDs
  concretos de ese modelo.
- Si el coche tiene un gateway de seguridad, puede bloquear direcciones
  CAN: el escaneo mostrará solo lo que el gateway permita.
- La lectura del avance de encendido (`0x0E`) se omite cuando la ECU
  devuelve valores físicamente imposibles con la escala estándar de OBD‑II.
- El escaneo de módulos tarda entre 10 y 30 s en CAN de 11 bits y algo más
  en CAN de 29 bits (sondea 256 direcciones físicas). Se puede cancelar con
  `CIRCLE`.
- La reconexión automática reintenta hasta 3 veces con 2 s de espera entre
  intentos. Si el adaptador no vuelve, hay que reintentar a mano con
  `TRIANGLE`.
- El HUD está diseñado para 480×272 y no se adapta a otras resoluciones.

## Documentación

| Documento | Contenido |
|---|---|
| [`docs/ARQUITECTURA.md`](docs/ARQUITECTURA.md) | Capas, concurrencia, protocolo ELM327, scheduler, errores y presupuesto de tiempo. |
| [`docs/COMPATIBILIDAD.md`](docs/COMPATIBILIDAD.md) | PIDs estándar, fórmulas, MAP/BOOST, barométrica, límites y política para PIDs de fabricante. |
| [`docs/UI_QV.md`](docs/UI_QV.md) | Tema QV, animación de arranque, pantallas, colores, tipografía y transiciones. |
| [`docs/PSPGRAPHICS_MIGRATION.md`](docs/PSPGRAPHICS_MIGRATION.md) | Migración de la pantalla de texto al dashboard 2D. |
| [`docs/ANALISIS_LOGS.md`](docs/ANALISIS_LOGS.md) | Análisis de sesiones reales en coche: desincronizaciones, cortes de bus y módulo a módulo. |
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | Fases, criterios de aceptación y riesgos. |
| [`docs/FASE0.md`](docs/FASE0.md) / [`docs/FASE1.md`](docs/FASE1.md) | Viabilidad técnica y núcleo OBD con datos en vivo. |
| [`profiles/ALFA_MITO_QV_2010.md`](profiles/ALFA_MITO_QV_2010.md) | Perfil del vehículo de referencia y valores por defecto. |
| [`RUN.txt`](RUN.txt) | Manual de uso completo. |

## Roadmap

- Lectura OBD no bloqueante con reconexión automática. *(hecho)*
- Cadencia de sondeo y suavizado recalibrados contra el enlace real. *(hecho)*
- Perfil de vehículo editable desde la PSP.
- Exportación CSV de la sesión con formato largo.
- Sondear en alta cadencia solo lo que la pantalla activa necesita. *(hecho)*
- Soporte de mas de una PSP (PSP‑E1000 sin WiFi, PSP‑Street).
- Perfiles específicos de fabricante, siempre documentados y validados.

## Aviso

Proyecto personal, **no afiliado** a Alfa Romeo, Fiat, Stellantis, Sony,
ELM Electronics ni a ninguna de las marcas o juegos referenciados en los
temas. Las marcas se usan únicamente de forma descriptiva.

No sustituye a la instrumentación del vehículo ni a un diagnóstico
profesional. **No manipules la PSP mientras conduces**: debe ir montada y
manejada por el acompañante.

## Licencia

**GPL‑3.0** — ver [`LICENSE`](LICENSE).

El código fuente de este repositorio está bajo esa licencia. Si redistribuyes
el binario o una versión modificada, publica también el código fuente
correspondiente.

## Desarrollo

Las pruebas del núcleo son la puerta de entrada de cualquier cambio:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_core.ps1
```

Y para compilar la PSP:

```powershell
powershell -ExecutionPolicy Bypass -File tools\build.ps1
```

---

[English version](README_EN.md) · [Documentación](docs/) · [Perfil del vehículo](profiles/ALFA_MITO_QV_2010.md)
