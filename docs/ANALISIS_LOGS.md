# Analisis de logs - MiTo QV 2010 (ELM327 WiFi)

Sesiones revisadas: phase1_009.log, phase1_011.log, phase1_013.log
(6-nov-2026 / 7-nov-2026).

## Mascaras reales de la ECU

Coinciden en todas las sesiones:

| Consulta | Respuesta   | Mascara    |
|----------|-------------|------------|
| 0100     | 4100BE3EB811| BE3EB811   |
| 0120     | 412080022001| 80022001   |
| 0140     | 4140C0C00000| C0C00000   |

### PIDs confirmados por la mascara 0140 = C0C00000

- 0x41 monitor status
- 0x42 tension del modulo (responde 12.1 - 14.2 V)
- **0x49 pedal del acelerador D (SOPORTADO)**
- **0x4A pedal del acelerador E (SOPORTADO)**
- 0x5A pedal relativo: NO soportado
- 0x5C temperatura de aceite: NO soportado
- 0x5E caudal de combustible: NO soportado

### Otros PIDs relevantes

- 0x10 MAF: NO soportado (la ECU usa MAP, no caudalimetro).
- 0x2F nivel de combustible: soportado; responde 0xFF (100 %) y
  0xFC (98.8 %). Con el motor parado puede devolver 0.
- 0x33 presion barometrica: soportada; 101-102 kPa.
- 0x11 mariposa: soportada; 12.5 - 89.4 %, ~14.5 % al ralenti.
- 0x0E avance de encendido: soportada segun mascara, pero los valores
  no son fisicamente posibles con la escala OBD-II (salen entre -64 y
  -0.5 grados en todas las condiciones). Se ha dejado de mostrar y de
  sondear hasta poder verificar la escala real de esta ECU.

## Estadisticas de la sesion larga (phase1_011.log)

| PID | Significado            | Min   | Max    |
|-----|------------------------|-------|--------|
| 04  | carga calculada        | 2 %   | 100 %  |
| 05  | refrigerante           | 64 C  | 87 C   |
| 0B  | MAP absoluta           | 20 kPa| 255 kPa|
| 0C  | RPM                    | 0     | 5736   |
| 0D  | velocidad              | 0     | 122 km/h|
| 0F  | temperatura admision   | 36 C  | 51 C   |
| 11  | mariposa               | 12.5 %| 89.4 % |
| 2F  | nivel combustible      | 98.8 %| 100 %  |
| 33  | barometrica            | 101   | 102 kPa|
| 42  | tension                | 12.1 V| 14.2 V |

## Conclusiones

1. El campo que se mostraba como "acelerador" era en realidad la
   **mariposa (0x11)**: por eso se quedaba en ~14 % y solo se movia al
   acelerar. El pedal real de esta ECU es **0x49** (y 0x4A redundante),
   que ahora se sondea y se muestra como ACELERADOR.
2. La mariposa se mantiene visible como MARIPOSA; es normal que marque
   ~15 % al ralenti.
3. El aceite (0x5C) no lo soporta esta ECU: se muestra NO SOP.
4. El caudal de combustible (0x5E) tampoco; no se muestra.
5. Las unidades y formulas del resto de PIDs son correctas.
6. `settings.txt` se corrompia al guardar: `snprintf` devolvia la
   longitud que habria tenido el texto completo (mayor que el buffer de
   192 bytes) y `sceIoWrite` escribia bytes de la pila. Corregido con un
   buffer mayor y limitando la escritura.

## Descubrimiento de modulos (pantalla MODULOS)

El escaneo usa solo servicios de lectura:

- `ATH1`, `ATS1`, `ATAL` para ver cabeceras, separar bytes y permitir
  mensajes ISO-TP largos.
- `0100`, `0120`, `0140` funcionales para listar los modulos OBD-II que
  contestan y sus mascaras.
- `ATSH 7E0` ... `ATSH 7E7` + `3E00` (Tester Present) para detectar
  modulos UDS en CAN de 11 bits.
- `22F190`, `22F197`, `22F187`, `22F188`, `22F18C` para leer VIN,
  nombre del sistema, referencia de pieza, software y numero de serie.
- Al terminar se restauran `ATSH 7DF`, `ATNL`, `ATH0`, `ATS0`, `ATE0` y
  `ATL0`. No se usa `ATD` para no forzar una nueva deteccion de
  protocolo.

En el log aparecen las lineas crudas y un resumen por modulo:

```
MOD 7E8      req=7E0  obd=1 uds=1 m1=BE3EB811 m2=80022001 m3=C0C00000 sistema='MOTOR' pieza='...' sw='...' vin='...' serie='...'
```

Si un modulo no aparece, puede ser porque:

- no habla OBD-II ni UDS estandar;
- esta detras de un gateway de seguridad;
- usa CAN de 29 bits (el sondeo UDS 7E0-7E7 se omite);
- requiere una sesion de diagnostico extendida que la app no abre.

Los valores de suspension, ABS, airbag o carroceria no se pueden mostrar
sin un perfil de fabricante con los DIDs concretos. El escaneo sirve para
identificar el modulo y obtener su referencia; despues se puede preparar
un perfil de lectura especifico con datos reales, sin inventar comandos.

## Renault Clio 1.5 dCi (2024, CAN 29 bits) - phase1_018.log

- ELM327 v1.5, `ATDPN` inicial `A0` y despues `A7` (ISO 15765-4 CAN
  29 bits / 500 kbit).
- Responden dos modulos OBD-II:
  - `18DAF115`: `0100=983B8017`, `0120=A01BA015`, `0140=ECD2809D`
  - `18DAF1E1`: `0100=98180013`, `0120=8001A001`, `0140=88000000`
- PIDs activos en esta sesion: 0C, 0D, 04, 05, 33 y 11 (mariposa).
  No soporta aceite (5C), combustible (5E) ni pedal (49/4A/5A).
- El escaneo de modulos detecto los dos modulos OBD, pero el sondeo UDS
  se omitio porque la version anterior solo lo intentaba en CAN de 11
  bits. Corregido: ahora tambien sondea CAN de 29 bits con
  `18DB33F1` (funcional) y `18DA00F1`-`18DAFFF1` (fisico), leyendo los
  DIDs estandar F190/F197/F187/F188/F18C cuando un modulo responde.
- La direccion de ECU se deriva de la respuesta OBD `18DAF1xx`: la
  peticion fisica es `18DAxxF1`.

## Renault Clio 1.5 dCi 2024 - phase1_020.log

- Tercera sesion con CAN de 29 bits (`ATDPN=A7`). El escaneo UDS
  funcional detecta los dos modulos reales:
  - `18DAF115` -> peticion `18DA15F1`: ECM (motor).
  - `18DAF1E1` -> peticion `18DAE1F1`: UDM (modulo de urea/AdBlue).
- **Fallo detectado y corregido**: en este ELM327 clonado los comandos
  `ATSH 18DA15F1` y `ATSH 18DB33F1` (8 digitos) devuelven `?`. La
  cabecera se quedaba en la funcional por defecto, asi que todos los
  sondeos fisicos 18DA00F1-18DAFFF1 recibian la respuesta de las dos
  ECUs y se marcaban como modulos nuevos. Ademas los textos F190/F197
  se concatenaban entre ambas ECUs.
- Correcciones en fase 2.4:
  - Cabecera de 29 bits compatible: `ATCP 18` + `ATSH DA15F1` (6
    digitos); si el adaptador no lo acepta se prueba el formato de 8.
  - El sondeo fisico solo acepta la respuesta cuya cabecera coincide
    con la ECU pedida (`18DAF1xx`), evitando modulos fantasma.
  - La extraccion de DIDs filtra por cabecera de respuesta, asi que
    cada modulo muestra su VIN, sistema, pieza, software y serie sin
    mezclarse con el otro.
  - El campo `req` del modulo pasa de 8 a 16 bytes para no truncar
    las peticiones de 29 bits.
- Datos reales observados en los DIDs (ya separados):
  - ECM: VIN `VF1RJA*********`, sistema tipo `ECM-EngineControl`.
  - UDM: VIN `VF1RJA*********`, sistema tipo `UDM-UreaDosingModule`.

## MiTo 1.4 170cv - phase1_048.log (11-nov-2026)

Sesion de 3,1 MB / 260 756 lineas, build `fase2.18`, tema NFS.
Protocolo `A7` (CAN 29 bits, 500 kbit), ELM327 v1.5.

- VIN `ZAR955*********`, 4 modulos detectados en el escaneo:
  `18DAF110` (motor, mascaras `BE3EB811 / 80022001 / C0C00000`),
  `18DAF128` y `18DAF130` (solo UDS, pieza), y `18DAF140` (VIN).
- La sesion arranca bien y luego sufre dos caidas de bus:
  1. `NO DATA` y despues ~67 000 lineas `CAN ERROR` entre t=586 s y
     t=2726 s (~36 min). Se recupera sola, sin reconectar, y vuelve a
     dar datos correctos.
  2. Al final (t=3247 s) vuelve el `CAN ERROR`, los 3 auto-reintentos
     fallan y la sesion termina como `INCOMPLETO`.
- Desincronizacion temprana en t=9-16 s: respuestas `STOPPED` y
  desplazadas (p. ej. `ATI` -> `?`). Se recupera al reconectar. Una de
  las reconexiones intermedidas acepta `mascara 0100: 00000000` y solo
  12 PIDs, mostrando `N/D` hasta la siguiente reconexion.

### Estadisticas validas (solo datos decodificados)

| PID | Significado          | Min    | Max    | Media |
|-----|----------------------|--------|--------|-------|
| 04  | carga calculada      | 0 %    | 100 %  | 17.5 %|
| 05  | refrigerante         | 27 C   | 81 C   | 66.5 C|
| 0B  | MAP absoluta         | 20 kPa | 236 kPa| 77.7  |
| 0C  | RPM                  | 0      | 5142   | 2241  |
| 0D  | velocidad            | 0      | 144    | 55.8  |
| 0F  | temperatura admision | 20 C   | 43 C   | 30.2 C|
| 11  | mariposa             | 12.5 % | 89.4 % | 34.7 %|
| 14  | sonda O2 banco 1     | 0.14 V | 0.99 V | 0.55 V|
| 15  | sonda O2 banco 2     | 0.00 V | 0.96 V | 0.38 V|
| 21  | distancia con MIL    | 689 km | 706 km | 698 km|
| 2F  | nivel combustible    | 0 %    | 84.7 % | 80.6 %|
| 33  | barometrica          | 96 kPa | 99 kPa | 97.6  |
| 42  | tension              | 12.0 V | 14.25 V| 14.1 V|

### Fallos corregidos en fase 2.19

1. **Pedal 0x49 nunca sondeado**: el planificador tenia 16 huecos y los
   PIDs avanzados (06, 07, 0E, 14, 15, 21) los agotaban antes de llegar
   al pedal. En el log hay 0 peticiones `0149`. Capacidad subida a 24.
2. **Sondas O2 0x14/0x15 mostraban N/D**: llegaban respuestas validas
   (`4114C400` -> 0.98 V) pero `pid.c` no decodificaba 0x13-0x1B. Anadido
   `A/200 V` (y corregido el tamano esperado de 0x13/0x1B a 1 byte).
3. **CAN ERROR no reconectaba**: solo se contaban las respuestas vacias
   (timeouts). Los `CAN ERROR` tienen bytes y reiniciaban el contador, por
   eso el bus pudo estar 36 min caido. Ahora 40 errores de bus seguidos
   fuerzan reconexion, y el log solo guarda 5 lineas y luego 1 de cada 200
   para no llenar la tarjeta.

### Codificacion no estandar (fase 2.20)

Se aplica **solo al MiTo 1.4 170cv**, detectado por la combinacion exacta
de mascaras `BE3EB811 / 80022001 / C0C00000`; el resto de coches sigue
con la formula OBD estandar.

- **Trims 0x06/0x07**: se interpretan como byte con signo dividido por
  1.28 (`0x00` = 0 %, `0xFB` = -3.9 %, `0x0D` = +10.2 %). Si el valor con
  signo quedase fuera de +/-60 % se conserva la formula OBD, para no
  mostrar un trim absurdo.
- **Avance 0x0E**: se usa `A/2` directamente (0-63.5 grados). El raw
  `0x7F` (y `0xFF`) se trata como "sin dato" y se muestra `N/D`, porque
  aparece en ralenti/retencion donde 63.5 grados no es fisico.
- **Nivel combustible 0x2F**: pendiente. Responde 0 % en algunas fases
  (tambien con el motor en marcha); conviene tratarlo como N/D si llega 0
  de forma aislada.

## Mazda MX-5 NB2 (2003) - phase1_001 a phase1_003 (17-nov-2026)

Primera sesion de la app en un coche K-line (ISO 9141-2) con un ELM327 v2.3.
Las tres sesiones cuentan la historia completa de tres fallos.

### phase1_001: nada funciona, y la culpa es de la app

- Red perfecta: `IP PSP 192.168.0.150`, SSID `V-LINK` abierta, gateway
  `192.168.0.10`, TCP OK, `ATRV` 13.6 V. El adaptador responde a todo en
  ~30 ms.
- `ATDPN` = `A0` en las 6 reconsultas: el adaptador nunca identifico el
  protocolo.
- 303 lineas `SEARCHING...`, todas cortadas a los 3003 ms (handshake) o
  2000 ms (sonda), que son exactamente los timeouts de la app.
- **876 respuestas `STOPPED`** y **0 respuestas `NO DATA`**. Que no haya ni
  un `NO DATA` es la clave: `NO DATA` significa "hable con la ECU y ese PID
  no existe"; cero significa que el adaptador nunca llego a la centralita.
- Sonda: 0/18 PID, mascaras a cero, sin VIN.

Diagnostico: el ELM327 tarda unos 4 s en encontrar el protocolo de un coche
K-line. La app lo abortaba a los 3 s, mandaba el comando siguiente en mitad
de la busqueda y el adaptador contestaba `STOPPED` y se quedaba atascado ahi
el resto de la sesion. Corregido en fase K1 (ver ROADMAP).

### phase1_002: el log se pierde al salir

235 bytes: solo las lineas de arranque. El buffer de 4 KB del log solo se
volcaba al llenarse o en unos pocos eventos, asi que salir de la app sin
pasar por el menu dejaba la sesion entera en RAM. Corregido en fase K1b: el
log se escribe en la tarjeta cada segundo.

### phase1_003: funciona

| Magnitud | Valor |
|---|---|
| Duracion | 317 s (157 s conectado, luego dos caidas) |
| Protocolo | `A3` = ISO 9141-2 (`ATDP`: AUTO, ISO 9141-2) |
| Respuesta a `0100` tras `ATSP0` | `4100BE1FB811` en **4315 ms** |
| Mascaras | `0100`=BE1FB811, `0120`=80000000, `0140` no |
| PIDs activos | 15 |
| Peticiones | 295 en 157 s = **1,9 req/s** |
| Ida y vuelta | **383-384 ms** |
| Cadencia del RPM | **4,4 s** (36 peticiones en 157 s) |
| FPS / CPU | 30-60 fps, 2-5 ms por frame |
| Sonda | 13/18 PID responden |

Valores al ralenti: 804-820 rpm, ECT 82-86 C, IAT 53 C, MAF 2,20-2,28 g/s,
carga 18 %, mariposa 0 %, LTFT +3,13 %, avance +7,5 grados, O2 0,11 V,
MIL apagado. Sin MAP, sin baro, sin tension de modulo, sin aceite y sin
combustible.

El SNR de cada PID a lo largo de la sesion (peticiones hasta t=157):
RPM 36, MAF 31, mariposa 30, velocidad 29, carga 28, refrigerante 23,
admision 21, LTFT 14, STFT 14, O2-2 13, O2-1 13, avance 13, MIL 12.

Consecuencia: el planificador esta saturado unas 7 veces. No es un fallo del
algoritmo, es que los periodos base estan calibrados para un enlace CAN de
12-14 req/s. Corregir en fase K2.

### Caidas del enlace

`t=157`: rtt 383 -> 500 ms, 9 timeouts, reconexion automatica al primer
intento. `t=233`: rtt 516 ms, 21 timeouts y los tres reintentos fallan;
el resto de la sesion queda `offline`. Pendiente de investigar (T4 en
PRUEBAS_PENDIENTES.md).

### Lo que la sonda dio de si

- Cabecera cruda en K-line: `48 6B 10 41 00 BE 1F B8 11 AA`. Los tres
  primeros bytes son la cabecera de ISO 9141-2 y es justo lo que el parser
  de descubrimiento no reconoce.
- Modo 09 vacio y con razon: el PID `09` no esta declarado en la mascara.
