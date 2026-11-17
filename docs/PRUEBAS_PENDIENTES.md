# Pruebas pendientes en hardware

Lista viva. Cada prueba dice que se quiere saber, como se hace, que hay que
mirar en el log y cuando se da por buena. Las que necesitan coche se hacen
con el motor al ralenti salvo que se diga otra cosa.

## Hechas

| # | Prueba | Resultado |
|---|---|---|
| T1 | Sonda completa en el MX-5 | OK: `A3` ISO 9141-2, 13/18 PID, cabecera `48 6B 10` |
| T2 | Busqueda de protocolo con espera de 20 s | OK: `0100` respondio en 4315 ms |
| T3 | Log escrito a la tarjeta cada segundo | OK en `phase1_003` (317 s completos) |

## Pendientes

### T4 - Caidas del enlace (prioridad alta)

**Que se quiere saber:** si las dos caidas de `phase1_003` son del adaptador,
del WiFi de la PSP o del coche, y si la reconexion automatica basta.

**Como:** 15 minutos con el motor en marcha, circulando o al ralenti, sin
tocar la app. Anotar la hora real de cualquier corte visible en pantalla.

**Que mirar en el log:** lineas `t=` con `(offline)`, `Auto-reintento N/3`,
el `rtt` justo antes (383 -> 500 ms es la firma de la primera caida), y
cuantas reconexiones hicieron falta.

**Criterio:** 15 minutos sin caidas, o caidas que se recuperan solas en el
primer intento.

### T5 - Cadencia del RPM con el reparto por capacidad (depende de K2)

**Que se quiere saber:** que el planificador va dimensionado al enlace.

**Como:** ralenti 2 minutos y luego una subida de vueltas mantenida 5 s.

**Que mirar:** segundos entre peticiones `010C` en el log, `rtt` medio, y la
linea de demanda del test del nucleo.

**Criterio:** RPM a >= 1 Hz y los cuatro valores de la pantalla activa a
>= 0,5 Hz, sin que el planificador pida mas de lo que el enlace da.

### T6 - Panel GENERAL sin MAP (depende de K3)

**Criterio:** en marcha, ningun dato fijo en `N/D` en la pantalla GENERAL;
MAF y carga se mueven de forma coherente con el acelerador.

### T7 - Codigos de averia (depende de la pantalla FALLOS)

**Como:** leer PID `01`, luego modo `03`, `07` y `0A`.

**Criterio:** con el testigo apagado la pantalla dice "sin codigos" y no una
lista vacia. Con codigos, coinciden con un lector comercial.

### T8 - Protocolo forzado (opcional)

**Como:** crear `ms0:/PSP/SAVEDATA/PSPTORQUE/protocol.txt` con `3` y luego
con `5`, lanzar la app cada vez.

**Que se quiere saber:** si el coche tambien contesta en KWP2000 (5). Si
contesta, puede haber mas modulos direccionables que en ISO 9141-2.

**Criterio:** el log dice `Protocolo: forzado (ATSPx)` y luego `ATDPN` con el
protocolo forzado, o un fallo limpio sin dejar el adaptador en `STOPPED`.

### T9 - Descubrimiento de modulos en K-line (depende de K4)

**Criterio:** la pantalla MODULOS encuentra la centralita del motor en el
MX-5 y muestra la mascara `BE1FB811`. En el MiTo (CAN) sigue igual que antes.

### T10 - Regresion en el MiTo (cada vez que se toque el nucleo)

**Por que:** la app se calibro para el MiTo y no se puede romper mientras se
anade soporte K-line.

**Como:** con el MiTo y su adaptador, una sesion de 5 minutos.

**Criterio:** mismos PIDs que antes, `BOOST` con signo correcto, sin
`STOPPED`, y el reparto por pantalla funcionando.

### T11 - Salida abrupta

**Como:** lanzar la app, dejarla 30 s, apagar la PSP con el boton de encendido
sin pasar por el menu.

**Criterio:** el log de esa sesion existe y llega hasta el ultimo segundo.
