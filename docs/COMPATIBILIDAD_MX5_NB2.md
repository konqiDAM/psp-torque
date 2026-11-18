# OBD-II compatibility — Mazda MX-5 NB2

The second reference car, after the Alfa Romeo MiTo QV 2010. The MiTo is CAN 11-bit
and is the car everything was calibrated on; the MX-5 is **K-line ISO 9141-2**, which
is a different world in both timing and available PIDs. This document records what was
measured, not what was expected.

## 1. Vehicle and link

| Component | Detail |
|---|---|
| Car | Mazda MX-5 NB2 (second generation, 2001–2005), naturally aspirated petrol |
| Console | PSP-2000/3000 with ARK4 custom firmware |
| Adapter | ELM327 **v2.3** over WiFi (clone), open network `V-LINK` |
| Address | `192.168.0.10:35000` (the app's factory default) |
| Supply | `ATRV` 13.6–13.8 V with the ignition on |

Still to be filled in by the owner: exact model, engine and VIN of the specific car.
Nothing is invented in its place.

## 2. Protocol

Measured, not assumed:

```
ATDPN  -> A3            (3 = ISO 9141-2)
ATDP   -> AUTO, ISO 9141-2
```

And the real header of the reply to `0100`, captured with `ATH1` + `ATS1`:

```
48 6B 10 41 00 BE 1F B8 11 AA
```

That line is the evidence behind the K-line module-discovery bug: the header is **three
bytes** (`48 6B 10`), not a CAN id of 3 or 8 hex digits, and `obd_disc_parse_line` does
not recognise it. See ROADMAP, phase K4.

### 2.1 Link timings

| Quantity | MiTo (CAN) | MX-5 (ISO 9141-2) |
|---|---|---|
| Round trip for one PID | 50–67 ms | **383 ms** (500–516 ms after a drop) |
| Protocol search (`0100` after `ATSP0`) | negligible | **4315 ms** |
| Requests per second, measured | 12–14 | **1.9** |
| Requests per second, ceiling | ~18 | **~2.6** (1/0.383) |

The protocol search is the reason the app did not work on this car: the 3-second
timeout aborted it 1.3 seconds before the adapter answered, which left the ELM327
stuck in `STOPPED` for the rest of the session (876 `STOPPED` replies on the first
attempt). Fixed in phase K1.

## 3. PIDs

Masks declared by the ECU:

```
0100 -> BE 1F B8 11
0120 -> 80 00 00 00
0140 -> not supported
```

Declared PIDs: `01 03 04 05 06 07 0C 0D 0E 0F 10 11 13 14 15 1C 20 21`.

Result of the `SONDA DEL COCHE` probe (18 PIDs, requested one at a time): **13 answer,
5 do not exist**. Values from session `phase1_003` at idle:

| PID | Value | State | Measured |
|---|---|---|---|
| 0C | RPM | yes | 801–820 |
| 0D | vehicle speed | yes | 0 |
| 05 | coolant | yes | 82–86 °C |
| 0F | intake air | yes | 53 °C |
| 10 | MAF | yes | 2.20–2.28 g/s |
| 11 | throttle | yes | 0 % |
| 04 | engine load | yes | 18 % |
| 06 | STFT | yes | +0.00 % |
| 07 | LTFT | yes | +3.13 % |
| 0E | timing advance | yes | +7.5° |
| 14 | O2 bank 1 sensor 1 | yes | 0.11 V |
| 15 | O2 bank 1 sensor 2 | yes | 0.11 V |
| 21 | distance with MIL | yes | 0 km |
| 0B | MAP | **no** | — |
| 33 | barometric pressure | **no** | — |
| 42 | module voltage | **no** | — |
| 5C | oil temperature | **no** | — |
| 2F | fuel level | **no** | — |

Also: `09` (vehicle information) **is not declared**, so VIN, CalID, CVN and ECU name
cannot be read through mode 09 on this car. The probe returning them empty is correct
behaviour, not a fault.

Watch item: 53 °C intake air at idle is high (heat soak in the engine bay). Worth
confirming it falls while driving before treating it as a bad reading.

## 3b. Vehicle profile

Since `fase2.45` the app recognises the car on connect and applies its numbers, so the
MX-5 no longer inherits the MiTo's tacho. The evidence is what the app already reads:
`ATDPN` plus the three masks. The session log carries exactly one line:

```
Vehiculo: Mazda MX-5 NB2 (detectado, proto A3, mascaras BE1FB811/80000000/--------) tacometro=8000 rojo=7000 corte=7200
```

| Profile key | MX-5 NB2 | Where it comes from |
|---|---|---|
| gauge scale | 8000 rpm | the needle keeps climbing past 7000; the useful band ends around 7500 |
| red zone | 7000 rpm | owner's decision, and it matches the car's own tacho |
| fuel cut | 7200 rpm | owner's decision; used for the "about to cut" warning |
| centre gauge | MAF | no `0B`, no `33`: boost cannot exist here, MAF can |

Detection is a mask match on `BE1FB811` with the protocol in the K-line class, so it does
not depend on the probe being run. `profile=MAZDA_MX5_NB2` in `settings.txt` forces it,
and the log then says `forzado`. On any car without a matching profile nothing changes:
the tacho and red zone stay as the person set them.

The centre gauge switch is wired since `fase2.46` (`g_vehicle_centre` chooses between
BOOST, MAF and load, and the GENERAL panel draws whichever it gets), so this car shows
MAF in the middle with or without a profile.

Until `fase2.49` the detection did not work on this car: it ran while the adapter still
answered `ATDPN` with `A0`, and with no protocol class no profile can match, so the log
said `Vehiculo: Generic K-line (generico, proto A0, ...) tacometro=7000 rojo=0 corte=0`
while the very next line already said `ATDPN (conexion): A3`. The protocol is now
reread before the scheduler is built, and detection refuses to latch while it is
unknown. `tests/test_vehicle.c` pins both sides of that border.

## 3c. What the 18/11/2026 session showed

`phase1_006.log`, 198 s, engine idling and warming up, car stationary, `fase2.48`:

```
Vehiculo: Generic K-line (generico, proto A0, mascaras BE1FB811/80000000/00000000)
ATDPN (conexion): A3
Enlace: 2.30 req/s entregadas, pedidas 2.44; plan 2.20 req/s (comprimido)
t=154 rpm=2137 ect=67 iat=27 spd=0 load=31 maf=3.01
```

| Reading | Value over the session |
|---|---|
| rpm | 1002 → 820 idle once warm, blips to 1433 and 2137 |
| coolant | 40 → 67 °C (warming from cold, never reached thermostat) |
| intake air | 23–27 °C |
| MAF | 3.0–3.7 g/s at idle |
| engine load | 12–18 % |
| timing advance | 43.5° |
| throttle | 0 % (the car has `11` but not `49`/`4A`, so this is the plate, not the pedal) |

Three things came out of it:

1. **The profile bug above.** The car answered `4100BE1FB811` exactly as expected and
   still came out generic. Fixed in `fase2.49`.
2. **A silent ECU left the screen frozen.** The last 28 s of the log are 55 consecutive
   `NO DATA` replies while the adapter itself answered every command: the engine was
   switched off and the app kept showing the last good rpm. A NO DATA reply carries
   bytes, so the dead-link counter never fired. Twenty in a row now escalate to the
   reconnect path. If the ignition comes back the app reconnects on its own; if it
   does not, the retries stop after three and `TRIANGLE` reconnects by hand.
3. **No freeze frame on this car.** `0100` bit `02` is clear, so mode 02 does not
   exist: `FALLOS` can list stored, pending and permanent codes but can never show the
   frame that was running when the fault set. The screen now says `SIN MODO 02` rather
   than leaving the row empty, and the app no longer spends five requests asking.

Not yet exercised on this car: the `FALLOS` screen itself (mode 03/07/0A), the gear
indicator (needs the profile, so it read `N/D` all session), and any driving data.

## 4. Design consequences

1. **`BOOST` cannot exist on this car.** `BOOST = MAP − barometric` and neither `0B`
   nor `33` exists. The centre of the GENERAL screen is condemned to `N/D`. What the
   car does have is MAF (g/s) and load (%), which is where the centre of that panel
   should go on a naturally aspirated engine.
2. **The voltmeter is dead too.** Without `42` there is no battery voltage over OBD;
   the only voltage available is the adapter's own (`ATRV`), which is not the car's.
3. **The period table does not apply.** It is calibrated for 12–14 req/s against a
   link that delivers 1.9: the scheduler is oversubscribed about 7×, so every PID
   refreshes far later than requested. Measured: RPM every 4.4 s (36 requests in
   157 s) instead of every 200 ms.
4. **Polling must be few and fast.** With a 2.6 req/s ceiling, four live values means
   one refresh each per 1.5 s. Everything else (trims, O2 sensors, advance) is noise
   while driving.
5. **The MODULOS and SNIFFER screens cannot work here as written.** The UDS scan is
   CAN-only and the header parser does not understand `48 6B 10`.

## 5. Link drops

`phase1_003` dropped the link twice, at `t=157` and `t=233`. On the first, `rtt` rose
from 383 to 500 ms, nine timeouts accumulated, and the automatic reconnect recovered on
the first attempt. On the second, all three attempts failed and the session stayed
`offline` to the end.

It is not yet known whether that is the adapter (the clone's TCP stack), the PSP's WiFi
power saving, or the car. See PRUEBAS_PENDIENTES, T4.

## 6. How to repeat the probe

1. Copy `EBOOT.PBP` to `ms0:/PSP/GAME/HOMEBREW/psp-torque/` (or wherever the app lives).
2. Adapter on, ignition on, engine off or idling.
3. Launch the app and wait for it to connect. The first connection can take up to
   20 s: it is waiting for the adapter to find the protocol.
4. Press `L` until the `SONDA` tab, then `X`. It takes 30–60 s.
5. Exit with `START > SALIR > X` and read
   `ms0:/PSP/SAVEDATA/PSPTORQUE/phase1_*.log`.

## 7. Adapter limits

- The ELM327 v2.3 is a clone. Its 383 ms round trip is more than triple what a genuine
  ELM327 does on K-line (100–150 ms typical). A better adapter would raise the 2.6
  req/s ceiling and ease the cadence problem, but not remove it.
- Mode 01 does not allow more than one PID per request. There is no shortcut.
- The car only speaks ISO 9141-2: there is no CAN bus to listen to.
