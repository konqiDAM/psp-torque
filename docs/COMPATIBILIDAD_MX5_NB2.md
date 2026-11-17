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
