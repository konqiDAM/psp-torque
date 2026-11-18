# Pending hardware tests

A living list. Each test says what we want to learn, how to run it, what to look for in
the log, and when it counts as done. Tests that need the car are done at idle unless
stated otherwise.

## Done

| # | Test | Result |
|---|---|---|
| T1 | Full probe on the MX-5 | OK: `A3` ISO 9141-2, 13/18 PIDs, header `48 6B 10` |
| T2 | Protocol search with the 20 s wait | OK: `0100` answered at 4315 ms |
| T3 | Log written to the card every second | OK in `phase1_003` (317 s complete) |
| T4 | Link drops (adapter, WiFi or car?) | Answered in `007`/`008`: 411 s with zero drops, zero reconnects. The silence in `006` was the ignition going off while the app polled |
| T12 | Clean exit writes the session summary | OK in `007` and `008`: `Salida solicitada: cierre limpio` plus the full `=== RESUMEN ===` block |
| T13 | Car identified from the masks | OK in `007`/`008`: `Vehiculo: Mazda MX-5 NB2 tacometro=8000 rojo=7000 corte=7200`, no longer generic |
| T14 | No `N/D` blanking between samples | OK in `008`: coolant 0/77, speed 0/77, intake 11/77 (was 45/127, 53/127, 57/127) |
| T15 | No late-prompt desyncs | OK in `007`/`008`: zero `STOPPED` and zero `sin prompt` over 411 s, after the K-line request timeout went to 1000 ms |

## Pending

### T5 — RPM cadence under the capacity-based split (needs K2)

**What we want to know:** that the scheduler is dimensioned to the link.

**How:** two minutes at idle, then a sustained rev for 5 s.

**What to look for:** seconds between `010C` requests in the log, average `rtt`, and
the demand line printed by the core test.

**Done when:** RPM at ≥ 1 Hz and the four values of the active screen at ≥ 0.5 Hz,
without asking for more than the link can give.

**Measured on the MX-5 (`008`, `fase2.54`):** RPM at 1.06 s holds. The four values of
the active screen do **not** reach 0.5 Hz — coolant 4.2 s, speed 4.4 s, intake ~9.7 s —
and on this bus they cannot: four values at 0.5 Hz is 2 requests/s plus rpm, and the
whole link carries 2.5. The criterion is a CAN criterion. On K-line the equivalent is
"rpm at ≥ 1 Hz, the values on screen inside their own refresh window, and nothing
blanking", which `008` meets. Reaching 0.5 Hz on this car needs a faster adapter, not
different code.

### T6 — GENERAL panel without MAP (needs K3)

**Done when:** while driving, no value on the GENERAL screen is stuck at `N/D`, and MAF
and load track the throttle.

### T7 — Fault codes (needs the FALLOS screen)

**How:** read PID `01`, then modes `03`, `07` and `0A`.

**Done when:** with the lamp off the screen says "no codes" rather than showing an empty
list; with codes present they match a commercial reader.

### T8 — Forced protocol (optional)

**How:** create `ms0:/PSP/SAVEDATA/PSPTORQUE/protocol.txt` containing `3`, launch, then
repeat with `5`.

**What we want to know:** whether the car also answers on KWP2000 (5). If it does, there
may be more addressable modules than under ISO 9141-2.

**Done when:** the log shows `Protocolo: forzado (ATSPx)` followed by an `ATDPN` of the
forced protocol, or a clean failure that does not leave the adapter in `STOPPED`.

### T9 — K-line module discovery (needs K4)

**Done when:** the MODULOS screen finds the engine ECU on the MX-5 and shows mask
`BE1FB811`, and the MiTo (CAN) still behaves as before.

### T10 — MiTo regression (whenever the core is touched)

**Why:** the app was calibrated on the MiTo and must not be broken while K-line support
goes in.

**How:** with the MiTo and its adapter, a five-minute session.

**Done when:** same PIDs as before, `BOOST` with the right sign, no `STOPPED`, and the
per-screen bandwidth split working.

### T11 — Abrupt exit

**How:** launch the app, leave it 30 s, then hold the PSP power switch up to kill it
without going through the menu.

**Done when:** that session's log exists and reaches the last second.
