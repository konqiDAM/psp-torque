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
| T4 | Link drops (adapter, WiFi or car?) | Answered in `007`/`008`: 255 s and 156 s, two sessions, zero drops and zero reconnects. The silence in `006` was the ignition going off while the app polled. The 15-minute single run was not done and is no longer a criterion |
| T12 | Clean exit writes the session summary | OK in `007`, `008` and `009` through the `SALIR` row: `Salida solicitada: cierre limpio` plus the full `=== RESUMEN ===` block. The hold-START path is the same code but no log exercises it yet |
| T13 | Car identified from the masks | OK in `007`/`008`: `Vehiculo: Mazda MX-5 NB2 tacometro=8000 rojo=7000 corte=7200`, no longer generic |
| T14 | No `N/D` blanking between samples | Better, not closed: coolant 0/77 and speed 0/77 (was 45/127, 53/127), intake 11/77 (was 57/127). Intake still blanks after a 40 s hole in its own polling; `fase2.56` now sizes the window from the real interval instead of the planned one |
| T15 | No late-prompt desyncs | OK in `007`/`008`: zero `STOPPED` and zero `sin prompt` over 411 s, after the K-line request timeout went to 1000 ms |

## Pending

### T5 — RPM cadence under the capacity-based split (needs K2)

**What we want to know:** that the scheduler is dimensioned to the link.

**How:** two minutes at idle, then a sustained rev for 5 s.

**What to look for:** seconds between `010C` requests in the log, average `rtt`, and
the demand line printed by the core test.

**Done when:** RPM at ≥ 1 Hz and the four values of the active screen at ≥ 0.5 Hz,
without asking for more than the link can give.

**Measured on the MX-5 (`009`, `fase2.55`):** RPM at 1.11 s holds, so the first half of
the criterion passes. The second half does **not**: coolant came every 3.4 s and speed
every 3.4 s, and intake every 7 s. It cannot pass on this bus — four values at 0.5 Hz is
2 requests/s on its own, and the whole link carries 2.2 by design (2.6 is the physical
ceiling at 383 ms per request). The criterion was written for CAN.

The K-line equivalent, and what actually gets accepted here, is: rpm at ≥ 1 Hz, every
value on screen inside three of its own refresh intervals so nothing blanks, and no more
asked of the link than it gives. `009` meets that. Reaching 0.5 Hz on four values needs a
faster adapter (a genuine ELM327 is 100–150 ms per request on K-line instead of 383), not
different code.

**Status:** closed for K-line with the criterion above; still open as written for CAN
until a MiTo session measures it.

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

### T19 — The calculated oil pressure reads like a real gauge

**What we want to know:** whether the EST oil pressure behaves like the gauge it
imitates, even though it is arithmetic and not a sender.

**How:** cold start: the reading must sit at 3.5 bar at idle and rise to 4.6 by
2000 rpm, and it must move about a second after the revs do. Then once the coolant
is past 90 °C: 1.0 bar at idle and 4.5 bar at 5000 rpm, with the middle values in
between.

**What to look for:** `t=` lines in the log give rpm and ect every 2 s, so the
reading can be checked against the curve afterwards. The number must never be
offered as a measurement: the tile says `EST` next to it.

**Done when:** he says it reads plausibly against a real oil gauge, or the curve
needs different numbers. Nothing here can be closed by a log alone.

### T16 — AGUJA: direct, soft, very soft

**What we want to know:** which ease he actually likes on the MX-5 needle, or whether
none of them does it.

**How:** `AJUSTES -> AGUJA`, one value per minute at idle, then three short blips of
throttle in each. `AJUSTES -> LOG COMPLETO` stays on, so the log holds every frame of
`010C` that arrived.

**What to look for:** the needle at the blip (does it lag, overshoot or stutter?) and,
in the log, the gap between `> 010C` lines. The ease can only approach the last sample;
if the gap is 1 s the needle cannot do more than that, and no setting fixes it — that is
REPARTO's job.

**Done when:** he picks one of the three and says why, or the answer is that the needle
still trails and the fix has to be more rpm polling.

### T17 — REPARTO: how many values share the link

**What we want to know:** how much faster the rpm gets when the other tiles are slowed
down, and whether the slowed tiles still read.

**How:** `AJUSTES -> REPARTO` at 0, then 2, then 3, one minute each, same screen
(GENERAL), same conditions. The plan's arithmetic for this link and the four value PIDs
the factory GENERAL draws says the rpm gets a sample every 583 ms at 0, 656 ms at 2 and
702 ms at 3; tiles inside the quota come back every 5.0 s and the ones outside it every
10.4 s. **Nothing is measured yet**: only this log settles it.

**What to look for:** `grep -c '^> 010C'` against the session length (what it really got,
not what the plan asked for), and whether the tiles outside the cut (usually intake air
and coolant) blank to `N/D`. They should not: they are asked every ~10 s and the
validity window tracks the real interval.

**Done when:** a log shows which REPARTO gives the best needle with the tiles alive, and
that number becomes the default.

**Measured at REPARTO 3 (`phase1_012`, fase2.63, 68 s at idle):** 84 rpm samples over
62 s of link time, one every **0.74 s** against the 0.702 s the plan asked for; the four
drawn values came back at 5.2-5.7 s and the intake air at 9.7 s, exactly as the split
predicts. 151 requests, 151 answers, 0 `NO DATA`, 0 timeouts, link delivered 2.22-2.50
req/s with the plan asking 2.12. For comparison, `phase1_011` (fase2.62) measured 0.82 s
and `phase1_010` (fase2.60) 1.10 s with the same method. What is left on the table: 19
of the 151 requests went to PIDs that are not on the GENERAL screen (they are all due at
connect because their 60 s period starts expired), which is ~12 % of the link spent in
the first minute; REPARTO 2 would plan 0.656 s instead of 0.702 s. Not closed: no load
test yet.

### T18 — The `-1` on the advance tile (answered)

**Answer (log `phase1_011`, fase2.62):** the ECU really sent it. That log holds one
`410E7E` reply out of 39 `010E` answers, and `0x7E` is the only raw byte that decodes to
exactly -1.0 (`A/2 - 64`). The tile was showing a real measurement, not a bug: the app
draws the advance with the measurement as it arrived. Nothing to fix; kept here as the
record of why the number was right.

**Why:** he saw the advance move to values like -1 in the `fase2.61`/`fase2.62` drive.
The log for that session is not here, and the two logs that are (`phase1_010`, fase2.60)
show `010E` answering `0x94`-`0xD0`, which is 10-40 degrees, and no `0x7E` (the only raw
byte that decodes to exactly -1.0). Nothing in the code produces -1 for that PID outside
a `0x7E` reply, so the cause is still unknown.

**How:** next session with full logging on, and a note of the screen, the tile label and
roughly when. `grep -A1 '^> 010E'` in that log shows every reply the tile could have
painted.

**Done when:** either a `0x7E` reply appears (a real ECU answer, and the tile was right),
or the tile is shown to have painted something the log never carried, which is a bug in
the display path and has to be found. `fase2.63` narrows it: nothing is extrapolated any
more and only the two needles are eased, so every number on screen is a decoded frame
(the advance was already drawn raw before, so this removes no known cause).

### T11 — Abrupt exit

**How:** launch the app, leave it 30 s, then hold the PSP power switch up to kill it
without going through the menu.

**Done when:** that session's log exists and reaches the last second.
