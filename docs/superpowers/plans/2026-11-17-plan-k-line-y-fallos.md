# Plan: K-line support and a fault-code screen

- **Date:** 2026-11-17
- **State:** plan of work; everything from K2 on is unstarted
- **Branch:** `feat/sonda-diagnostico` on `09159e1` (fork `konqiDAM/psp-torque`)
- **Measured basis:** `docs/COMPATIBILIDAD_MX5_NB2.md`, `docs/ANALISIS_LOGS.md`
  (sessions `phase1_001` to `phase1_003`), `docs/PRUEBAS_PENDIENTES.md`

## 0. Summary

The second car is a Mazda MX-5 NB2 on ISO 9141-2 with an ELM327 v2.3. The app already
connects and reads 13 PIDs, but the scheduler is dimensioned for a link seven times
faster, the centre of the dashboard shows a boost this car cannot produce, the MODULOS
and SNIFFER screens do not work on K-line, and the app cannot say why the check-engine
lamp is on. This plan closes those four gaps without breaking the MiTo.

Already done and not revisited: the `SONDA DEL COCHE` probe screen, the protocol-search
wait, `protocol.txt`, the per-second log and the `ms0:/PSP/SAVEDATA/PSPTORQUE` data
folder.

## 1. Decisions taken

1. **No invented data.** No reading means `N/D`. No interpolating a needle, no
   estimating a boost the car does not have.
2. **Read-only.** Services `01`, `02`, `03`, `06`, `07`, `09`, `0A` and AT commands.
   Never `04` (clear codes), never `2E`/`2F`/`31`, never an actuator.
3. **Periods are decided by link capacity**, not by what the car "should" deliver. The
   current constant budget (`PROFILE_LINK_BUDGET_REQ_S` = 14) is only valid for CAN.
4. **The panel adapts to the car from the PIDs that exist**, not from a hand-written
   profile: no MAP means MAF and load in the centre.
5. **Screens that do not apply say so**, rather than showing an empty box.
6. **The MiTo is the safety net.** Every core change is validated against it (T10 in
   `docs/PRUEBAS_PENDIENTES.md`).

## 2. Work packages

### K2 — Capacity-based period split (first)

**Why first:** without it the app is a dashboard that refreshes every 4.4 s, and
everything added on top looks broken.

**Starting data:** 383 ms per request, 1.9 req/s measured, 15 active PIDs, RPM every
4.4 s. With a 2.6 req/s ceiling, four live values are one refresh each per 1.5 s.

**Files:** `src/core/profile.c` (`g_base_periods`, `obd_profile_period`),
`include/obd/profile.h` (budget), `src/psp/main.c` (protocol detection in
`scan_refresh_proto`, application in `apply_screen_schedule`), `tests/test_main.c`.

**Proposed design:**
- A budget per link class: CAN (14 req/s, as today) and K-line (~2 req/s). The class
  comes from `ATDPN`: `1–5` is J1850/K-line, `6–9` is CAN.
- On K-line, the base table is replaced by a short one: RPM 700 ms, MAF/load 1000 ms,
  coolant 3000 ms, and **everything off-screen** (trims, O2, advance) at 30 s or
  disabled.
- New test `test_demand_by_link`: for each class, the active screen's demand fits the
  budget, and RPM is always the fastest PID.

**Done when:** the core test passes for both classes; in the car RPM ≥ 1 Hz and the
four values of the active screen ≥ 0.5 Hz with no `STOPPED`; on the MiTo nothing
changes (T10).

**Risk:** someone wants trims visible on a K-line car. Answer: screen priority — on
DIAG the trims come up and everything else drops.

### K3 — GENERAL panel without MAP

**Why:** `BOOST` needs `0B` and `33`; this car has neither, so the centre gauge is dead
and so is the voltmeter (`42` does not exist).

**Files:** `src/psp/main.c` (`draw_qv`, `draw_tacho`, `widget_available`, the default
`g_slot_item`, `boost_learn_update`), `src/core/boost.c` (already reports `N/D`; touch
only if needed), `RUN.txt`, `README.md`.

**Proposed design:**
- If the car does not declare `0B`, the default centre slot becomes MAF (bar) and the
  second becomes load (%). The MAF full scale is learned, as the boost scale already is.
- `TURBO AUTO` is disabled and never stores a maximum boost that does not exist.
- The voltmeter is hidden when `42` is missing; the adapter voltage (`ATRV`) moves to
  the INFO screen, labelled as such.

**Done when:** while driving, no value on GENERAL is stuck at `N/D` and the readings
track the throttle (T6).

### K4 — K-line module discovery

**Why:** `obd_disc_parse_line` only recognises CAN headers (3 or 8 hex digits). The
K-line header is **three bytes** and is already captured:
`48 6B 10 41 00 BE 1F B8 11 AA`.

**Files:** `src/core/discovery.c` (line parser), `include/obd/discovery.h`,
`tests/test_discovery.c`, `src/psp/main.c` (`scan_obd`, `draw_modules`).

**Proposed design:**
- Recognise a three-byte header when the protocol is not CAN: the first three
  two-digit hex tokens.
- If no header is recognised, **search for the `0x41` byte** in the frame and take the
  next four bytes as the mask — which is exactly what the probe does today, and why the
  probe works.
- A unit test with the real MX-5 line returning `BE1FB811`, and another with the usual
  CAN line (`7E8 06 41 00 BE 3E B8 11`) so the MiTo does not regress.
- The MODULOS screen states which protocol it is on and that the UDS scan is CAN-only,
  instead of showing an empty list.

**Done when:** the test passes for both lines and MODULOS finds the ECU on the MX-5 (T9).

### K5 — Link stability

**Why:** two drops in five minutes, the second without recovery. This is what will
annoy most in real use, and it is not fixed by waiting longer.

**Files:** `src/psp/main.c` (`elm_rx_step`, the reconnect block in `run_dashboard`,
`connect_network_and_elm`).

**Work:** before writing code, run T4 for fifteen minutes and read the pattern: does
`rtt` climb before the drop, does the socket die or just go quiet, and does `ATI`
answer after it. That decides between more retries, reopening the AP, or resetting the
adapter with `ATZ`.

**Done when:** fifteen minutes without drops, or drops recovered on the first retry,
with the log naming the cause.

### F5 — FALLOS screen (on-board diagnostics)

**Why:** this is the feature that actually gets used in an older car. Today the app
cannot say why the warning lamp is on.

**Files:** `src/psp/main.c` (a new screen, or inside DIAG), a new core module to decode
codes (`dtc.c`/`dtc.h`) with its test and description table, `RUN.txt`, `README.md`.

**Work, in order:**
1. PID `01`: lamp state and stored code count. One request.
2. Mode `03`: stored codes as `43 01 XX YY …`, decoded to `P0xxx`.
3. Modes `07` and `0A`: pending and permanent codes.
4. Mode `02`: freeze frame for the first code.
5. Mode `06`: on-board monitor results, only if cheap on K-line.

**Done when:** with the lamp off it says "no codes"; with codes present they match a
commercial reader (T7). No path ever sends mode `04`.

## 3. Order and dependencies

```
K2 (periods)  ->  K3 (panel)
     |                  |
     +----> K5 (link)   +----> F5 (faults) -> K4 (modules)
```

K2 goes first because it changes the timings of everything else. K4 is independent and
can be done at any time. F5 needs nothing beyond a screen to live on.

## 4. Working conventions

- New code, comments, documents and commit messages are in **English**. The repository's
  existing Spanish text is left alone; new text in an edited file is English.
- One change = one branch = one commit. Never mix a screen and the core in one commit.
- Zero warnings under `-Wall -Wextra` (see `docs/DESARROLLO.md`).
- Every core change ships with a test in `tests/`.
- Check a changed screen in PPSSPP before flashing the PSP.
- Update `RUN.txt` (user manual) and `README.md`/`README_EN.md` (features) when what the
  user sees changes. Measurements go to `docs/ANALISIS_LOGS.md`; per-car behaviour to
  `docs/COMPATIBILIDAD_*.md`; criteria to `docs/ROADMAP.md`.
- The log is the proof: every acceptance criterion in this plan is demonstrated with a
  log, never with "seems to work".

## 5. Out of scope for this phase

- Writing to the ECU, clearing codes, actuators. None of it.
- Interpolating the needle or estimating a boost the car cannot produce.
- Manufacturer PIDs (mode 22) without verifiable documentation.
- UDS scanning over K-line.
- Replacing the adapter: the ELM327 v2.3 is what we have, and its ceiling is documented.
