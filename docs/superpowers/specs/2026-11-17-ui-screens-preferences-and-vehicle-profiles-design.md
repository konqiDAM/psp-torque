# Design: screens, preferences and vehicle profiles

- **Date:** 2026-11-17
- **State:** proposed, awaiting approval. No code written.
- **Scope:** navigation model, the settings that persist, and per-car detection
- **Related:** `docs/ROADMAP.md` phase 6, `docs/superpowers/plans/2026-11-17-plan-k-line-y-fallos.md`

## 1. Problems

1. **Eight tabs on a 480 px bar.** GENERAL, MOTOR, DIAG, TELEM, MODULOS, SNIFFER, INFO,
   SONDA. The names barely fit, the live status text now has to shrink to a clock to
   avoid overlapping them, and three of those eight are diagnostic tools nobody uses
   while driving. On a K-line car two of them cannot work at all.
2. **SELECT does exactly nothing of its own.** In `handle_ui_input`, SELECT and START
   are handled in the same branch: both open the settings overlay, both save it when
   closing. A whole button doing a duplicate job.
3. **Preferences are saved, but not the ones that matter per car.** `settings.txt` holds
   theme, colours, language, contrast, animation, peak hold, smoothing, shift light,
   tacho range, units, turbo learning, FPS and the six dashboard slots. It does not hold
   *which screen to open on*, and there is one single layout for all cars: a dashboard
   arranged for the MiTo is overwritten the moment it is arranged for the MX-5.
4. **The app assumes one car.** Centre gauge is boost, tacho tops out where the MiTo
   does, and the whole reference design is a turbo car. On a naturally aspirated car with
   no MAP sensor those choices are wrong and currently there is no way to tell the app
   apart from editing each slot by hand at every launch.

## 2. Goals and non-goals

**Goals**

- A top bar that stays legible and shows only screens that make sense for the current car.
- SELECT gets a distinct, useful job.
- Preferences persist per car, including the screen the app opens on.
- The app works out which car it is in from what the ECU declares, and picks dials and
  defaults that fit — with a manual override that always wins.

**Non-goals**

- No new gauge drawing here; boost → MAF/load is K3 and this spec only decides *which*
  dial appears where.
- One car at a time. Not simultaneous profiles.
- No detection that needs the internet; a local table only.
- No change to the read-only rule or to the polling rules.

## 3. Screen model: two tiers plus a picker

**Tier 1, driving** — cycled with L/R, and the only thing the top bar shows:

| Screen | Purpose |
|---|---|
| GENERAL | The main dashboard |
| MOTOR | Detailed engine page |
| TELEM | Telemetry traces |
| DIAG | Trims, O2, advance, distance, and later fault codes |

**Tier 2, tools** — occasional, not in the top bar:

| Screen | Purpose | Conditions |
|---|---|---|
| MODULOS | ECU discovery | CAN today; K-line after K4 |
| SNIFFER | Bus capture | CAN only |
| SONDA | Vehicle probe | always |
| INFO | Parameter reference | always |

Behaviour:

- L/R cycles tier 1 only. It no longer walks into the diagnostics screens by accident
  while driving, which is a real hazard today.
- When a tier-2 screen is active, the top bar shows the tier-1 tabs replaced by that
  screen's name and a small "TOOLS" marker, so it is obvious where you are and that L/R
  will take you back to driving screens.
- **SELECT opens the screen picker**: every screen in both tiers, each with its name and
  a one-line purpose, current one highlighted, unavailable ones greyed with the reason
  ("SNIFFER — CAN only, this car is ISO 9141-2"). D-pad to move, X or START to open,
  SELECT again to close.
- START keeps its current job: settings overlay, with exit inside it.

**Why a picker rather than icons.** Eight icons at 480 px need art that does not exist
yet and would not be localised; a list with names is readable in sunlight and easy to
extend as screens are added. It also gives the buttons a clean division: L/R is "next
driving screen", SELECT is "go somewhere specific", START is "change how this looks".

**Alternative rejected:** keep eight tabs and shrink the bar. It fixes legibility but
not the accident risk of trigger-cycling past SNIFFER, and it keeps dead screens visible
on a car that cannot use them.

## 4. Preferences

### 4.1 Storage

Keep one file, `ms0:/PSP/SAVEDATA/PSPTORQUE/settings.txt`, and keep the flat
`key=value` format and its parser: it already works, it is inside `main.c` today and
rewriting it is not what this phase is for. Two additions:

- **`version=2`** as the first line, so a later format change has something to migrate
  from. A file without it is version 1 and is read as today.
- **Prefixes for per-vehicle keys**: `v.<profile_id>.<key>=<value>`. Unknown prefixes are
  ignored, so an older build still reads the global keys of a newer file.

Every global key keeps its current name and meaning, so an existing `settings.txt` loads
as the global defaults of every profile.

### 4.2 What gets added

| Key | Values | Meaning |
|---|---|---|
| `start` | `last` or a screen name | Which screen to open on |
| `last` | screen name | Written on exit, used when `start=last` |
| `profile` | `auto` or a profile id | Manual override of the detected car |
| `hidden` | screen names, comma separated | Screens the user chose to hide |
| `v.<id>.slot<n>`, `v.<id>.style<n>` | as today | Dashboard layout, per car |
| `v.<id>.rpm_max`, `v.<id>.redline` | rpm | Tacho range, per car |

The current global slot keys keep working as the default layout for a car that has no
per-vehicle block yet, so nothing is lost on the first run.

### 4.3 Rules

- Changing a slot on a car saves it under that car's profile, never globally.
- `start=last` never resumes into a screen that is unavailable for the car; it falls back
  to GENERAL and says so in the log.
- The picker writes `last` when a screen is opened from it.

## 5. Vehicle profiles

### 5.1 Inputs

After the handshake the app already knows, and already logs: `ATDPN` (the protocol
number), and the three masks `0100`/`0120`/`0140`. That is enough. The probe adds the
per-PID answers, but detection must work without the user running the probe.

### 5.2 Profiles

A const table in `src/core/vehicle.c`:

```c
typedef struct {
    const char *id;          /* "MAZDA_MX5_NB2" */
    const char *name;        /* "Mazda MX-5 NB2" */
    int         proto;       /* 3 = ISO 9141-2, 5 = KWP, 0 = any CAN, -1 = any */
    uint32_t    mask1;       /* 0 = do not care */
    uint32_t    mask2;
    uint32_t    mask3;
    int         rpm_max;     /* tacho default */
    int         redline;     /* red zone default */
    uint8_t     centre;      /* CENTRE_BOOST, CENTRE_MAF, CENTRE_LOAD */
} obd_vehicle_profile;
```

Initial rows:

| id | protocol | masks | gauge scale | red zone | fuel cut | centre |
|---|---|---|---|---|---|---|
| `ALFA_MITO_QV_2010` | CAN | `BE3EB811`/`80022001`/`C0C00000` | 7000 | 6500 | — | BOOST |
| `MAZDA_MX5_NB2` | 3 | `BE1FB811`/`80000000`/— | 8000 | 7000 | 7200 | MAF |
| `GENERIC_CAN` | any CAN | — | 7000 | 6500 | — | from capability |
| `GENERIC_KLINE` | any K-line | — | 7000 | 6500 | — | from capability |

The two generic rows match anything and derive the centre from the declared PIDs rather
than from identity.

### 5.3 Matching

1. A profile whose three masks all match wins, and only if its protocol class matches
   too. This is the same evidence the app already uses to enable the MiTo's sign
   decoding, so it is not new risk.
2. Otherwise, capability only: a generic profile, with the centre gauge chosen from the
   declared PIDs — `MAP` and baro present means BOOST is possible; no `MAP` but `MAF`
   means MAF; neither means LOAD.
3. A manual `profile=` setting always wins, and the log records both what was detected
   and what was forced.

Whatever the outcome, the session log gets one line: `Vehicle: Mazda MX-5 NB2 (detected,
proto A3, masks BE1FB811/80000000/--------)` or `(generic K-line, no profile matched)`.
Without that line, a wrong layout would be unexplainable.

### 5.4 What a profile changes

- The default dashboard layout (which values in which slots, and their style).
- Tacho range and red zone.
- The centre gauge: boost, MAF or load.
- Which screens are offered: SNIFFER only on CAN, MODULOS only where discovery can work.
- Which PID groups are polled, which ties into K2.

What it does not change: units, language, theme, ELM gap. Those are the person's
preferences, not the car's.

## 6. Files

| File | Change |
|---|---|
| `src/core/vehicle.c`, `vehicle.h` | new: profile table, matcher, capability flags |
| `src/core/profile.c` | the MiTo mask constants move to the vehicle table |
| `tests/test_vehicle.c` | new: exact match, generic fallback, override, no false MiTo |
| `src/psp/main.c` | `handle_ui_input` (SELECT), `draw_top_bar` (tiers), new `draw_picker`, `settings_load/save` (version, prefixes, `start`, `last`), `run_dashboard` (opening screen, layout per profile) |
| `RUN.txt`, `README.md`/`README_EN.md` | controls, picker, start screen, per-car layout |
| `docs/COMPATIBILIDAD_MX5_NB2.md` | the detection line and what the MX-5 profile sets |

## 7. Acceptance criteria

- SELECT opens the picker from any screen, and the picker lists every screen with its
  state; choosing one switches immediately and records it as `last`.
- On the MX-5 the picker does not offer SNIFFER and states the reason; the top bar shows
  only driving screens.
- L/R never leaves the driving tier.
- The start screen preference survives a power cycle; `last` resumes, and a screen that
  is no longer available falls back to GENERAL and says so in the log.
- A layout arranged on the MiTo is intact when the MX-5 is used next, and vice versa.
- Detection names each car from its masks, and with `profile=` set by hand that choice
  wins and is logged.
- An MX-5 session never enables the MiTo sign decoding (mask-gated, tested).
- MiTo regression: everything the MiTo did before still works, with the new bar.

## 8. Decisions

Resolved on 2026-11-17:

2. **Start screen: `last`.** The app reopens on the screen it was left on, falling back
   to GENERAL when that screen no longer exists for the current car.
3. **Unavailable screens are greyed in the picker with the reason**, and absent from the
   top bar. Silent removal teaches nothing.
4. **MX-5: red zone at 7000, fuel cut at 7200.** The gauge therefore needs a full scale
   above the cut — 8000 proposed — because the current code derives the red zone as
   `rpm_max - 500`, which cannot express "red at 7000, cut at 7200". The red zone becomes
   an explicit per-profile value instead of a derived one, defaulting to `rpm_max - 500`
   so nothing changes for the MiTo. The MiTo profile keeps its current 7000 scale and
   6500 red zone, and gets no cut value.

Still open:

1. **Top bar with four driving tabs plus the picker** — approve, or keep all eight visible
   and use SELECT only as a fast switcher? Awaiting the screen-by-screen review.
