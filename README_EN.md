# PSP Torque

**A read-only OBD‑II telemetry dashboard for the PSP.**

Reads real data from your car through a **WiFi ELM327 adapter** and renders it
on a 480×272 graphical dashboard with video-game themes, boost gauges,
telemetry charts and diagnostics.

Developed and validated on an **Alfa Romeo MiTo Quadrifoglio Verde 2010**
(1.4 Turbo MultiAir, 170 hp) with a PSP‑2000/3000 running ARK4 CFW, but the
core is **universal**: it only uses standard OBD‑II (SAE J1979) and adapts to
whatever your ECU actually supports.

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

## Features

- **Read-only, always.** It never writes to the ECU, never clears diagnostic
  trouble codes (DTCs), never codes and never commands actuators.
- **Universal.** It discovers which PIDs exist by reading the real
  `0100`/`0120`/`0140` support masks from the ECU. If a PID is not there, it
  shows `N/D` or `NO SOP.` — never a made-up `0`.
- **Correct boost.** `BOOST = MAP − barometric pressure`. The reference
  pressure comes from PID `0x33`, from a KEY‑ON capture with the engine off,
  or, as a last resort, from an estimate explicitly marked as `EST`.
- **Universal mode without manufacturer PIDs.** It does not use mode 22 or
  proprietary DIDs: they are not universal and they are never invented.
- **7 screens:** `GENERAL`, `MOTOR`, `DIAG`, `TELEM`, `MODULOS`, `SNIFFER`,
  `INFO`.
- **10 themes.** The game themes change the structure of the gauge, the needle
  and the indicators — not just the colours.
- **Telemetry panel** with 5 simultaneous series, collision-free colours and
  a max/min per series.
- **Editable dashboard:** every slot can be `RELOJ` (gauge), `BARRA` (bar),
  `TEXTO` (text) or `OCULTO` (hidden), and you pick which value it shows.
- **ECU module discovery** (OBD‑II and UDS identification) and a **CAN bus
  sniffer** that captures the whole session traffic.
- **Vehicle probe** (`SONDA DEL COCHE`): a diagnostic screen that reads the
  adapter and ECU identity, the PID masks, the raw header and then asks for
  every PID once. It tells you what an unknown car can actually deliver
  instead of tuning the scheduler blind.
- **Demo mode** with a ~130 s synthetic driving profile, so you can try the
  interface without a car.
- **Per-session log** on the Memory Stick, including performance metrics
  (CPU, wait, VBlanks, vertices) and a final session summary.
- **Low latency:** non-blocking OBD reads, only one request in flight, and a
  draw cadence locked to VBlank. Polling **does not** depend on the frame:
  the reply is detected in ~2 ms instead of waiting for the next frame, and
  each PID's rate is calibrated against the link's real capacity, not against
  what the car "should" deliver.

## Requirements

| Component | Detail |
|---|---|
| PSP | PSP‑2000 / PSP‑3000 / PSP‑E1000 with WiFi. CFW (ARK4) is required for network configuration. |
| Adapter | **WiFi** ELM327 (clone `v1.5` validated). It must create its own access point. |
| Network | The PSP joins the adapter's network and talks **TCP** to it. |
| Car | Any vehicle with read-capable OBD‑II. |
| Port | `35000` by default (some clones use `23` or `2000`). |

> **Hardware limitation:** the stock PSP does not support WPA2. The adapter
> must run an open network, WEP or WPA‑PSK/TKIP. With CFW and the `wpa2psp`
> plugin you can use WPA2‑AES, or you can put a travel router/ESP32 in
> between. See [`docs/COMPATIBILIDAD.md`](docs/COMPATIBILIDAD.md).

### Pointing the app at your adapter

The app reads `ms0:/PSP/SAVEDATA/PSPTORQUE/host.txt`. If that file is missing,
it falls back to `192.168.0.10:35000`.

```
# Format: IP PORT
192.168.0.10 35000
```

You can also change the IP and port from the in-app menu.

## Installation

1. Copy the `PSPTorqueOBD` folder to `ms0:/PSP/GAME/PSPTorqueOBD/` on your
   Memory Stick. It should end up as:

   ```
   ms0:/PSP/GAME/PSPTorqueOBD/EBOOT.PBP
   ms0:/PSP/SAVEDATA/PSPTORQUE/              (data folder: created by the app)
   ms0:/PSP/SAVEDATA/PSPTORQUE/host.txt      (optional)
   ```

2. Power on the ELM327 and turn the **ignition on** with the engine **off**.
3. On the PSP: **Game → Memory Stick → PSP Torque**.
4. The app opens the dashboard immediately and, in the background, connects to
   the saved WiFi profile on its own. You will see `CONECTANDO WIFI` and
   `BUSCANDO ELM327` drawn over the dashboard itself.
5. If it does not connect: `TRIANGLE` retries with the saved profile,
   `SQUARE` opens the PSP WiFi dialog.
6. Start the engine and let it idle for 30–60 s.

### Pre-built binary

A **Release** is attached with a prebuilt `EBOOT.PBP`, but it is `fase2.39`:
it has neither the MX-5 profile nor the clean exit nor the FALLOS screen this
page describes. For what is documented here you need to build it (or ask for the
`EBOOT` from the latest session).

## Controls

| Button | Action |
|---|---|
| `L` / `R` / `◀` / `▶` | Change driving screen (GENERAL, MOTOR, DIAG, TELEM) |
| `SELECT` | Open / close the screen picker |
| `START` | Open / close the settings menu |
| `START` (hold 1.2 s) | Quit from any screen, including the ones that run their own loop (sniffer, FALLOS, scan). A running scan is cancelled with `CIRCLE` |
| `TRIANGLE` | Reconnect using the saved WiFi profile |
| `SQUARE` | Open the PSP WiFi dialog and reconnect |
| `X` or `START` | Skip the boot animation · confirm · leave demo mode |
| `L` + `R` (1.5 s) | Toggle **demo mode** |
| `CIRCLE` | Cancel a running scan · clear the learned boost peak |

To quit: hold `START` for 1.2 s, or `START` → scroll down to `SALIR` → `X`. Both
are clean exits: they save the settings and write the session summary at the end
of the log. The PSP HOME button skips all of that, and the log then ends without
a summary.

From a tool screen, `L`/`R` goes back to GENERAL.

## The screen picker

`SELECT` opens a full list of the nine screens with one line of purpose each. Up and
down move, `X` or `START` opens the chosen screen, `SELECT` or `CIRCLE` closes the list.
Screens the car cannot use are greyed with the reason: on a pre-CAN car, MODULOS and
SNIFFER need CAN and stay out of reach.

The bar and `L`/`R` carry only the four driving screens. From a tool the bar shows the
screen name and `L/R: VOLVER`, so the way back is always on screen.

## Where the app starts

It opens on the driving screen it was left on. The choice is written to
`ms0:/PSP/SAVEDATA/PSPTORQUE/settings.txt` as `last=`, once per change and once on exit.
If it was left on a tool screen it opens GENERAL instead.

## The screens

| Screen | Content |
|---|---|
| **GENERAL** | Tachometer, speedometer, boost and ECT / IAT / MAP tiles. On a car with no MAP, the car fills the three slots that would be empty: MAF in the centre, GEAR beside it and TIMING on the other. |
| **MOTOR** | Tile grid: MAP (or MAF), BOOST, ECT, IAT, accelerator, throttle/load, voltage, oil and fuel. Unavailable boxes are compacted away — they leave no gaps. |
| **DIAG** | Advanced read-only diagnostics: STFT, LTFT, timing, O₂ voltages, fuel, voltage, distance with MIL and battery voltage. |
| **TELEM** | A 5-series chart (rpm, boost, timing and throttle by default) plus session max/min values. |
| **MODULOS** | OBD‑II and UDS ECU discovery. Press `X` to scan. Works on K-line cars too, where it reads the OBD‑II masks and skips the CAN-only probe. |
| **FALLOS** | Fault codes: lamp state, stored (03), pending (07) and permanent (0A) codes with their meaning, decoded locally, plus the freeze frame (02) that was stored with the first code. `X` reads, `CIRCLE` clears the screen — never the ECU. When the lamp is on it also shows as a red MIL in the top bar for the rest of the session. |
| **SNIFFER** | Capture of all CAN bus traffic. `X` starts or stops it. |
| **INFO** | What each parameter means, with the standard PID it comes from. |
| **SONDA** | Reads every PID one at a time and reports which ones the ECU answers, which return `NO DATA` and which do not exist. Takes 30–60 s and is cancelled with `CIRCLE`. |

### Settings

`START` opens the menu: `TEMA`, `COLOR`, `IDIOMA` (auto/Spanish/English),
`CONTRASTE`, `ANIMACION`, `MUESTREO`, `EDITAR VISTA`, `PICO MAX`, `SHIFT RPM`,
`RPM MAX`, `UNIDADES` (metric/imperial), `TURBO AUTO`, `BOOST MAX`, `HUECO ELM`,
`PANTALLA INICIAL`, `DEBUG`, `LOG`, `MODO SHOW`, `AGUJA`, `REPARTO` and `SALIR`.

`AGUJA` picks how much the tachometer and speedometer needles are eased
(`DIRECT`, `SOFT` at 150 ms or `VERY SOFT` at 400 ms); numbers and bars always
show the measurement as it arrives. `REPARTO` (0 to 4) decides how many
on-screen values share the fast part of the link: lower means a faster needle.
On a CAN car the split is not used (the base table fits whole), so `REPARTO`
changes nothing on the MiTo.

`LOG` decides what reaches the card: `NO` opens no file at all, `SESION` keeps
the boot lines, the status lines and the final summary, and `COMPLETO` (the
factory value) also keeps every command and every reply. With logging switched
off on purpose the `SIN LOG` warning does not appear: turning it off is a
decision, not a failure.

`MODO SHOW` boots the dashboard with no network and no adapter: it enters demo
mode, changes screen every 12 s and theme every 30 s, and writes neither the
log nor the settings. Any button press holds the dashboard still for a minute
and then it keeps turning. `SELECT` and `START` held for two seconds exit the
app and put a normal boot back on the card; the triggers, `TRIANGLE` and
`SQUARE` do nothing in that mode, so nobody at a show can leave the dashboard
in a menu or asking for data. It is switched on in the settings menu and takes
effect on the next power-on.

They are stored in `ms0:/PSP/SAVEDATA/PSPTORQUE/settings.txt` and applied
immediately.

## Themes

`NORMAL` (with a colour of your choice: red, green, amber, blue or mono) plus
nine game themes: **FORZA**, **NFS**, **GRAN TURISMO**, **INITIAL D**,
**CYBERPUNK**, **RETRO 80**, **NOTHING**, **BACK FUTURE** and **TODO GAS**.

The game themes are not a palette swap: they change the structure of the
gauge (double rings, blocky scales, technical bezel), the shape of the needle
(triangular, double arrow-tipped, red with counterweight), the typography
(italic with halo, bold, retro) and the indicators (chunked bars, pink
stripes, segmented neon bars). Each one also adds its own effect:

| Theme | Effect |
|---|---|
| INITIAL D | Chime warning at 100 km/h (fixed) with a blinking `AVISO`. |
| NFS | Blue tint, side trails and a nitrous bar that follows your throttle. |
| FORZA | A `SKILL` marker with dots that climb while you accelerate hard. |
| GRAN TURISMO | Calculated G-meter and a red warning in the redline zone. |

> **`NOTHING` theme:** dark background, no ornament.

## Demo mode

Hold `L` + `R` for 1.5 seconds. The header shows `DEMO` and the dashboard
displays a simulated car (idle, accelerating to 130 km/h, cruising, pulling
to 185 km/h, then braking). Values are tagged `SIM` in the log.

It needs neither car nor adapter: it is for trying themes, scales, units and
the boost learning. The same combo turns it off.

## Building from source

You need Docker (with the `pspdev/pspdev` image) or a natively installed
PSPSDK.

```bash
# Linux / macOS / Git Bash
MSYS_NO_PATHCONV=1 docker run --rm \
  -v "D:/path/to/project:/src" -w /src \
  pspdev/pspdev \
  bash -lc 'export PATH=/usr/local/pspdev/bin:$PATH; make'
```

```powershell
# Windows
powershell -ExecutionPolicy Bypass -File tools\build.ps1
```

This produces `EBOOT.PBP` in the project root and copies it next to
`host.txt` and `RUN.txt` into `dist\PSPTorqueOBD\`, ready to copy to the
Memory Stick.

#### Fallback without Docker: WSL

If Docker is unavailable (or breaks), the toolchain also works inside a
Linux WSL distribution:

```bash
# 1. Extract the pspdev/pspdev image from the Docker Hub registry
#    (216 MB; no container engine needed) into a directory, say
#    D:/pspdev-root
# 2. Copy the toolchain to the path it expects:
cp -a /mnt/d/pspdev-root/usr/local/pspdev /usr/local/
# 3. The musl loader looks for libraries in its path file:
echo /mnt/d/pspdev-root/usr/lib > /etc/ld-musl-x86_64.path
echo /mnt/d/pspdev-root/lib     >> /etc/ld-musl-x86_64.path
# 4. Build:
sh /mnt/d/psp_torqueOBD/tools/build_wsl.sh
```

`tools/build_wsl.sh` cleans, compiles, copies into `dist/` and prints the
SHA-256.

### Core tests

The portable core has no PSP dependency and is tested on the PC with `gcc`:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_core.ps1
```

220 assertions covering the ELM327 parser, the PID decoder, the boost/baro
calculation, the scheduler, the sample store and module discovery.

### Simulator

`tools/elm327_sim.ps1` starts a TCP server on your PC that speaks the ELM327
protocol with synthetic data. It lets you work on the interface without a car:

```powershell
powershell -ExecutionPolicy Bypass -File tools\elm327_sim.ps1
```

## Project layout

```
.
├── include/obd/     Portable core headers (C99, no PSP dependency)
├── src/core/        ELM327 parser, PIDs, boost/baro, scheduler, samples, discovery, smoothing
├── src/psp/         PSP layer: GU, fonts, input, audio, network, main loop
├── tests/           Core unit tests (native gcc)
├── tools/           Docker build, ELM327 simulator, log reader
├── profiles/        Vehicle profile with its default values
├── docs/            Technical design documentation
├── assets/          EBOOT icon
├── host.txt         ELM327 IP/port target
└── RUN.txt          User manual copied to the Memory Stick
```

## Architecture

```
┌──────────────────────────── PSP ────────────────────────────┐
│  GUI (GU, 480×272, 30/60 fps)                               │
│      ▲ samples                                              │
│  PID Scheduler ──► 1 request in flight ──► ELM327 (TCP)     │
│      │                                   ▲                   │
│  Sample store ◄── decoders ◄──────────────┘                 │
│      │                                                      │
│  Session log ◄─── ELM327 parser ◄─── non-blocking socket    │
└──────────────────────────────────────────────────────────────┘
```

- **Portable C99 core** (`src/core/`): ELM327 protocol parser, standard PID
  decoding, boost and barometric computation, the polling scheduler, the
  sample store and module discovery. Not a single PSP dependency, which is
  exactly why it can be tested on the PC.
- **PSP layer** (`src/psp/`): the GU pipeline, a built-in 8×8 font, input,
  synthesised audio, `sceNetInet`/`sceNetApctl` sockets and the main loop.
- **Non-blocking reads:** the core keeps exactly one OBD request in flight and
  advances its state once per loop iteration. A new command is never sent
  while another one is pending, and the interface never blocks waiting for a
  reply.
- **Polling decoupled from rendering:** while a request is in flight the loop
  waits 2 ms instead of a full VBlank (16.7 ms). That wait used to add
  ~26 ms to every request and capped the link at 12.6 requests/s; now it
  delivers ~18. The draw cadence does not change: it still counts VBlanks.
- **Two adaptive loops set the rate.** The first divides the link budget
  between what the active screen shows and what it does not, and is recomputed
  on every screen change or capacity change. The second measures what the
  adapter actually delivers and raises or lowers that budget every 10 s. On CAN
  the plan targets 16.5 requests/s; on the MX-5's K-line, 2.5 (383 ms per
  request is 2.6 physical). A core test computes the demand and fails if it goes
  over budget.
- **Smoothed needle, nothing invented:** the tachometer and speedometer needles
  move toward the last measurement every frame and stop there: they never
  extrapolate past the last reading or invent a value. Numbers, bars and the
  graph always show the measurement as it arrives. `NEEDLE` in the menu
  picks `DIRECT`, `SOFT` (150 ms) or `VERY SOFT` (400 ms).
- **Panel that follows the car:** on the MX-5, which has no MAP sensor, the
  GENERAL screen replaces the centre tile with a block holding airflow (MAF),
  the **calculated** oil pressure (marked `EST`: that car has no sender) and
  coolant; the MAP tile becomes timing advance. A BOOST or MAP tile on a car
  without that sensor is substituted even with the car's own saved layout; the
  block and the gear arrive with the factory view or through the migration of
  an old card.
- **Link split (`SPLIT`):** how many on-screen values share the fast part of the
  link, from 0 to 4. Every visible value reserves its own floor and the rpm takes
  the rest, so a lower `SPLIT` gives a faster needle; values that are drawn but do
  not make the cut come back every ~10 s instead of blanking to `N/D`. Only what
  is on screen counts, not everything the profile marks visible.
- **The store keeps the measurement as it arrived:** the log, the peaks and the
  graph history are evidence, not presentation. Smoothing lives only in what is
  drawn.
- **VBlank cadence:** drawing is synchronised with `sceDisplayGetVcount()`
  instead of with timers, and the number of VBlanks per frame adapts
  automatically to the CPU load.
- **Memory:** static buffers and tables; no `malloc` in the render loop.

Full details in [`docs/ARQUITECTURA.md`](docs/ARQUITECTURA.md).

## Project rules

These rules are the reason the project exists and they are not negotiable:

1. **Read-only.** No write commands are ever sent to the ECU: no DTC
   clearing, no coding, no actuator commands, no parameter changes.
2. **Never invent readings.** An unavailable PID is shown as `N/D`
   (not available), never as `0`.
3. **Never invent proprietary PIDs, CAN addresses or manufacturer commands.**
   Extended parameters are only added with well-sourced documentation and
   validation on the real vehicle.
4. **Simulated data is always tagged `SIM`.** The simulator is a development
   tool, not a data source presented as real.
5. **`MAP` and `BOOST` are different parameters.** `MAP` is absolute
   pressure (kPa/absolute bar); `BOOST` is `MAP − barometric`. If the
   barometric reading is not reliable, it is marked `EST`.

## Known limitations

- The stock PSP does not support WPA2 (see above).
- Suspension data, target boost pressure, DPF, EGR and other manufacturer
  parameters **are not universal** and are not displayed. The module
  discoverer can detect those ECUs and read their standard identification,
  but not their live data without knowing that specific model's DIDs.
- If your car has a security gateway it may block CAN addresses: the scan
  will only show what the gateway lets through.
- The ignition timing reading (`0x0E`) is skipped when the ECU returns
  physically impossible values under the standard OBD‑II scale.
- The module scan takes 10–30 s on 11-bit CAN and a bit longer on 29-bit
  CAN (it probes 256 physical addresses). You can cancel it with `CIRCLE`.
- Automatic reconnection retries up to 3 times with 2 s between attempts. If
  the adapter does not come back, reconnect manually with `TRIANGLE`.
- The HUD is designed for 480×272 and does not adapt to other resolutions.

## Documentation

| Document | Contents |
|---|---|
| [`docs/ARQUITECTURA.md`](docs/ARQUITECTURA.md) | Layers, concurrency, ELM327 protocol, scheduler, errors and time budget. |
| [`docs/COMPATIBILIDAD.md`](docs/COMPATIBILIDAD.md) | Standard PIDs, formulas, MAP/BOOST, barometric, limits and the policy for manufacturer PIDs. |
| [`docs/UI_QV.md`](docs/UI_QV.md) | QV theme, boot animation, screens, colours, typography and transitions. |
| [`docs/PSPGRAPHICS_MIGRATION.md`](docs/PSPGRAPHICS_MIGRATION.md) | Migrating the text screen to the 2D dashboard. |
| [`docs/ANALISIS_LOGS.md`](docs/ANALISIS_LOGS.md) | Analysis of real in-car sessions: desyncs, bus drops and module by module. |
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | Phases, acceptance criteria and risks. |
| [`docs/FASE0.md`](docs/FASE0.md) / [`docs/FASE1.md`](docs/FASE1.md) | Technical feasibility and the OBD core with live data. |
| [`profiles/ALFA_MITO_QV_2010.md`](profiles/ALFA_MITO_QV_2010.md) | Reference vehicle profile and default values. |
| [`RUN.txt`](RUN.txt) | Full user manual. |

## Roadmap

- Non-blocking OBD reads with automatic reconnection. *(done)*
- Polling rate and smoothing recalibrated against the real link. *(done)*
- Vehicle profile editable from the PSP.
- Session CSV export in long format.
- Poll at high rate only what the visible screen needs. *(done)*
- Support for more PSP models (PSP‑E1000 has no WiFi, PSP‑Street).
- Manufacturer-specific profiles, always documented and validated.

## Disclaimer

Personal project, **not affiliated** with Alfa Romeo, Fiat, Stellantis, Sony,
ELM Electronics or any of the brands or games referenced in the themes. Brand
names are used descriptively only.

This is not a substitute for the car's own instrumentation or for a
professional diagnostic. **Do not operate the PSP while driving:** it must be
mounted and handled by a passenger.

## License

**GPL‑3.0** — see [`LICENSE`](LICENSE).

The source code in this repository is under that license. If you redistribute
the binary or a modified version, publish the corresponding source code too.

## Development

The core tests are the entry gate for any change:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_core.ps1
```

And to build the PSP app:

```powershell
powershell -ExecutionPolicy Bypass -File tools\build.ps1
```

---

[Versión en español](README.md) · [Documentation](docs/) · [Vehicle profile](profiles/ALFA_MITO_QV_2010.md)
