# Development environment

How this app is built, tested and installed on the current development machine
(Windows with WSL). The repository's `tools/build.ps1` drives Docker, which is not
installed here; what follows works without it.

## 1. Toolchain

The official pspdev toolchain is published as a tarball on GitHub. It does not run on
Ubuntu 22.04 (it needs glibc 2.38 and that release ships 2.35), so use an
**Ubuntu-24.04** WSL distribution:

```powershell
wsl --install -d Ubuntu-24.04 --no-launch
```

Inside the distribution (as root; no user account needed):

```sh
apt-get update && apt-get install -y make gcc
cd /root
curl -sL -o pspdev.tar.gz https://github.com/pspdev/pspdev/releases/download/v20261101/pspdev-ubuntu-latest-x86_64.tar.gz
mkdir -p pspdev && tar xzf pspdev.tar.gz -C pspdev --strip-components=1
```

`PSPDEV` is what points the Makefile at the toolchain: it defaults to
`/usr/local/pspdev` and derives `psp/sdk/lib/build.mak` from it.

## 2. Building

The repository lives on the Windows disk, so build from a copy on the Linux filesystem
(faster than `/mnt/c`):

```sh
rm -rf /root/work/psp-torque-fork
cp -r /mnt/c/Users/void/psp/psp-torque-fork /root/work/psp-torque-fork
cd /root/work/psp-torque-fork
export PATH=/root/pspdev/bin:$PATH
make clean PSPDEV=/root/pspdev
make PSPDEV=/root/pspdev
```

- **Zero warnings** under `-Wall -Wextra -std=c99`. A new warning is a failure.
- `make clean` also needs `PSPDEV`: without it the Makefile cannot find `build.mak` and
  fails before doing anything.
- The output is `EBOOT.PBP` in the repository root, with a `PARAM.SFO` generated at
  `MEMSIZE=2` (the pspdev default).

## 3. Core tests

These need no PSP toolchain, only a host compiler:

```sh
make -C tests clean all
```

It prints `OK: todas las pruebas pasan` plus a line with the per-screen demand. Any
change under `src/core/` or to the period table comes with its own test here.

## 4. Testing without a PSP (PPSSPP)

PPSSPP is installed at `C:/Program Files/PPSSPP`. Its memory stick is
`C:/Users/void/Documents/PPSSPP/PSP`, with the same layout as the console (`GAME`,
`SAVEDATA`, `SYSTEM`).

- An EBOOT can be launched directly: `PPSSPPWindows64.exe path\EBOOT.PBP`.
- Keyboard bindings live in `Documents/PPSSPP/PSP/SYSTEM/controls.ini`. Useful codes:
  Cross = 54 (`Z`), Circle = 52 (`X`), Square = 29 (`A`), Triangle = 47 (`S`),
  L = 45 (`Q`), R = 51 (`W`), Start = 62 (space).
- The app's log appears in `Documents/PPSSPP/PSP/SAVEDATA/PSPTORQUE`. That is the cheap
  way to confirm a new screen draws and the log writes, before touching hardware.

## 5. Installing on the PSP

The PSP appears as a removable drive (here `F:`). The app lives at:

```
F:/PSP/GAME/HOMEBREW/psp-torque/EBOOT.PBP
```

Data (log, settings, host file, chime) goes to `F:/PSP/SAVEDATA/PSPTORQUE/`, which the
app creates itself. Nothing else needs copying.

After copying, press `O` on the console to leave USB mode before unplugging.

## 6. Collecting the log

The log is written every second, so it survives even an abrupt shutdown:

```
F:/PSP/SAVEDATA/PSPTORQUE/phase1_AAAAMMDD_HHMMSS.log
```

If the console clock is unset it numbers sequentially instead (`phase1_001.log`,
`phase1_002.log`…). Prefer exiting with `START > SALIR > X`.

## 7. Git

- `konqiDAM/psp-torque` is the fork we work in; `autografia/psp-torque` is the original,
  which will receive a pull request once there is something to review.
- One branch per piece of work. Commit messages are in English from now on, while the
  existing history is Spanish.
- The current K-line branch is `feat/sonda-diagnostico`.
