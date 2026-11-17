# Entorno de desarrollo

Como se compila, se prueba y se instala esta app en la maquina de desarrollo
actual (Windows con WSL). El `tools/build.ps1` del repositorio usa Docker, que
en esta maquina no esta instalado: lo de abajo funciona sin Docker.

## 1. Toolchain

El toolchain oficial de pspdev se publica como tar.gz en GitHub. En Ubuntu
22.04 no arranca (necesita glibc 2.38 y alli hay 2.35), asi que se usa una
distro **Ubuntu-24.04** en WSL:

```powershell
wsl --install -d Ubuntu-24.04 --no-launch
```

Dentro de la distro (como root, no hace falta usuario):

```sh
apt-get update && apt-get install -y make gcc
cd /root
curl -sL -o pspdev.tar.gz https://github.com/pspdev/pspdev/releases/download/v20261101/pspdev-ubuntu-latest-x86_64.tar.gz
mkdir -p pspdev && tar xzf pspdev.tar.gz -C pspdev --strip-components=1
```

`PSPDEV` es la variable que define el toolchain: el Makefile usa
`PSPDEV ?= /usr/local/pspdev` y de ahi saca `psp/sdk/lib/build.mak`.

## 2. Compilar

El repositorio esta en el disco de Windows, asi que conviene compilar sobre
una copia en el sistema de ficheros de Linux (mas rapido que `/mnt/c`):

```sh
rm -rf /root/work/psp-torque
cp -r /mnt/c/Users/void/psp/psp-torque-fork /root/work/psp-torque-fork
cd /root/work/psp-torque-fork
export PATH=/root/pspdev/bin:$PATH
make clean PSPDEV=/root/pspdev
make PSPDEV=/root/pspdev
```

- **Cero avisos** con `-Wall -Wextra -std=c99`. Un aviso nuevo es un fallo.
- `make clean` tambien necesita `PSPDEV`: sin el, el Makefile no encuentra
  `build.mak` y falla antes de hacer nada.
- El resultado es `EBOOT.PBP` en la raiz, con un `PARAM.SFO` generado con
  `MEMSIZE=2` (valor por defecto de pspdev).

## 3. Pruebas del nucleo

No necesitan toolchain de PSP, solo un compilador de PC:

```sh
make -C tests clean all
```

Sale `OK: todas las pruebas pasan` y una linea con la demanda de cada
pantalla. Cualquier cambio en `src/core/` o en la tabla de periodos tiene que
ir con su prueba aqui.

## 4. Probar sin PSP (PPSSPP)

PPSSPP esta instalado en `C:/Program Files/PPSSPP`. Su tarjeta de memoria es
`C:/Users/void/Documents/PPSSPP/PSP`, con la misma estructura que la PSP
(`GAME`, `SAVEDATA`, `SYSTEM`).

- Se puede lanzar un EBOOT directamente:
  `PPSSPPWindows64.exe ruta\EBOOT.PBP`.
- Los mandos por teclado estan en
  `Documents/PPSSPP/PSP/SYSTEM/controls.ini`. Codigos utiles: Cruz = 54
  (tecla `Z`), Circulo = 52 (`X`), Cuadrado = 29 (`A`), Triangulo = 47 (`S`),
  L = 45 (`Q`), R = 51 (`W`), Start = 62 (espacio).
- El log de la app aparece en `Documents/PPSSPP/PSP/SAVEDATA/PSPTORQUE`. Es la
  forma barata de comprobar que una pantalla nueva dibuja y que el log escribe,
  antes de tocar hardware.

## 5. Instalar en la PSP

La PSP aparece como unidad extraible (aqui `F:`). La app vive en:

```
F:/PSP/GAME/HOMEBREW/psp-torque/EBOOT.PBP
```

Los datos (log, ajustes, host, chime) van a `F:/PSP/SAVEDATA/PSPTORQUE/`, que
crea la propia app. No hay que copiar nada mas.

Tras copiar, pulsar `O` en la PSP para salir del modo USB antes de
desconectar.

## 6. Recoger el log

El log se escribe cada segundo, asi que existe aunque la app muera de golpe:

```
F:/PSP/SAVEDATA/PSPTORQUE/phase1_AAAAMMDD_HHMMSS.log
```

Si el reloj de la PSP no esta puesto, numera seguido (`phase1_001.log`,
`phase1_002.log`...). Salir siempre que se pueda con `START > SALIR > X`.

## 7. Git

- `konqiDAM/psp-torque` es el fork donde se trabaja; `autografia/psp-torque`
  es el original, al que se abrira un PR cuando haya algo que revisar.
- Una rama por cambio, mensajes en espanol como el resto del historial.
- La rama actual del trabajo K-line es `feat/sonda-diagnostico`.
