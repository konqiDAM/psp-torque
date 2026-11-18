TARGET = PSPTorqueOBD
OBJS = src/psp/main.o \
       src/core/elm327.o \
       src/core/pid.o \
       src/core/boost.o \
       src/core/scheduler.o \
       src/core/samples.o \
       src/core/profile.o \
       src/core/discovery.o \
       src/core/smooth.o \
       src/core/vehicle.o \
       src/psp/gui.o \
       src/psp/gfx.o \
       src/psp/audio.o \
       src/psp/splash_mx5.o

BUILD_PRX = 1
PSP_FW_VERSION = 660

CFLAGS = -O2 -G0 -Wall -Wextra -std=c99 -Iinclude
CXXFLAGS = $(CFLAGS) -fno-exceptions -fno-rtti
ASFLAGS = $(CFLAGS)

LIBS = -lpspgu -lpsppower -lpspaudio -lm

EXTRA_TARGETS = EBOOT.PBP
# Nombre visible del juego en el XMB. El nombre de la carpeta y del modulo
# sigue siendo PSPTorqueOBD para no romper la instalacion existente.
PSP_EBOOT_TITLE = PSP Torque
PSP_EBOOT_ICON = assets/icon0.png

PSPDEV ?= /usr/local/pspdev
PSPSDK ?= $(PSPDEV)/psp/sdk

include $(PSPSDK)/lib/build.mak
