CC ?= cc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2 -I../include
LDFLAGS ?= -lm

SRC = test_main.c test_parser.c test_pid.c test_boost.c \
      test_scheduler.c test_samples.c test_discovery.c test_smooth.c \
      test_forced_set.c \
      test_vehicle.c \
      ../src/core/elm327.c ../src/core/pid.c ../src/core/boost.c \
      ../src/core/scheduler.c ../src/core/samples.c ../src/core/profile.c \
      ../src/core/discovery.c ../src/core/smooth.c  \
      ../src/core/vehicle.c

BIN = build/run_tests

all: run

$(BIN): $(SRC) ../include/obd/elm327.h ../include/obd/pid.h \
        ../include/obd/boost.h ../include/obd/scheduler.h \
        ../include/obd/samples.h ../include/obd/profile.h \
        ../include/obd/discovery.h ../include/obd/smooth.h  \
        ../include/obd/vehicle.h
	@mkdir -p build
	$(CC) $(CFLAGS) -o $@ $(SRC) $(LDFLAGS)

run: $(BIN)
	./$(BIN)

clean:
	rm -rf build

.PHONY: all run clean
