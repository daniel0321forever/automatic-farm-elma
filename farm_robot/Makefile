# Compilers - match root for C++17
CC          := g++ -std=c++17 -Wno-psabi

# Directories and paths
SRCEXT      := cc
CFLAGS      := -ggdb
LIB         := -lelma -lssl -lcrypto -lpthread
INCLUDE     := -I../include
LIBDIR      := -L../lib
ELMALIB     := ../lib/libelma.a

# Targets: main executable links with all sources
TARGETDIR   := ./bin
MODULE_SRCS := $(wildcard modules/vehicle/infra/mock/*.cc modules/vehicle/infra/elintech/*.cc \
                modules/checkpoint_sensor/infra/mock/*.cc modules/checkpoint_sensor/infra/elintech/*.cc \
                modules/apriltag_sensor/*.cc modules/robotarm/*.cc \
                modules/vision/planting/*.cc modules/vision/harvest/*.cc)
APP_SRCS    := application/vehicle_controller.cc application/checkpoint_sensor_controller.cc application/robot_arm_controller.cc application/communicator.cc application/robot_state_machine.cc
CORE_SRCS   := cores/shared_state.cc
FULL_TARGETS := $(TARGETDIR)/main

# Default: build farm_robot binary (main + application + cores + modules)
all: dirs $(FULL_TARGETS)

$(TARGETDIR):
	@mkdir -p $@

dirs: $(TARGETDIR)

# Clean binaries
clean:
	@$(RM) -rf $(TARGETDIR)

# Link main.cc and all sources into bin/main
$(TARGETDIR)/main: main.cc $(MODULE_SRCS) $(APP_SRCS) $(CORE_SRCS) $(ELMALIB)
	$(CC) $(CFLAGS) $(INCLUDE) main.cc $(MODULE_SRCS) $(APP_SRCS) $(CORE_SRCS) $(LIBDIR) $(LIB) -o $@

.PHONY: all dirs clean
