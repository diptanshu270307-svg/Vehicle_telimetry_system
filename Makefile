# Telemetry & Anomaly Reconstruction Engine
# Pure C11 — gcc, standard library + POSIX (Winsock on Windows). No third-party libs.
#
#   make / mingw32-make          build engine
#   make demo                    one-command hardware-free auto-demo + browser dashboard
#   make run                     demo collision at t=8s
#   make test                    unit + CSV replay + benchmark tests
#   make clean                   remove build artifacts

CC      := gcc
CFLAGS  ?= -Wall -Wextra -std=c11 -O2 -g
LIBS    ?= -lm

ifeq ($(OS),Windows_NT)
  LIBS   += -lws2_32
  EXE    := .exe
  DEVNULL := NUL
else
  EXE    :=
  DEVNULL := /dev/null
endif

SRC_DIR  := src
TEST_DIR := tests
BUILD    := build

ENGINE_SRCS := \
  $(SRC_DIR)/hal_sim.c \
  $(SRC_DIR)/circular_queue.c \
  $(SRC_DIR)/linked_list.c \
  $(SRC_DIR)/detector.c \
  $(SRC_DIR)/state_machine.c \
  $(SRC_DIR)/security.c \
  $(SRC_DIR)/modem_at.c \
  $(SRC_DIR)/publisher.c \
  $(SRC_DIR)/simulator.c \
  $(SRC_DIR)/http_server.c \
  $(SRC_DIR)/main.c

ENGINE_OBJS := \
  $(BUILD)/hal_sim.o \
  $(BUILD)/circular_queue.o \
  $(BUILD)/linked_list.o \
  $(BUILD)/detector.o \
  $(BUILD)/state_machine.o \
  $(BUILD)/security.o \
  $(BUILD)/modem_at.o \
  $(BUILD)/publisher.o \
  $(BUILD)/simulator.o \
  $(BUILD)/http_server.o \
  $(BUILD)/main.o

.PHONY: all run demo test clean dirs sync-esp32

all: dirs $(BUILD)/engine$(EXE)

dirs:
ifeq ($(OS),Windows_NT)
	-@if not exist $(BUILD) mkdir $(BUILD)
	-@if not exist data mkdir data
else
	@mkdir -p $(BUILD) data
endif

$(BUILD)/%.o: $(SRC_DIR)/%.c | dirs
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/engine$(EXE): $(ENGINE_OBJS)
	$(CC) $(CFLAGS) -o $@ $(ENGINE_OBJS) $(LIBS)

run: all
	./$(BUILD)/engine$(EXE) --scenario collision --at 8

demo: all
ifeq ($(OS),Windows_NT)
	./$(BUILD)/engine$(EXE) --auto-demo --seed 42
else
	sh ./run_demo.sh
endif

$(BUILD)/test_circular_queue$(EXE): $(TEST_DIR)/test_circular_queue.c $(SRC_DIR)/circular_queue.c | dirs
	$(CC) $(CFLAGS) -o $@ $(TEST_DIR)/test_circular_queue.c $(SRC_DIR)/circular_queue.c $(LIBS)

$(BUILD)/test_linked_list$(EXE): $(TEST_DIR)/test_linked_list.c $(SRC_DIR)/linked_list.c $(SRC_DIR)/publisher.c $(SRC_DIR)/modem_at.c $(SRC_DIR)/security.c $(SRC_DIR)/detector.c $(SRC_DIR)/hal_sim.c | dirs
	$(CC) $(CFLAGS) -o $@ $(TEST_DIR)/test_linked_list.c $(SRC_DIR)/linked_list.c $(SRC_DIR)/publisher.c $(SRC_DIR)/modem_at.c $(SRC_DIR)/security.c $(SRC_DIR)/detector.c $(SRC_DIR)/hal_sim.c $(LIBS)

$(BUILD)/test_detector$(EXE): $(TEST_DIR)/test_detector.c $(SRC_DIR)/detector.c $(SRC_DIR)/hal_sim.c | dirs
	$(CC) $(CFLAGS) -o $@ $(TEST_DIR)/test_detector.c $(SRC_DIR)/detector.c $(SRC_DIR)/hal_sim.c $(LIBS)

$(BUILD)/test_replay$(EXE): $(TEST_DIR)/test_replay.c $(SRC_DIR)/detector.c $(SRC_DIR)/hal_sim.c | dirs
	$(CC) $(CFLAGS) -o $@ $(TEST_DIR)/test_replay.c $(SRC_DIR)/detector.c $(SRC_DIR)/hal_sim.c $(LIBS)

$(BUILD)/test_state_machine$(EXE): $(TEST_DIR)/test_state_machine.c $(SRC_DIR)/state_machine.c $(SRC_DIR)/detector.c $(SRC_DIR)/hal_sim.c | dirs
	$(CC) $(CFLAGS) -o $@ $(TEST_DIR)/test_state_machine.c $(SRC_DIR)/state_machine.c $(SRC_DIR)/detector.c $(SRC_DIR)/hal_sim.c $(LIBS)

$(BUILD)/test_security_modem$(EXE): $(TEST_DIR)/test_security_modem.c $(SRC_DIR)/security.c $(SRC_DIR)/modem_at.c $(SRC_DIR)/detector.c $(SRC_DIR)/hal_sim.c | dirs
	$(CC) $(CFLAGS) -o $@ $(TEST_DIR)/test_security_modem.c $(SRC_DIR)/security.c $(SRC_DIR)/modem_at.c $(SRC_DIR)/detector.c $(SRC_DIR)/hal_sim.c $(LIBS)

$(BUILD)/test_benchmark$(EXE): $(TEST_DIR)/test_benchmark.c $(SRC_DIR)/circular_queue.c $(SRC_DIR)/linked_list.c $(SRC_DIR)/detector.c $(SRC_DIR)/security.c $(SRC_DIR)/hal_sim.c | dirs
	$(CC) $(CFLAGS) -o $@ $(TEST_DIR)/test_benchmark.c $(SRC_DIR)/circular_queue.c $(SRC_DIR)/linked_list.c $(SRC_DIR)/detector.c $(SRC_DIR)/security.c $(SRC_DIR)/hal_sim.c $(LIBS)

test: $(BUILD)/test_circular_queue$(EXE) $(BUILD)/test_linked_list$(EXE) $(BUILD)/test_detector$(EXE) $(BUILD)/test_replay$(EXE) $(BUILD)/test_state_machine$(EXE) $(BUILD)/test_security_modem$(EXE) $(BUILD)/test_benchmark$(EXE)
	./$(BUILD)/test_circular_queue$(EXE)
	./$(BUILD)/test_linked_list$(EXE)
	./$(BUILD)/test_detector$(EXE)
	./$(BUILD)/test_replay$(EXE)
	./$(BUILD)/test_state_machine$(EXE)
	./$(BUILD)/test_security_modem$(EXE)
	./$(BUILD)/test_benchmark$(EXE)

clean:
ifeq ($(OS),Windows_NT)
	-@if exist $(BUILD) rmdir /S /Q $(BUILD)
else
	@rm -rf $(BUILD)
endif
	@echo cleaned

sync-esp32:
	@cp $(SRC_DIR)/config.h $(SRC_DIR)/hal.h $(SRC_DIR)/structs.h \
	    $(SRC_DIR)/circular_queue.h $(SRC_DIR)/circular_queue.c \
	    $(SRC_DIR)/linked_list.h $(SRC_DIR)/linked_list.c \
	    $(SRC_DIR)/detector.h $(SRC_DIR)/detector.c \
	    $(SRC_DIR)/state_machine.h $(SRC_DIR)/state_machine.c \
	    $(SRC_DIR)/publisher.h $(SRC_DIR)/publisher.c \
	    $(SRC_DIR)/security.h $(SRC_DIR)/security.c \
	    $(SRC_DIR)/modem_at.h $(SRC_DIR)/modem_at.c \
	    esp32_firmware/main_car_controller/
	@echo "Synced C11 core into esp32_firmware/main_car_controller/"
