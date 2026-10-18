CC      ?= cc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Wconversion -Werror
CFLAGS  += -Iinclude

BUILD   := build
PORT    := ports/nucleo_sx1262
MOCKS   := tests/mocks
TESTFLAGS := -Wno-conversion -fsanitize=address,undefined -g
MOCKHAL  := -DFHSS_HAL_HEADER='"mock_hal.h"'

CORE_SRC := src/fhss.c
PORT_SRC := $(PORT)/sx1262.c $(PORT)/fhss_link.c
EMU_SRC  := $(MOCKS)/sx1262_emu.c
CAN_DRV  := drivers/mcp2515
BRIDGE_SRC := src/can_bridge.c src/can_msg.c

.PHONY: all test clean

all: test

$(BUILD)/fhss.o: $(CORE_SRC) include/fhss.h | $(BUILD)
	$(CC) $(CFLAGS) -c $(CORE_SRC) -o $@
	$(CC) $(CFLAGS) -fsyntax-only src/can_msg.c
	$(CC) $(CFLAGS) -fsyntax-only src/can_bridge.c
	$(CC) $(CFLAGS) $(MOCKHAL) -I$(MOCKS) -I$(CAN_DRV) -fsyntax-only $(CAN_DRV)/mcp2515.c

# Port sources under the strict flags, against the mock HAL header.
$(BUILD)/port.stamp: $(PORT_SRC) $(PORT)/*.h $(MOCKS)/mock_hal.h | $(BUILD)
	for f in $(PORT_SRC); do \
		$(CC) $(CFLAGS) $(MOCKHAL) -I$(PORT) -I$(MOCKS) -fsyntax-only $$f || exit 1; \
	done
	touch $@

$(BUILD)/test_fhss: $(CORE_SRC) tests/test_fhss.c include/fhss.h | $(BUILD)
	$(CC) $(CFLAGS) $(TESTFLAGS) $(CORE_SRC) tests/test_fhss.c -o $@

$(BUILD)/test_port: $(CORE_SRC) $(PORT_SRC) $(EMU_SRC) $(BRIDGE_SRC) tests/test_port_sx1262.c \
		$(PORT)/*.h $(MOCKS)/*.h include/can_bridge.h | $(BUILD)
	$(CC) $(CFLAGS) $(TESTFLAGS) $(MOCKHAL) -I$(PORT) -I$(MOCKS) \
		$(CORE_SRC) $(PORT_SRC) $(EMU_SRC) $(BRIDGE_SRC) tests/test_port_sx1262.c -o $@

$(BUILD)/test_can_msg: src/can_msg.c tests/test_can_msg.c include/can_msg.h | $(BUILD)
	$(CC) $(CFLAGS) $(TESTFLAGS) src/can_msg.c tests/test_can_msg.c -o $@

$(BUILD)/test_can_bridge: src/can_bridge.c src/can_msg.c tests/test_can_bridge.c include/can_bridge.h \
		include/can_msg.h | $(BUILD)
	$(CC) $(CFLAGS) $(TESTFLAGS) src/can_bridge.c src/can_msg.c tests/test_can_bridge.c -o $@

$(BUILD)/test_mcp2515: $(CAN_DRV)/mcp2515.c $(CAN_DRV)/mcp2515.h $(MOCKS)/mcp2515_emu.c \
		$(MOCKS)/mcp2515_emu.h tests/test_mcp2515.c | $(BUILD)
	$(CC) $(CFLAGS) $(TESTFLAGS) $(MOCKHAL) -I$(MOCKS) -I$(CAN_DRV) \
		$(CAN_DRV)/mcp2515.c $(MOCKS)/mcp2515_emu.c tests/test_mcp2515.c -o $@

# MATLAB tools compare against sequences from the real C code; fail if the
# checked-in reference has drifted from src/fhss.c
$(BUILD)/matlab_ref.stamp: $(CORE_SRC) include/fhss.h tools/matlab/gen_ref_sequences.c \
		tools/matlab/ref_sequences.csv | $(BUILD)
	$(CC) $(CFLAGS) tools/matlab/gen_ref_sequences.c $(CORE_SRC) -o $(BUILD)/gen_ref
	./$(BUILD)/gen_ref | diff -q - tools/matlab/ref_sequences.csv || \
		(echo "tools/matlab/ref_sequences.csv is stale: ./$(BUILD)/gen_ref > tools/matlab/ref_sequences.csv"; exit 1)
	touch $@

test: $(BUILD)/matlab_ref.stamp $(BUILD)/fhss.o $(BUILD)/port.stamp $(BUILD)/test_fhss $(BUILD)/test_port \
		$(BUILD)/test_can_msg $(BUILD)/test_can_bridge $(BUILD)/test_mcp2515
	./$(BUILD)/test_fhss
	./$(BUILD)/test_port
	./$(BUILD)/test_can_msg
	./$(BUILD)/test_can_bridge
	./$(BUILD)/test_mcp2515

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)
