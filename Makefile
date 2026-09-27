CC      ?= cc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Wconversion -Werror
CFLAGS  += -Iinclude

BUILD   := build
PORT    := ports/nucleo_sx1262
MOCKS   := tests/mocks
TESTFLAGS := -Wno-conversion -fsanitize=address,undefined -g

CORE_SRC := src/fhss.c
PORT_SRC := $(PORT)/sx1262.c $(PORT)/fhss_link.c
EMU_SRC  := $(MOCKS)/sx1262_emu.c

.PHONY: all test clean

all: test

$(BUILD)/fhss.o: $(CORE_SRC) include/fhss.h | $(BUILD)
	$(CC) $(CFLAGS) -c $(CORE_SRC) -o $@

# Port sources under the strict flags, against the mock HAL header.
$(BUILD)/port.stamp: $(PORT_SRC) $(PORT)/*.h $(MOCKS)/stm32f4xx_hal.h | $(BUILD)
	for f in $(PORT_SRC); do \
		$(CC) $(CFLAGS) -I$(PORT) -I$(MOCKS) -fsyntax-only $$f || exit 1; \
	done
	touch $@

$(BUILD)/test_fhss: $(CORE_SRC) tests/test_fhss.c include/fhss.h | $(BUILD)
	$(CC) $(CFLAGS) $(TESTFLAGS) $(CORE_SRC) tests/test_fhss.c -o $@

$(BUILD)/test_port: $(CORE_SRC) $(PORT_SRC) $(EMU_SRC) tests/test_port_sx1262.c \
		$(PORT)/*.h $(MOCKS)/*.h | $(BUILD)
	$(CC) $(CFLAGS) $(TESTFLAGS) -I$(PORT) -I$(MOCKS) \
		$(CORE_SRC) $(PORT_SRC) $(EMU_SRC) tests/test_port_sx1262.c -o $@

test: $(BUILD)/fhss.o $(BUILD)/port.stamp $(BUILD)/test_fhss $(BUILD)/test_port
	./$(BUILD)/test_fhss
	./$(BUILD)/test_port

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)
