CC      ?= cc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Wconversion -Werror
CFLAGS  += -Iinclude

BUILD   := build

.PHONY: all test clean

all: test

$(BUILD)/test_fhss: src/fhss.c tests/test_fhss.c include/fhss.h | $(BUILD)
	$(CC) $(CFLAGS) -Wno-conversion -fsanitize=address,undefined -g \
		src/fhss.c tests/test_fhss.c -o $@

$(BUILD)/fhss.o: src/fhss.c include/fhss.h | $(BUILD)
	$(CC) $(CFLAGS) -c src/fhss.c -o $@

test: $(BUILD)/fhss.o $(BUILD)/test_fhss
	./$(BUILD)/test_fhss

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)
