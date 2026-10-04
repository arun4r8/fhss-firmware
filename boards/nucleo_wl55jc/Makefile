# NUCLEO-WL55JC + SX1262 shield firmware. Needs only arm-none-eabi-gcc
# (brew install arm-none-eabi-gcc arm-none-eabi-binutils).
#
#   make            build build/selftest.bin
#   make flash      copy it onto the Nucleo's USB drive (drag-and-drop flashing)
#   make monitor    open the board's serial console (Ctrl-A then K to quit)

PREFIX  ?= arm-none-eabi-
CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

ROOT  := ../..
BUILD := build

CFLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Os -g -std=c99 \
          -Wall -Wextra -Werror -ffreestanding -ffunction-sections -fdata-sections \
          -fno-tree-loop-distribute-patterns \
          -DCORE_CM4 -DSTM32WL55xx -DFHSS_HAL_HEADER='"hal_shim.h"' \
          -I. -Ilibc -I$(ROOT)/include -I$(ROOT)/ports/nucleo_sx1262 -I$(ROOT)/drivers/mcp2515 \
          -I$(ROOT)/third_party/cmsis_core -I$(ROOT)/third_party/cmsis_device_wl
LDFLAGS := -T stm32wl55jc_cm4.ld -nostdlib -Wl,--gc-sections -Wl,-Map=$(BUILD)/selftest.map

SRCS := startup.c board.c hal_shim.c libc_min.c selftest.c \
        $(ROOT)/src/fhss.c $(ROOT)/ports/nucleo_sx1262/sx1262.c $(ROOT)/ports/nucleo_sx1262/fhss_link.c \
        $(ROOT)/drivers/mcp2515/mcp2515.c

.PHONY: all flash monitor clean

all: $(BUILD)/selftest.bin

$(BUILD)/selftest.elf: $(SRCS) $(wildcard *.h) $(wildcard libc/*.h) stm32wl55jc_cm4.ld | $(BUILD)
	$(CC) $(CFLAGS) $(SRCS) $(LDFLAGS) -lgcc -o $@
	$(SIZE) $@

$(BUILD)/selftest.bin: $(BUILD)/selftest.elf
	$(OBJCOPY) -O binary $< $@

flash: $(BUILD)/selftest.bin
	@vol=$$(ls -d /Volumes/NOD_* 2>/dev/null | head -1); \
	if [ -z "$$vol" ]; then echo "Nucleo USB drive (/Volumes/NOD_...) not found. Is the board plugged in?"; exit 1; fi; \
	echo "flashing via $$vol"; cp $< "$$vol/" && sync && echo "done: the board resets and runs the new firmware"

monitor:
	@port=$$(ls /dev/cu.usbmodem* 2>/dev/null | head -1); \
	if [ -z "$$port" ]; then echo "no /dev/cu.usbmodem* serial port found"; exit 1; fi; \
	screen $$port 115200

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)
