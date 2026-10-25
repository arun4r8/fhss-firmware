# NUCLEO-WL55JC + SX1262 shield firmware. Needs only arm-none-eabi-gcc
# (brew install arm-none-eabi-gcc arm-none-eabi-binutils).
#
#   make                         build build/selftest.bin
#   make bridge ROLE=pod         build build/bridge-pod.bin (CAN -> radio)
#   make bridge ROLE=ground      build build/bridge-ground.bin (radio -> console/CAN)
#   make bridge ROLE=pod DEMO=1  build build/bridge-demo.bin (one board, no CAN bus)
#   make flash [APP=bridge ROLE=... DEMO=1]
#                                copy it onto the Nucleo's USB drive (drag-and-drop)
#   make flash-swd [APP=... SERIAL=...]
#                                flash over the ST-LINK debug link with OpenOCD
#                                (brew install open-ocd): works when the USB drive
#                                is blocked or the old firmware won't let go.
#                                SERIAL picks one board when several are plugged in.
#   make monitor                 open the board's serial console (Ctrl-A then K to quit)

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
LDFLAGS := -T stm32wl55jc_cm4.ld -nostdlib -Wl,--gc-sections

COMMON := startup.c board.c hal_shim.c libc_min.c \
          $(ROOT)/src/fhss.c $(ROOT)/ports/nucleo_sx1262/sx1262.c $(ROOT)/ports/nucleo_sx1262/fhss_link.c \
          $(ROOT)/drivers/mcp2515/mcp2515.c
DEPS   := $(COMMON) $(wildcard *.h) $(wildcard libc/*.h) $(wildcard $(ROOT)/include/*.h) \
          $(wildcard $(ROOT)/ports/nucleo_sx1262/*.h) $(ROOT)/drivers/mcp2515/mcp2515.h stm32wl55jc_cm4.ld

ROLE ?= pod
DEMO ?= 0
ifeq ($(ROLE),pod)
  BRIDGE_DEFS := -DBRIDGE_ROLE=1
else ifeq ($(ROLE),ground)
  BRIDGE_DEFS := -DBRIDGE_ROLE=2
else
  $(error ROLE must be pod or ground)
endif
ifeq ($(DEMO),1)
  ifneq ($(ROLE),pod)
    $(error DEMO=1 only applies to ROLE=pod)
  endif
  BRIDGE_DEFS += -DBRIDGE_DEMO=1
  BRIDGE_NAME := bridge-demo
else
  BRIDGE_NAME := bridge-$(ROLE)
endif
BRIDGE_SRCS := bridge.c $(ROOT)/src/can_bridge.c $(ROOT)/src/can_msg.c

APP ?= selftest
FLASH_BIN := $(BUILD)/$(if $(filter bridge,$(APP)),$(BRIDGE_NAME),selftest).bin

.PHONY: all bridge bridges flash flash-swd monitor clean

all: $(BUILD)/selftest.bin

bridge: $(BUILD)/$(BRIDGE_NAME).bin

# every bridge variant (CI)
bridges:
	$(MAKE) bridge ROLE=pod
	$(MAKE) bridge ROLE=ground
	$(MAKE) bridge ROLE=pod DEMO=1

$(BUILD)/selftest.elf: selftest.c $(DEPS) | $(BUILD)
	$(CC) $(CFLAGS) $(COMMON) selftest.c $(LDFLAGS) -Wl,-Map=$(BUILD)/selftest.map -lgcc -o $@
	$(SIZE) $@

# the role is baked in at compile time, so always rebuild
.PHONY: $(BUILD)/$(BRIDGE_NAME).elf
$(BUILD)/$(BRIDGE_NAME).elf: | $(BUILD)
	$(CC) $(CFLAGS) $(BRIDGE_DEFS) $(COMMON) $(BRIDGE_SRCS) $(LDFLAGS) \
		-Wl,-Map=$(BUILD)/$(BRIDGE_NAME).map -lgcc -o $@
	$(SIZE) $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

flash: $(FLASH_BIN)
	@vol=$$(ls -d /Volumes/NOD_* 2>/dev/null | head -1); \
	if [ -z "$$vol" ]; then echo "Nucleo USB drive (/Volumes/NOD_...) not found. Is the board plugged in?"; exit 1; fi; \
	echo "flashing via $$vol"; cp $< "$$vol/" && sync && echo "done: the board resets and runs the new firmware"

flash-swd: $(FLASH_BIN)
	openocd -f interface/stlink.cfg $(if $(SERIAL),-c "adapter serial $(SERIAL)") \
		-f target/stm32wlx.cfg -c "program $(FLASH_BIN:.bin=.elf) verify reset exit"

monitor:
	@port=$$(ls /dev/cu.usbmodem* 2>/dev/null | head -1); \
	if [ -z "$$port" ]; then echo "no /dev/cu.usbmodem* serial port found"; exit 1; fi; \
	screen $$port 115200

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)
