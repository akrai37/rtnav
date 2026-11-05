# Builds the rtnav firmware for the ATmega328P (Arduino Uno) with avr-gcc.
#
#   make            build/rtnav.elf and build/rtnav.hex, then print the footprint
#   make clean
#
# No Arduino core: registers and avr-libc only.

MCU   ?= atmega328p
F_CPU ?= 16000000UL
CC    := avr-gcc
OBJCOPY := avr-objcopy
SIZE  := avr-size

BUILD := build
SRC   := main.c
OBJ   := $(SRC:%.c=$(BUILD)/%.o)

CFLAGS := -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu11 -g \
          -Wall -Wextra -Werror -Wshadow -Wstrict-prototypes -Wmissing-prototypes \
          -ffunction-sections -fdata-sections -I../include
LDFLAGS := -mmcu=$(MCU) -Wl,--gc-sections

.PHONY: all clean size
all: $(BUILD)/rtnav.hex size

$(BUILD):
	mkdir -p $@

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/rtnav.elf: $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@

$(BUILD)/rtnav.hex: $(BUILD)/rtnav.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

size: $(BUILD)/rtnav.elf
	$(SIZE) -C --mcu=$(MCU) $<

clean:
	rm -rf $(BUILD)
