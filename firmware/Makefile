# ENGR 290 hovercraft firmware (ATmega328P / Arduino Nano, 16 MHz)
#   make          build build/hovercraft.hex
#   make upload   build and flash over USB
#   make monitor  show the debug output (9600 baud), Ctrl+C to exit
#   make clean

MCU     := atmega328p
F_CPU   := 16000000UL
TARGET  := build/hovercraft
SOURCES := main.c imu.c

# Use avr-gcc/avrdude from PATH, or else the copies the Arduino IDE installed.
AVR_DIR  := $(firstword $(wildcard $(HOME)/.arduino15/packages/arduino/tools/avr-gcc/*/bin))
AVR_CC   := $(or $(shell command -v avr-gcc),$(AVR_DIR)/avr-gcc)
OBJCOPY  := $(or $(shell command -v avr-objcopy),$(AVR_DIR)/avr-objcopy)
SIZE     := $(or $(shell command -v avr-size),$(AVR_DIR)/avr-size)
DUDE_DIR := $(firstword $(wildcard $(HOME)/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino*))
AVRDUDE  := $(or $(shell command -v avrdude),$(DUDE_DIR)/bin/avrdude)
DUDE_CONFIG := $(if $(shell command -v avrdude),,-C $(DUDE_DIR)/etc/avrdude.conf)

PORT ?= $(or $(firstword $(wildcard /dev/ttyUSB* /dev/ttyACM*)),/dev/ttyUSB0)
# 57600 = old Nano bootloader; use BAUD=115200 for the new one
BAUD ?= 57600
# matches uart_init() in main.c
MONITOR_BAUD := 9600

OBJECTS := $(SOURCES:%.c=build/%.o)
CFLAGS  := -std=c11 -Os -Wall -Wextra -mmcu=$(MCU) -DF_CPU=$(F_CPU) \
           -ffunction-sections -fdata-sections -MMD -MP
LDFLAGS := -mmcu=$(MCU) -Wl,--gc-sections

.PHONY: all upload monitor clean

all: $(TARGET).hex
	$(SIZE) -C --mcu=$(MCU) $(TARGET).elf

build:
	mkdir -p build

build/%.o: %.c | build
	$(AVR_CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS)
	$(AVR_CC) $(LDFLAGS) $^ -lm -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

upload: all
	$(AVRDUDE) $(DUDE_CONFIG) -p $(MCU) -c arduino -P $(PORT) -b $(BAUD) -D -U flash:w:$(TARGET).hex:i

monitor:
	@test -c "$(PORT)" || { echo "No serial device at $(PORT). Try PORT=/dev/ttyUSB0"; exit 1; }
	@stty -F "$(PORT)" $(MONITOR_BAUD) raw -echo -ixon -ixoff -crtscts
	@echo "Listening on $(PORT) at $(MONITOR_BAUD) baud. Ctrl+C to exit."
	@cat "$(PORT)"

clean:
	$(RM) -r build

-include $(OBJECTS:.o=.d)
