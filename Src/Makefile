#
#
#

# /etc/avrdude.conf :
#programmer
#  id    = "usbasp-clone";
#  desc  = "Any usbasp clone with correct VID/PID";
#  type  = usbasp;
#  usbvid    = 0x16C0; # VOTI
#  usbpid    = 0x05DC; # Obdev's free shared PID
#  #usbvendor  = "VOTI";
#  #usbproduct = "";
#;
# /etc/udev/rules.d/60-UsbAsp.rules
# SUBSYSTEMS=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="05dc", GROUP="users", MODE="0666"
#

FIRMWARE	:=PidController
MCU		:=atmega328p

#Internal RC Osc
#HZ		:=8000000U
#External ceramic resonator
HZ		:=8000000U


#
# Fuse value
# See http://www.engbedded.com/fusecalc
# 8MHz internal osc (Default + CKDIV8=0)

LFUSE		:=0xE2
HFUSE		:=0xD9
EFUSE       :=0xFF


#
# Tool chain
CC		    :=avr-gcc
OBJCOPY		:=avr-objcopy
CFLAGS		:=-Os -D F_CPU=$(HZ)
CFLAGS		+=-g -mmcu=$(MCU) -Wall -Wstrict-prototypes
HEADERS         := $(wildcard *.h)
#SOURCES         := $(wildcard *.c)
SOURCES        := main.c timer0.c i2c.c uart_printf.c ssd1306.c analogIn.c PID.c
OBJECTS        := $(patsubst %.c,%.o,$(SOURCES))

all: $(FIRMWARE).hex


$(FIRMWARE).hex: $(FIRMWARE).elf
	@printf "Generating $(FIRMWARE).hex...\n"
	@$(OBJCOPY) -R .eeprom -O ihex $(FIRMWARE).elf  \
		$(FIRMWARE).hex
	@echo "done."

$(FIRMWARE).elf: $(OBJECTS)
	@printf "Linking    $(FIRMWARE)...\n"
	@$(CC) $(CFLAGS) -o $(FIRMWARE).elf \
		-Wl,-Map,$(FIRMWARE).map $(OBJECTS)
	@echo "done."

.c.o:  $(HEADERS) $(SOURCES)
#	@printf "Compiling  $<...\n"
	$(CC) $(CFLAGS) -Os -c $< -o $@
	@echo "done."

asm: $(FIRMWARE).elf
	@printf "Generating assembler source file...\n"
	@avr-objdump -D -S $(FIRMWARE).elf > $(FIRMWARE).s
	@echo "done."

bin: $(FIRMWARE).elf
	@printf "Generating $(FIRMWARE).hex...\n"
	@$(OBJCOPY) -R .eeprom -O binary $(FIRMWARE).elf  \
		$(FIRMWARE).bin
	@echo "done."
#
# Configuration du programmateur ISP (avrdude)

# Initial fuses reading / programming
#  avrdude -c usbasp-clone -p m328p -P usb -B 3
# Use  -B 3 to reduce bitclock

ISP := -c usbasp-clone -p m328pb -P usb -B 3
#ISP:=-i10 -c ponyser -p m328p -P /dev/ttyS0

# Via programmer
LOADER	:= avrdude -c usbasp-clone -p m328p -P usb -B 3
#Arduino NANO:
#LOADER	:= avrdude -c usbasp-clone -p m328p -P usb -B 3

# Via bootloader
#ARDUINO_LOADER	:= avrdude -c arduino -p m328p -P/dev/ttyS0 -b115200 -D
# Works also at 115200 by  flashing 8MHz bootloader running at 16 Mhz
#ARDUINO_LOADER	:= avrdude -c arduino -p m328p -P/dev/ttyS0 -b57600 -D
#Arduino NANO:
ARDUINO_LOADER	:= avrdude -c arduino -p m328p -P/dev/ttyUSB0 -b57600 -D

LOAD	:=$(LOADER) -i 5 -U flash:w:$(FIRMWARE).hex
DUMP	:=$(LOADER) -i 5 -U flash:r:$(FIRMWARE)_.hex:i
VERIFY	:=$(LOADER) -i 5 -U flash:v:$(FIRMWARE).hex
ERASE	:=$(LOADER) -i 5 -e

#BOOTLOADER_IMAGE := /usr/share/arduino/hardware/arduino/avr/bootloaders/atmega/ATmegaBOOT_168_atmega328.hex
#Bootloader with WATCHDOG_MODS compile option
BOOTLOADER_IMAGE := ../bootloader/ATmegaBOOT_168_atmega328_8.hex


upload:
	@printf "Loading firmware via arduino bootloader ...\n"
	$(ARDUINO_LOADER) -i 5 -U flash:w:$(FIRMWARE).hex
	@echo "done."
flash:
	@printf "Loading firmware...\n"
	$(LOADER) -i 5 -U flash:w:$(FIRMWARE).hex
	@echo "done."
dump:
	@printf "Reading $(MCU) device...\n"
	$(LOADER) -i 5 -U flash:r:$(FIRMWARE)_.hex:i
	@echo "done."
verify:
	@printf "Verify $(MCU) device...\n"
	$(LOADER) -i 5 -U flash:v:$(FIRMWARE).hex
	@echo "done."
erase:
	@printf "Erasing $(MCU) device...\n"
	$(LOADER) -i 5 -e
	@echo "done."
rfuse:
	@printf "Reading fuse...\n"
	$(LOADER)
	@echo "done."
wfuse:
	@printf "Writing fuse...\n"
	$(LOADER) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m
	@echo "done."
wbl:
	@printf "Writing arduino bootloader...\n"
	$(LOADER) -i 5 -U flash:w:$(BOOTLOADER_IMAGE)
	@echo "done."
#
# Clean up
clean :
	@printf "Cleaning source tree...\n"
	@rm -f *.map *.bin *~ *.elf *.gch *.hex *.vcd *.o \
		$(FIRMWARE) $(FIRMWARE).asm
	@echo "done."

