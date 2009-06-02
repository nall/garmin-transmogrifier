# Name: Makefile
# Author: Jon Nall
# Copyright: Copyright (c) 2009, Jon Nall. All rights reserved.
# License: LGPL Version 3

# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

#############################
# Begin configuration section
#
DEVICE     = at90usb647
CLOCK      = 4800000
#
# End configuration section
#############################

PROGRAMMER = -c stk500v2 -P /dev/tty.usbmodemfd13411

OBJDIR     = obj
SRCDIR     = src
SRC        = nmeagen.c main.c usb_host.c serial_disp.c

OBJECTS    = $(patsubst %.c,%.o,$(SRC))
FUSES      = -U hfuse:w:0xff:m -U lfuse:w:0x69:m
# ATTiny13 fuse bits (fuse bits for other devices are different!):
#
# Fuse high byte:
# 0xd9 = 1 1 0 1   1 0 0 1 <-- RSTDISBL (if set to 0, RESET pin is disabled)
#        ^ ^ ^ ^   ^ ^ ^------ BODLEVEL0 
#        | | | |   | +-------- BODLEVEL1
#        | | | |   +---------- DWEN (debugWire Enable)
#        | | | +-------------- SELFPRGEN (Self Program Enable)
#        | | +---------------- UNUSED
#        | +------------------ UNUSED
#        +-------------------- UNUSED
# Fuse low byte:
# 0x24 = 0 0 1 0   0 1 0 0
#        ^ ^ ^ ^   \-/ \-/
#        | | | |    |   +----- CKSEL 1..0 (8M internal RC)
#        | | | |    |+-------- SUT 1..0 (Select startup time)
#        | | | +-------------- CKDIV8 (Divide clock by 8)
#        | | +---------------- WDTON (Watchdog Timer Always On)
#        | +------------------ EESAVE (Preseve EEPROM memory though chip erase)
#        +-------------------- SPIEN (Enable Serial Programming and Data Downloading)

# Example for 12 MHz external crystal:
# Fuse high byte:
# 0xc9 = 1 1 0 0   1 0 0 1 <-- BOOTRST (boot reset vector at 0x0000)
#        ^ ^ ^ ^   ^ ^ ^------ BOOTSZ0
#        | | | |   | +-------- BOOTSZ1
#        | | | |   +---------- EESAVE (set to 0 to preserve EEPROM over chip erase)
#        | | | +-------------- CKOPT (clock option, depends on oscillator type)
#        | | +---------------- SPIEN (if set to 1, serial programming is disabled)
#        | +------------------ WDTON (if set to 0, watchdog is always on)
#        +-------------------- RSTDISBL (if set to 0, RESET pin is disabled)
# Fuse low byte:
# 0x9f = 1 0 0 1   1 1 1 1
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (external >8M crystal)
#        | |  +--------------- SUT 1..0 (crystal osc, BOD enabled)
#        | +------------------ BODEN (if 0, brown-out detector is enabled)
#        +-------------------- BODLEVEL (if 0: 4V, if 1: 2.7V)


# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -std=c99 -g -Wall -Os -Isrc\
	-DF_CPU=$(CLOCK) \
	-mmcu=$(DEVICE)

# symbolic targets:
all: main.hex

$(OBJECTS): %.o: $(SRCDIR)/%.c
	@mkdir -p obj
	$(COMPILE) -std=c99 -c $< -o $(OBJDIR)/$@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	@echo "FIX FUSES"
	#$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID main.hex

clean:
	rm -rf main.hex main.elf main.bin $(OBJDIR)

# file targets:
main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(patsubst %.o,$(OBJDIR)/%.o,$(OBJECTS)) -lm

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-objcopy -j .text -j .data -O binary main.elf main.bin
	ls -l main.bin
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c
