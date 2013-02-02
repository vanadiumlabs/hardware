# simple makefile for Atmel ATmega8 processor
# Originally from Seattle Robotics
#
# on the command line:
#   make all      - build the program
#   make clean    - remove all built files
#   make program  - download program (using AVRDUDE defines below)
#   make setfuse  - set fuse bits (using AVRDUDE defines below)
#	make size	  - run avr-size on .elf file
#
# stuff you may want to edit:
#
#   PRG         - the name of the program
#   SRC         - the list of C files to be compiled
#   MCU_TARGET  - the microprocessor for which we're building
#
#   AVRDUDE_PART        - AVRDUDE's part id for the microprocessor
#   AVRDUDE_PROGRAMMER  - the programmer used for downloading
#   AVRDUDE_PORT        - the port the programmer is plugged into
#
# revisions:
#   02-07-08	added avr-size option

PRG            = gripper
SRC            = main.c ax_device.c
MCU_TARGET     = atmega328p
F_CPU 		   = 16000000

OBJ            = $(SRC:.c=.o)
OPTIMIZE       = -Os
DEFS           =
LIBS           =

# compile of .c auto with $(CC) -c $(CPPFLAGS) $(CFLAGS)
# compile of .cpp auto with (CXX) -c $(CPPFLAGS) $(CXXFLAGS)
CC             = avr-gcc
CXX            = avr-g++

# Override is only needed by avr-lib build system.
override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override CPPFLAGS      = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
AVRDUDE        = avrdude     # for downloading

AVRDUDE_PART       = m328p
ifndef AVRDUDE_PROGRAMMER
AVRDUDE_PROGRAMMER = avrispmkii
endif
ifndef AVRDUDE_PORT
AVRDUDE_PORT       = /dev/ttyUSB0
endif
AVRDUDE_FLAGS      = -p $(AVRDUDE_PART) -c $(AVRDUDE_PROGRAMMER) -P $(AVRDUDE_PORT) 

# (this is the default rule since it's first)
#all: $(PRG).elf bin
all: $(PRG).elf lst bin

clean:
#	rm -rf *.o $(PRG).elf *.bin
	rm -rf *.o $(PRG).elf *.lst *.map *.bin
	
size:
	avr-size $(PRG).elf

program: $(PRG).bin
	avrdude $(AVRDUDE_FLAGS) -e -U flash:w:$(PRG).bin:r

# make sure fuse bits are set for full swing crystal
setfuse:
	avrdude $(AVRDUDE_FLAGS) -u -U lfuse:w:0xF7:m
#	avrdude $(AVRDUDE_FLAGS) -u -U lfuse:w:0xC2:m
#	avrdude $(AVRDUDE_FLAGS) -u -U hfuse:w:0xDD:m

readeeprom:
	avrdude $(AVRDUDE_FLAGS) -U eeprom:r:eeprom1.hex:i

# binary file (to download to robot)
bin:  $(PRG).bin

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# code/asm listing
lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

