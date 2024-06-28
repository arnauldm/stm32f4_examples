# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=blinky

CC=arm-none-eabi-gcc

FLAGS  = -mlittle-endian -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

CFLAGS  = -g -Os -Wall
CFLAGS += $(FLAGS)
CFLAGS += -Isrc/ -I./src/include/

LDFLAGS = -Tstm32_flash.ld -nostdlib $(FLAGS)

# Sources files
# C
SRC = $(wildcard src/*.c)
OBJ = $(patsubst %.c, build/%.o, $(notdir $(SRC)))

# Assembly
SSRC = src/startup_stm32f4xx.s
SOBJ = $(patsubst %.s, build/%.o, $(notdir $(SSRC)))

all: build blinky.elf

build:
	mkdir -p $@

build/%.o: src/%.s
	$(CC) -c $(CFLAGS) -o $@ $<

build/%.o: src/%.c
	$(CC) -c $(CFLAGS) -o $@ $<

blinky.elf: $(OBJ) $(SOBJ)
	$(CC) $(LDFLAGS) -o blinky.elf $(SOBJ) $(OBJ)

clean:
	rm -f $(OBJ) $(SOBJ) blinky.elf

flash:
	openocd -f /usr/share/openocd/scripts/board/stm32f4discovery.cfg -f ocd.cfg

