
GCC_PREFIX = arm-none-eabi
CC = gcc 
OBJECT = $(APP).elf
BOARD = cc2538
CORE = cortex-m3
LINKER_SCRIPT =  $(BOARD).ld
BUILD = $(APP).bin
FLASHER = ubimote-bsl
DEV = /dev/ttyUSB0
OPTIMIZATION = -O2
BASE = ../../../sdk
ALL_PATHS = -I$(BASE)/bsp/ -I$(BASE)/include -I$(BASE)/drivers/ -I$(BASE)/rf/ -I$(BASE)/utils/
CFLAGS = -mthumb -mlittle-endian -fshort-enums -fomit-frame-pointer -fno-strict-aliasing -ffunction-sections -fdata-sections -std=c99 -Wpointer-sign
LFLAGS = -mthumb -nostartfiles -T cc2538.ld -Wl,-Map=main_cc2538dk.map,--cref,--no-warn-mismatch -Wl,--gc-sections
MAKEFLAGS = --silent
include Makefile.app

all:
	$(GCC_PREFIX)-cpp -imacros "$(BASE)/linker_script_def.h" -P -E $(BASE)/$(BOARD).lds -o $(LINKER_SCRIPT)
	$(GCC_PREFIX)-$(CC) $(OPTIMIZATION) -mcpu=$(CORE) $(CFLAGS) -c $(BASE)/startup_gcc.c -o $(BASE)/startup_gcc.so
	$(GCC_PREFIX)-$(CC) $(OPTIMIZATION) -mcpu=$(CORE) $(CFLAGS) -I. $(ALL_PATHS) -c $(BASE)/drivers/*.c
	$(GCC_PREFIX)-$(CC) $(OPTIMIZATION) -mcpu=$(CORE) $(CFLAGS) -I. $(ALL_PATHS) -c $(BASE)/bsp/*.c
	$(GCC_PREFIX)-$(CC) $(OPTIMIZATION) -mcpu=$(CORE) $(CFLAGS) -I. $(ALL_PATHS) -c $(BASE)/rf/*.c
	$(GCC_PREFIX)-$(CC) $(OPTIMIZATION) -mcpu=$(CORE) $(CFLAGS) -I. $(ALL_PATHS) -c $(BASE)/utils/*.c
	$(GCC_PREFIX)-$(CC) $(OPTIMIZATION) -mcpu=$(CORE) $(CFLAGS) -I. $(ALL_PATHS) -c *.c
	$(GCC_PREFIX)-$(CC) $(OPTIMIZATION) -mcpu=$(CORE) $(LFLAGS) $(BASE)/startup_gcc.so *.o -o $(OBJECT)
	$(GCC_PREFIX)-objcopy -O binary --gap-fill 0xff $(OBJECT) $(BUILD)
	-@rm -f Makefile.app
	@echo "saving app"
	@echo >Makefile.app "APP = $(APP)"
	@echo "successfully builded"
	-@rm -f *.o *.co *.ld *.map

flash:
	$(BASE)/$(FLASHER) -p $(DEV) -e -w -v $(BUILD) 
erase:
	$(FLASHER) -p $(DEV) -e
clean: 
	rm -f *.ld *.co *.o *.elf *.map *.bin

