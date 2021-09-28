MACH = cortex-m4
CFLAGS = -c -g -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -o0 -Wall
LDFLAGS= -g -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nano.specs -T linker.ld -Wl,-Map=memory.map

BASEDIR = $(shell pwd)
SRCDIR = $(BASEDIR)/src
DRIVERDIR = $(BASEDIR)/drivers

LIBSRC += $(wildcard $(DRIVERDIR)/src/*.c)
LIBSRC_O = $(patsubst %.c, %.o, $(LIBSRC))
LIBINC += -I $(DRIVERDIR)/inc/ 

SRC += $(wildcard $(SRCDIR)/*.c)
SRC_O = $(patsubst %.c, %.o, $(SRC))
SRC_ELF = $(patsubst %.c, %.elf, $(SRC))

CROSS_COMPILE = arm-none-eabi-
AR = $(CROSS_COMPILE)ar
CC = $(CROSS_COMPILE)gcc
AS = $(CROSS_COMPILE)as
NM = $(CROSS_COMPILE)nm
LD = $(CROSS_COMPILE)gcc
GDB = $(CROSS_COMPILE)gdb
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump

all: $(SRC_ELF)

%.elf: $(LIBSRC_O) %.o startup.o syscalls.o
	$(LD) $(LDFLAGS) -o $@ $^

%.o: %.c 
	$(CC) $(CFLAGS) -o $@ $^ $(LIBINC)

driver_gpio.o: driver_gpio.c
	$(CC) $(CFLAGS) -o $@ $^ $(LIBINC)

driver_spi.o: driver_spi.c
	$(CC) $(CFLAGS) -o $@ $^ $(LIBINC)

driver_i2c.o: driver_i2c.c
	$(CC) $(CFLAGS) -o $@ $^ $(LIBINC)

driver_rcc.o: driver_rcc.c
	$(CC) $(CFLAGS) -o $@ $^ $(LIBINC)

startup.o: startup.c
	$(CC) $(CFLAGS) -o $@ $^

syscalls.o: syscalls.c
	$(CC) $(CFLAGS) -o $@ $^

load:
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

clean:
	rm -rf $(wildcard $(SRCDIR)/*.o)
	rm -rf $(wildcard $(DRIVERDIR)/src/*.o)
	rm -rf $(wildcard *.o)
	rm -rf $(wildcard $(SRCDIR)/*.elf)
	rm -rf $(wildcard *.elf)
	rm -rf $(wildcard *.map)

.PHONY: all load clean
