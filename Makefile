MACH = cortex-m4
CFLAGS = -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -o0 -Wall
LDFLAGS= -mcpu=$(MACH) -mthumb -mfloat-abi=soft -nostdlib -T linker.ld -Wl,-Map=memory.map

BASEDIR = $(shell pwd)
SRCDIR = $(BASEDIR)/src
DRIVERDIR = $(BASEDIR)/drivers

SRCALL += $(wildcard $(SRCDIR)/*.c)
SRCALL_O = $(patsubst %.c, %.o, $(SRC))
SRCGPIO = $(SRCDIR)/gpio_led_toggle.c
SRCGPIO_O = $(patsubst %.c, %.o, $(SRCGPIO))

LIBSRC += $(wildcard $(DRIVERDIR)/src/*.c)
LIBSRC_O = $(patsubst %.c, %.o, $(LIBSRC))
LIBINC += -I $(DRIVERDIR)/inc/ 

CROSS_COMPILE = arm-none-eabi-
AR = $(CROSS_COMPILE)ar
CC = $(CROSS_COMPILE)gcc
AS = $(CROSS_COMPILE)as
NM = $(CROSS_COMPILE)nm
LD = $(CROSS_COMPILE)gcc
GDB = $(CROSS_COMPILE)gdb
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump

GPIO: gpio_led_toggle.elf

gpio_led_toggle.elf: $(LIBSRC_O) $(SRCGPIO_O) startup.o
	$(LD) $(LDFLAGS) -o $@ $^

$(LIBSRC_O): $(LIBSRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBINC)

$(SRCGPIO_O): $(SRCGPIO)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBINC)

startup.o: startup.c
	$(CC) $(CFLAGS) -o $@ $^

load:
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

clean:
	rm -rf $(wildcard $(SRCDIR)/*.o)
	rm -rf $(wildcard $(DRIVERDIR)/src/*.o)
	rm -rf $(wildcard *.o)
	rm -rf $(wildcard *.elf)
	rm -rf $(wildcard *.map)

.PHONY: GPIO clean
