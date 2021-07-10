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
LD = $(CROSS_COMPILE)ld
GDB = $(CROSS_COMPILE)gdb
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump

GPIO: $(SRCGPIO_O) $(LIBSRC_O)
# TO DO: make gpio elf

$(LIBSRC_O): $(LIBSRC)
	$(CC) -c -o $@ $^ $(LIBINC)

$(SRCGPIO_O): $(SRCGPIO)
	$(CC) -c -o $@ $^ $(LIBINC)

clean:
	rm -rf $(wildcard $(SRCDIR)/*.o)
	rm -rf $(wildcard $(DRIVERDIR)/src/*.o)

.PHONY: GPIO clean
