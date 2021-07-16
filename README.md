# STM32F410RB Drivers
Drivers include:
* GPIO
* SPI
* I2C
* USART

## Compiling and Linking
Makefile commands to build.
* make clean
* make

## Flash and Debug
Connect board to host PC via USB. Onboard ST-Link, OpenOCD and GDB will be used for debugging and flashing program into the target.
* make load

Run below commands on another terminal.
* arm-none-eabi-gdb
* target remote :3333
* monitor flash write_image erase {path to elf file}
