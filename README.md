# STM32F410RB Drivers
Drivers include:
* GPIO
* SPI
* I2C
* USART

## Compiling and Linking
Makefile commands to build all the source files. ELF files are generated in src directory.
* make clean
* make

## Flash and Debug
Connect board to host PC via USB. Onboard ST-Link, OpenOCD and GDB will be used for debugging and flashing program into the target. Run below command to start OpenOCD server.
* make load

Run below command on another terminal to launch GDB client.
* arm-none-eabi-gdb {path to elf file}

Run below commands on GDB client.
* target remote :3333
* monitor reset init
* monitor flash write_image erase {path to elf file}
* monitor reset halt
* monitor resume
