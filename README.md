# STM32F410RB Drivers
No IDE is used. Drivers include:
* GPIO
* SPI
* I2C
* USART

## Compiling and Linking
Cross compiler arm-none-eabi-gcc is used for compiling and linking.
* make clean
* make
* make load

## Flash and Debug
OpenOCD and GDB will be used for debugging and flashing program into the target. Run below commands on another terminal.
* arm-none-eabi-gdb {name of elf file}
* monitor reset init
* monitor flash write_image {name of elf file}
