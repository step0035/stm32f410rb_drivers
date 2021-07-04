test:
	arm-none-eabi-gcc -c -o main.o -I drivers/inc src/main.c

clean:
	rm -rf *.o
