#include "driver_gpio.h"

void delay(void) {
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void) {
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA; // User LED2 of stm32f410rb is on PA5
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_NO_PUPD;

	GPIO_PeriClkCtl(GpioLed.pGPIOx, ENABLE);
	GPIO_Init(&GpioLed);	

	while(1) {
		GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
		delay();
	}

	return  0;	
}
