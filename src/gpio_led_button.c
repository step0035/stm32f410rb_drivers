#include "driver_gpio.h"

#define HIGH		1
#define LOW		0
#define BTN_PRESSED	HIGH

void delay(void) {
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void) {
	GPIO_Handle_t GpioLed, GpioBtn;
	
	GpioLed.pGPIOx = GPIOA; // User LED2 of stm32f410rb is on PA5
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_NO_PUPD;

	GpioBtn.pGPIOx = GPIOC; // Button is on PC13
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_PIN_PD;

	GPIO_PeriClkCtl(GpioLed.pGPIOx, ENABLE);
	GPIO_PeriClkCtl(GpioBtn.pGPIOx, ENABLE);

	GPIO_Init(&GpioLed);	
	GPIO_Init(&GpioBtn);	

	while(1) {
		if(GPIO_ReadInputPin(GpioBtn.pGPIOx, GPIO_PIN_13) == BTN_PRESSED) {
			delay();
			GPIO_WriteOutputPin(GpioLed.pGPIOx, GPIO_PIN_5, HIGH);
		}
		else{
			delay();
			GPIO_WriteOutputPin(GpioLed.pGPIOx, GPIO_PIN_5, LOW);
		}
	}
	return  0;	
}
