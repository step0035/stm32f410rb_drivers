#include <string.h>
#include "driver_gpio.h"

#define HIGH		1
#define LOW		0
#define BTN_PRESSED	HIGH

void delay(void) {
	for(uint32_t i = 0; i < 100000; i++);
}

int main(void) {
	GPIO_Handle_t GpioLed, GpioBtn;
	// Initialize fields to 0 first
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioBtn, 0, sizeof(GpioBtn));
	
	GpioLed.pGPIOx = GPIOA; // User LED2 of stm32f410rb is on PA5
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_NO_PUPD;

	GpioBtn.pGPIOx = GPIOC; // Button is on PC13
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_PIN_PD;

	GPIO_PeriClkCtl(GpioLed.pGPIOx, ENABLE);
	GPIO_PeriClkCtl(GpioBtn.pGPIOx, ENABLE);

	GPIO_Init(&GpioLed);	
	GPIO_Init(&GpioBtn);	

	// Interupt configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI10);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);
	
	return  0;	
}

void EXTI15_10_Handler(void) {
	delay();
	GPIO_IRQHandler(GPIO_PIN_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
}
