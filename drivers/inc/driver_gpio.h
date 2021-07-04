#ifndef DRIVER_GPIO_H_
#define DRIVER_GPIO_H_

#include <stdint.h>
#include "stm32f410rb.h"

/*
 * Configuration structure for GPIO pin
 */

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * Handle structure for GPIO
 */

typedef struct {
	GPIO_RegDef_t *pGPIOx;			// Base address of GPIO port to which the pin belongs to
	GPIO_PinConfig_t GPIO_PinConfig;	// GPIO pin configuration settings
} GPIO_Handle_t;

/*
 * GPIO APIs prototype
 */

void GPIO_PeriClkCtl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif
