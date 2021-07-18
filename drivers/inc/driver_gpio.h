#ifndef DRIVER_GPIO_H_
#define DRIVER_GPIO_H_

#include <stdint.h>
#include "stm32f410rb.h"

/*
 * Configuration structure for GPIO pin
 */

typedef struct {
	uint8_t GPIO_PinNumber;			//@GPIO_PIN_NUMBER	
	uint8_t GPIO_PinMode;			//@GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			//@GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;		//@GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;			//@GPIO_OUTPUT_TYPE
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

/* @GPIO_PIN_NUMBER
 * GPIO Pin number
 */

#define GPIO_PIN_0		0	
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12	
#define GPIO_PIN_13		13	
#define GPIO_PIN_14		14	
#define GPIO_PIN_15		15

/* @GPIO_PIN_MODES
 * GPIO modes
 */

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/* @GPIO_OUTPUT_TYPE
 * GPIO output types
 */

#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/* @GPIO_PIN_SPEED
 * GPIO speed
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/* @GPIO_PIN_PUPD
 * GPIO pin pull up and pull down configs
 */

#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU		1
#define GPIO_PIN_PD		2

#endif
