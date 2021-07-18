#include <stdint.h>
#include "driver_gpio.h"

/*********************************************************************
 * @fn 			- GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_PeriClkCtl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
	}

	else if(ENorDI == DISABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
		
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp = 0;

	// configure pin mode
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// non interrupt mode
		temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode) << (2 * ( pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle -> pGPIOx -> MODER &= ~(0x3 << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle -> pGPIOx -> MODER |= temp;

	}
	
	else {
		// interrupt mode	
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;		
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;		
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (4 * temp2);

		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}	

	temp = 0;

	// configure pin speed
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed) << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> OSPEEDR &= ~(0x3 << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> OSPEEDR |= temp;

	temp = 0;

	// configure pin pull up pull down	
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinPuPdControl) << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> PUPDR &= ~(0x3 << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> PUPDR |= temp;

	temp = 0;

	// configure pin output type
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinOPType) << (1 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> OTYPER &= ~(0x3 << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> OTYPER |= temp;

	temp  = 0;

	// configure the alt functionality
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		uint8_t temp1, temp2;
		
		temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle -> pGPIOx -> AFR[temp1] &= ~(0xF << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle -> pGPIOx -> AFR[temp1] |= pGPIOHandle -> GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);
	}
	
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	
	if(pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -
 */

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value;
	value = (uint8_t) (((pGPIOx -> IDR) >> PinNumber) & 0x00000001);

	return value;
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	value = (uint16_t) (pGPIOx -> IDR); 

	return value;

}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if(Value == GPIO_PIN_SET) {
		pGPIOx -> ODR |= (1 << PinNumber);
	}
	else {
		pGPIOx -> ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx -> ODR = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx -> ODR ^= (1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		if(IRQNumber <= 31) {
			//ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) {
			//ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			//ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else {
		if(IRQNumber <= 31) {
			//ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) {
			//ICER1
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			//ICER2
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}

}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount); 
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_IRQHandling(uint8_t PinNumber) {
	//clear pending register corresponding to pin number
	if(EXTI->PR & (1 << PinNumber)) {
		EXTI->PR |= (1 << PinNumber);
	}
}
