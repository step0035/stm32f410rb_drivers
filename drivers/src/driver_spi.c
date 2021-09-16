#include <stdint.h>
#include "driver_spi.h"

/*********************************************************************
 * @fn 			- SPI_PeriClkCtl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void SPI_PeriClkCtl(SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		if(pSPIx == SPI1) {
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI5) {
			SPI5_PCLK_EN();
		}
	}

	else if(ENorDI == DISABLE) {
		if(pSPIx == SPI1) {
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2) {
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI5) {
			SPI5_PCLK_DI();
		}
		
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
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

void SPI_Init(SPI_Handle_t *pSPIHandle) {
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
 * @fn      		  - SPI_DeInit
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

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	
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
 * @fn      		  - SPI_SendData
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

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {

}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
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

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {

}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
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

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI) {
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount); 
}

/*********************************************************************
 * @fn      		  - SPI_IRQHandler
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

void SPI_IRQHandler(SPI_Handle_t *pSPIHandle) {
	//clear pending register corresponding to pin number
	if(EXTI->PR & (1 << PinNumber)) {
		EXTI->PR |= (1 << PinNumber);
	}
}
