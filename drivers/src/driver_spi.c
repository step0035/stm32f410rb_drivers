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
	uint32_t tempreg = 0;

    // Enable peripheral clock
    SPI_PeriClkCtl(pSPIHandle->pSPIx, ENABLE);

	// configure device mode
    tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << 2;

    // configure bus config
    if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
        tempreg &= ~(1 << 15); // clear BIDI
    }
    else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
        tempreg |= (1 << 15); // set BIDI
    }
    else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
        tempreg &= ~(1 << 15); // clear BIDI
        tempreg |= (1 << 10); // set RXONLY
    }

    // configure spi serial clock speed (baud rate)
    tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << 3;

    // configure DFF
    tempreg |= pSPIHandle->SPI_Config.SPI_DFF << 3;

    // configure CPOL
    tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << 1;

    // configure CPHA
    tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

    pSPIHandle->pSPIx->CR1 = tempreg;
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
	
	if(pSPIx == SPI1) {
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI5) {
		SPI5_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
    if (pSPIx->SR & FlagName)
        return FLAG_SET;
    else
        return FLAG_RESET;
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
 * @Note              - This is a blocking call
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
    while(Len > 0) {
        // wait until TXE is set 
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        // check DFF bit in CR1
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
            // 16bit DFF
            // load data into DR
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            Len--;
            Len--;
            (uint16_t*)pTxBuffer++;
        }
        else {
            // 8bit DFF
            // load data into DR
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
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

}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
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

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
    if (ENorDI == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}
