#include <stdint.h>
#include "driver_spi.h"

/*
 * Private functions for interrupt handling
 */

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);  // prevents interrupt from TXE flag
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);  // prevents interrupt from RXNEIE flag
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv) {
	// This is a weak implementation. The user application may override this function.
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
    // check DFF bit in CR1
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
        // 16bit DFF
        // load data into DR
        pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen--;
        pSPIHandle->TxLen--;
        (uint16_t*)pSPIHandle->pTxBuffer++;
    }
    else {
        // 8bit DFF
        // load data into DR
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }
    // Note: this handler only transmits one byte of data per interrupt

    if (!pSPIHandle->TxLen) {
        // close the SPI transmission and inform app that TX is over
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLT);
    }
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
    // check DFF bit in CR1
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
        // 16bit DFF
        // load data from DR into RxBuffer
        *((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->RxLen--;
        (uint16_t*)pSPIHandle->pRxBuffer++;
    }
    else {
        // 8bit DFF
        // load data from DR into RxBuffer
        *pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }

    if (!pSPIHandle->TxLen) {
        // close the SPI transmission and inform app that RX is over
        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLT);
    }
}

static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	uint8_t temp;
	// clear the ovr flag if not in busy state, else clear ovr flag from application
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	// inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

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
    while(Len > 0) {
        // wait until RXNE is set 
        while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

        // check DFF bit in CR1
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
            // 16bit DFF
            // load data from DR into RxBuffer
            *((uint16_t*)pRxBuffer) = (uint16_t)pSPIx->DR;
            Len--;
            Len--;
            (uint16_t*)pRxBuffer++;
        }
        else {
            // 8bit DFF
            // load data from DR into RxBuffer
            *pRxBuffer = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }

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

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
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

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
    if (ENorDI == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Data transmission will be handled inside the ISR code
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {
        // Save the TxBuffer address and Len information in some global variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;

        // Mark SPI state as busy in TX so no other code can take over the same SPI peripheral
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        // Enable the TXEIE control bit to enable the interrupt when TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }

    return pSPIHandle->TxState;
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
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

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {
    if (pSPIHandle->RxState != SPI_BUSY_IN_RX) {
        // Save the RxBuffer address and Len information in some global variables
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;

        // Mark SPI state as busy in TX so no other code can take over the same SPI peripheral
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        // Enable the TXEIE control bit to enable the interrupt when TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }

    return pSPIHandle->RxState;
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
    uint8_t temp1, temp2;

    // Check for TXE
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if (temp1 && temp2) {
        // Handle TXE
        spi_txe_interrupt_handle(pSPIHandle);
    }
    
    // Check for RXNE
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if (temp1 && temp2) {
        // Handle RXNE
        spi_rxne_interrupt_handle(pSPIHandle);
    }

    // Check for OVR
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if (temp1 && temp2) {
        // Handle RXNE
        spi_ovr_err_interrupt_handle(pSPIHandle);
    }
}
