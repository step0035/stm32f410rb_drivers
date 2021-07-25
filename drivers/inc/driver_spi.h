#ifndef DRIVER_SPI_H_
#define DRIVER_SPI_H_

#include <stdint.h>
#include "stm32f410rb.h"

/*
 * Configuration structure for GPIO pin
 */

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

/*
 * Handle structure for GPIO
 */

typedef struct {
	SPI_RegDef_t *pSPIx;			
	SPI_Config_t SPI_Config;	// SPI configuration settings
} GPIO_Handle_t;


/*
 * SPI APIs prototype
 */
void SPI_PeriClkCtl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef *pSPIx);
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle);
#endif
