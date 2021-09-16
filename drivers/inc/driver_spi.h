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
} SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE       0
#define SPI_DEVICE_MODE_MASTER      1

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD                   1
#define SPI_BUS_CONFIG_HD                   2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY       3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2        0
#define SPI_SCLK_SPEED_DIV4        1
#define SPI_SCLK_SPEED_DIV8        2
#define SPI_SCLK_SPEED_DIV16       3
#define SPI_SCLK_SPEED_DIV32       4
#define SPI_SCLK_SPEED_DIV64       5
#define SPI_SCLK_SPEED_DIV128      6
#define SPI_SCLK_SPEED_DIV256      7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS       0
#define SPI_DFF_16BITS      1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW        0
#define SPI_CPOL_HIGH       1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW        0
#define SPI_CPHA_HIGH       1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI      0
#define SPI_SSM_EN      1

/*
 * SPI APIs prototype
 */
void SPI_PeriClkCtl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle);
#endif
