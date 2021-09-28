#include <stdint.h>
#include "driver_i2c.h"
#include "driver_rcc.h"

/*
 * Private functions
 */

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
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

void I2C_PeriClkCtl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		if(pI2Cx == I2C1) 
			I2C1_PCLK_EN();
		else if (pI2Cx == I2C2)
			I2C2_PCLK_EN();
		else if (pI2Cx == I2C4)
			I2C4_PCLK_EN();
	}
	else {
		if(pI2Cx == I2C1) 
			I2C1_PCLK_DI();
		else if (pI2Cx == I2C2)
			I2C2_PCLK_DI();
		else if (pI2Cx == I2C4)
			I2C4_PCLK_DI();
	}

}

/*********************************************************************
 * @fn          - I2C_Init 
 *
 * @brief             - 
 *
 * @param[in]         - 
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void I2C_Init(I2C_Handle_t *pI2CHandle) {
    uint32_t tempreg = 0;

    // Enable I2Cx peripheral clock
    I2C_PeriClkCtl(pI2CHandle->pI2Cx, ENABLE);

    // ACK control bit
    tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
    pI2CHandle->pI2Cx->CR1 = tempreg;

    // FREQ field of CR2
    tempreg = 0;
    tempreg = RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = tempreg & 0x3F; // 5 bits only

    // Program device's own address
    tempreg = 0;
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempreg |= (1 << 14); // bit 14 must be set

    // CCR calculations
    uint16_t ccr_value = 0;
    tempreg = 0;

    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempreg |= (ccr_value & 0xFFF);
	}
    else {
		//mode is fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
        else {
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}

		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

    // TRISE configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		//mode is standard mode
		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;
	}
    else {
		//mode is fast mode
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/*********************************************************************
 * @fn                - I2C_DeInit
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

void I2C_DeInit(I2C_RegDef_t *pI2Cx) {

    if(pI2Cx == I2C1) 
        I2C1_REG_RESET();
    else if (pI2Cx == I2C2) 
        I2C2_REG_RESET();
    else if (pI2Cx == I2C4) 
        I2C4_REG_RESET();
}
