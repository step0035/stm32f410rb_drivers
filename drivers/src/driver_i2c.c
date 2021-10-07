#include <stdint.h>
#include "driver_i2c.h"
#include "driver_rcc.h"

/*
 * Private functions
 */

static void I2C_PeriClkCtl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI) {
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

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/w bit=0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/w bit=1
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle ) {
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL)) {
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if(pI2CHandle->RxSize  == 1) {
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else {
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}
	}
	else {
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
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

/*********************************************************************
 * @fn                - I2C_GetFlagStatus 
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

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName) {
	if(pI2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn                - I2C_MasterSendData
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

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr) {
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3. Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. send the data until len becomes 0
	while(Len > 0) {
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR )
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}
