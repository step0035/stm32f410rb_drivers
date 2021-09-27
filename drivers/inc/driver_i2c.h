#ifndef DRIVER_I2C_H_
#define DRIVER_I2C_H_

#include <stdint.h>
#include "stm32f410rb.h"

/*
 * Configuration structure for I2C peripheral 
 */
typedef struct {
    uint32_t I2C_SCLSpeed;
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_ACKControl;
    uint16_t I2C_FMDutyCycle;
} I2C_Config_t;

/*
 * Handle structure for I2C peripheral
 */
typedef struct {
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
} I2C_Handle_T;

/*
 * I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM        100000
#define I2C_SCL_SPEED_FM2k      200000
#define I2C_SCL_SPEED_FM4k      400000

/*
 * I2C_ACKControl
 */
#define I2C_ACK_DISABLE         0
#define I2C_ACK_ENABLE          1

/*
 * I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2           0
#define I2C_FM_DUTY_16_9        1

/*
 * I2C APIs prototype
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
#endif
