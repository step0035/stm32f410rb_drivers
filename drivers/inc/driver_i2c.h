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

#endif
