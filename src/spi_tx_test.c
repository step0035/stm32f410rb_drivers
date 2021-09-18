#include <string.h>
#include "driver_gpio.h"
#include "driver_spi.h"
#include "stm32f410rb.h"

/*
 * PB12 -> SPI2_NSS
 * PB13 -> SPI2_SCLK
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * ALT function mode 5
 */

void SPI2_GPIOInit(void) {
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // NSS (not required in this example)
    // SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    // GPIO_Init(&SPIPins);
    
    // SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
    GPIO_Init(&SPIPins);

    // MISO (not required in this example)
    // SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    // GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
    GPIO_Init(&SPIPins);
}

void SPI2_Init(void) {
    SPI_Handle_t SPI2Handle;

    SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //generates sclk of 8MHz
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN; 

	SPI_Init(&SPI2Handle);
}

int main(void) {
    char user_data[] = "Hello world";

    SPI2_GPIOInit();                        // Init the gpio pins to behave as SPI2 pins
    SPI2_Init();                            // Init the SPI2 peripheral parameters
    SPI_PeripheralControl(SPI2, ENABLE);   // Enable SPI2 peripheral 

    SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

    while(1);

    return 0;
}
