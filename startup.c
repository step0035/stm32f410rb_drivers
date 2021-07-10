#include <stdio.h>

#define SRAM_START	0x20000000U
#define SRAM_SIZE	(32U * 1024U) //32KB
#define SRAM_END	((SRAM_START) + (SRAM_SIZE))

#define STACK_START	SRAM_END

void Reset_Handler(void);
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Systick_Handler(void) __attribute__((weak, alias("Default_Handler")));
void WWDG_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PVD_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI21_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI22_Handler(void) __attribute__((weak, alias("Default_Handler")));
void FLASH_Handler(void) __attribute__((weak, alias("Default_Handler")));
void RCC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI0_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI2_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI3_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI4_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream0_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream2_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream3_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream4_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream5_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream6_Handler(void) __attribute__((weak, alias("Default_Handler")));
void ADC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_TIM9 _Handler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM11_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_Handler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SPI2_Handler(void) __attribute__((weak, alias("Default_Handler")));
void USART1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void USART2_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI17_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream7_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TIM5_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TIM6_DAC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream0_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream2_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream3_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream4_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream5_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream6_Handler(void) __attribute__((weak, alias("Default_Handler")));
void USART6_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI20_Handler(void) __attribute__((weak, alias("Default_Handler")));
void RNG_Handler(void) __attribute__((weak, alias("Default_Handler")));
void FPU_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SPI5_Handler(void) __attribute__((weak, alias("Default_Handler")));
void I2C4_EV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void I2C4_ER_Handler(void) __attribute__((weak, alias("Default_Handler")));
void LPTIM1_Handler(void) __attribute__((weak, alias("Default_Handler")));

uint32_t vectors[] __attribute__((section(".isr_vector"))) = {
	STACK_START,
	(uint32_t) &Reset_Handler,
	(uint32_t) &NMI_Handler,
	(uint32_t) &HardFault_Handler,
	(uint32_t) &MemManage_Handler,
	(uint32_t) &BusFault_Handler,
	(uint32_t) &UsageFault_Handler,
	0,
	0,
	0,
	0,
	(uint32_t) &SVCall_Handler,
	(uint32_t) &DebugMon_Handler,
	0,
	(uint32_t) &PendSV_Handler,
	(uint32_t) &Systick_Handler,
	(uint32_t) &WWDG_Handler,
	(uint32_t) &PVD_Handler,
	(uint32_t) &EXTI21_Handler,
	(uint32_t) &EXTI22_Handler,
	(uint32_t) &FLASH_Handler,
	(uint32_t) &RCC_Handler,
	(uint32_t) &EXTI0_Handler,
	(uint32_t) &EXTI1_Handler,
	(uint32_t) &EXTI2_Handler,
	(uint32_t) &EXTI3_Handler,
	(uint32_t) &EXTI4_Handler,
	(uint32_t) &DMA1_Stream0_Handler,
	(uint32_t) &DMA1_Stream1_Handler,
	(uint32_t) &DMA1_Stream2_Handler,
	(uint32_t) &DMA1_Stream3_Handler,
	(uint32_t) &DMA1_Stream4_Handler,
	(uint32_t) &DMA1_Stream5_Handler,
	(uint32_t) &DMA1_Stream6_Handler,
	(uint32_t) &ADC_Handler,
	0,
	0,
	0,
	0,
	(uint32_t) &EXTI9_5_Handler,
	(uint32_t) &TIM1_BRK_TIM9_Handler,
	(uint32_t) &TIM1_UP_Handler,
	(uint32_t) &TIM1_TRG_COM_TIM11_Handler,
	(uint32_t) &TIM1_CC_Handler,
	0,
	0,
	0,
	(uint32_t) &I2C1_EV_Handler,
	(uint32_t) &I2C1_ER_Handler,
	(uint32_t) &I2C2_EV_Handler,
	(uint32_t) &I2C2_ER_Handler,
	(uint32_t) &SPI1_Handler,
	(uint32_t) &SPI2_Handler,
	(uint32_t) &USART1_Handler,
	(uint32_t) &USART2_Handler,
	0,
	(uint32_t) &EXTI15_10_Handler,
	(uint32_t) &EXTI17_Handler,
	0,
	0,
	0,
	0,
	0,
	(uint32_t) &DMA1_Stream7_Handler,
	0,
	0,
	(uint32_t) &TIM5_Handler,
	0,
	0,
	0,
	(uint32_t) &TIM6_DAC_Handler,
	0,
	(uint32_t) &DMA2_Stream0_Handler,
	(uint32_t) &DMA2_Stream1_Handler,
	(uint32_t) &DMA2_Stream2_Handler,
	(uint32_t) &DMA2_Stream3_Handler,
	(uint32_t) &DMA2_Stream4_Handler,
	0,
	(uint32_t) &EXTI19_Handler,
	0,
	0,
	0,
	0,
	0,
	(uint32_t) &DMA2_Stream5_Handler,
	(uint32_t) &DMA2_Stream6_Handler,
	(uint32_t) &DMA2_Stream7_Handler,
	(uint32_t) &USART6_Handler,
	0,
	0,
	0,
	0,
	(uint32_t) &EXTI20_Handler,
	0,
	0,
	0,
	(uint32_t) &RNG_Handler,
	(uint32_t) &FPU_Handler,
	0,
	0,
	0,
	(uint32_t) &SPI5_Handler,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	(uint32_t) &I2C4_EV_Handler,
	(uint32_t) &I2C4_ER_Handler,
	(uint32_t) &LPTIM1_Handler,
}

void Default_Handler(void) {
	while(1);
}

void Reset_Handler(void) {

}
