/*
 * stm32f410rb.h
 * device header file
 */

#include <stdint.h>

/*
 * FLASH and SRAM
 */

#define FLASH_BASEADDR		0x00000000U
#define SRAM_BASEADDR		0x20000000U
#define ROM_BASEADDR		0x1FFF0000U


/*
 * AHBx and APBx bus peripherals
 */

#define PERI_BASEADDR		0x40000000U
#define APB1PERI_BASEADDR	PERI_BASEADDR
#define APB2PERI_BASEADDR	0x40010000U
#define AHB1PERI_BASEADDR	0x40020000U


/*
 * GPIO (AHB1 peripheral)
 */

#define GPIOA_BASEADDR		AHB1PERI_BASEADDR
#define GPIOB_BASEADDR		(AHB1PERI_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERI_BASEADDR + 0x0800)
#define GPIOH_BASEADDR		(AHB1PERI_BASEADDR + 0x0C00)
#define RCC_BASEADDR		(AHB1PERI_BASEADDR + 0x1000)


/******************************************************************
 * peripheral data structures
 */


