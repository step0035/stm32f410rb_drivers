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

/*
 * GPIO
 */

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDER;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_RegDef_t;

/*
 * RCC
 */

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    uint32_t RSVD0[3];
    volatile uint32_t APB1STR;
    volatile uint32_t APB2STR;
    uint32_t RSVD1[2];
    volatile uint32_t AHB1ENR;
    uint32_t RSVD2[3];
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    uint32_t RSVD3[2];
    volatile uint32_t AHB1LPENR;
    uint32_t RSVD4[3];
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    uint32_t RSVD5[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    uint32_t RSVD6[2];
    volatile uint32_t SSCGR;
    uint32_t RSVD7[2];
    volatile uint32_t DCKCFGR;
    uint32_t RSVD8;
    volatile uint32_t DCKCFGR2;
} RCC_RegDef_t;

/**************************************************
 * Definitions
 */

/*
 * GPIO
 */

#define GPIOA  				((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define GPIOA_PCLK_EN()      (RCC -> AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()      (RCC -> AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()      (RCC -> AHB1ENR |= (1 << 2))
#define GPIOH_PCLK_EN()      (RCC -> AHB1ENR |= (1 << 7))

#define GPIOA_PCLK_DI()      (RCC -> AHB1ENR &= (~(1 << 0)))
#define GPIOB_PCLK_DI()      (RCC -> AHB1ENR &= (~(1 << 1)))
#define GPIOC_PCLK_DI()      (RCC -> AHB1ENR &= (~(1 << 2)))
#define GPIOH_PCLK_DI()      (RCC -> AHB1ENR &= (~(1 << 7)))

#define GPIOA_REG_RESET()     do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()     do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()     do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOH_REG_RESET()     do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
