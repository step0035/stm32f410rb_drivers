/*
 * stm32f410rb.h
 * device header file
 */
#ifndef STM32F410RB_H_
#define STM32F410RB_H_

#include <stddef.h>
#include <stdint.h>

/*
 * NVIC registers
 */

#define NVIC_ISER0		((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1		((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2		((volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3		((volatile uint32_t*) 0xE000E10C)
#define NVIC_ISER4		((volatile uint32_t*) 0xE000E110)
#define NVIC_ISER5		((volatile uint32_t*) 0xE000E114)
#define NVIC_ISER6		((volatile uint32_t*) 0xE000E118)
#define NVIC_ISER7		((volatile uint32_t*) 0xE000E11C)

#define NVIC_ICER0		((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1		((volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2		((volatile uint32_t*) 0xE000E188)
#define NVIC_ICER3		((volatile uint32_t*) 0xE000E18C)
#define NVIC_ICER4		((volatile uint32_t*) 0xE000E190)
#define NVIC_ICER5		((volatile uint32_t*) 0xE000E194)
#define NVIC_ICER6		((volatile uint32_t*) 0xE000E198)
#define NVIC_ICER7		((volatile uint32_t*) 0xE000E19C)

#define NVIC_PR_BASE_ADDR	((volatile uint32_t*) 0xE000E400)

#define NO_PR_BITS_IMPLEMENTED	4

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

/*
 * RCC (AHB1 peripheral)
 */

#define RCC_BASEADDR		(AHB1PERI_BASEADDR + 0x3800)

/*
 * I2C (APB1 peripheral)
 */

#define I2C1_BASEADDR		(APB1PERI_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1PERI_BASEADDR + 0x5800)
#define I2C4_BASEADDR		(APB1PERI_BASEADDR + 0x6000)

/*
 * USART (APB1 AND APB2 peripheral)
 */

#define USART1_BASEADDR		(APB2PERI_BASEADDR + 0x1000)	
#define USART2_BASEADDR		(APB1PERI_BASEADDR + 0x4400)	
#define USART6_BASEADDR		(APB2PERI_BASEADDR + 0x1400)	

/*
 * SPI (APB1 and ABP2 peripheral)
 */

#define SPI1_BASEADDR		(APB2PERI_BASEADDR + 0x3000)
#define SPI2_BASEADDR		(APB1PERI_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1PERI_BASEADDR + 0x3C00)
#define SPI5_BASEADDR		(APB2PERI_BASEADDR + 0x5000)

/*
 * EXTI (ABP2 PERIPHERAL)
 */

#define EXTI_BASEADDR		(APB2PERI_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR		(APB2PERI_BASEADDR + 0x3800)

/******************************************************************
 * peripheral data structures
 */

/*
 * GPIO
 */

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
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
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
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

/*
 * EXTI
 */

typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR; 
	volatile uint32_t RTSR;
	volatile uint32_t FTSR; 
	volatile uint32_t SWIER; 
	volatile uint32_t PR;
} EXTI_RegDef_t;

/*
 * SYSCFG
 */

typedef struct {
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
	volatile uint32_t CFGR;
} SYSCFG_RegDef_t;

/*
 * SPI
 */

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
} SPI_RegDef_t;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} I2C_RegDef_t;

/**************************************************
 * Definitions
 */

/*
 * RCC, EXTI, SYSCFG
 */

#define RCC			    ((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG			((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)
/* 
 * SYSCFG
 */

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

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

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ?0:\
					 (x == GPIOB) ?1:\
					 (x == GPIOC) ?2:\
					 (x == GPIOH) ?3:0)


/*
 * I2C
 */

#define I2C1                ((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2                ((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C4                ((I2C_RegDef_t*) I2C4_BASEADDR)

#define I2C1_PCLK_EN()		(RCC -> APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC -> APB1ENR |= (1 << 22))
#define I2C4_PCLK_EN()		(RCC -> APB1ENR |= (1 << 24))

#define I2C1_PCLK_DI()		(RCC -> APB1ENR &= (~(1 << 21)))
#define I2C2_PCLK_DI()		(RCC -> APB1ENR &= (~(1 << 22)))
#define I2C4_PCLK_DI()		(RCC -> APB1ENR &= (~(1 << 24)))

#define I2C1_REG_RESET()     do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB2RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()     do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C4_REG_RESET()     do{ (RCC->APB1RSTR |= (1 << 24)); (RCC->APB1RSTR &= ~(1 << 24)); }while(0)

/*
 * USART
 */

#define USART1_PCLK_EN()	(RCC -> APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC -> APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()	(RCC -> APB2ENR |= (1 << 5))

#define USART1_PCLK_DI()	(RCC -> APB2ENR &= (~(1 << 4)))
#define USART2_PCLK_DI()	(RCC -> APB1ENR &= (~(1 << 17)))
#define USART6_PCLK_DI()	(RCC -> APB2ENR &= (~(1 << 5)))

/*
 * SPI
 */

#define SPI1			((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI5			((SPI_RegDef_t*) SPI5_BASEADDR)

#define SPI1_PCLK_EN()		(RCC -> APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC -> APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC -> APB1ENR |= (1 << 15))
#define SPI5_PCLK_EN()		(RCC -> APB2ENR |= (1 << 20))
	
#define SPI1_PCLK_DI()		(RCC -> APB2ENR &= (~(1 << 12)))
#define SPI2_PCLK_DI()		(RCC -> APB1ENR &= (~(1 << 14)))
#define SPI3_PCLK_DI()		(RCC -> APB1ENR &= (~(1 << 15)))
#define SPI5_PCLK_DI()		(RCC -> APB2ENR &= (~(1 << 20)))

#define SPI1_REG_RESET()     do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()     do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()     do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI5_REG_RESET()     do{ (RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR &= ~(1 << 20)); }while(0)
/*
 * IRQ numbers
 */

#define IRQ_NO_EXTI0		(6)
#define IRQ_NO_EXTI1		(7)
#define IRQ_NO_EXTI2		(8)
#define IRQ_NO_EXTI3		(9)
#define IRQ_NO_EXTI4		(10)
#define IRQ_NO_EXTI9_5		(23)
#define IRQ_NO_EXTI15_10	(40)
#define IRQ_NO_EXTI19		(62)
#define IRQ_NO_EXTI20		(76)
#define IRQ_NO_EXTI23		(97)
#define IRQ_NO_SPI1         (35)
#define IRQ_NO_SPI2         (36)
#define IRQ_NO_SPI5         (85)
#define IRQ_NO_USART1       (37)
#define IRQ_NO_USART2       (38)
#define IRQ_NO_USART6       (71)
#define IRQ_NO_I2C1_EV      (31)
#define IRQ_NO_I2C1_ER      (32)
#define IRQ_NO_I2C2_EV      (33)
#define IRQ_NO_I2C2_ER      (34)
#define IRQ_NO_I2C4_EV      (95)
#define IRQ_NO_I2C4_ER      (96)

/*
 * IRQ Priority Levels
 */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14	
#define NVIC_IRQ_PRI15		15

/*
 * Misc
 */

#define ENABLE		    	1
#define DISABLE		    	0
#define SET			        ENABLE
#define RESET			    DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_SET            SET
#define FLAG_RESET          RESET

/********************************************
 * Bit position definitions of SPI peripheral
 ********************************************/

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE         6
#define SPI_CR1_LSBFIRST    7
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RXONLY      10
#define SPI_CR1_DFF         11
#define SPI_CR1_CRCNEXT     12
#define SPI_CR1_CRCEN       13
#define SPI_CR1_BIDIOE      14
#define SPI_CR1_BIDIMODE    15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

#endif

/********************************************
 * Bit position definitions of I2C peripheral
 ********************************************/

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				0
#define I2C_OAR1_ADD71 				 	1
#define I2C_OAR1_ADD98  			 	8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15
