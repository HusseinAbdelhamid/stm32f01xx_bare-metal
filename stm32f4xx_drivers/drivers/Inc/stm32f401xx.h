/*
 * stm32f401xx.h
 *
 *  Created on: Dec 15, 2023
 *      Author: hussein
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_
#include <stdint.h>

#define __vo volatile


/***** processor specific details *****/

/*
 * ARM Cortex M4 porcessor number of priority bits implemented
 */
#define NO_PR_BITS_IMPLEMENTED		4
/*
 * ARM Cortex M4 Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex M4 Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2					((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3					((__vo uint32_t*)0xE000E18C)

/*
 * ARM Cortex M4 Processor NVIC IPRx registers
 */
#define NVIC_IPR_BASEADDR			0xE000E400
#define NVIC_IPRx(n)                ((__vo uint32_t*)(NVIC_IPR_BASEADDR + (int)(n/4)*(4)))

/*generic macros*/
#define ENABLE 			 1
#define DISABLE			 0
#define SET				 ENABLE
#define RESET 			 DISABLE
#define GPIO_PIN_SET 	 SET
#define GPIO_PIN_RESET	 RESET


/*
 * base address of FLASH and SRAM memories
 */

#define FLASH_BASEADDR				0x08000000U		/* flash base address */
#define SRAM1_BASEADDR				0x20000000U		/* sram1 base address */
#define ROM_BASEADDR				0x1FFF0000U	    /* rom base address   */
#define SRAM_BASEADDR				SRAM1_BASEADDR  /* sram base address  */


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE 				0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE
#define APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIPH_BASE				0x50000000U

/*
 * Base adresses of peripherals hanging on AHB1 bus
 */
#define GPIOA_BASEADDR				(AHB1PERIPH_BASE)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASE + 0x1C00)
#define RCC_BASEADDR				(AHB1PERIPH_BASE + 0x3800)

/*
 * Base adresses of peripherals hanging on APB1 bus
 */

#define USART2_BASEADDR				(APB1PERIPH_BASE + 0x4400)
#define I2C1_BASEADDR 				(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR 				(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR 				(APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR				(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASE + 0x3C00)
/*
 * Base adresses of peripherals hanging on APB2 bus
 */

#define USART1_BASEADDR				(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASE + 0x1400)
#define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR				(APB2PERIPH_BASE + 0x3400)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASE + 0x3800)
#define EXTI_BASEADDR				(APB2PERIPH_BASE + 0x3C00)

/********* peripheral register definition structures *********/

typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;
}GPIO_RegDef_t;

typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	const uint32_t res1[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	const uint32_t res2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	const uint32_t res3[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	const uint32_t res4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	const uint32_t res5[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	const uint32_t res6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	const uint32_t res7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	const uint32_t res8;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

typedef struct{
    __vo uint32_t MEMRMP;
    __vo uint32_t PMC;
    __vo uint32_t EXTICR[4];
    __vo uint16_t reserved;
    __vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

/*
 * peripheral definitions
 */

#define GPIOA 						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH 						((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC 						((RCC_RegDef_t*)RCC_BASEADDR)
#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define EXTI 						((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SPI1						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4						((SPI_RegDef_t*)SPI4_BASEADDR)



/* clock enable macros*/

/*AHB1*/
#define GPIOA_PCLK_EN()				(	RCC->AHB1ENR |= (1<<0)	   )
#define GPIOB_PCLK_EN()				(	RCC->AHB1ENR |= (1<<1)      )
#define GPIOC_PCLK_EN()				(	RCC->AHB1ENR |= (1<<2)      )
#define GPIOD_PCLK_EN()				(	RCC->AHB1ENR |= (1<<3)      )
#define GPIOE_PCLK_EN()				(	RCC->AHB1ENR |= (1<<4)      )
#define GPIOH_PCLK_EN()				(	RCC->AHB1ENR |= (1<<7)      )

#define SPI1_PCLK_EN()				(	RCC->APB2ENR |= (1<<12)    )
#define SPI4_PCLK_EN()				(	RCC->APB2ENR |= (1<<13)    )
#define SPI2_PCLK_EN()				(	RCC->APB1ENR |= (1<<14)    )
#define SPI3_PCLK_EN()				(	RCC->APB1ENR |= (1<<15)    )

#define USART1_PCLK_EN()			(	RCC->APB2ENR |= (1<<4)     )
#define USART2_PCLK_EN()			(	RCC->APB1ENR |= (1<<17)    )
#define USART6_PCLK_EN()			(	RCC->APB2ENR |= (1<<5)     )

#define I2C1_PCLK_EN()				(	RCC->APB1ENR |= (1<<21)    )
#define I2C2_PCLK_EN()				(	RCC->APB1ENR |= (1<<22)    )
#define I2C3_PCLK_EN()				(	RCC->APB1ENR |= (1<<23)    )

#define CRC_CLK_EN()				(	RCC->AHB1ENR |= (1<<12)    )
#define SYSCFG_PCLK_EN()			(	RCC->APB2ENR |=	(1<<14)	   )


/*clock disable macros*/
#define GPIOA_PCLK_DIS()			(	RCC->AHB1ENR &= ~(1<<0)	   )
#define GPIOB_PCLK_DIS()			(	RCC->AHB1ENR &= ~(1<<1)    )
#define GPIOC_PCLK_DIS()			(	RCC->AHB1ENR &= ~(1<<2)    )
#define GPIOD_PCLK_DIS()			(	RCC->AHB1ENR &= ~(1<<3)    )
#define GPIOE_PCLK_DIS()			(	RCC->AHB1ENR &= ~(1<<4)    )
#define GPIOH_PCLK_DIS()			(	RCC->AHB1ENR &= ~(1<<7)    )

#define SPI1_PCLK_DIS()				(	RCC->APB2ENR &= ~(1<<12)   )
#define SPI2_PCLK_DIS()				(	RCC->APB1ENR &= ~(1<<14)   )
#define SPI3_PCLK_DIS()				(	RCC->APB1ENR &= ~(1<<15)   )
#define SPI4_PCLK_DIS()				(	RCC->APB2ENR &= ~(1<<13)   )

#define USART1_PCLK_DIS()			(	RCC->APB2ENR &= ~(1<<4)    )
#define USART2_PCLK_DIS()			(	RCC->APB1ENR &= ~(1<<17)   )
#define USART6_PCLK_DIS()			(	RCC->APB2ENR &= ~(1<<5)    )

#define I2C1_PCLK_DIS()				(	RCC->APB1ENR &= ~(1<<21)   )
#define I2C2_PCLK_DIS()				(	RCC->APB1ENR &= ~(1<<22)   )
#define I2C3_PCLK_DIS()				(	RCC->APB1ENR &= ~(1<<23)   )

#define CRC_CLK_DIS()				(	RCC->AHB1ENR &= ~(1<<12)   )
#define SYSCFG_PCLK_DIS()			(	RCC->APB2ENR &= ~(1<<14)   )

/*GPIO register reset*/
#define GPIOA_RESET()	   			do{(	RCC->AHB1RSTR |= (1<<0)	   ) ;   (	RCC->AHB1RSTR &= ~(1<<0)	)  ; }while(0)
#define GPIOB_RESET()	   			do{(	RCC->AHB1RSTR |= (1<<1)    ) ;   (	RCC->AHB1RSTR &= ~(1<<1)    )  ; }while(0)
#define GPIOC_RESET()	   			do{(	RCC->AHB1RSTR |= (1<<2)    ) ;   (	RCC->AHB1RSTR &= ~(1<<2)    )  ; }while(0)
#define GPIOD_RESET()	   			do{(	RCC->AHB1RSTR |= (1<<3)    ) ;   (	RCC->AHB1RSTR &= ~(1<<3)    )  ; }while(0)
#define GPIOE_RESET()	   			do{(	RCC->AHB1RSTR |= (1<<4)    ) ;   (	RCC->AHB1RSTR &= ~(1<<4)    )  ; }while(0)
#define GPIOH_RESET()	   			do{(	RCC->AHB1RSTR |= (1<<7)    ) ;   (	RCC->AHB1RSTR &= ~(1<<7)    )  ; }while(0)

/*I2C registers reset*/
#define I2C1_RESET()				do{(	RCC->APB1RSTR |= (1<<21)   ) ;   (	RCC->APB1RSTR &= ~(1<<21)   )  ;}while(0)
#define I2C2_RESET()				do{(	RCC->APB1RSTR |= (1<<22)   ) ;   (	RCC->APB1RSTR &= ~(1<<22)   )  ;}while(0)
#define I2C3_RESET()				do{(	RCC->APB1RSTR |= (1<<23)   ) ;   (	RCC->APB1RSTR &= ~(1<<23)   )  ;}while(0)

/*SPI registers reset*/
#define SPI1_RESET()				do{(	RCC->APB2RSTR |= (1<<12)   ) ;   (	RCC->APB2RSTR &= ~(1<<12)   )  ;}while(0)
#define SPI2_RESET()				do{(	RCC->APB1RSTR |= (1<<14)   ) ;   (	RCC->APB1RSTR &= ~(1<<14)   )  ;}while(0)
#define SPI3_RESET()				do{(	RCC->APB1RSTR |= (1<<15)   ) ;   (	RCC->APB1RSTR &= ~(1<<15)   )  ;}while(0)
#define SPI4_RESET()				do{(	RCC->APB2RSTR |= (1<<13)   ) ;   (	RCC->APB2RSTR &= ~(1<<13)   )  ;}while(0)

/*USART registers reset*/
#define USART1_RESET()				do{(	RCC->APB2RSTR |= (1<<4)    ) ;   (	RCC->APB2RSTR &= ~(1<<4)    )  ;}while(0)
#define USART2_RESET()				do{(	RCC->APB1RSTR |= (1<<17)   ) ;   (	RCC->APB1RSTR &= ~(1<<17)   )  ;}while(0)
#define USART6_RESET()				do{(	RCC->APB2RSTR |= (1<<5)    ) ;   (	RCC->APB2RSTR &= ~(1<<5)    )  ;}while(0)


/*GPIO port code for SYSCFG_EXTICR register*/
#define GPIO_ADDR_TO_CODE(x)		((x==GPIOA) ? 0:\
									(x==GPIOB) ? 1:\
									(x==GPIOC) ? 2:\
									(x==GPIOD) ? 3:\
									(x==GPIOE) ? 4:\
									(x==GPIOH) ? 7:0)

/*IRQ number for stm32f401xx (from reference manual table (position column)*/
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84





#endif /* INC_STM32F401XX_H_ */
