/*
 * stm32f411re.h
 *
 *  Created on: Dec 24, 2023
 *      Author: adhip
 */

#ifndef INC_STM32F411RE_H_
#define INC_STM32F411RE_H_
#define __vo 						volatile
#include <stdint.h>

/*
 * ARM Cortex M4 processor specific details
 */
//Interrupt Set-enable Registers
#define NVIC_ISER_BASE_ADDR			0xE000E100U
#define NVIC_REG_OFFSET				0x04U
#define NVIC_ISER0_ADDR				((__vo uint32_t *)NVIC_ISER_BASE_ADDR)
#define NVIC_ISER1_ADDR				((__vo uint32_t *)(NVIC_ISER_BASE_ADDR + NVIC_REG_OFFSET))
#define NVIC_ISER2_ADDR				((__vo uint32_t *)(NVIC_ISER_BASE_ADDR + NVIC_REG_OFFSET*2))
//Interrupt Clear-enable Registers
#define NVIC_ICER_BASE_ADDR			0xE000E180U
#define NVIC_ICER0_ADDR				((__vo uint32_t *)NVIC_ICER_BASE_ADDR)
#define NVIC_ICER1_ADDR				((__vo uint32_t *)(NVIC_ICER_BASE_ADDR + NVIC_REG_OFFSET))
#define NVIC_ICER2_ADDR				((__vo uint32_t *)(NVIC_ICER_BASE_ADDR + NVIC_REG_OFFSET*2))
//Interrupt Priority Registers
#define NVIC_IPR_BASE_ADDR			0xE000E400U
#define NVIC_IPR0_ADDR				((__vo uint32_t *)NVIC_IPR_BASE_ADDR)

//Interrupt Priority in STM32F411RE
#define NO_PR_BITS_IMPL				4

/*Base address of Flash, SRAM and System Memory*/
#define FLASH_BASE_ADDRESS          0x08000000U
#define SRAM1_BASE_ADDRESS          0x20000000U
#define SRAM                        SRAM1_BASE_ADDRESS
#define SRAM2_BASE_ADDRESS			0X20001C00U //SRAM + 128Kb
#define ROM_BASE_ADDRESS			0x1FFF0000U

/*Base address of all the buses interfaces present in STM32F411RE*/
#define PERIPH_BASE_ADDRESS		 	0x40000000U
#define PERIPH_OFFSET				0X10000U
#define APB1_PERIPH_BASE_ADDRESS	PERIPH_BASE_ADDRESS
#define APB2_PERIPH_BASE_ADDRESS	(APB1_PERIPH_BASE_ADDRESS + PERIPH_OFFSET)
#define AHB1_PERIPH_BASE_ADDRESS	(APB2_PERIPH_BASE_ADDRESS + PERIPH_OFFSET)
#define AHB2_PERIPH_BASE_ADDRESS	0X50000000U

/*Defining base address of the peripherals which are hanging to AHB1 bus*/
#define GPIO_OFFSET					0x400U
#define GPIOA_BASE_ADDRESS			AHB1_PERIPH_BASE_ADDRESS
#define GPIOB_BASE_ADDRESS			(AHB1_PERIPH_BASE_ADDRESS + GPIO_OFFSET)
#define GPIOC_BASE_ADDRESS			(AHB1_PERIPH_BASE_ADDRESS + GPIO_OFFSET*2)
#define GPIOD_BASE_ADDRESS			(AHB1_PERIPH_BASE_ADDRESS + GPIO_OFFSET*3)
#define GPIOE_BASE_ADDRESS			(AHB1_PERIPH_BASE_ADDRESS + GPIO_OFFSET*4)
#define GPIOH_BASE_ADDRESS			(AHB1_PERIPH_BASE_ADDRESS + GPIO_OFFSET*7) //GPIO F and G not defined in reference manual
#define RCC_BASE_ADDRESS			0X40023800U

/*Defining base address of the peripherals which are hanging to APB1 bus*/
#define SPI2_BASE_ADDRESS			(APB1_PERIPH_BASE_ADDRESS + 0x3800U)
#define SPI3_BASE_ADDRESS			(APB1_PERIPH_BASE_ADDRESS + 0x3C00U)
#define USART2_BASE_ADDRESS			(APB1_PERIPH_BASE_ADDRESS + 0x4400U)
#define I2C1_BASE_ADDRESS			(APB1_PERIPH_BASE_ADDRESS + 0x5400U)
#define I2C2_BASE_ADDRESS			(APB1_PERIPH_BASE_ADDRESS + 0x5800U)
#define I2C3_BASE_ADDRESS			(APB1_PERIPH_BASE_ADDRESS + 0x5C00U)

/*Defining base address of the peripherals which are hanging to APB2 bus*/
#define USART1_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0x1000U)
#define USART6_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0x1400U)
#define SPI1_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0x3000U)
#define SPI4_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0x3400U)
#define SYSCFG_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0x3800U)
#define EXTI_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0x3C00U)
#define SPI5_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0x5000U)

/*Defining the base address of all the specific registers of the peripherals using a structure*/
typedef struct{						//As the size of variables is uint32 so an offset of 0x04 is directly applied while dereferencing
__vo uint32_t MODER;				//GPIO port mode register
__vo uint32_t OTYPER;				//GPIO port output type register
__vo uint32_t OSPEEDR;				//GPIO port output speed register
__vo uint32_t PUPDR;				//GPIO port pull-up/pull-down register
__vo uint32_t IDR;					//GPIO port input data register
__vo uint32_t ODR;					//GPIO port output data register
__vo uint32_t BSRR;					//GPIO port bit set/reset register
__vo uint32_t LCKR;					//GPIO port configuration lock register
__vo uint32_t AFRL;					//GPIO alternate function low register
__vo uint32_t AFRH;					//GPIO alternate function high register
}GPIO_RegDef_t;

typedef struct{					//As the size of variables is uint32 so an offset of 0x04 is directly applied while dereferencing
__vo uint32_t CR;				//RCC clock control register
__vo uint32_t PLLCFGR;			//RCC PLL configuration register
__vo uint32_t CFGR;				//RCC clock configuration register
__vo uint32_t CIR;				//RCC clock interrupt register
__vo uint32_t AHB1RSTR;			//RCC AHB1 peripheral reset register
__vo uint32_t AHB2RSTR;			//RCC AHB2 peripheral reset register
__vo uint32_t Reserved1[2];		//
__vo uint32_t APB1RSTR;			//RCC APB1 peripheral reset register
__vo uint32_t APB2RSTR;			//RCC APB2 peripheral reset register
__vo uint32_t Reserved2[2];		//
__vo uint32_t AHB1ENR;			//RCC AHB1 peripheral clock enable register
__vo uint32_t AHB2ENR;			//RCC AHB2 peripheral clock enable register
__vo uint32_t Reserved3[2];		//
__vo uint32_t APB1ENR;			//RCC APB1 peripheral clock enable register
__vo uint32_t APB2ENR;			//RCC APB2 peripheral clock enable register
__vo uint32_t Reserved4[2];		//
__vo uint32_t AHB1LPENR;		//RCC AHB1 peripheral clock enable in low power mode register
__vo uint32_t AHB2LPENR;		//RCC AHB2 peripheral clock enable in low power mode register
__vo uint32_t Reserved5[2];		//
__vo uint32_t APB1LPENR;		//RCC APB1 peripheral clock enable in low power mode register
__vo uint32_t APB2LPENR;		//RCC APB2 peripheral clock enabled in low power mode register
__vo uint32_t Reserved6[2];		//
__vo uint32_t BDCR;				//RCC Backup domain control register
__vo uint32_t CSR;				//RCC clock control & status register
__vo uint32_t Reserved7[2];		//
__vo uint32_t SSCGR;			//RCC spread spectrum clock generation register
__vo uint32_t PLLI2SCFGR;		//RCC PLLI2S configuration register
__vo uint32_t Reserved8;		//
__vo uint32_t DCKCFGR;			//RCC Dedicated Clocks Configuration Register
}RCC_RegDef_t;

typedef struct{					//As the size of variables is uint32 so an offset of 0x04 is directly applied while dereferencing
__vo uint32_t IMR;			//Interrupt mask register
__vo uint32_t EMR;			//Event mask register
__vo uint32_t RTSR;		//Rising trigger selection register
__vo uint32_t FTSR;		//Falling trigger selection register
__vo uint32_t SWIER;		//Software interrupt event register
__vo uint32_t PR;			//Pending register
}EXTI_RegDef_t;

typedef struct{				//As the size of variables is uint32 so an offset of 0x04 is directly applied while dereferencing
__vo uint32_t MEMRMP;		//SYSCFG memory remap register
__vo uint32_t PMC;			//SYSCFG peripheral mode configuration register
__vo uint32_t EXTICR[4];	//SYSCFG external interrupt configuration register 1, 2, 3, 4
__vo uint32_t Reserved1[2];
__vo uint32_t CMPCR;		//Compensation cell control register
}SYSCHG_RegDef_t;

#define GPIOA 	((GPIO_RegDef_t*)GPIOA_BASE_ADDRESS)
#define GPIOB 	((GPIO_RegDef_t*)GPIOB_BASE_ADDRESS)
#define GPIOC 	((GPIO_RegDef_t*)GPIOC_BASE_ADDRESS)
#define GPIOD 	((GPIO_RegDef_t*)GPIOD_BASE_ADDRESS)
#define GPIOE 	((GPIO_RegDef_t*)GPIOE_BASE_ADDRESS)
#define GPIOH 	((GPIO_RegDef_t*)GPIOH_BASE_ADDRESS)
#define RCC	 	((RCC_RegDef_t*)RCC_BASE_ADDRESS)
#define EXTI	((EXTI_RegDef_t*)EXTI_BASE_ADDRESS)
#define SYSCFG  ((SYSCHG_RegDef_t*)SYSCFG_BASE_ADDRESS	)

/*Clock enable and disable macro for GPIO MACRO*/
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1<<0)) //GPIOA peripheral clock enable
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1<<1)) //GPIOB peripheral clock enable
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1<<2)) //GPIOC peripheral clock enable
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1<<3)) //GPIOD peripheral clock enable
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1<<4)) //GPIOE peripheral clock enable
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1<<7)) //GPIOH peripheral clock enable

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1<<0)) //GPIOA peripheral clock disable
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1<<1)) //GPIOB peripheral clock disable
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1<<2)) //GPIOC peripheral clock disable
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1<<3)) //GPIOD peripheral clock disable
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1<<4)) //GPIOE peripheral clock disable
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1<<7)) //GPIOH peripheral clock disable

/*Clock enable macro for I2C MACRO*/
#define I2C1_PCLK_EN() 	(RCC->APB1ENR |= (1<<21)) //I2C1 peripheral clock enable
#define I2C2_PCLK_EN() 	(RCC->APB1ENR |= (1<<22)) //I2C2 peripheral clock enable
#define I2C3_PCLK_EN() 	(RCC->APB1ENR |= (1<<23)) //I2C3 peripheral clock enable

#define I2C1_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<21)) //I2C1 peripheral clock disable
#define I2C2_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<22)) //I2C2 peripheral clock disable
#define I2C3_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<23)) //I2C3 peripheral clock disable

/*Clock enable macro for SPI MACRO*/
#define SPI1_PCLK_EN() 	(RCC->APB2ENR |= (1<<12)) //SPI1 peripheral clock enable
#define SPI2_PCLK_EN() 	(RCC->APB1ENR |= (1<<14)) //SPI2 peripheral clock enable
#define SPI3_PCLK_EN() 	(RCC->APB1ENR |= (1<<15)) //SPI3 peripheral clock enable
#define SPI4_PCLK_EN() 	(RCC->APB2ENR |= (1<<13)) //SPI3 peripheral clock enable
#define SPI5_PCLK_EN() 	(RCC->APB2ENR |= (1<<20)) //SPI3 peripheral clock enable

#define SPI1_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<12)) //SPI1 peripheral clock disable
#define SPI2_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<14)) //SPI2 peripheral clock disable
#define SPI3_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<15)) //SPI3 peripheral clock disable
#define SPI4_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<13)) //SPI3 peripheral clock disable
#define SPI5_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<20)) //SPI3 peripheral clock disable

/*Clock enable macro for USART MACRO*/
#define USART1_PCLK_EN() 	(RCC->APB2ENR |= (1<<4)) 	//USART1 peripheral clock enable
#define USART2_PCLK_EN() 	(RCC->APB1ENR |= (1<<17)) 	//USART2 peripheral clock enable
#define USART6_PCLK_EN() 	(RCC->APB2ENR |= (1<<5)) 	//USART6 peripheral clock enable

#define USART1_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<4)) 	//USART1 peripheral clock disable
#define USART2_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<17)) 	//USART2 peripheral clock disable
#define USART6_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<5)) 	//USART6 peripheral clock disable

/*Clock enable macro for SYSCFG MACRO*/
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1<<14)) 	//SYSCFG peripheral clock enable

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1<<14)) 	//SYSCFG peripheral clock disable

/*Macros to reset GPIO Peripherals*/
#define GPIOA_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)

/*
 * This macro returns a code corresponding to the GPIO port base address
 */
#define GPIO_BASEADDR_TO_CODE(x) 	 (	(x == GPIOA) ? 0: \
										(x == GPIOB) ? 1: \
										(x == GPIOC) ? 2: \
										(x == GPIOD) ? 3: \
										(x == GPIOE) ? 4: \
										(x == GPIOH) ? 7: 0	)

/*
 * Interrupt Request number for STM32F411RE
 */
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI5_9 		23
#define IRQ_NO_EXTI10_15	40

#define NVIC_IRQ_PRI0 		0
#define NVIC_IRQ_PRI1 		1
#define NVIC_IRQ_PRI2 		2
#define NVIC_IRQ_PRI3 		3
#define NVIC_IRQ_PRI4 		4
#define NVIC_IRQ_PRI5 		5
#define NVIC_IRQ_PRI6 		6
#define NVIC_IRQ_PRI7 		7
#define NVIC_IRQ_PRI8 		8
#define NVIC_IRQ_PRI9 		9
#define NVIC_IRQ_PRI10 		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12 		12
#define NVIC_IRQ_PRI13 		13
#define NVIC_IRQ_PRI14 		14
#define NVIC_IRQ_PRI15 		15

#define ENABLE 				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET 		RESET


#include "STM32F411RE_DRIVER_GPIO.h"



#endif /* INC_STM32F411RE_H_ */
