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
#include <stddef.h>
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

/*Defining the base address of all the specific registers of the GPIO peripherals using a structure*/
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

/*Defining the base address of all the specific RCC registers*/
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

/*Defining the base address of all the specific EXTI registers using a structure*/
typedef struct{					//As the size of variables is uint32 so an offset of 0x04 is directly applied while dereferencing
__vo uint32_t IMR;				//Interrupt mask register
__vo uint32_t EMR;				//Event mask register
__vo uint32_t RTSR;				//Rising trigger selection register
__vo uint32_t FTSR;				//Falling trigger selection register
__vo uint32_t SWIER;			//Software interrupt event register
__vo uint32_t PR;				//Pending register
}EXTI_RegDef_t;

/*Defining the base address of all the specific SYSCFG registers using a structure*/
typedef struct{					//As the size of variables is uint32 so an offset of 0x04 is directly applied while dereferencing
__vo uint32_t MEMRMP;			//SYSCFG memory remap register
__vo uint32_t PMC;				//SYSCFG peripheral mode configuration register
__vo uint32_t EXTICR[4];		//SYSCFG external interrupt configuration register 1, 2, 3, 4
__vo uint32_t Reserved1[2];
__vo uint32_t CMPCR;			//Compensation cell control register
}SYSCHG_RegDef_t;

/*Defining the base address of all the specific SPI registers using a structure*/
typedef struct{
__vo uint32_t SPI_CR1;			// SPI control register 1
__vo uint32_t SPI_CR2;			// SPI control register 2
__vo uint32_t SPI_SR;			// SPI status register
__vo uint32_t SPI_DR;			// SPI data register
__vo uint32_t SPI_CRCPR;		// SPI CRC polynomial register
__vo uint32_t SPI_RXCRCR;		// SPI RX CRC register
__vo uint32_t SPI_TXCRCR;		// SPI TX CRC register
__vo uint32_t SPI_I2SCFGR;		// SPI_I2S configuration register
__vo uint32_t SPI_I2SPR;		// SPI_I2S prescaler register
}SPI_RegDef_t;

/*Defining the base address of all the specific I2C registers using a structure*/
typedef struct{
__vo uint32_t I2C_CR1;				//I2C Control register 1
__vo uint32_t I2C_CR2;				//I2C Control register 2
__vo uint32_t I2C_OAR1;				//I2C Own address register 1
__vo uint32_t I2C_OAR2;				//2C Own address register 2
__vo uint32_t I2C_DR;				//I2C Data register
__vo uint32_t I2C_SR1;				//I2C Status register 1
__vo uint32_t I2C_SR2;				//I2C Status register 2
__vo uint32_t I2C_CCR;				//I2C Clock control register
__vo uint32_t I2C_TRISE;			//I2C TRISE register
__vo uint32_t I2C_FLTR;				//I2C FLTR register
}I2C_RegDef_t;

/*Defining the base address of all the specific UART register using a structure*/
typedef struct{
__vo uint32_t USART_SR;				//Status register
__vo uint32_t USART_DR;				//Data register
__vo uint32_t USART_BRR;			//Baud rate register
__vo uint32_t USART_CR1;			//Control register 1
__vo uint32_t USART_CR2;			//Control register 2
__vo uint32_t USART_CR3;			//Control register 3
__vo uint32_t USART_GTPR;			//Guard time and prescaler register
}USART_RegDef_t;

/*Declaring the base peripheral address as their specific structure type*/
#define GPIOA 	((GPIO_RegDef_t*)GPIOA_BASE_ADDRESS)
#define GPIOB 	((GPIO_RegDef_t*)GPIOB_BASE_ADDRESS)
#define GPIOC 	((GPIO_RegDef_t*)GPIOC_BASE_ADDRESS)
#define GPIOD 	((GPIO_RegDef_t*)GPIOD_BASE_ADDRESS)
#define GPIOE 	((GPIO_RegDef_t*)GPIOE_BASE_ADDRESS)
#define GPIOH 	((GPIO_RegDef_t*)GPIOH_BASE_ADDRESS)
#define RCC	 	((RCC_RegDef_t*)RCC_BASE_ADDRESS)
#define EXTI	((EXTI_RegDef_t*)EXTI_BASE_ADDRESS)
#define SYSCFG  ((SYSCHG_RegDef_t*)SYSCFG_BASE_ADDRESS)
#define SPI1	((SPI_RegDef_t*)SPI1_BASE_ADDRESS)
#define SPI2	((SPI_RegDef_t*)SPI2_BASE_ADDRESS)
#define SPI3	((SPI_RegDef_t*)SPI3_BASE_ADDRESS)
#define SPI4	((SPI_RegDef_t*)SPI4_BASE_ADDRESS)
#define SPI5	((SPI_RegDef_t*)SPI5_BASE_ADDRESS)
#define I2C1	((I2C_RegDef_t*)I2C1_BASE_ADDRESS)
#define I2C2	((I2C_RegDef_t*)I2C2_BASE_ADDRESS)
#define I2C3	((I2C_RegDef_t*)I2C3_BASE_ADDRESS)
#define USART1	((USART_RegDef_t*)USART1_BASE_ADDRESS)
#define USART2	((USART_RegDef_t*)USART2_BASE_ADDRESS)
#define USART6	((USART_RegDef_t*)USART6_BASE_ADDRESS)

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

/*Macros to reset GPIO Peripherals******************* To reset first the bit has to be set to 1 then reset to 0*/
#define GPIOA_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)

/*Macros to reset SPI Peripherals*/
#define SPI1_REG_RESET() 	do{(RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET() 	do{(RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET() 	do{(RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET() 	do{(RCC->APB2RSTR |= (1<<13)); (RCC->APB2RSTR &= ~(1<<13));}while(0)
#define SPI5_REG_RESET() 	do{(RCC->APB2RSTR |= (1<<20)); (RCC->APB2RSTR &= ~(1<<20));}while(0)

/*Macros to reset I2C Peripherals*/
#define I2C1_REG_RESET() 	do{(RCC->APB1RSTR |= (1<<21)); (RCC->APB1RSTR &= ~(1<<21));}while(0)
#define I2C2_REG_RESET() 	do{(RCC->APB1RSTR |= (1<<22)); (RCC->APB1RSTR &= ~(1<<22));}while(0)
#define I2C3_REG_RESET() 	do{(RCC->APB1RSTR |= (1<<23)); (RCC->APB1RSTR &= ~(1<<23));}while(0)

/*Macros to rest USART peripherals*/
#define USART1_REG_RESET() 	do{(RCC->APB2RSTR |= (1<<04)); (RCC->APB1RSTR &= ~(1<<04));}while(0)
#define USART2_REG_RESET() 	do{(RCC->APB1RSTR |= (1<<17)); (RCC->APB1RSTR &= ~(1<<17));}while(0)
#define USART6_REG_RESET() 	do{(RCC->APB2RSTR |= (1<<05)); (RCC->APB1RSTR &= ~(1<<05));}while(0)

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

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84
#define IRQ_NO_SPI5			85

#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73

#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART6		71

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

/*
 * Macros that can be used when calling APIs
 */
#define ENABLE 				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE

/*
 * Macros that can be used when calling	GPIOs
 */
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET 		RESET
/*
 *
 */
#define PULL_TO_GND			RESET
#define PULL_TO_HIGH		SET
/*
 * Macros used for defining the bits for various SPI register
 */
#define SPI_CR1_CPHA		0			//Clock phase
#define SPI_CR1_CPOL		1			//Clock polarity
#define SPI_CR1_MSTR		2			//Master selection
#define SPI_CR1_BR			3			//Baud rate control
#define SPI_CR1_SPE			6			//SPI enable
#define SPI_CR1_SSI			8			//Internal slave select
#define SPI_CR1_SSM			9			//Software slave management
#define SPI_CR1_RXONLY		10			//Receive only
#define SPI_CR1_DFF			11			//Data frame format
#define SPI_CR1_BIDIOE		14			//Output enable in bidirectional mode
#define SPI_CR1_BIDIMODE	15			//Bidirectional data mode enable

#define SPI_CR2_RXDMAEN		0			//Rx buffer DMA enable
#define SPI_CR2_TXDMAEN		1			//Tx buffer DMA enable
#define SPI_CR2_SSOE		2			//SS output enable
#define SPI_CR2_FRF			3			//Frame format
#define SPI_CR2_ERRIE		4			//Error interrupt enable
#define SPI_CR2_RXNEIE		6			//RX buffer not empty interrupt enable
#define SPI_CR2_TXEIE		7			//Tx buffer empty interrupt enable

#define SPI_SR_RXNE			0			//Receive buffer not empty
#define SPI_SR_TXE			1			//Transmit buffer empty
#define SPI_SR_CHSIDE		2			//Channel side
#define SPI_SR_UDR			3			//Underrun flag
#define SPI_SR_CRCERR		4			//CRC error flag
#define SPI_SR_MODF			5			//Mode fault
#define SPI_SR_OVR			6			//Overrun flag
#define SPI_SR_BSY			7			//Busy flag
#define SPI_SR_FRE			8			//Frame format error

#define I2C_CR1_SWRST		15			//Software Reset
#define	I2C_CR1_Res1		14			//Reserved 1
#define I2C_CR1_Alert		13			//SMBus alert
#define I2C_CR1_PEC			12			//Packet Error Checking
#define I2C_CR1_POS			11			//Acknowledgement PEC position
#define I2C_CR1_ACK			10			//Acknowledge enable
#define I2C_CR1_STOP		9			//STOP Generation
#define I2C_CR1_START		8			//Start Generation
#define I2C_CR1_NOSTRETCH	7			//Clock stretching disable (Slave mode)
#define I2C_CR1_ENGC		6			//General call enable
#define I2C_CR1_ENPEC		5			//PEC enable
#define I2C_CR1_ENARP		4			//ARP enable
#define I2C_CR1_SMBTYPE		3			//SMBus type
#define I2C_CR1_Res2		2			//Reserved 2
#define I2C_CR1_SMBUS		1			//SMBus mode
#define I2C_CR1_PE			0			//Peripheral enable

#define I2C_CR2_LAST		12			//DMA last transfer
#define I2C_CR2_DMAEN		11			//DMA requests enable
#define I2C_CR2_ITBUFEN		10			//Buffer interrupt enable
#define I2C_CR2_ITEVTEN		9			//Event interrupt enable
#define I2C_CR2_ITERREN		8			//Error interrupt enable
#define I2C_CR2_FREQ		0			//Peripheral clock frequency

#define I2C_SR1_SMBALERT	15			//SMBus alert
#define I2C_SR1_TIMEOUT		14			//Timeout or Tlow error
#define I2C_SR1_PECERR		12			//PEC Error in reception
#define I2C_SR1_OVR			11			//Overrun/Underrun
#define I2C_SR1_AF			10			//Acknowledge failure
#define I2C_SR1_ARLO		9			//Arbitration lost (master mode)
#define I2C_SR1_BERR		8			//Bus error
#define I2C_SR1_TxE			7			//Data register empty (transmitters)
#define I2C_SR1_RxNE		6			//Data register not empty (receivers)
#define I2C_SR1_STOPF		4			//Stop detection (slave mode)
#define I2C_SR1_ADD10		3			//10-bit header sent (Master mode)
#define I2C_SR1_BTF			2			//Byte transfer finished
#define I2C_SR1_ADDR		1			//Address sent (master mode)/matched (slave mode)
#define I2C_SR1_SB			0			//Start bit (Master mode)

#define I2C_SR2_PEC			8			//Packet error checking register
#define I2C_SR2_DUALF		7			//Dual flag (Slave mode)
#define I2C_SR2_SMBHOST		6			//SMBus host header (Slave mode)
#define I2C_SR2_SMBDEFAULT	5			//SMBus device default address (Slave mode)
#define I2C_SR2_GENCALL		4			//General call address (Slave mode)
#define I2C_SR2_TRA			2			//Transmitter/receiver
#define I2C_SR2_BUSY		1			//Bus busy
#define I2C_SR2_MSL			0			//Master/slave

#define I2C_CCR_FS			15			//I2C master mode selection
#define I2C_CCR_DUTY		14			//DUTY: Fm mode duty cycle
#define I2C_CCR_CCR			0			//Clock control register in Fm/Sm mode (Master mode)

#define UASRT_SR_CTS		9			//CTS flag
#define UASRT_SR_LBD		8			//LIN break detection flag
#define UASRT_SR_TXE		7			//Transmit data register empty
#define UASRT_SR_TC			6			//Transmission complete
#define UASRT_SR_RXNE		5			// Read data register not empty
#define UASRT_SR_IDLE		4			//IDLE line detected
#define UASRT_SR_OVR		3			//Overrun error
#define UASRT_SR_NF			2			//Noise detected flag
#define UASRT_SR_FE			1			//Framing error
#define UASRT_SR_PE			0			//Parity error

#define UASRT_CR1_OVER8		15			//Oversampling mode
#define UASRT_CR1_UE		13			//USART enable
#define UASRT_CR1_M			12			//Word length
#define UASRT_CR1_WAKE		11			//Wakeup method
#define UASRT_CR1_PCE		10			//Parity control enable
#define UASRT_CR1_PS		9			//Parity selection
#define UASRT_CR1_PEIE		8			//PE interrupt enable
#define UASRT_CR1_TXEIE		7			//TXE interrupt enable
#define UASRT_CR1_TCIE		6			//Transmission complete interrupt enable
#define UASRT_CR1_RXNEIE	5			//RXNE interrupt enable
#define UASRT_CR1_IDLEIE	4			//IDLE interrupt enable
#define UASRT_CR1_TE		3			//Transmitter enable
#define UASRT_CR1_RE		2			//Receiver enable
#define UASRT_CR1_RWU		1			//Receiver wakeup
#define UASRT_CR1_SBK		0			//Send break

#define UASRT_CR2_LINEN		14			//LIN mode enable
#define UASRT_CR2_STOP		12			//STOP bits
#define UASRT_CR2_CLKEN		11			//Clock enable
#define UASRT_CR2_CPOL		10			//Clock polarity
#define UASRT_CR2_CPHA		9			//Clock phase
#define UASRT_CR2_LBCL		8			//Last bit clock pulse
#define UASRT_CR2_LBDIE		6			//LIN break detection interrupt enable
#define UASRT_CR2_LBDL		5			//lin break detection length
#define UASRT_CR2_ADD		0			//Address of the USART node

#define UASRT_CR3_ONEBIT	11			//One sample bit method enable
#define UASRT_CR3_CTSIE		10			//CTS interrupt enable
#define UASRT_CR3_CTSE		9			//CTS enable
#define UASRT_CR3_RTSE		8			//RTS enable
#define UASRT_CR3_DMAT		7			//DMA enable transmitter
#define UASRT_CR3_DMAR		6			//DMA enable receiver
#define UASRT_CR3_SCEN		5			//Smartcard mode enable
#define UASRT_CR3_NACK		4			//Smartcard NACK enable
#define UASRT_CR3_HDSEL		3			//Half-duplex selection
#define UASRT_CR3_IRLP		2			//SIrDA low-power
#define UASRT_CR3_IREN		1			//IrDA mode enable
#define UASRT_CR3_EIE		0			//Error interrupt enable

#include "STM32F411RE_DRIVER_GPIO.h"
#include "STM32F411RE_Driver_SPI.h"
#include "STM32F411RE_Driver_I2C.h"
#include "STM32F411RE_Driver_USART.h"
#include "STM32F411RE_Driver_RCC.h"

#endif /* INC_STM32F411RE_H_ */
