/*
 * STM32F411RE_GPIO.c
 *
 *  Created on: Dec 24, 2023
 *      Author: adhip
 */

#include "STM32F411RE_DRIVER_GPIO.h"

/*Peripheral Clock Setup*/
/*******************************************************************
 * @fn 				- GPIO_PeriClockControl
 * @brief 			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of gpio peripheral
 * @param[in]		- ENABLE or DISABLE macro
 * @return 			- none
 *
 * @Note			- none
********************************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnORDi){
	if(EnORDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}

/*GPIO Init*/
/*******************************************************************
 * @fn 				- GPIO_Init
 * @brief 			- This function initializes GPIO pin's mode, speed, pull up/ pull down, output type, alternate functionality
 *
 * @param[in]		- GPIO Pin handle
 * @param[in]		-
 * @return 			- none
 *
 * @Note			- none
********************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	// Setting pin mode
	uint32_t temp =0;
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE); //Enabling the clock peripheral inside init

	if(pGPIOHandle->GPIO_Config.GPIO_PinMode<=GPIO_MODE_ANALOG){
		temp = (pGPIOHandle->GPIO_Config.GPIO_PinMode << (2 * pGPIOHandle->GPIO_Config.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2 * pGPIOHandle->GPIO_Config.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else{ //These are interrupt modes
		if(pGPIOHandle->GPIO_Config.GPIO_PinMode==GPIO_MODE_IT_FT){
		//1. Configuring the FTSR
			EXTI->FTSR &= ~(0X01<<pGPIOHandle->GPIO_Config.GPIO_PinNumber);
			EXTI->RTSR &= ~(0X01<<pGPIOHandle->GPIO_Config.GPIO_PinNumber);
			EXTI->FTSR |=  (0X01<<pGPIOHandle->GPIO_Config.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_Config.GPIO_PinMode==GPIO_MODE_IT_RT){
		//1. Configuring the RTSR
			EXTI->FTSR &= ~(0X01<<pGPIOHandle->GPIO_Config.GPIO_PinNumber);
			EXTI->RTSR &= ~(0X01<<pGPIOHandle->GPIO_Config.GPIO_PinNumber);
			EXTI->RTSR |=  (0X01<<pGPIOHandle->GPIO_Config.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_Config.GPIO_PinMode==GPIO_MODE_IT_RFT){
		//1. Configuring the RFTS
			EXTI->RTSR |=  (0X01<<pGPIOHandle->GPIO_Config.GPIO_PinNumber);
			EXTI->FTSR |=  (0X01<<pGPIOHandle->GPIO_Config.GPIO_PinNumber);
		}

		//2. Configure the GPIO port. By default EXTI is configured to port A for every pin
		uint8_t temp1 = pGPIOHandle->GPIO_Config.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_Config.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);

		//3. Enabling the EXTI interrupt delivery using IMR
		EXTI->IMR |=  (0X01<<pGPIOHandle->GPIO_Config.GPIO_PinNumber);
	}

	// Setting pin speed
	temp = pGPIOHandle->GPIO_Config.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;


	// Setting pin pull up/ pull down
	temp = pGPIOHandle->GPIO_Config.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;


	// Setting pin output type
	temp = pGPIOHandle->GPIO_Config.GPIO_PinOPType << (pGPIOHandle->GPIO_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x03 << (2 * pGPIOHandle->GPIO_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;


	// Setting pin alternate function
	if(pGPIOHandle->GPIO_Config.GPIO_PinMode==GPIO_MODE_ALTFN){

		if(pGPIOHandle->GPIO_Config.GPIO_PinNumber<8){
			temp = pGPIOHandle->GPIO_Config.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_Config.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->AFRL &= ~(0x0f << (4 * pGPIOHandle->GPIO_Config.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFRL |= temp;
		} else {
			temp = pGPIOHandle->GPIO_Config.GPIO_PinAltFunMode << (4 *(pGPIOHandle->GPIO_Config.GPIO_PinNumber - 8));
			pGPIOHandle->pGPIOx->AFRH &= ~(0x0f << (4 * (pGPIOHandle->GPIO_Config.GPIO_PinNumber%8)));
			pGPIOHandle->pGPIOx->AFRH |= temp;
		}
	}
}


/*GPIO DeInit*/
/*******************************************************************
 * @fn 				- GPIO_Init
 * @brief 			- This function initializes resets the particular GPIO port
 *
 * @param[in]		- GPIO Register base address
 * @param[in]		-
 * @return 			- none
 *
 * @Note			- none
********************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
}

/*GPIO Read and Write*/
/*******************************************************************
 * @fn 				- GPIO_Init
 * @brief 			- This function initializes resets the particular GPIO port
 *
 * @param[in]		- GPIO Register base address
 * @param[in]		- GPIO PIN Number
 * @return 			- 0 or 1
 *
 * @Note			- none
********************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR; //Check if this need to be shifted by 16 bits
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){//Write 1
		pGPIOx->ODR |= (1<<PinNumber);
	}else{//white0
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR=Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR = pGPIOx->ODR ^ (1<<PinNumber);
}

/*GPIO IRQ and ISR handling*/
void GPIO_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnORDi){
	if (EnORDi == ENABLE){
		if(IRQNumber<32){
			//Program ISER0 register
			*NVIC_ISER0_ADDR |= (1<<IRQNumber%32);
		}else if (IRQNumber>31 && IRQNumber<64){
			//Program ISER1 register
			*NVIC_ISER1_ADDR |= (1<<IRQNumber%32);
		}else if (IRQNumber>63 && IRQNumber<96){
			//Program ISER2 register
			*NVIC_ISER0_ADDR |= (1<<IRQNumber%32);
		}
	} else {
		if(IRQNumber<32){
			//Program ICER0 register
			*NVIC_ICER0_ADDR |= (1<<IRQNumber%32);
		}else if (IRQNumber>31 && IRQNumber<64){
			//Program ICER1 register
			*NVIC_ICER1_ADDR |= (1<<IRQNumber%32);
		}else if (IRQNumber>63 && IRQNumber<96){
			//Program ICER2 register
			*NVIC_ICER0_ADDR |= (1<<IRQNumber%32);
		}
	}
}

void GPIO_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority){
	if (IRQNumber<=59){
		uint8_t RegNum = IRQNumber/4;
		uint8_t RegOffest = IRQNumber%4;
		*(NVIC_IPR0_ADDR + RegNum) |= (IRQPriority<<((RegOffest*8) + (8-NO_PR_BITS_IMPL)));
	}
}

void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR & (1<<PinNumber)){
		EXTI->PR |= (1<<PinNumber);
	}
}


