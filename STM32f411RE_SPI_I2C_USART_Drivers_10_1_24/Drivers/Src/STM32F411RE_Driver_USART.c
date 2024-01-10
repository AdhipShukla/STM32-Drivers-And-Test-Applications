#include "STM32F411RE_Driver_USART.h"
#define __weak __attribute__((weak))

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnORDi){
	if(EnORDi == ENABLE){
		if(pUSARTx == USART1){
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2){
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART6){
			USART6_PCLK_EN();
		}
	}else{
		if(pUSARTx == USART1){
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2){
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART6){
			USART6_PCLK_DI();
		}
	}
}

void USART_Init(USART_Handle_t *pUSARTHandle){
	uint32_t tempReg = 0;
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enabling the UART engine
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX){
		tempReg|=(1<<UASRT_CR1_RE);
	} else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX){
		tempReg|=(1<<UASRT_CR1_TE);
	} else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX){
		tempReg|=((1<<UASRT_CR1_TE) | (1<<UASRT_CR1_RE));
	}

	//Configuring the word length
	tempReg |=	pUSARTHandle->USART_Config.USART_WordLength<<UASRT_CR1_M;

	//Configuring the parity control bit field
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN){
		//Implement the code to enable parity control
		tempReg |=(1<<UASRT_CR1_PCE);
		tempReg &= ~(1<<UASRT_CR1_PS);
	}else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD){
		//Implement the code to enable parity control
		tempReg |=(1<<UASRT_CR1_PCE);
		tempReg |=(1<<UASRT_CR1_PS);
	}

	//Program the CR1 register
	pUSARTHandle->pUSARTx->USART_CR1 = tempReg;

	//Configuring the CR2 register
	tempReg=0;
	//Configuring the number of STOP bits
	tempReg |= pUSARTHandle->USART_Config.USART_NoOfStopBits<<UASRT_CR2_STOP;
	pUSARTHandle->pUSARTx->USART_CR2 = tempReg;

	//Configuring the CR3 register
	tempReg=0;
	//Configuring USART hardware flow control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS){
		tempReg |= (1<<UASRT_CR3_CTSE);
	} else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS){
		tempReg |= (1<<UASRT_CR3_RTSE);
	}	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS){
		tempReg |= ((1<<UASRT_CR3_RTSE) | (1<<UASRT_CR3_CTSE));
	}
	pUSARTHandle->pUSARTx->USART_CR3 = tempReg;

	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

void USART_DeInit(USART_RegDef_t *pUSARTx){
	if(pUSARTx == USART1){
		USART1_REG_RESET();
	}
	else if(pUSARTx == USART2){
		USART2_REG_RESET();
	}
	else if(pUSARTx == USART6){
		USART6_REG_RESET();
	}
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName){
	if(pUSARTx->USART_SR & StatusFlagName){
			return 1;
		}
		return 0;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName){
	pUSARTx->USART_SR &= ~StatusFlagName;
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx,  uint8_t EnORDi){
	if(EnORDi == ENABLE)
		pUSARTx->USART_CR1 |= (1<<UASRT_CR1_UE);
	else
		pUSARTx->USART_CR1 &= ~(1<<UASRT_CR1_UE);
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){
	uint32_t PCLKx;
	uint32_t usartDiv;
	uint32_t M_part, F_part;
	uint32_t tempReg=0;

	if(pUSARTx == USART1 || pUSARTx == USART6){ //APB2
		PCLKx = RCC_GetPCLK2Value();
	} else{
		PCLKx = RCC_GetPCLK1Value();
	}
	if(pUSARTx->USART_CR1 & (1<<UASRT_CR1_OVER8)){
		usartDiv = ((25*PCLKx)/(2*BaudRate)); // (PCLKx/8/BaudRate)*100
	} else {
		usartDiv = ((25*PCLKx)/(4*BaudRate)); // (PCLKx/16/BaudRate)*100
	}
	M_part = (int)(usartDiv/100);
	tempReg|= (M_part<<4); //vacating the bytes for the fraction part
	F_part = (usartDiv - M_part*100);

	if (pUSARTx->USART_CR1 & (1<<UASRT_CR1_OVER8)){
		F_part = ((F_part*8 + 50)/100) & (uint8_t)(0x07); //Last 3 bytes
	} else {
		F_part = ((F_part*16 + 50)/100) & (uint8_t)(0x0f); //Last 4 bytes
	}
	tempReg|=F_part;
	pUSARTx->USART_BRR = tempReg;
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint16_t *pdata;
	for (uint32_t i = 0; i<Len; i++){
		//Wait until TXE flag is set
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,UASRT_FLAG_TXE));
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
			//if the data length is 9 bits then load DR with 2 bytes masking the bits other than first 9 bits
			pdata = (uint16_t *)pTxBuffer;
			pUSARTHandle->pUSARTx->USART_DR = ((*pdata) & (uint16_t)0x1FF);
			//Checking for USART parity control
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				//No parity is used in this transfer. So 9 bits of User data  will be sent
				pTxBuffer++; pTxBuffer++;
			} else {
				//The 9th bit will be replaced by the parity bit by the hardware
				pTxBuffer++;
			}
		} else {
			//8 bit of data transfer
			pUSARTHandle->pUSARTx->USART_DR = (uint32_t)(*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, UASRT_FLAG_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	for(uint32_t i=0; i<Len; i++){
		//Waiting for the data to be arrived in the data register
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, UASRT_FLAG_RXNE));
		//Checking whether to receive 8 or 9 bits
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
			//Checking for USART parity control
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				//No parity is used in this transfer. So 9 bits of User data  are received
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->USART_DR & (uint16_t)0x01FF);
				//Incrementing the RxBuffer twice
				pRxBuffer++; pRxBuffer++;
			} else {
				//The 9th bit will be replaced by the parity bit by the hardware
				*pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		} else {
			//8 bit of data transfer
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				//No parity is used in this transfer. So 8 bits of User data  are received
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
			} else {
				//The 7th bit will be replaced by the parity bit by the hardware
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0x7F);
			}
			//Incrementing the RxBuffer
			pRxBuffer++;
		}
	}
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t txState = pUSARTHandle->TxBusyState;
	if(txState != USART_BUSY_IN_TX){
		pUSARTHandle->TxLen	= Len;
		pUSARTHandle->pTxBuffer	= pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Enabling the interrupt for TXE
		pUSARTHandle->pUSARTx->USART_CR1 |= (1<<UASRT_CR1_TXEIE);
		//Enabling the interrupt for Transmission complete
		pUSARTHandle->pUSARTx->USART_CR1 |= (1<<UASRT_CR1_TCIE);
	}
	return txState;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t rxState = pUSARTHandle->RxBusyState;
	if(rxState != USART_BUSY_IN_RX){
		pUSARTHandle->RxLen	= Len;
		pUSARTHandle->pRxBuffer	= pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;
		(void)pUSARTHandle->pUSARTx->USART_DR; // ???
		//Enabling the interrupt for RXNE
		pUSARTHandle->pUSARTx->USART_CR1 |= (1<<UASRT_CR1_RXNEIE);
	}
	return rxState;
}
void USART_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnORDi){
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

void USART_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t RegNum = IRQNumber/4;
	uint8_t RegOffest = IRQNumber%4;
	*(NVIC_IPR0_ADDR + RegNum) |= (IRQPriority<<((RegOffest*8) + (8-NO_PR_BITS_IMPL)));
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle){
	uint32_t temp1, temp2, temp3;
	uint16_t *pData;
	//Check for TC flag
	temp1 = pUSARTHandle->pUSARTx->USART_SR & (1<<UASRT_SR_TC);
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1<<UASRT_CR1_TCIE);
	if(temp1 && temp2){
		//Interrupt because of the transmission complete
		if(pUSARTHandle->TxBusyState == I2C_BUSY_IN_TX){
			if(!pUSARTHandle->TxLen){
				//Clearing the TC flag
				pUSARTHandle->pUSARTx->USART_SR &= ~UASRT_FLAG_TC;
				//Clearing the TCIE control bit
				pUSARTHandle->pUSARTx->USART_CR1 &= ~(1<<UASRT_CR1_TCIE);
				//Reset the application state
				pUSARTHandle->TxBusyState = USART_BUSY_READY;
				//Reset the buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;
				//Reset the length of the buffer
				pUSARTHandle->TxLen = 0;
				//Call the application call back with the event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	//Check for TXE flag
	temp1 = pUSARTHandle->pUSARTx->USART_SR & UASRT_FLAG_TXE;
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1<<UASRT_CR1_TXEIE);
	if(temp1 && temp2){
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX){
			if(pUSARTHandle->TxLen>0){
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
					pData = (uint16_t*)pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->USART_DR = (*pData & (uint16_t)0x01FF);
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						pUSARTHandle->pTxBuffer += 2;
						pUSARTHandle->TxLen -= 2;
					} else {
						pUSARTHandle->pTxBuffer += 1;
						pUSARTHandle->TxLen -= 1;
					}
				} else {
					pUSARTHandle->pUSARTx->USART_DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);
					pUSARTHandle->pTxBuffer += 1;
					pUSARTHandle->TxLen -= 1;
				}
			}
			if (pUSARTHandle->TxLen==0){
				pUSARTHandle->pUSARTx->USART_CR1 &= ~(1<<UASRT_CR1_TXEIE);
			}
		}
	}

	//Check for the RXNE
	temp1 = pUSARTHandle->pUSARTx->USART_SR & UASRT_FLAG_RXNE;
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1<<UASRT_CR1_RXNEIE);
	if(temp1 && temp2){
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX){
			if(pUSARTHandle->RxLen > 0){
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						*(uint16_t*)pUSARTHandle->pRxBuffer = pUSARTHandle->pUSARTx->USART_DR & (uint16_t)0x1FF;
						pUSARTHandle->pRxBuffer += 2;
						pUSARTHandle->RxLen -= 2;
					} else {
						*pUSARTHandle->pRxBuffer = pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF;
						pUSARTHandle->pRxBuffer += 1;
						pUSARTHandle->RxLen -= 1;
					}
				} else {
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						*pUSARTHandle->pRxBuffer = pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF;
						pUSARTHandle->pRxBuffer += 1;
						pUSARTHandle->RxLen -= 1;
					} else {
						*pUSARTHandle->pRxBuffer = pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0x7F;
						pUSARTHandle->pRxBuffer += 1;
						pUSARTHandle->RxLen -= 1;
					}
				}
			}
			if(pUSARTHandle->RxLen == 0){
				pUSARTHandle->pUSARTx->USART_CR1 &= ~(1<<UASRT_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_BUSY_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	//Check for CTS status
	temp1 = pUSARTHandle->pUSARTx->USART_SR & UASRT_FLAG_CTS;
	temp2 = pUSARTHandle->pUSARTx->USART_CR3 & (1<<UASRT_CR3_CTSE);
	temp3 = pUSARTHandle->pUSARTx->USART_CR3 & (1<<UASRT_CR3_CTSIE);
	if(temp1 && temp2 && temp3){
		pUSARTHandle->pUSARTx->USART_SR &= ~UASRT_FLAG_RXNE;
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

	//Check for Idle Detection
	temp1 = pUSARTHandle->pUSARTx->USART_SR & UASRT_FLAG_IDLE;
	temp2 =	pUSARTHandle->pUSARTx->USART_CR1 & (1<<UASRT_CR1_IDLEIE);
	uint32_t temp;
	if(temp1 && temp2){
		temp = pUSARTHandle->pUSARTx->USART_SR;
		temp = pUSARTHandle->pUSARTx->USART_DR;
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	//Check for overrun Detection
	temp1 = pUSARTHandle->pUSARTx->USART_SR & UASRT_FLAG_OVR;
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1<<UASRT_CR1_RXNEIE);
	if(temp1 && temp2){
		/*temp = pUSARTHandle->pUSARTx->USART_SR;
		temp = pUSARTHandle->pUSARTx->USART_DR;*/ //This can be cleared by the application
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

	//Error in multi-buffer case
	//Check for Idle Detection
	temp2 =	pUSARTHandle->pUSARTx->USART_CR3 & (1<<UASRT_CR3_EIE);
	if(temp2){
		temp1 = pUSARTHandle->pUSARTx->USART_SR;
		if(temp1 & UASRT_FLAG_FE){//Set by hardware when a break character is detected.
			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
		}
		if(temp1 & UASRT_FLAG_NF){//Hardware noise is detected on the receive frame
			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
		}
		if(temp1 & UASRT_FLAG_OVR){
			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
		}
	}
}

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv){

}
