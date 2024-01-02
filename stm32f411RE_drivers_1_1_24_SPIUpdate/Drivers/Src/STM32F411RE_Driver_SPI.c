/*
 * STM32F411RE_Driver_SPI.c
 *
 *  Created on: Dec 26, 2023
 *      Author: adhip
 */

#include "STM32F411RE_Driver_SPI.h"
#include <string.h>
#define __weak __attribute__((weak))
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
/*Peripheral Clock Setup*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnORDi){
	if(EnORDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
		else if(pSPIx == SPI5){
			SPI5_PCLK_EN();
		}
	}else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}
		else if(pSPIx == SPI5){
			SPI5_PCLK_DI();
		}
	}
}

/*Init and De-init*/
void SPI_Init(SPI_Handle_t *pSPIHandle){
	uint32_t tempReg=0;

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE); //Enabling the clock peripheral inside init

	//Configuring device mode
	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	//Configuring the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		tempReg &= ~(1<<SPI_CR1_BIDIMODE);//Clearing the BIDI value
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		tempReg |= (1<<SPI_CR1_BIDIMODE);//Setting the BIDI value
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		tempReg &= ~(1<<SPI_CR1_BIDIMODE);//Device is set in Full duplex with only single wire
		tempReg |= (1<<SPI_CR1_RXONLY);//Rx only for master
	}

	//Configuring SPI clock speed baud-rate
	tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR;

	//Configure the DFF
	tempReg |= pSPIHandle->SPIConfig.SPI_DFF<<SPI_CR1_DFF	;

	//Configure the CPOL
	tempReg |= pSPIHandle->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL;

	//Configure the CPHA
	tempReg |= pSPIHandle->SPIConfig.SPI_CPHA<<SPI_CR1_CPHA;

	//Configure the SSM
	tempReg |= pSPIHandle->SPIConfig.SPI_SSM<<SPI_CR1_SSM;

	//Setting the value of CR1 Reg
	pSPIHandle->pSPIx->SPI_CR1 = tempReg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4){
		SPI4_REG_RESET();
	}
	else if(pSPIx == SPI5){
		SPI5_REG_RESET();
	}
}

/* SPI Enable API, which is to be called after initialization*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnORDi){
	if (EnORDi == ENABLE){
		pSPIx->SPI_CR1 |= 1<<SPI_CR1_SPE;
	}else{
	//Write the code for disable following the routine
		pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

/*Data Send and Receive*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t ReverseByteOrder[2];
	//char Response[15] = {0};
	//uint8_t i=0;
	while(Len>0){
		while((~pSPIx->SPI_SR) & 1<<SPI_SR_TXE); //Making sure Tx buffer is empty
		if(pSPIx->SPI_CR1 & 1<<SPI_CR1_DFF){ //Check if the data frame format is 16 bits
			ReverseByteOrder[0]= *(pTxBuffer+1);
			ReverseByteOrder[1]= *(pTxBuffer);
			memcpy((void*)&pSPIx->SPI_DR,(void*)ReverseByteOrder,2);
			if(Len==1)
				Len -=1;
			else
				Len -=2;
			pTxBuffer +=2;
		}else{
			memcpy((void*)&pSPIx->SPI_DR,(void*)pTxBuffer,1);
			//memcpy((void*)(Response+i), (void*)&pSPIx->SPI_DR, 1);
			//pSPIx->SPI_DR = (uint32_t)*(uint8_t*)pTxBuffer;
			//Response[i] = pSPIx->SPI_DR;
			Len -=1;
			pTxBuffer +=1;
			//i++;
		}
	}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t ReverseByteOrder[2];
	while(Len>0){
		while(~(pSPIx->SPI_SR) & 1<<SPI_SR_RXNE); // Waiting for RxBuffer to get non-empty
		if(pSPIx->SPI_CR1 & 1<<SPI_CR1_DFF){ //Check if the data frame format is 16 bits
			memcpy((void*)ReverseByteOrder,(void*)&pSPIx->SPI_DR,2);
			*(pRxBuffer+1) = ReverseByteOrder[0];
			*(pRxBuffer) = ReverseByteOrder[1];
			if(Len==1)
				Len -=1;
			else
				Len -=2;
			pRxBuffer +=2;
		}else{
			memcpy((void*)pRxBuffer, (void*)&pSPIx->SPI_DR, 1);
			//pSPIx->SPI_DR = (uint32_t)*(uint8_t*)pTxBuffer;
			Len -=1;
			pRxBuffer +=1;
		}
	}
}

/*
 * Setting the SII pin when node's SSM bit is set
 */
void SPI_SSISet(SPI_RegDef_t *pSPIx, uint8_t EnORDi){
	if (EnORDi == SET){
		pSPIx->SPI_CR1 |= 1<<SPI_CR1_SSI;
	}else{
		pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_SSI);
	}
}
/*
 * SSOE Configuration
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnORDi){
	if (EnORDi == ENABLE){
		pSPIx->SPI_CR2 |= 1<<SPI_CR2_SSOE;
	}else{
		pSPIx->SPI_CR2 &= ~(1<<SPI_CR2_SSOE);
	}
}

/*SPI IRQ and ISR handling*/
void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnORDi){
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

void SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority){
		uint8_t RegNum = IRQNumber/4;
		uint8_t RegOffest = IRQNumber%4;
		*(NVIC_IPR0_ADDR + RegNum) |= (IRQPriority<<((RegOffest*8) + (8-NO_PR_BITS_IMPL)));
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){ //Only update if SPI state is free
		//Save Tx buffer and length information in global buffer
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen= Len;
		//Mark SPI state as busy so that no other code take over the same SPI peripheral
		pSPIHandle->TxState=SPI_BUSY_IN_TX;
		//Enable the TXEIE control bit to get  interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1<<SPI_CR2_TXEIE);
	}
	return pSPIHandle->TxState;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
	if(pSPIHandle->RxState != SPI_BUSY_IN_RX){ //Only update if SPI state is free
		//Save Rx buffer and length information in global buffer
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen= Len;
		//Mark SPI state as busy so that no other code take over the same SPI peripheral
		pSPIHandle->RxState=SPI_BUSY_IN_RX;
		//Enable the RXNEIE control bit to get  interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1<<SPI_CR2_RXNEIE);
	}
	return pSPIHandle->RxState;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t temp1, temp2;
	//TXEIE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1<<SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1<<SPI_CR2_TXEIE);
	if(temp1 && temp2){
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}
	//RXEIE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1<<SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1<<SPI_CR2_RXNEIE);
	if(temp1 && temp2){
		//handle TXE
		spi_rxne_interrupt_handle(pSPIHandle);
	}
	//OVR
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1<<SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1<<SPI_CR2_ERRIE);
	if(temp1 && temp2){
		//handle TXE
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}
//Some helper function implementations
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t ReverseByteOrder[2];
	if(pSPIHandle->pSPIx->SPI_CR1 & 1<<SPI_CR1_DFF){ //Check if the data frame format is 16 bits
		ReverseByteOrder[0]= *(pSPIHandle->pTxBuffer+1);
		ReverseByteOrder[1]= *(pSPIHandle->pTxBuffer);
		memcpy((void*)&pSPIHandle->pSPIx->SPI_DR,(void*)ReverseByteOrder,2);
		if(pSPIHandle->TxLen==1)
			pSPIHandle->TxLen -=1;
		else
			pSPIHandle->TxLen -=2;

		pSPIHandle->pTxBuffer +=2;
	}else{
		memcpy((void*)&pSPIHandle->pSPIx->SPI_DR,(void*)pSPIHandle->pTxBuffer,1);
		pSPIHandle->TxLen -=1;
		pSPIHandle->pTxBuffer +=1;
	}

	if(pSPIHandle->TxLen == 0){
		//When all the data is transfered length is zero and then we have to close the transmission tell the application
		SPI_CloseTransmission (pSPIHandle);
		SPI_ApllicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t ReverseByteOrder[2];
	if(pSPIHandle->pSPIx->SPI_CR1 & 1<<SPI_CR1_DFF){ //Check if the data frame format is 16 bits
		memcpy((void*)ReverseByteOrder,(void*)&pSPIHandle->pSPIx->SPI_DR,2);
		*(pSPIHandle->pRxBuffer+1) = ReverseByteOrder[0];
		*(pSPIHandle->pRxBuffer) = ReverseByteOrder[1];
		if(pSPIHandle->RxLen==1)
			pSPIHandle->RxLen -=1;
		else
			pSPIHandle->RxLen -=2;

		pSPIHandle->pRxBuffer +=2;
	}else{
		memcpy((void*)pSPIHandle->pRxBuffer, (void*)&pSPIHandle->pSPIx->SPI_DR, 1);
		//pSPIx->SPI_DR = (uint32_t)*(uint8_t*)pTxBuffer;
		pSPIHandle->RxLen -=1;
		pSPIHandle->pRxBuffer +=1;
	}

	if(pSPIHandle->RxLen == 0){
		//When all the data is received then length is zero and then we have to close the reception and notify the application
		SPI_CloseReception(pSPIHandle);
		SPI_ApllicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}

}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;
	SPI_ApllicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission (SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0; //Already Zero
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception (SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0; //Already Zero
	pSPIHandle->RxState = SPI_READY;
}
void SPI_ClearOVRFlag (SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->SPI_DR; 	//Reading the data register
	temp = pSPIx->SPI_SR;	//Reading the status register
	(void)temp;
}


__weak void SPI_ApllicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv){

}
