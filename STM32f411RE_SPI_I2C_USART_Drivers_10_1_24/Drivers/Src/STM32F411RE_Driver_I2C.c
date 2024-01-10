#include "STM32F411RE_Driver_I2C.h"
#define __weak __attribute__((weak))

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnORDi){
	if(EnORDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}
void I2C_AckSet(I2C_Handle_t *pI2CHandle, uint8_t setAck){
	if (setAck){
		pI2CHandle->pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	} else {
		pI2CHandle->pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
	}
}
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempReg = 0;

	//Enabling the clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//Reseting the I2C peripherals
	pI2CHandle->pI2Cx->I2C_CR1 |= 1<<I2C_CR1_SWRST;
	pI2CHandle->pI2Cx->I2C_CR1 &= ~(1<<I2C_CR1_SWRST);

	//Configuring Ack Control Bit
	//This may not be set as the peripheral is not initialized yet
	tempReg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->I2C_CR1 |= tempReg;

	//Configure the  FREQ field  of CR2
	tempReg = 0;
	tempReg |= RCC_GetPCLK1Value()/1000000U; //Converting to Mega Hz
	pI2CHandle->pI2Cx->I2C_CR2 = (tempReg & 0x3F);

	//Configuring the Self Address
	tempReg = 0;
	tempReg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempReg |= (1<14);
	pI2CHandle->pI2Cx->I2C_OAR1 = tempReg;

	//CCR Calculation
	uint16_t ccr_value = 0;
	tempReg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed<=I2C_SCL_SPEED_SM){ //Low speed standard mode
		ccr_value = (RCC_GetPCLK1Value()/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed)); //APB1Clock/(2*ConfigSpeed)
		tempReg |= (ccr_value & 0xFFF); //Masking for last 12 bits
	} else {
		tempReg |= (1<<I2C_CCR_FS);
		tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle<<I2C_CCR_DUTY);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = (RCC_GetPCLK1Value()/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed)); //APB1Clock/((2+1)*ConfigSpeed)
		} else {
			ccr_value = (RCC_GetPCLK1Value()/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed)); //APB1Clock/((9+16)*ConfigSpeed)
		}
		tempReg |= (ccr_value & 0xFFF); //Masking for last 12 bits
	}
	pI2CHandle->pI2Cx->I2C_CCR = tempReg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){// Checking if standard mode or fast mode
		tempReg = (RCC_GetPCLK1Value()/1000000U) + 1; //(1000*10^-9)/Clk_Time_Period + 1
	} else {
		tempReg = (RCC_GetPCLK1Value()*3/10000000U) + 1; //(300*10^-9)/Clk_Time_Period + 1
	}
	pI2CHandle->pI2Cx->I2C_TRISE = (tempReg & 0x3F);
}


void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnORDi){
	if (EnORDi == ENABLE){
		pI2Cx->I2C_CR1 |= 1<<I2C_CR1_PE;
	}else{
	//Write the code for disable following the routine
		pI2Cx->I2C_CR1 &= ~(1<<I2C_CR1_PE);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
	if(pI2Cx->I2C_SR1 & FlagName){
		return 1;
	}
	return 0;
}

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->I2C_CR1 |= (1<<I2C_CR1_START);
}
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t WriteCmd){
	if (WriteCmd ==1){
		SlaveAddr = SlaveAddr << 1;
		SlaveAddr &= ~(1); //Slave address and read and write bit
		pI2Cx->I2C_DR = SlaveAddr;
	} else {
		SlaveAddr = SlaveAddr << 1;
		SlaveAddr |= (1); //Slave address and read and write bit
		pI2Cx->I2C_DR = SlaveAddr;
	}
}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->pI2Cx->I2C_SR2 & (1<<I2C_SR2_MSL)){//Check if the mode is master
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){//
			if(pI2CHandle->RxSize ==1){
				I2C_AckSet(pI2CHandle, DISABLE);
				/*//This is just read of SR1 register after the SR2 register
				uint32_t dummyRead = pI2Cx->I2C_SR1;
				dummyRead = pI2Cx->I2C_SR2;
				(void)dummyRead;*/
			}
		} else { //If the node state is Tx
			/*//This is just read of SR1 register after the SR2 register
			uint32_t dummyRead = pI2Cx->I2C_SR1;
			dummyRead = pI2Cx->I2C_SR2;
			(void)dummyRead;*/
		}
	} else { //If the node is slave
		/*//This is just read of SR1 register after the SR2 register
		uint32_t dummyRead = pI2Cx->I2C_SR1;
		dummyRead = pI2Cx->I2C_SR2;
		(void)dummyRead;*/
	}
	//This is just read of SR1 register after the SR2 register
	uint32_t dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
	dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
	(void)dummyRead;
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->I2C_CR1 |= (1<<I2C_CR1_STOP);
}

static void I2C_ClearSTOPFFlag(I2C_RegDef_t *pI2Cx){
	//This is just read of SR1 register and writing of CR register
	uint32_t dummyRead = pI2Cx->I2C_SR1;
	(void)dummyRead;
	I2C_GenerateStopCondition(pI2Cx);
}


void I2C_SlaveSendDataOld(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint32_t SlaveAddr, uint8_t SR){
	//Hardware automatically acknowledges if the address on the bus is the same as the salve address
	//First step for the slave is to clear the ADDR bit once ADDR is set
	uint8_t *originalBuffer = pTxBuffer;
	uint8_t originalLen = Len;
	NewData:
	Len = originalLen;
	pTxBuffer = originalBuffer;
	int WriteCmd = 1;
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));
	//Clear the ADDR bit
	I2C_ClearADDRFlag(pI2CHandle);
	//Write the data to be sent to Master if Master commands for the data
	if(WriteCmd){//Can be handled in application
		while(1){
			if(Len>0){
				while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
				pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;	//Filling the DR with the data to be sent
				pTxBuffer++;
				Len--;
				if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_AF)){//Check if acknowledegement failed
					pI2CHandle->pI2Cx->I2C_SR1 |= I2C_FLAG_AF;
				}
			} else if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)) {
				pI2CHandle->pI2Cx->I2C_DR = 0x05;//Writing Dummy data
			} else if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_AF)){
				pI2CHandle->pI2Cx->I2C_SR1 &= ~(I2C_FLAG_AF);
				break;
			}
		}
	} else {
		pI2CHandle->pI2Cx->I2C_DR = 0x04;//*pTxBuffer; //Writing Dummy data
	}
	//Check if the STOPF bit is set
	while(1){
	if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_AF)){
		I2C_ClearSTOPFFlag(pI2CHandle->pI2Cx);
		return;
	}
	if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))//In case there is no stop condition from the master and the same data is requested again
		goto NewData;
	}
	return;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint32_t SlaveAddr, uint8_t SR){
	//Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Check for start bit(SB) flag in SR1 register
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//Send the address of the slave with r/nw bit set to w(0)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, WRITECMD);

	//Confirm that the address is completed by checking the ADDR flag
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//Clear the ADDR flag according to its software sequence
	// Until ADDR is cleared  SCL will be  stretched (pulled to low)
	I2C_ClearADDRFlag(pI2CHandle);

	//Send the data until the length becomes zero
	while(Len>0){
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//When the Len becomes zero wait for TXE=1 and BTF=1 indicating both SR and DR are empty
	//when BTF is one SCL is extended
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//Generate stop condition and master need to wait for the completetion of stop condition
	//Generating stop automatically clears BTF
	if(SR==I2C_SR_DISABLE)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_SlaveReceiveDataOld(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len){
	//Hardware automatically acknowledges if the address on the bus is the same as the salve address
	//First step for the slave is to clear the ADDR bit once ADDR is set
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));
	//Clear the ADDR bit
	I2C_ClearADDRFlag(pI2CHandle);
	while(Len>0){
		//Waiting for data to arrive in Rx buffer and setting of RXNE
		while(1){
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF)){
			I2C_ClearSTOPFFlag(pI2CHandle->pI2Cx);
			return;
		}
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
			break;
		}
		//Getting the data
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		Len--;

	}
	//Check if the STOPF bit is set
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF));
	I2C_ClearSTOPFFlag(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint32_t SlaveAddr, uint8_t SR){
	//Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Check for start bit(SB) flag in SR1 register
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//Send the address of the slave with r/nw bit set to w(0)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, READCMD);

	//Confirm that the address is completed by checking the ADDR flag
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	if(Len ==1){
		//Diabling ACK to stop the slave from sending any more bytes
		I2C_AckSet(pI2CHandle, I2C_ACK_DISABLE);
		//Clearing the ADDR  flag
		I2C_ClearADDRFlag(pI2CHandle);
		//Waiting for data to arrive in Rx buffer and setting of RXNE
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
		//Generating stop condition
		if(SR==I2C_SR_DISABLE)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		//Getting the data
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
	} else {
		//Clearing the ADDR  flag
		I2C_ClearADDRFlag(pI2CHandle);
		for(uint32_t i =Len; i>0; i--){
			//Waiting for data to arrive in Rx buffer and setting of RXNE
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
			if(i==2){
				//Diabling ACK to stop the slave from sending any more bytes
				I2C_AckSet(pI2CHandle, I2C_ACK_DISABLE);
				//Generating stop condition
				if(SR==I2C_SR_DISABLE)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
			pRxBuffer++;
		}
	}
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)//If the Ack was set in config then set it again
		I2C_AckSet(pI2CHandle, I2C_ACK_ENABLE);
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data){
	pI2C->I2C_DR =data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C){
	return (uint8_t)pI2C->I2C_DR;
}

void I2C_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnORDi){
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

void I2C_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t RegNum = IRQNumber/4;
	uint8_t RegOffest = IRQNumber%4;
	*(NVIC_IPR0_ADDR + RegNum) |= (IRQPriority<<((RegOffest*8) + (8-NO_PR_BITS_IMPL)));
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint32_t SlaveAddr, uint8_t SR){
	uint8_t busystate = pI2CHandle->TxRxState;
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = SR;

		//Generating stat condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//Enabling Buffer interrupt
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<<I2C_CR2_ITBUFEN);
		//Enabling Buffer interrupt
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<<I2C_CR2_ITEVTEN);
		//Enabling Error interrupt
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<<I2C_CR2_ITERREN);
	}
	return busystate;
}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint32_t SlaveAddr, uint8_t SR){
	uint8_t busystate = pI2CHandle->TxRxState;
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->RxSize = Len;
		pI2CHandle->Sr = SR;

		//Generating start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//Enabling Buffer interrupt
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<<I2C_CR2_ITBUFEN);
		//Enabling Buffer interrupt
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<<I2C_CR2_ITEVTEN);
		//Enabling Error interrupt
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<<I2C_CR2_ITERREN);
	}
	return busystate;
}
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnORDi){
	if(EnORDi == ENABLE){
		//Enabling Buffer interrupt
		pI2Cx->I2C_CR2 |= (1<<I2C_CR2_ITBUFEN);
		//Enabling Event interrupt
		pI2Cx->I2C_CR2 |= (1<<I2C_CR2_ITEVTEN);
		//Enabling Error interrupt
		pI2Cx->I2C_CR2 |= (1<<I2C_CR2_ITERREN);
	} else {
		//Disabling Buffer interrupt
		pI2Cx->I2C_CR2 &= ~(1<<I2C_CR2_ITBUFEN);
		//Disabling Event interrupt
		pI2Cx->I2C_CR2 &= ~(1<<I2C_CR2_ITEVTEN);
		//Disabling Error interrupt
		pI2Cx->I2C_CR2 &= ~(1<<I2C_CR2_ITERREN);
	}
}
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){
	//Disabling ITBUFEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1<<I2C_CR2_ITBUFEN);
	//Disabling ITEVTEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1<<I2C_CR2_ITEVTEN);
	//Reseting the parameters of the
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		I2C_AckSet(pI2CHandle, ENABLE);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){
	//Disabling ITBUFEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1<<I2C_CR2_ITBUFEN);
	//Disabling ITEVTEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1<<I2C_CR2_ITEVTEN);
	//Reseting the parameters of the
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1<<I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1<<I2C_CR2_ITBUFEN);

	//Handle interrupt generated by SB event
	//This event is only triggered in master mode
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<<I2C_SR1_SB);
	if(temp1 && temp3){
		//After the SB now let's execute the address phase
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, WRITECMD);
		} else {
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, READCMD);
		}
	}

	//Handle interrupt generated by ADDR event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<<I2C_SR1_ADDR);
	if(temp1 && temp3){
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//Handle interrupt generated by BTF event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<<I2C_SR1_BTF);
	if(temp1 && temp3){
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			if(pI2CHandle->pI2Cx->I2C_SR1 & (1<<I2C_SR1_TxE)){ //This condition should always be true of BTF is 1
				if(pI2CHandle->TxLen == 0){
					if(pI2CHandle->Sr == I2C_SR_DISABLE){//Generate stop request
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					//Reset all the members of the handle structure
					I2C_CloseSendData(pI2CHandle);
					//Notify the application about the transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		} else {//No action required
			;
		}
	}

	//Handle interrupt generated by STOPF event
	//This is event is only triggered in Slave mode when STOP is detected on the bus
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<<I2C_SR1_STOPF);
	if(temp1 && temp3){
		//STOP flag is set
		//Clear  the STOPF by reading SR1 and writing CR1
		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000; //Dummy Write
	}

	//Handle interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<<I2C_SR1_TxE);
	if(temp1 && temp2 && temp3){
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1<<I2C_SR2_MSL)){
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
				if(pI2CHandle->TxLen>0){
					//Load the data in to DR
					pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);
					//Decrement the TxLen
					pI2CHandle->TxLen--;
					//Increment the buffer address
					pI2CHandle->pTxBuffer++;
				}
			}
		} else {
			if(pI2CHandle->pI2Cx->I2C_SR2 & (1<<I2C_SR2_TRA)){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	//Handle interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<<I2C_SR1_RxNE);
	if(temp1 && temp2 && temp3){
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1<<I2C_SR2_MSL)){
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				if(pI2CHandle->RxSize == 1){
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
					pI2CHandle->RxLen--;
				}
				if(pI2CHandle->RxSize>1){
					if(pI2CHandle->RxLen==2){
						//Diabling ACK to stop the slave from sending any more bytes
						I2C_AckSet(pI2CHandle, I2C_ACK_DISABLE);
					}
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxLen--;
				}
				if(pI2CHandle->RxLen==0){
					if(pI2CHandle->Sr == I2C_SR_DISABLE){
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					I2C_CloseReceiveData(pI2CHandle);
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
				}
			}
		} else {
			if(!(pI2CHandle->pI2Cx->I2C_SR2 & (1<<I2C_SR2_TRA))){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1 , temp2;
	//Check the status of the ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & (1<<I2C_CR2_ITERREN);

	//Check for the bus error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & (1<<I2C_SR1_BERR);
	if(temp1 && temp2){
		//Clearing the bus error
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1<<I2C_SR1_BERR);
		//Notifying the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	//Check for arbitration loss
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & (1<<I2C_SR1_ARLO);
	if(temp1 && temp2){
		//Clearing the arbitration error
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1<<I2C_SR1_ARLO);
		//Notifying the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	//Check for acknowledgement fail
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & (1<<I2C_SR1_AF);
	if(temp1 && temp2){
		//Clearing the acknowledgement fail error
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1<<I2C_SR1_AF);
		//Notifying the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	//Check for overrun and underrun error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & (1<<I2C_SR1_OVR);
	if(temp1 && temp2){
		//Clearing the overrun and underrun error
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1<<I2C_SR1_OVR);
		//Notifying the overrun and underrun error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}
	//Check for Timeout error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & (1<<I2C_SR1_TIMEOUT);
	if(temp1 && temp2){
		//Clearing the Timeout error
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1<<I2C_SR1_TIMEOUT);
		//Notifying the Timeout error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv){

}
