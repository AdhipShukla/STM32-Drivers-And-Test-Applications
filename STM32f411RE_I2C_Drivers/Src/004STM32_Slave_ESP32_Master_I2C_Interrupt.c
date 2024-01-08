/*
 * 004STM32_Slave_ESP32_Master_I2C_Interrupt.c
 *
 *  Created on: Jan 8, 2024
 *      Author: adhip
 */

#include <stdint.h>
#include "stm32f411re.h"
#include <string.h>
#include <stdio.h>
extern void initialise_monitor_handles(void);
#define MasterExample	1
#define SLAVE_ADDR 	0x3E
#define MY_ADDR 	SLAVE_ADDR
#define PRESSED 	1
uint8_t Message[] = "I2C Occupy Mars!\n";
uint8_t ExpMes[] = "Let's Capture Mars!!\n";
uint8_t RecByte;
uint8_t CommandByte;
I2C_Handle_t I2C1Handle;
uint8_t Rec_Buf[32];
uint8_t len;
uint8_t i = 0;
uint8_t NotSame = 0;
void delay(void){
	for(uint32_t i=0; i<500000; i++);
}

int main(void)
{
	initialise_monitor_handles();
	printf("Application is running\n");
	//GPIO Button for message sending
	GPIO_Handle_t GPIOButton;
	GPIOButton.pGPIOx= GPIOC;
	GPIOButton.GPIO_Config.GPIO_PinNumber=GPIO_PIN_NUM_13;
	GPIOButton.GPIO_Config.GPIO_PinMode=GPIO_MODE_IN;
	GPIOButton.GPIO_Config.GPIO_PinOPType=GPIO_OP_TYPES_PP;
	GPIOButton.GPIO_Config.GPIO_PinSpeed=GPIO_OP_SPEED_FAST;
	GPIOButton.GPIO_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOButton);

	//LED
	GPIO_Handle_t InternalLED;
	InternalLED.pGPIOx= GPIOA;
	InternalLED.GPIO_Config.GPIO_PinNumber=GPIO_PIN_NUM_5;
	InternalLED.GPIO_Config.GPIO_PinMode=GPIO_MODE_OUT;
	InternalLED.GPIO_Config.GPIO_PinOPType=GPIO_OP_TYPES_PP;
	InternalLED.GPIO_Config.GPIO_PinSpeed=GPIO_OP_SPEED_FAST;
	InternalLED.GPIO_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&InternalLED);

	//GPIO INIT FOR I2C
	//PB8 - I2C1_SCL
	//PB9 - I2C1_SDA
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_Config.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_Config.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_Config.GPIO_PinOPType = GPIO_OP_TYPES_OD;
	I2CPins.GPIO_Config.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	I2CPins.GPIO_Config.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	I2CPins.GPIO_Config.GPIO_PinNumber = 8; //I2C1_SCL
	GPIO_Init(&I2CPins);

	I2CPins.GPIO_Config.GPIO_PinNumber = 9; //I2C1_SDA
	GPIO_Init(&I2CPins);

	//I2CInit
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;	//Self address does not matter as master mode
	I2C1Handle.I2C_Config.I2C_FMDutyCycle =  I2C_FM_DUTY_2; //Does not matter as it is in standard mode
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;  //Standard mode i.e. 100k speed
	I2C_Init(&I2C1Handle);

	//IRQ Enable
	I2C_IRQ_Interrupt_Config(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQ_Interrupt_Config(IRQ_NO_I2C1_ER, ENABLE);

	//Enabling interrupt
	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	//Enable peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_AckSet(&I2C1Handle, I2C1Handle.I2C_Config.I2C_ACKControl);

	while(1);
}


void I2C1_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C1_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){
	static uint8_t commandCode = 0;
	static uint8_t cnt = 0;
	if(AppEv == I2C_EV_DATA_REQ){
		//Master wants some data slave has to send it
		if(commandCode == 0x51){
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Message));
		} else if(commandCode == 0x52){
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Message[cnt++]);
		}
	} else if(AppEv == I2C_EV_DATA_RCV){
		//Master send some data that slave has to read from the buffer
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	} else if(AppEv == I2C_ERROR_AF){
		//Master has sent Nack when slave was transmitting. Slave considers this as no more data required by the master and slave stop sending data
		commandCode = 0xff; //Reseting
		cnt = 0;
	} else if (AppEv == I2C_EV_STOP){
		//Mater ends the I2C communication with slave while transmitting
	}
}
