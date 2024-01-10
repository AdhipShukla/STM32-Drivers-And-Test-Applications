/*
 * 005STM32Tx_ESP32Rx_UART.c
 *
 *  Created on: Jan 9, 2024
 *      Author: adhip
 */

#include <stdint.h>
#include "stm32f411re.h"
#include <string.h>
#include <stdio.h>
#define PRESSED 	1
uint8_t Message[] = "USART Occupy Mars!\n";
uint8_t RecByte;
USART_Handle_t USART1Handle;

void delay(void){
	for(uint32_t i=0; i<500000; i++);
}

int main(void)
{
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

	//GPIO INIT FOR USART
	//PA2 - USART1_Tx
	//PA3 - USART1_Rx
	GPIO_Handle_t USARTPins;
	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_Config.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_Config.GPIO_PinAltFunMode = 7;
	USARTPins.GPIO_Config.GPIO_PinOPType = GPIO_OP_TYPES_PP;
	USARTPins.GPIO_Config.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USARTPins.GPIO_Config.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	USARTPins.GPIO_Config.GPIO_PinNumber = 9; //USART1_Tx
	GPIO_Init(&USARTPins);

	USARTPins.GPIO_Config.GPIO_PinNumber = 10; //USART1_Rx
	GPIO_Init(&USARTPins);

	//USARTInit
	USART1Handle.pUSARTx = USART1;
	USART1Handle.USART_Config.USART_Baud = USASRT_STD_BAUD_9600;
	USART1Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART1Handle.USART_Config.USART_NoOfStopBits =  USART_STOPBITS_1;
	USART1Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART1Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART1Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART_Init(&USART1Handle);
	//Enable peripheral
	USART_PeripheralControl(USART1, ENABLE);
	for(;;){
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13)==PRESSED);

		delay();

		USART_SendData(&USART1Handle, (uint8_t*)Message, strlen((char *)Message));
	}
	return 0;
}
