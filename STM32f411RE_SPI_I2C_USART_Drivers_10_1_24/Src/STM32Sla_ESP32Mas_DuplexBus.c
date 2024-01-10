/*
 * STM32M_ESP32S_DuplexBus.c
 *
 *  Created on: Dec 28, 2023
 *      Author: adhip
 */

#include "stm32f411re.h"
#include <string.h>
extern void initialise_monitor_handles();

#define PRESSED 0
#define CMD_LED_ON        0x50
#define CMD_BUT_READ      0x51
#define CMD_LED_OFF       0x52
#define CMD_POT_READ      0x53

#define LED_ON	1
#define LED_OFF	0
#define LED_PIN	2
#define SigMas  1

uint8_t SPI_VerifyRespose(uint8_t ackbyte){
	if (ackbyte == 0xF5){
		return 1;
	}
	return 0;
}
void delay(void){
	for(uint32_t i=0; i<500000; i++);
}

int main(void){

	//initialise_monitor_handles();
	//printf("Application Running!\n");
	GPIO_Handle_t GPIOButton;
	GPIOButton.pGPIOx= GPIOC;
	GPIOButton.GPIO_Config.GPIO_PinNumber=GPIO_PIN_NUM_13;
	GPIOButton.GPIO_Config.GPIO_PinMode=GPIO_MODE_IN;
	GPIOButton.GPIO_Config.GPIO_PinOPType=GPIO_OP_TYPES_PP;
	GPIOButton.GPIO_Config.GPIO_PinSpeed=GPIO_OP_SPEED_FAST;
	GPIOButton.GPIO_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOButton);

	/*
	 *Potentiometer Signal
	 */
	GPIO_Handle_t GPIOIn;
	GPIOIn.pGPIOx= GPIOC;
	GPIOIn.GPIO_Config.GPIO_PinNumber=GPIO_PIN_NUM_8;
	GPIOIn.GPIO_Config.GPIO_PinMode=GPIO_MODE_IN;
	GPIOIn.GPIO_Config.GPIO_PinOPType=GPIO_OP_TYPES_PP;
	GPIOIn.GPIO_Config.GPIO_PinSpeed=GPIO_OP_SPEED_FAST;
	GPIOIn.GPIO_Config.GPIO_PinPuPdControl=GPIO_PIN_PD;
	//GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOIn);

	/*
	 *Configuring Communication Complete GPIO Signal
	 */
	GPIO_Handle_t ExternLED;
	ExternLED.pGPIOx= GPIOA;
	ExternLED.GPIO_Config.GPIO_PinNumber=GPIO_PIN_NUM_6;
	ExternLED.GPIO_Config.GPIO_PinMode=GPIO_MODE_OUT;
	ExternLED.GPIO_Config.GPIO_PinOPType=GPIO_OP_TYPES_PP;
	ExternLED.GPIO_Config.GPIO_PinSpeed=GPIO_OP_SPEED_FAST;
	ExternLED.GPIO_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&ExternLED);

	/*
	 *Configuring Interrupt for signal for Master
	 */
	GPIO_Handle_t ExternSig;
	ExternSig.pGPIOx= GPIOC;
	ExternSig.GPIO_Config.GPIO_PinNumber=GPIO_PIN_NUM_6;
	ExternSig.GPIO_Config.GPIO_PinMode=GPIO_MODE_OUT;
	ExternSig.GPIO_Config.GPIO_PinOPType=GPIO_OP_TYPES_PP;
	ExternSig.GPIO_Config.GPIO_PinSpeed=GPIO_OP_SPEED_FAST;
	ExternSig.GPIO_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&ExternSig);

	/*
	 * Configuring the required pins for SPI communication
	 */
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_Config.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_Config.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_Config.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_Config.GPIO_PinOPType = GPIO_OP_TYPES_PP;
	SPIPins.GPIO_Config.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_Config.GPIO_PinNumber =  GPIO_PIN_NUM_13;
	GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_Config.GPIO_PinNumber =  GPIO_PIN_NUM_15;
	GPIO_Init(&SPIPins);
	//MISO
	SPIPins.GPIO_Config.GPIO_PinNumber =  GPIO_PIN_NUM_14;
	GPIO_Init(&SPIPins);
	//NSS
	SPIPins.GPIO_Config.GPIO_PinNumber =  GPIO_PIN_NUM_12;
	GPIO_Init(&SPIPins);

	/*
	 * Configuring the SPI peripheral
	 */
	SPI_Handle_t SPICh2;
	SPICh2.pSPIx = SPI2;
	SPICh2.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPICh2.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	//SPICh2.SPIConfig.SPI_SSM = SPI_SSM_DI;
	SPICh2.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
	SPICh2.SPIConfig.SPI_DFF = SPI_DEF_8BITS;
	SPICh2.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	//SPICh2.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV16;
	SPI_Init(&SPICh2);

	SPI_SSOEConfig(SPI2, ENABLE);
	//printf("Initialization done!\n");
	uint8_t RxMes;
	uint8_t PotBool=0;
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NUM_6, !SigMas);
	while(1){

		SPI_PeripheralControl(SPI2, ENABLE);
		while(~(SPI2->SPI_SR) & 1<<SPI_SR_RXNE){
			GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NUM_6, SigMas);
			GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NUM_6, !SigMas);
		}
		SPI_ReceiveData(SPI2, &RxMes, 1);
		if(RxMes==CMD_LED_ON){
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NUM_6, LED_ON);
			PotBool=(uint8_t)0xff;
			SPI_SendData(SPI2, &PotBool, 1);
			//printf("LED ON\n");
		} else if(RxMes==CMD_LED_OFF){
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NUM_6, LED_OFF);
			PotBool=(uint8_t)0xef;
			SPI_SendData(SPI2, &PotBool, 1);
			//printf("LED OFF\n");
		} else if(RxMes==CMD_BUT_READ){
			PotBool = (uint8_t)(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_8)*12);
			//PotBool=(uint8_t)0x7F;
			SPI_SendData(SPI2, &PotBool, 1);
			//printf("Button Read\n");
		} else {
			PotBool=(uint8_t)0x1F;
			SPI_SendData(SPI2, &PotBool, 1);
			//printf("Dummy Read\n");
		}
		while(SPI2->SPI_SR & (1<<SPI_SR_BSY));
		/*while((SPI2->SPI_SR) & 1<<SPI_SR_RXNE);
		while((SPI2->SPI_SR) & 1<<SPI_SR_TXE);*/
		SPI_PeripheralControl(SPI2, DISABLE);
	}
}
