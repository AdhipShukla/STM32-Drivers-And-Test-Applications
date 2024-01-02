/*
 * SPI_ESP32Slave_STM32Master.c
 *
 *  Created on: Dec 28, 2023
 *      Author: adhip
 */

#include "stm32f411re.h"
#include <string.h>
#define PRESSED 0

void delay(void){
	for(uint32_t i=0; i<500000; i++);
}

int main(void){
	char Mes[13] = "Hello World!\n";

	/*
	* Configuring the Internal GPIO BUTTON
	*/
	GPIO_Handle_t GPIOButton;
	GPIOButton.pGPIOx= GPIOC;
	GPIOButton.GPIO_Config.GPIO_PinNumber=GPIO_PIN_NUM_13;
	GPIOButton.GPIO_Config.GPIO_PinMode=GPIO_MODE_IN;
	GPIOButton.GPIO_Config.GPIO_PinOPType=GPIO_NO_PUPD;
	GPIOButton.GPIO_Config.GPIO_PinSpeed=GPIO_OP_SPEED_FAST;
	GPIOButton.GPIO_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOButton);

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
	SPICh2.SPIConfig.SPI_SSM = SPI_SSM_DI;
	SPICh2.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPICh2.SPIConfig.SPI_DFF = SPI_DEF_16BITS;
	SPICh2.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPICh2.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI_Init(&SPICh2);

	SPI_SSOEConfig(SPI2, ENABLE);
	//uint8_t datalen = strlen(Mes);
	while(1){
	while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13)==PRESSED){
			delay();
			SPI_PeripheralControl(SPI2, ENABLE);
			//SPI_SendData(SPI2, &datalen, 1);
			SPI_SendData(SPI2, (uint8_t *)Mes, strlen(Mes));
			while(SPI2->SPI_SR & (1<<SPI_SR_BSY));
			SPI_PeripheralControl(SPI2, DISABLE);
		}
	}
	return 0;
}
