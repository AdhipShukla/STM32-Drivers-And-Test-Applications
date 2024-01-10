/*
 * InterruptSPI_ESP32_STM32.c
 *
 *  Created on: Jan 1, 2024
 *      Author: adhip
 */

#include "stm32f411re.h"
#include <string.h>
//extern void initialise_monitor_handles();

#define PRESSED 0
#define CMD_LED_ON        0x50
#define CMD_BUT_READ      0x51
#define CMD_LED_OFF       0x52
#define CMD_POT_READ      0x53

#define LED_ON	1
#define LED_OFF	0
#define LED_PIN	2

#define MAX_LEN 			500

SPI_Handle_t SPI2handle;
char RecvBuffer[MAX_LEN];
volatile char ReadByte;
volatile char rcvStop = 0;
volatile uint8_t dataAvailable = 0;

void delay(void){
	for(uint32_t i=0; i<500000; i++);
}
void ButtonInit(){
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
}

void ExternLEDInit(){
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
}
void Slave_GPIO_InterruptPinInit(){
	/*
	 *This is the pin on which slave sends the interrupt
	 */
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin, 0, sizeof(spiIntPin));
	spiIntPin.pGPIOx= GPIOC;
	spiIntPin.GPIO_Config.GPIO_PinNumber=GPIO_PIN_NUM_8;
	spiIntPin.GPIO_Config.GPIO_PinMode=GPIO_MODE_IT_FT;
	spiIntPin.GPIO_Config.GPIO_PinSpeed=GPIO_OP_SPEED_LOW;
	spiIntPin.GPIO_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_Init(&spiIntPin);
	GPIO_IRQ_Priority_Config(IRQ_NO_EXTI5_9, NVIC_IRQ_PRI15);
	GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI5_9, ENABLE);
}
void SPI2_GPIOInits(){
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
}

void SPI2_Inits(){
	/*
		 * Configuring the SPI peripheral
		 */
		SPI_Handle_t SPICh2;
		SPICh2.pSPIx = SPI2;
		SPICh2.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
		SPICh2.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
		SPICh2.SPIConfig.SPI_SSM = SPI_SSM_DI;
		SPICh2.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
		SPICh2.SPIConfig.SPI_DFF = SPI_DEF_8BITS;
		SPICh2.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
		SPICh2.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
		SPI_Init(&SPICh2);
		SPI_SSOEConfig(SPI2, ENABLE);
}
int main(void){
	uint8_t dummy = 0xff;

	ButtonInit();
	ExternLEDInit();
	Slave_GPIO_InterruptPinInit();
	SPI2_GPIOInits();
	SPI2_Inits();

	SPI_IRQ_Interrupt_Config(IRQ_NO_SPI2, ENABLE);
	while(1){
		rcvStop = 0;

		while(!dataAvailable); // wait till data is available from the slave

		GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI5_9, DISABLE);
		SPI_PeripheralControl(SPI2, ENABLE);
		while(!rcvStop){//Getting one byte after another
			while(SPI_SendDataIT(&SPI2handle, &dummy, 1)==SPI_BUSY_IN_TX);
			while(SPI_ReceiveDataIT(&SPI2handle, &ReadByte, 1)==SPI_BUSY_IN_RX);
		}

		while(SPI2->SPI_SR & (1<<SPI_SR_BSY));
		SPI_PeripheralControl(SPI2, DISABLE);

		dataAvailable = 0;
		GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI5_9, ENABLE);
	}
	return 0;
}

void SPI_IRQHandler(void){ //This function implementation will override the weak implementation in startup file
	SPI_IRQHandling(&SPI2handle);
}

void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
	//This function is implemented after the SPI_IRQHandler-> IRQHandling -> IRQHandle is already implemented
	//The main work here is to save the bytes from the Rx buffer to RcvBuff
	static uint32_t i = 0;
	if (AppEv == SPI_EVENT_RX_CMPLT){
		RecvBuffer[i++] = ReadByte;
		if(ReadByte == '\0' || (i==MAX_LEN)){
			rcvStop = 1;	//This is set to stop reading anymore bytes after the null character
			RecvBuffer[i-1] = '\0';
			i=0;
		}
	}
}

void EXTI9_5_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NUM_8);
	dataAvailable = 1;
}
