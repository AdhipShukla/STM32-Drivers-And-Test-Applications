/*
 * spi_tx_testing.c
 *
 *  Created on: Dec 27, 2023
 *      Author: adhip
 */
/*
 * PB15 -> SPI2_MOSI
 * PB14	-> SPI2_MISO
 * PB13	-> SPI2_SCLK
 * PB12	-> SPI2_NSS
 * Alt Functionality mode 5
 */
#include "stm32f411re.h"
#include <string.h>
int main(void){
	char Mes[13] = "Hello World!";

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
	SPICh2.SPIConfig.SPI_SSM = SPI_SSM_EN;
	SPICh2.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPICh2.SPIConfig.SPI_DFF = SPI_DEF_8BITS;
	SPICh2.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPICh2.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI_Init(&SPICh2);
	/*
	 * Pull SSI to high to avoid current node consideration as slave
	 */
	SPI_SSISet(SPI2, PULL_TO_HIGH);
	/*
	 * Enabling the SPI
	 */
	SPI_PeripheralControl(SPI2, ENABLE);

	/*
	 * Sending the data on SPI bus
	 */
	SPI_SendData(SPI2, (uint8_t *)Mes, strlen(Mes));
	while(1);
	return 0;
}
