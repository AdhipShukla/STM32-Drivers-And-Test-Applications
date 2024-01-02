/*
 * STM32F411RE_Driver_SPI.h
 *
 *  Created on: Dec 26, 2023
 *      Author: adhip
 */

#ifndef INC_STM32F411RE_DRIVER_SPI_H_
#define INC_STM32F411RE_DRIVER_SPI_H_
#include "stm32f411re.h"

typedef struct {
	uint8_t SPI_DeviceMode;  		//Set as Master or Slave
	uint8_t SPI_BusConfig;			//Bus can be Full Duplex, Half Duplex or One Way
	uint8_t SPI_SclkSpeed;			//Bus clock speed based on the peripheral bus(APB1, APB2, AHB) internal clock
	uint8_t SPI_DFF;				//Bus data frame can be 8 or 16 bits
	uint8_t SPI_CPOL;				//Polarity of SPI data bits compared to clock signal
	uint8_t SPI_CPHA;				//Phase of SPI data bits compared to clock signal
	uint8_t SPI_SSM;				//Software slave setting, used for single slave to save GPIO pins
}SPI_Config_t;

typedef struct{
	SPI_Config_t SPIConfig;
	SPI_RegDef_t *pSPIx;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
//#define SPI_BUS_CONFIG_Simplex_TXONLY		2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/*
 * @SPI_DFF
 */
#define SPI_DEF_8BITS						0
#define SPI_DEF_16BITS						1

/*
 * @CPOL
 */
#define	SPI_CPOL_HIGH						1
#define	SPI_CPOL_LOW						0

/*
 * @CPHA
 */
#define	SPI_CPHA_HIGH						1
#define	SPI_CPHA_LOW						0

/*
 * @SSI_SSM
 */
#define	SPI_SSM_EN							1
#define	SPI_SSM_DI							0

/*
 *  @SSI_Application_State
 */
#define SPI_READY							0
#define SPI_BUSY_IN_RX						1
#define SPI_BUSY_IN_TX						2

/*
 *  @SSI_Application_Events
 */
#define SPI_EVENT_TX_CMPLT					1
#define SPI_EVENT_RX_CMPLT					2
#define SPI_EVENT_OVR_ERR					3

/*Peripheral Clock Setup*/
void SPI_PeriClockControl(SPI_RegDef_t *pGPIOx, uint8_t EnORDi);

/*Init and De-init*/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIHandle);

/*Enabling and Disabling SPI*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnORDi);

/*Setting the SII pin when node's SSM bit is set*/
void SPI_SSISet(SPI_RegDef_t *pSPIx, uint8_t EnORDi);

/*Setting the SSOE bit*/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnORDi);

/*Data Send and Receive*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*API to clear overrun flag*/
void SPI_ClearOVRFlag (SPI_RegDef_t *pSPIx);

/*SPI close receive and transmission*/
void SPI_CloseTransmission (SPI_Handle_t *pSPIHandle);
void SPI_CloseReception (SPI_Handle_t *pSPIHandle);

/*IRQ Configuration and ISR Handling*/
void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnORDi);
void SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *PinNumber);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Application interrupt handle callback
 */
void SPI_ApllicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_STM32F411RE_DRIVER_SPI_H_ */
