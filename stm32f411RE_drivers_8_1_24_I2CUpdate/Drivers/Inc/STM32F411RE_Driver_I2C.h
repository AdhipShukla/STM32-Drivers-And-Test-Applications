#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_
#include "stm32f411re.h"

//Configuration structure for I2Cx peripheral
typedef struct{
	uint32_t 	I2C_SCLSpeed;
	uint8_t  	I2C_DeviceAddress;
	uint8_t 	I2C_ACKControl;
	uint16_t	I2C_FMDutyCycle;
}I2C_Config_t;

//Handle structure for I2Cx peripheral
typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t 	*pTxBuffer;		//To store the application Tx buffer state
	uint8_t 	*pRxBuffer;		//To store the application Rx buffer state
	uint32_t 	TxLen;			//To store the Tx Len
	uint32_t 	RxLen;			//To store the Rx Len
	uint8_t 	TxRxState;		//To store the communication state
	uint8_t 	DevAddr;		//To store slave/device address
	uint32_t 	RxSize;			//To store Rx size
	uint8_t 	Sr;				//To store the repeated start value
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM4K		400000
#define I2C_SCL_SPEED_FM2K		200000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE			ENABLE
#define I2C_ACK_DISABLE			DISABLE
/*
 *@I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1
/*
 *
 */
#define READCMD					0
#define WRITECMD				1
/*
 * Macros for setting and reseting the start repeat
 */
#define I2C_SR_ENABLE				ENABLE
#define I2C_SR_DISABLE				DISABLE
/*
 * @I2CRelated Status flag definitions
 */
#define I2C_FLAG_TIMEOUT	(1<<I2C_SR1_TIMEOUT)
#define I2C_FLAG_OVR		(1<<I2C_SR1_OVR)
#define I2C_FLAG_AF			(1<<I2C_SR1_AF)
#define I2C_FLAG_ARLO		(1<<I2C_SR1_ARLO)
#define I2C_FLAG_BERR		(1<<I2C_SR1_BERR)
#define I2C_FLAG_TXE		(1<<I2C_SR1_TxE)
#define I2C_FLAG_RXNE		(1<<I2C_SR1_RxNE)
#define I2C_FLAG_STOPF		(1<<I2C_SR1_STOPF)
#define I2C_FLAG_ADD10		(1<<I2C_SR1_ADD10)
#define I2C_FLAG_BTF		(1<<I2C_SR1_BTF)
#define I2C_FLAG_ADDR		(1<<I2C_SR1_ADDR)
#define I2C_FLAG_SB			(1<<I2C_SR1_SB)

/*
 * I2C Applicaiton States
 */
#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2

/*
 * I2C Application Events
 */
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_ERROR_BERR		3
#define I2C_ERROR_ARLO		4
#define I2C_ERROR_AF		5
#define I2C_ERROR_OVR		6
#define I2C_ERROR_TIMEOUT	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9

/*Peripheral Clock Setup*/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnORDi);

/*Init and De-init*/
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*Setting the acknowledgement bit*/
void I2C_AckSet(I2C_Handle_t *pI2CHandle, uint8_t setAck);

/*Enabling and Disabling I2C*/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnORDi);

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);

/*IRQ Configuration and ISR Handling*/
void I2C_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnORDi);
void I2C_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnORDi);

/*Master Data Send and Receive*/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint32_t SlaveAddr, uint8_t SR);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint32_t SlaveAddr, uint8_t SR);

/*Master Data Send and Receive Interrupt APIs*/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint32_t SlaveAddr, uint8_t SR);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint32_t SlaveAddr, uint8_t SR);

/*Slave Data Send and Receive*/
void I2C_SlaveSendDataOld(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint32_t SlaveAddr, uint8_t SR);
void I2C_SlaveReceiveDataOld(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*Close Send and Receive Data*/
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

/*Checking status flag*/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/* Application interrupt handle callback*/
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

#endif
