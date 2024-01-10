#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_
#include "stm32f411re.h"

//Configuration structure for I2Cx peripheral
typedef struct{
	uint8_t 	USART_Mode;
	uint32_t  	USART_Baud;
	uint8_t 	USART_NoOfStopBits;
	uint8_t		USART_WordLength;
	uint8_t 	USART_ParityControl;
	uint8_t		USART_HWFlowControl;
}USART_Config_t;

//Handle structure for I2Cx peripheral
typedef struct{
	USART_RegDef_t 	*pUSARTx;
	USART_Config_t 	USART_Config;
	uint8_t 		*pTxBuffer;			//To store the application Tx buffer state
	uint8_t 		*pRxBuffer;			//To store the application Rx buffer state
	uint32_t 		TxLen;				//To store the Tx Len
	uint32_t 		RxLen;				//To store the Rx Len
	uint8_t 		TxBusyState;		//To store the transmission busy state
	uint8_t 		RxBusyState;		//To store the reception busy state
}USART_Handle_t;

//Possible USART modes
#define USART_MODE_ONLY_TX	0
#define USART_MODE_ONLY_RX	1
#define USART_MODE_TXRX		2

//Possible USART Baud Rate
#define USASRT_STD_BAUD_1200	1200
#define USASRT_STD_BAUD_2400	2400
#define USASRT_STD_BAUD_9600	9600
#define USASRT_STD_BAUD_19200	19200
#define USASRT_STD_BAUD_38400	38400
#define USASRT_STD_BAUD_57600	57600
#define USASRT_STD_BAUD_115200	115200
#define USASRT_STD_BAUD_230400	230400
#define USASRT_STD_BAUD_460800	460800
#define USASRT_STD_BAUD_2M		2000000
#define USASRT_STD_BAUD_3M		3000000

//Possible option for UART parity control
#define USART_PARITY_DISABLE	0
#define USART_PARITY_EN_EVEN	1
#define USART_PARITY_EN_ODD		2

//Possible option for USART word length
#define USART_WORDLEN_8BITS		0
#define USART_WORDLEN_9BITS		1

//Possible options for USART_NoOfStopBits
#define USART_STOPBITS_1		0
#define USART_STOPBITS_0_5		1
#define USART_STOPBITS_2		2
#define USART_STOPBITS_1_5		3

//Possible option for USART_HWflowControl
#define USART_HW_FLOW_CTRL_NONE		0
#define USART_HW_FLOW_CTRL_CTS		1
#define USART_HW_FLOW_CTRL_RTS		2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

//Application state
#define USART_BUSY_READY			0
#define USART_BUSY_IN_RX			1
#define USART_BUSY_IN_TX			2

//Interrupt Event and Errors
#define USART_EVENT_TX_CMPLT		0
#define USART_EVENT_RX_CMPLT		1
#define USART_EVENT_IDLE			2
#define USART_EVENT_CTS				3
#define USART_EVENT_PE				4
#define USART_ERR_FE				5
#define USART_ERR_NE				6
#define USART_ERR_ORE				7

//SR Flags
#define UASRT_FLAG_CTS		(1<<UASRT_SR_CTS)			//CTS flag
#define UASRT_FLAG_LBD		(1<<UASRT_SR_LBD)			//LIN break detection flag
#define UASRT_FLAG_TXE		(1<<UASRT_SR_TXE)			//Transmit data register empty
#define UASRT_FLAG_TC		(1<<UASRT_SR_TC)			//Transmission complete
#define UASRT_FLAG_RXNE		(1<<UASRT_SR_RXNE)			// Read data register not empty
#define UASRT_FLAG_IDLE		(1<<UASRT_SR_IDLE)			//IDLE line detected
#define UASRT_FLAG_OVR		(1<<UASRT_SR_OVR)			//Overrun error
#define UASRT_FLAG_NF		(1<<UASRT_SR_NF)			//Noise detected flag
#define UASRT_FLAG_FE		(1<<UASRT_SR_FE)			//Framing error
#define UASRT_FLAG_PE		(1<<UASRT_SR_PE)			//Parity error

/*
 * All the User Level APIs Supported by this Driver
 */
//Peripheral clock setup
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnORDi);

//Init and Deinit
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

//Data Send and receive
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

//Other peripheral control APIs
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx,  uint8_t EnORDi);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

//Interrupt configuration APIs
void USART_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnORDi);
void USART_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);


//Application Interrupt Callback
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv);

#endif
