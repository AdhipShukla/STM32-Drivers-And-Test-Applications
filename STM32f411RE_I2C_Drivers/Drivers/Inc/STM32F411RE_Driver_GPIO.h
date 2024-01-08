/*
 * STM32F411RE_Driver_GPIO.h
 *
 *  Created on: Dec 24, 2023
 *      Author: adhip
 */

#ifndef SRC_STM32F411RE_DRIVER_GPIO_H_
#define SRC_STM32F411RE_DRIVER_GPIO_H_

#include "stm32f411re.h"

typedef struct{
	uint8_t GPIO_PinNumber;			/* Possible values from @GPIO_PIN_PinNum  */
	uint8_t GPIO_PinMode;			/* Possible values from @GPIO_PIN_MODES   */
	uint8_t GPIO_PinSpeed;			/* Possible values from @GPIO_PIN_SPEEDS  */
	uint8_t GPIO_PinPuPdControl;	/* Possible values from @GPIO_PIN_PUPD_Cn */
	uint8_t GPIO_PinOPType;			/* Possible values from @GPIO_PIN_OutType */
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx; //This hold the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_Config; //This holds GPIO pin configuration  settings
}GPIO_Handle_t;

/*GPIO PIN Possible Modes
 *@GPIO_PIN_MODES
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT 	4	//Interrupt for falling edge
#define GPIO_MODE_IT_RT 	5 	//Interrupt for rising edge
#define GPIO_MODE_IT_RFT	6	//Interrupt for rising and falling edge

/*GPIO PIN Possible Output Types
 * @GPIO_PIN_OutType
 */
#define GPIO_OP_TYPES_PP	0	//Push-Pull
#define GPIO_OP_TYPES_OD	1	//Open Drain

/*GPIO PIN Possible Output Speeds
 * @GPIO_PIN_SPEEDS
 */
#define GPIO_OP_SPEED_LOW	0	//Low Speed
#define GPIO_OP_SPEED_MED	1	//Med Speed
#define GPIO_OP_SPEED_FAST	2	//Fast Speed
#define GPIO_OP_SPEED_HIFGH	3	//High Speed

/*GPIO PIN Pull-up and Pull-down resistor configuration
 * @GPIO_PIN_PUPD_CONFIG
 */
#define GPIO_NO_PUPD		0	//No pull up or pull down resistor
#define GPIO_PIN_PU			1	//Pull Up
#define GPIO_PIN_PD			2	//Pull Down

/*GPIO PIN Number configuration
 * @GPIO_PIN_PinNum
 */
#define GPIO_PIN_NUM_0 		0
#define GPIO_PIN_NUM_1 		1
#define GPIO_PIN_NUM_2 		2
#define GPIO_PIN_NUM_3 		3
#define GPIO_PIN_NUM_4 		4
#define GPIO_PIN_NUM_5 		5
#define GPIO_PIN_NUM_6 		6
#define GPIO_PIN_NUM_7 		7
#define GPIO_PIN_NUM_8 		8
#define GPIO_PIN_NUM_9 		9
#define GPIO_PIN_NUM_10 	10
#define GPIO_PIN_NUM_11 	11
#define GPIO_PIN_NUM_12 	12
#define GPIO_PIN_NUM_13 	13
#define GPIO_PIN_NUM_14 	14
#define GPIO_PIN_NUM_15 	15



/*Following are all the APIs which are supported by this driver*/

/*Peripheral Clock Setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnORDi);

/*GPIO Init and DeInit*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*GPIO Read and Write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*GPIO IRQ and ISR handling*/
void GPIO_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnORDi);
void GPIO_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* SRC_STM32F411RE_DRIVER_GPIO_H_ */
