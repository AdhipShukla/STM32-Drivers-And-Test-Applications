#include "stm32f411re.h"
#define PRESSED 0

void delay(void){
	for(uint32_t i=0; i<1000000; i++);
}

int main(void)
{
	GPIO_Handle_t GPIOLED;
	GPIOLED.pGPIOx= GPIOC;
	GPIOLED.GPIO_Config.GPIO_PinNumber=GPIO_PIN_NUM_8;
	GPIOLED.GPIO_Config.GPIO_PinMode=GPIO_MODE_OUT;
	GPIOLED.GPIO_Config.GPIO_PinOPType=GPIO_OP_TYPES_PP;
	GPIOLED.GPIO_Config.GPIO_PinSpeed=GPIO_OP_SPEED_FAST;
	GPIOLED.GPIO_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOLED);

	GPIO_Handle_t GPIOButton;
	GPIOButton.pGPIOx= GPIOC;
	GPIOButton.GPIO_Config.GPIO_PinNumber=GPIO_PIN_NUM_7;
	GPIOButton.GPIO_Config.GPIO_PinMode=GPIO_MODE_IT_RT;
	//GPIOButton.GPIO_Config.GPIO_PinOPType=GPIO_OP_TYPES_PP;
	GPIOButton.GPIO_Config.GPIO_PinSpeed=GPIO_OP_SPEED_FAST;
	GPIOButton.GPIO_Config.GPIO_PinPuPdControl=GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOButton);

	GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI5_9, ENABLE);
	GPIO_IRQ_Priority_Config(IRQ_NO_EXTI5_9, NVIC_IRQ_PRI15);



	for(;;){
		/*if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_7)==PRESSED){
			GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NUM_8, GPIO_PIN_SET);
		}else{
			GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NUM_8, GPIO_PIN_RESET);
		}*/
		//delay();
	}
}

void EXTI9_5_IRQHandler(void){
	delay(); //Avoid switch debouncing
	GPIO_IRQHandling(GPIO_PIN_NUM_7);
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NUM_8);
	//GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI5_9, DISABLE);
}

