#include "STM32F411RE_Driver_RCC.h"

uint16_t AHB_Prescalar[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB_Prescalar[4] = {2, 4, 8, 16};

uint32_t RCC_GetLIOutputClock(){
	return 16000000; //Here implement the function to get the output of the PLI clock
}

//APB1 clock frequency calculation based on all the programmed registers
uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp;
	uint16_t AHBPreScalar, APB1PreScalar;
	clksrc = ((RCC->CFGR>>2) & 0x03); //Clock source status registered maintained by the hardware
	if(clksrc == 0){//HSI as system clock
		SystemClk =  16000000;
	} else if(clksrc == 1){//HSI as system clock
		SystemClk =  8000000;
	} else if (clksrc == 2){//PLL as system clock
		SystemClk = RCC_GetLIOutputClock();
	}

	temp = ((RCC->CFGR>>4) & 0xf);
	if(temp<8){
		AHBPreScalar = 1;
	} else{
		AHBPreScalar = AHB_Prescalar[temp-8];
	}
	temp = ((RCC->CFGR>>10) & 0x7);
	if(temp<4){
		APB1PreScalar = 1;
	} else {
		APB1PreScalar = APB_Prescalar[temp-4];
	}
	pclk1 = SystemClk/AHBPreScalar/APB1PreScalar;

	return pclk1;
}

//APB2 clock frequency calculation based on all the programmed registers
uint32_t RCC_GetPCLK2Value(void){
	uint32_t pclk2, SystemClk;
	uint8_t clksrc, temp;
	uint16_t AHBPreScalar, APB2PreScalar;
	clksrc = ((RCC->CFGR>>2) & 0x03); //Clock source status registered maintained by the hardware
	if(clksrc == 0){//HSI as system clock
		SystemClk =  16000000;
	} else if(clksrc == 1){//HSI as system clock
		SystemClk =  8000000;
	} else if (clksrc == 2){//PLL as system clock
		SystemClk = RCC_GetLIOutputClock();
	}

	temp = ((RCC->CFGR>>4) & 0xf);
	if(temp<8){
		AHBPreScalar = 1;
	} else{
		AHBPreScalar = AHB_Prescalar[temp-8];
	}
	temp = ((RCC->CFGR>>10) & 0x7);
	if(temp<4){
		APB2PreScalar = 1;
	} else {
		APB2PreScalar = APB_Prescalar[temp-4];
	}
	pclk2 = SystemClk/AHBPreScalar/APB2PreScalar;

	return pclk2;
}
