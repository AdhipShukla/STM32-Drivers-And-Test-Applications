#include<stdint.h>

#define RCC_BASE_ADDRESS 0x40023800UL
#define RCC_CFGR_REG_OFFSET 0x08UL
#define RCC_CFGR_REG_ADDRESS (RCC_BASE_ADDRESS+RCC_CFGR_REG_OFFSET)
#define GPIOA_BASE_ADDRESS 0x40020000
int main(void)
{
    /* Loop forever */
	//First configuring the MCO Master clock output 1 to take input from HSI
	uint32_t *p_RCC_CFGR = (uint32_t *)RCC_CFGR_REG_ADDRESS;
	*p_RCC_CFGR = *p_RCC_CFGR & ~(0x03<<21); //This will set the 21 and 22 bit of p_RCC_CFGR to 1

	//Setting GPIO Cock which is AHB1 bus
	uint32_t *pRCCAhb1Enr = (uint32_t *)(RCC_BASE_ADDRESS + 0x30);
	*pRCCAhb1Enr = *pRCCAhb1Enr|(1<<0);

	//Configure the mode of GPIO pin 8 as alternate function mode
	uint32_t *pGPIOModeReg =(uint32_t *)(GPIOA_BASE_ADDRESS+0x00);
	*pGPIOModeReg &= ~(0x03<<16);
	*pGPIOModeReg |= (0x02<<16);

	//Configure the alternate function to set the mode0 which is MSO1 for PA8
	uint32_t *pGPIOAAltFunHighReg = (uint32_t*)(GPIOA_BASE_ADDRESS + 0x24);
	*pGPIOAAltFunHighReg &=~(0xf << 0);

	for(;;);
}
