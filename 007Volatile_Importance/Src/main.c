#include <stdint.h>
#include <stdio.h>

#define SRAM_Addr 0x20000004

int main(void)
{
    /* Loop forever */
	volatile uint32_t *pSRAMAddr = (uint32_t *)SRAM_Addr;
	while(1){ //Comparing the values at address
		if(*pSRAMAddr){
			break;
		}
	}
	while(1);
}
