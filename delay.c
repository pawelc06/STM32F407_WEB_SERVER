#include "delay.h"

static __IO uint32_t sysTickCounter;

void SysTick_Init(void) {
	/****************************************
	 *SystemFrequency/1000      1ms         *
	 *SystemFrequency/100000    10us        *
	 *SystemFrequency/1000000   1us         *
	 *****************************************/
	while (SysTick_Config(SystemCoreClock / 1000000) != 0) {
	} // One SysTick interrupt now equals 1us

}

/**
 * This method needs to be called in the SysTick_Handler
 */
void TimeTick_Decrement(void) { //called every 1ms
	if (sysTickCounter != 0x00) {
		sysTickCounter--;
	}
}

void delay_us(u32 n) {
	sysTickCounter = n;
	while (sysTickCounter != 0) {
	}
}

void delay_1ms(void) {
	sysTickCounter = 1000;
	//sysTickCounter = 1;
	while (sysTickCounter != 0) {
	}
}

void delay_ms(u32 n) {
	while (n--) {
		delay_1ms();
	}
}

void delay_200ns(void){
	uint8_t i;


	for(i=0;i<8;i++)
	 asm("nop");


/*
	 asm("nop");
	 asm("nop");
	 asm("nop");
	 asm("nop");

	 asm("nop");
	 asm("nop");
	 asm("nop");
	 asm("nop");
	 asm("nop");

	 asm("nop");
	 asm("nop");
	 asm("nop");
	 asm("nop");
	 asm("nop");

	 asm("nop");
	 asm("nop");
	 asm("nop");
	 asm("nop");
	 asm("nop");

	 asm("nop");
	 asm("nop");
*/


}



