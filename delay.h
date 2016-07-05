#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f4xx.h"

void SysTick_Init(void);
void TimeTick_Decrement(void);
void delay_us(u32 n);
void delay_1ms(void);
void delay_ms(u32 n);
void delay_200ns(void);

#endif
