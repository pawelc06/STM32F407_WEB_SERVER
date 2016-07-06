/*
 * systimer.h
 *
 *  Created on: 21 maj 2016
 *      Author: Pawe³
 */

#ifndef INC_SYSTIMER_H_
#define INC_SYSTIMER_H_

//#include "stm32f10x_conf.h"
#include "stm32f4xx.h"



/** Wait for X milliseconds.
 *
 * @param ms Milliseconds
 */
void delay_ms(unsigned ms);


/** Initialize the millisecond timer. */
void mstimer_init(void);


/** Return the number of milliseconds since start.
 *
 * @return Milliseconds
 */
uint32_t mstimer_get(void);


#endif /* INC_SYSTIMER_H_ */
